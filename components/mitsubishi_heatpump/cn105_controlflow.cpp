#include "cn105_controlflow.h"

#include "cn105_logging.h"

#include "esphome.h"

namespace devicestate {

    static const char* TAG = "CN105ControlFlow"; // Logging tag

    CN105ControlFlow::CN105ControlFlow(
            CN105Connection* connection,
            CN105State* hpState,
            RequestScheduler::TimeoutCallback timeoutCallback,
            RequestScheduler::TerminateCallback terminateCallback,
            RetryCallback retryCallback
        ): connection_{connection},
            timeoutCallback_{timeoutCallback},
            retryCallback_{retryCallback},
            scheduler_(
                // send_callback: envoie un paquet via buildAndSendInfoPacket
                [this](uint8_t code) {
                    this->buildAndSendInfoPacket(code);
                },
                timeoutCallback,
                terminateCallback,
                [this]() -> CN105State* { return this->hpState_; }
            ),
            hpProtocol{} {
        this->hpState_ = hpState;
        this->debounce_delay_ = 0;
        this->remote_temp_timeout_ = 4294967295;    // uint32_t max
    }

    void CN105ControlFlow::set_debounce_delay(uint32_t delay) {
        this->debounce_delay_ = delay;
        //ESP_LOGI(LOG_ACTION_EVT_TAG, "set_debounce_delay is set to %lu", delay);
        log_info_uint32(LOG_ACTION_EVT_TAG, "set_debounce_delay is set to ", delay);
    }

#ifndef USE_ESP32
    /**
     * This methode emulates the esp32 lock_guard feature with a boolean variable
     *
    */
    void CN105ControlFlow::emulateMutex(const char* retryName, std::function<void()>&& f) {
        retryCallback_(retryName, 100, 10, [this, f, retryName](uint8_t retry_count) {
            if (this->wantedSettingsMutex) {
                if (retry_count < 1) {
                    ESP_LOGW(retryName, "10 retry calls failed because mutex was locked, forcing unlock...");
                    this->wantedSettingsMutex = false;  // Fixed: was incorrectly set to true
                    f();
                    this->wantedSettingsMutex = false;
                    return esphome::RetryResult::DONE;
                }
                ESP_LOGI(retryName, "wantedSettingsMutex is already locked, defferring...");
                return esphome::RetryResult::RETRY;
            } else {
                this->wantedSettingsMutex = true;
                ESP_LOGD(retryName, "emulateMutex normal behaviour, locking...");
                f();
                ESP_LOGD(retryName, "emulateMutex unlocking...");
                this->wantedSettingsMutex = false;
                return esphome::RetryResult::DONE;
            }
            });
    }
#endif

    void CN105ControlFlow::sendWantedSettingsDelegate() {
        this->hpState_->getWantedSettings().sent();

        wantedHeatpumpSettings wantedSettings = this->hpState_->getWantedSettings();
        debugSettings("wantedSettings", wantedSettings);
        // and then we send the update packet

        uint8_t packet[PACKET_LEN] = {};
        hpProtocol.createPacket(packet, *this->hpState_);
        this->connection_->writePacket(packet, PACKET_LEN);
        hpPacketDebug(packet, 22, "WRITE_SETTINGS");

        this->hpState_->updateCurrentSettings(wantedSettings);

        // as soon as the packet is sent, we reset the settings
        //this->hpState_->getWantedSettings().resetSettings();
        this->hpState_->resetWantedSettings();
    }

    bool CN105ControlFlow::sendWantedSettings() {
        if (!this->connection_->ensureActiveConnection()) {
            return false;
        }

        auto callback = [this]() {
            this->sendWantedSettingsDelegate();
        };
        this->acquireWantedSettingsLock(callback);
        return true;
    }

    void CN105ControlFlow::checkPendingWantedSettings(cycleManagement& loopCycle) {
        long now = CUSTOM_MILLIS;

        if (!this->hpState_->getWantedSettings().hasChanged) {
            ESP_LOGI(LOG_ACTION_EVT_TAG, "Skipping checkPendingWantedSettings: %s", TRUEFALSE(this->hpState_->getWantedSettings().hasChanged));
            return;
        }

        // Skip if not enough time has passed since last change (debounce)
        if ((now - this->hpState_->getWantedSettings().lastChange) < this->debounce_delay_) {
            ESP_LOGI(LOG_ACTION_EVT_TAG, "Skipping checkPendingWantedSettings: debounce delay not elapsed");
            return;
        }

        ESP_LOGI(LOG_ACTION_EVT_TAG, "checkPendingWantedSettings - wanted settings have changed, sending them to the heatpump...");

        if (this->sendWantedSettings()) {
            // as we've just sent a packet to the heatpump, we let it time for process
            // this might not be necessary but, we give it a try because of issue #32
            // https://github.com/echavet/MitsubishiCN105ESPHome/issues/32
            loopCycle.deferCycle();
        }
    }

    bool CN105ControlFlow::sendWantedRunStates() {
        /*
        uint8_t packet[PACKET_LEN] = {};

        hpProtocol.prepareSetPacket(packet, PACKET_LEN);

        packet[5] = 0x08;
        if (hpState.getWantedRunStates().airflow_control != nullptr) {
            ESP_LOGD(TAG, "airflow control -> %s", getAirflowControlSetting());
            packet[11] = AIRFLOW_CONTROL[lookupByteMapIndex(AIRFLOW_CONTROL_MAP, 3, getAirflowControlSetting(), "run state (write)")];
            packet[6] += RUN_STATE_PACKET_1[4];
        }
        if (hpState.getWantedRunStates().air_purifier > -1) {
            if (getAirPurifierRunState() != currentRunStates.air_purifier) {
                ESP_LOGI(TAG, "air purifier switch state -> %s", getAirPurifierRunState() ? "ON" : "OFF");
                packet[17] = getAirPurifierRunState() ? 0x01 : 0x00;
                packet[7] += RUN_STATE_PACKET_2[1];
            }
        }
        if (hpState.getWantedRunStates().night_mode > -1) {
            if (getNightModeRunState() != currentRunStates.night_mode) {
                ESP_LOGI(TAG, "night mode switch state -> %s", this->getNightModeRunState() ? "ON" : "OFF");
                packet[18] = getNightModeRunState() ? 0x01 : 0x00;
                packet[7] += RUN_STATE_PACKET_2[2];
            }
        }
        if (hpState.getCirculatorRunState().circulator > -1) {
            if (hpState.getCirculatorRunState() != currentRunStates.circulator) {
                ESP_LOGI(TAG, "circulator switch state -> %s", getCirculatorRunState() ? "ON" : "OFF");
                packet[19] = hpState.getCirculatorRunState() ? 0x01 : 0x00;
                packet[7] += RUN_STATE_PACKET_2[3];
            }
        }

        // Add the checksum
        uint8_t chkSum = checkSum(packet, 21);
        packet[21] = chkSum;
        ESP_LOGD(LOG_SET_RUN_STATE, "Sending set run state package (0x08)");
        writePacket(packet, PACKET_LEN);

        //this->publishWantedRunStatesStateToHA();

        this->wantedRunStates.resetSettings();
        */
        return false;
    }

    void CN105ControlFlow::checkPendingWantedRunStates(cycleManagement& loopCycle) {
        long now = CUSTOM_MILLIS;
        if (!(this->hpState_->getWantedRunStates().hasChanged) || (now - this->hpState_->getWantedRunStates().lastChange < this->debounce_delay_)) {
            return;
        }
        ESP_LOGI(LOG_ACTION_EVT_TAG, "checkPendingWantedRunStates - wanted run states have changed, sending them to the heatpump...");
        if (this->sendWantedRunStates()) {
            // as we've just sent a packet to the heatpump, we let it time for process
            // this might not be necessary but, we give it a try because of issue #32
            // https://github.com/echavet/MitsubishiCN105ESPHome/issues/32
            loopCycle.deferCycle();
        }
    }

    void CN105ControlFlow::buildAndSendRequestPacket(int packetType) {
        // Legacy path kept temporarily if some callsites still pass packetType indices.
        // Map legacy indices to real codes and delegate to buildAndSendInfoPacket.
        uint8_t code = 0x02; // default to settings
        switch (packetType) {
        case 0: code = 0x02; break; // RQST_PKT_SETTINGS
        case 1: code = 0x03; break; // RQST_PKT_ROOM_TEMP
        case 2: code = 0x04; break; // RQST_PKT_UNKNOWN
        case 3: code = 0x05; break; // RQST_PKT_TIMERS
        case 4: code = 0x06; break; // RQST_PKT_STATUS
        case 5: code = 0x09; break; // RQST_PKT_STANDBY
        case 6: code = 0x42; break; // RQST_PKT_HVAC_OPTIONS
        default: code = 0x02; break;
        }
        this->buildAndSendInfoPacket(code);
    }

    void CN105ControlFlow::buildAndSendInfoPacket(uint8_t code) {
        uint8_t packet[PACKET_LEN] = {};
        hpProtocol.createInfoPacket(packet, code);
        this->connection_->writePacket(packet, PACKET_LEN);
    }

    void CN105ControlFlow::buildAndSendRequestsInfoPackets(cycleManagement& loopCycle) {
        if (this->connection_->isConnected()) {
            ESP_LOGV(LOG_UPD_INT_TAG, "triggering infopacket because of update interval tick");
            ESP_LOGV("CONTROL_WANTED_SETTINGS", "hasChanged is %s", TRUEFALSE(this->hpState_->getWantedSettings().hasChanged));
            loopCycle.cycleStarted();
            //this->nbCycles_++;
            // Send the first activatable request (the list is registered once in the constructor)
            this->scheduler_.send_next_after(0x00); // 0x00 -> start, pick first eligible
        } else {
            this->connection_->reconnectIfConnectionLost();
        }
    }

    void CN105ControlFlow::loop(cycleManagement& loopCycle) {
        // Bootstrap connexion CN105 (UART + CONNECT) depuis loop()
        this->connection_->ensureConnection();

        // While the connection has not succeeded, we don't start ANY cycle/write (otherwise it short-circuits the delay).
        // We still continue to read/process input to detect 0x7A/0x7B (connection success).
        const bool can_talk_to_hp = this->connection_->isConnected();
        if (!this->connection_->processInput(
                [this](const uint8_t* packet, const int dataLength) {
                    const uint8_t code = packet[0];
                    if (this->scheduler_.process_response(code)) {
                        return;
                    }
                    ESP_LOGW(TAG, "Scheduler failed to process response.");
                    this->hpProtocol.getDataFromResponsePacket(packet, dataLength, *this->hpState_);
                })) {                                            // if we don't get any input: no read op

            if (!can_talk_to_hp) {
                return;
            }

            if (this->hpState_->getWantedSettings().hasChanged && (!loopCycle.isCycleRunning())) {
                //ESP_LOGI(TAG, "Settings changed, updating: %s", TRUEFALSE(this->hpState_->getWantedSettings().hasChanged));
                this->checkPendingWantedSettings(loopCycle);
            } else if (this->hpState_->getWantedRunStates().hasChanged && (!loopCycle.isCycleRunning())) {
                ESP_LOGD(TAG, "Run states changed, updating: %s", TRUEFALSE(this->hpState_->getWantedRunStates().hasChanged));
                this->checkPendingWantedRunStates(loopCycle);
            } else {
                if (loopCycle.isCycleRunning()) {                         // if we are  running an update cycle
                    loopCycle.checkTimeout();
                } else { // we are not running a cycle
                    if (loopCycle.hasUpdateIntervalPassed()) {
                        this->buildAndSendRequestsInfoPackets(loopCycle);            // initiate an update cycle with this->cycleStarted();
                    }
                }
            }
        }
    }

    void CN105ControlFlow::registerInfoRequests() {
        scheduler_.clear_requests();

        // 0x02 Settings
        InfoRequest r_settings("settings", "Settings", 0x02, 3, 0);
        r_settings.onResponse = [this](CN105State& self) {
            this->hpProtocol.parseSettings0x02(this->connection_->getData(), self);
        };
        scheduler_.register_request(r_settings);

        // 0x03 Room temperature
        InfoRequest r_room("room_temp", "Room temperature", 0x03, 3, 0);
        r_room.onResponse = [this](CN105State& self) {
            this->hpProtocol.parseStatus0x03(this->connection_->getData(), self);
         };
        scheduler_.register_request(r_room);

        // 0x06 Status
        InfoRequest r_status("status", "Status", 0x06, 3, 0);
        r_status.onResponse = [this](CN105State& self) {
            this->hpProtocol.parseStatus0x06(this->connection_->getData(), self);
        };
        scheduler_.register_request(r_status);

        // 0x09 Standby/Power
        InfoRequest r_power("standby", "Power/Standby", 0x09, 3, 500);
        r_power.onResponse = [this](CN105State& self) {
            this->hpProtocol.parseStatus0x06(this->connection_->getData(), self);
        };
        scheduler_.register_request(r_power);

        // 0x42 HVAC options
        InfoRequest r_hvac_opts("hvac_options", "HVAC options", 0x42, 3, 500);
        r_hvac_opts.canSend = [this](const CN105State& self) {
            (void)self;
            //return (this->air_purifier_switch_ != nullptr || this->night_mode_switch_ != nullptr || this->circulator_switch_ != nullptr);
            return false;
        };
        r_hvac_opts.onResponse = [this](CN105State& self) {
            //(void)self; this->getHVACOptionsFromResponsePacket();
            ESP_LOGW(TAG, "hvac_options");
        };
        r_hvac_opts.disabled = true;
        scheduler_.register_request(r_hvac_opts);

        // Placeholders
        InfoRequest r_unknown("unknown", "Unknown", 0x04, 1, 0);
        r_unknown.disabled = true;
        scheduler_.register_request(r_unknown);

        InfoRequest r_timers("timers", "Timers", 0x05, 1, 0);
        r_timers.disabled = true;
        scheduler_.register_request(r_timers);

        // Call to the new dedicated method
        //this->registerHardwareSettingsRequests();
    }

    bool CN105ControlFlow::getOffsetDirection() {
        if (this->hpState_->getCurrentSettings().mode == nullptr) {
            return false;
        }

        if (std::strcmp(this->hpState_->getCurrentSettings().mode, "HEAT") == 0) {
            return true;
        }
        return false;
    }

    void CN105ControlFlow::setRemoteTemperature(const float current) {
        if (std::isnan(current)) {
            ESP_LOGW(LOG_REMOTE_TEMP, "Remote temperature is NaN, ignoring.");
            return;
        }

        const float normalizedRemoteTemp = this->getOffsetDirection()
            ? std::ceil(current * 2.0) / 2.0
            : std::floor(current * 2.0) / 2.0;
        
        this->remoteTemperature_ = normalizedRemoteTemp;
        this->shouldSendExternalTemperature_ = true;
        ESP_LOGD(LOG_REMOTE_TEMP, "setting remote temperature to %f", this->remoteTemperature_);
    }

    void CN105ControlFlow::completeCycle() {
        if (this->shouldSendExternalTemperature_) {
            // We will receive ACK packet for this.
            // Sending WantedSettings must be delayed in this case (lastSend timestamp updated).        
            ESP_LOGI(LOG_REMOTE_TEMP, "Sending remote temperature...");
            this->sendRemoteTemperature();
        }
    }

    void CN105ControlFlow::sendRemoteTemperature() {
        this->shouldSendExternalTemperature_ = false;

        uint8_t packet[PACKET_LEN] = {};
        hpProtocol.prepareSetPacket(packet, PACKET_LEN);

        packet[5] = 0x07;
        if (this->remoteTemperature_ > 0) {
            packet[6] = 0x01;
            float temp = round(this->remoteTemperature_ * 2);
            packet[7] = static_cast<uint8_t>(temp - 16);
            packet[8] = static_cast<uint8_t>(temp + 128);
        } else {
            packet[8] = 0x80; //MHK1 send 80, even though it could be 00, since ControlByte is 00
        }
        // add the checksum
        uint8_t chkSum = checkSum(packet, 21);
        packet[21] = chkSum;
        ESP_LOGD(LOG_REMOTE_TEMP, "Sending remote temperature packet... -> %f", this->remoteTemperature_);
        this->connection_->writePacket(packet, PACKET_LEN);

        // this resets the timeout
        this->pingExternalTemperature();
    }

    void CN105ControlFlow::pingExternalTemperature() {
        timeoutCallback_(SCHEDULER_REMOTE_TEMP_TIMEOUT, this->remote_temp_timeout_, [this]() {
            ESP_LOGW(LOG_REMOTE_TEMP, "Remote temperature timeout occured, fall back to internal temperature!");
            this->setRemoteTemperature(0);
        });
    }

    void CN105ControlFlow::acquireWantedSettingsLock(AcquireCallback callback) {
#ifdef USE_ESP32
        std::lock_guard<std::mutex> guard(wantedSettingsMutex);
        callback();
#else
        this->emulateMutex("WRITE_SETTINGS", std::move(callback));
#endif
    }

}
