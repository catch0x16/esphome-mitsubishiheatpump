#include "cn105_controlflow.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105ControlFlow"; // Logging tag

    CN105ControlFlow::CN105ControlFlow(
            CN105Connection* connection,
            CN105State* hpState,
            RequestScheduler::TimeoutCallback timeoutCallback,
            RequestScheduler::TerminateCallback terminateCallback
        ): connection_{connection},
            scheduler_(
                // send_callback: envoie un paquet via buildAndSendInfoPacket
                [this](uint8_t code) {
                    ESP_LOGI(TAG, "scheduled code: %d", code);
                    this->buildAndSendInfoPacket(code);
                },
                timeoutCallback,
                terminateCallback,
                [this]() -> CN105State* { return this->hpState_; }
            ),
            hpProtocol{} {
        this->hpState_ = hpState;
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
            ESP_LOGV("CONTROL_WANTED_SETTINGS", "hasChanged is %s", wantedSettings.hasChanged ? "true" : "false");
            loopCycle.cycleStarted();
            //this->nbCycles_++;
            // Envoie la première requête activable (la liste est enregistrée une fois au constructeur)
            this->scheduler_.send_next_after(0x00); // 0x00 -> start, pick first eligible
        } else {
            this->connection_->reconnectIfConnectionLost();
        }
    }

    void CN105ControlFlow::loop(cycleManagement& loopCycle) {
        // Bootstrap connexion CN105 (UART + CONNECT) depuis loop()
        this->connection_->ensureConnection();

        // Tant que la connexion n'a pas réussi, on ne lance AUCUN cycle/écriture (sinon ça court-circuite le délai).
        // On continue quand même à lire/processer l'input afin de détecter le 0x7A/0x7B (connection success).
        const bool can_talk_to_hp = this->connection_->isConnected();
        if (!this->connection_->processInput([this, &loopCycle]() {
                    // let's say that the last complete cycle was over now
                    loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
                    this->hpState_->getCurrentSettings().resetSettings();      // each time we connect, we need to reset current setting to force a complete sync with ha component state and receievdSettings
                    this->hpState_->getCurrentRunStates().resetSettings();
                }, 
                [this](const uint8_t* packet, const int dataLength) {
                    const uint8_t code = packet[0];
                    if (this->scheduler_.process_response(code)) {
                        return;
                    }
                    this->hpProtocol.getDataFromResponsePacket(packet, dataLength, *this->hpState_);
                })) {                                            // if we don't get any input: no read op

            if (!can_talk_to_hp) {
                return;
            }

            if ((this->hpState_->getWantedSettings().hasChanged) && (!loopCycle.isCycleRunning())) {
                //this->checkPendingWantedSettings();
            } else if ((this->hpState_->getWantedRunStates().hasChanged) && (!loopCycle.isCycleRunning())) {
                //this->checkPendingWantedRunStates();
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

}
