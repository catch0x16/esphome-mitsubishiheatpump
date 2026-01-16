#include "cn105_protocol.h"

#include "esphome.h"

#define SAFE_STR(s) ((s) ? (s) : "(null)")

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105Protocol"; // Logging tag

    // Write Protocol

    void CN105Protocol::prepareSetPacket(uint8_t* packet, int length) {
        ESP_LOGV(TAG, "preparing Set packet...");
        memset(packet, 0, length * sizeof(uint8_t));

        for (int i = 0; i < HEADER_LEN && i < length; i++) {
            packet[i] = HEADER[i];
        }
    }

    void CN105Protocol::createInfoPacket(uint8_t* packet, uint8_t code) {
        ESP_LOGD(TAG, "creating Info packet");
        // add the header to the packet
        for (int i = 0; i < INFOHEADER_LEN; i++) {
            packet[i] = INFOHEADER[i];
        }

        // directly set requested info code (0x02, 0x03, 0x06, 0x09, 0x42, ...)
        packet[5] = code;

        // pad the packet out
        for (int i = 0; i < 15; i++) {
            packet[i + 6] = 0x00;
        }

        // add the checksum
        uint8_t chkSum = checkSum(packet, 21);
        packet[21] = chkSum;
    }

    void CN105Protocol::prepareInfoPacket(uint8_t* packet, int length) {
        memset(packet, 0, length * sizeof(uint8_t));
        
        for (int i = 0; i < INFOHEADER_LEN && i < length; i++) {
            packet[i] = INFOHEADER[i];
        }  
    }

    void CN105Protocol::createPacket(uint8_t* packet, CN105State& hpState) {
        prepareSetPacket(packet, PACKET_LEN);

        //ESP_LOGD(TAG, "checking differences bw asked settings and current ones...");
        ESP_LOGD(TAG, "building packet for writing...");

        wantedHeatpumpSettings wantedSettings = hpState.getWantedSettings();
        if (wantedSettings.power != nullptr) {
            ESP_LOGD(TAG, "power -> %s", SAFE_STR(hpState.getPowerSetting()));
            int idx = lookupByteMapIndex(POWER_MAP, 2, hpState.getPowerSetting(), "power (write)");
            if (idx >= 0) { packet[8] = POWER[idx]; packet[6] += CONTROL_PACKET_1[0]; } else { ESP_LOGW(TAG, "Ignoring invalid power setting while building packet"); }
        }

        if (wantedSettings.mode != nullptr) {
            ESP_LOGD(TAG, "heatpump mode -> %s", SAFE_STR(hpState.getModeSetting()));
            int idx = lookupByteMapIndex(MODE_MAP, 5, hpState.getModeSetting(), "mode (write)");
            if (idx >= 0) { packet[9] = MODE[idx]; packet[6] += CONTROL_PACKET_1[1]; } else { ESP_LOGW(TAG, "Ignoring invalid mode setting while building packet"); }
        }

        if (wantedSettings.temperature != -1) {
            if (!hpState.getTempMode()) {
                ESP_LOGD(TAG, "temperature (tempmode is false) -> %f", hpState.getTemperatureSetting());
                int idx = lookupByteMapIndex(TEMP_MAP, 16, hpState.getTemperatureSetting(), "temperature (write)");
                if (idx >= 0) { packet[10] = TEMP[idx]; packet[6] += CONTROL_PACKET_1[2]; } else { ESP_LOGW(TAG, "Ignoring invalid temperature setting while building packet"); }
            } else {
                ESP_LOGD(TAG, "temperature (tempmode is true) -> %f", hpState.getTemperatureSetting());
                float temp = (hpState.getTemperatureSetting() * 2) + 128;
                packet[19] = (int)temp;
                packet[6] += CONTROL_PACKET_1[2];
            }
        }

        if (wantedSettings.fan != nullptr) {
            ESP_LOGD(TAG, "heatpump fan -> %s", SAFE_STR(hpState.getFanSpeedSetting()));
            int idx = lookupByteMapIndex(FAN_MAP, 6, hpState.getFanSpeedSetting(), "fan (write)");
            if (idx >= 0) { packet[11] = FAN[idx]; packet[6] += CONTROL_PACKET_1[3]; } else { ESP_LOGW(TAG, "Ignoring invalid fan setting while building packet"); }
        }

        if (wantedSettings.vane != nullptr) {
            ESP_LOGD(TAG, "heatpump vane -> %s", SAFE_STR(hpState.getVaneSetting()));
            int idx = lookupByteMapIndex(VANE_MAP, 7, hpState.getVaneSetting(), "vane (write)");
            if (idx >= 0) { packet[12] = VANE[idx]; packet[6] += CONTROL_PACKET_1[4]; } else { ESP_LOGW(TAG, "Ignoring invalid vane setting while building packet"); }
        }

        if (wantedSettings.wideVane != nullptr) {
            ESP_LOGD(TAG, "heatpump widevane -> %s", SAFE_STR(hpState.getWideVaneSetting()));
            int idx = lookupByteMapIndex(WIDEVANE_MAP, 8, hpState.getWideVaneSetting(), "wideVane (write)");
            if (idx >= 0) { packet[18] = WIDEVANE[idx] | (hpState.shouldWideVaneAdj() ? 0x80 : 0x00); packet[7] += CONTROL_PACKET_2[0]; } else { ESP_LOGW(TAG, "Ignoring invalid wideVane setting while building packet"); }
        }

        // add the checksum
        uint8_t chkSum = checkSum(packet, 21);
        packet[21] = chkSum;
        //ESP_LOGD(TAG, "debug before write packet:");
        //this->hpPacketDebug(packet, 22, "WRITE");
    }

    // Write Protocol

    // Read Protocol

    void CN105Protocol::parseSettings0x02(uint8_t* packet, CN105State &hpState) {
        heatpumpSettings receivedSettings{};
        heatpumpRunStates receivedRunStates{};
        ESP_LOGD("Decoder", "[0x02 is settings]");

        receivedSettings.connected = true;
        receivedSettings.power = lookupByteMapValue(POWER_MAP, POWER, 2, packet[3], "power reading");
        receivedSettings.iSee = packet[4] > 0x08 ? true : false;
        receivedSettings.mode = lookupByteMapValue(MODE_MAP, MODE, 5, receivedSettings.iSee ? (packet[4] - 0x08) : packet[4], "mode reading");

        ESP_LOGD("Decoder", "[Power : %s]", SAFE_STR(receivedSettings.power));
        ESP_LOGD("Decoder", "[iSee  : %d]", receivedSettings.iSee);
        ESP_LOGD("Decoder", "[Mode  : %s]", SAFE_STR(receivedSettings.mode));

        if (packet[11] != 0x00) {
            int temp = packet[11];
            temp -= 128;
            receivedSettings.temperature = (float)temp / 2;
            hpState.setTempMode(true);
        } else {
            receivedSettings.temperature = lookupByteMapValue(TEMP_MAP, TEMP, 16, packet[5], "temperature reading");
        }

        ESP_LOGD("Decoder", "[Temp °C: %f]", receivedSettings.temperature);

        receivedSettings.fan = lookupByteMapValue(FAN_MAP, FAN, 6, packet[6], "fan reading");
        ESP_LOGD("Decoder", "[Fan: %s]", SAFE_STR(receivedSettings.fan));

        receivedSettings.vane = lookupByteMapValue(VANE_MAP, VANE, 7, packet[7], "vane reading");
        ESP_LOGD("Decoder", "[Vane: %s]", SAFE_STR(receivedSettings.vane));

        // --- START OF MODIFIED SECTION - Reverted widevane section back to more or less original state
        if (packet[10] != 0) {    // wideVane is not always supported
            receivedSettings.wideVane = lookupByteMapValue(WIDEVANE_MAP, WIDEVANE, 8, packet[10] & 0x0F, "wideVane reading");
            hpState.setWideVaneAdj((packet[10] & 0xF0) == 0x80 ? true : false);
            ESP_LOGD("Decoder", "[wideVane: %s (adj:%d)]", SAFE_STR(receivedSettings.wideVane), hpState.shouldWideVaneAdj());
        }
        // --- END OF MODIFIED SECTION ---

        // --- AIRFLOW CONTROL START
        /*
        if (this->airflow_control_select_ != nullptr) {
            if (packet[10] == 0x80) {
                if (receivedSettings.iSee) {
                    receivedRunStates.airflow_control = lookupByteMapValue(AIRFLOW_CONTROL_MAP, AIRFLOW_CONTROL, 3, packet[14], "airflow control reading");
                } else {
                    // For some reason packet[10] is 0x80, but the i-See sensor is not active. 
                    // Some units let us do this, but the real mode is unknown (might be powersave) and the i-See sensor does not get activated.
                    //receivedRunStates.airflow_control = "N/A";
                    ESP_LOGD("Decoder", "i-See sensor not present/active.");
                    receivedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
                }
            } else {
                receivedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
            }
            if (!this->currentRunStates.airflow_control || strcmp(receivedRunStates.airflow_control, this->currentRunStates.airflow_control) != 0) {
                this->currentRunStates.airflow_control = receivedRunStates.airflow_control;
                this->airflow_control_select_->publish_state(receivedRunStates.airflow_control);
            }
        }
        */
        hpState.updateCurrentSettings(receivedSettings);
    }

    void CN105Protocol::parseStatus0x03(uint8_t* packet, CN105State& hpState) {
        //ESP_LOGD("Decoder", "[0x03 room temperature]");
        //this->last_received_packet_sensor->publish_state("0x62-> 0x03: Data -> Room temperature");
        //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
        // FC 62 01 30 10 03 00 00 0E 00 94 B0 B0 FE 42 00 01 0A 64 00 00 A9
        //                         RT    OT RT SP ?? ?? ?? RM RM RM
        // FC 62 01 30 10 03 00 00 0B 00 94 AB 00 00 00 00 00 00 00 00 00 10 
        // RT = room temperature (in old format and in new format)
        // OT = outside air temperature
        // SP = room setpoint temperature?
        // RM = indoor unit operating time in minutes

        heatpumpStatus receivedStatus{};
        if (packet[5] > 1) {
            receivedStatus.outsideAirTemperature = (packet[5] - 128) / 2.0f;
        } else {
            receivedStatus.outsideAirTemperature = NAN;
        }

        if (packet[6] != 0x00) {
            int temp = packet[6];
            temp -= 128;
            receivedStatus.roomTemperature = temp / 2.0f;
            ESP_LOGD(LOG_TEMP_SENSOR_TAG, "packet[6]  --> [Room °C: %f]", receivedStatus.roomTemperature);
        } else {
            receivedStatus.roomTemperature = lookupByteMapValue(ROOM_TEMP_MAP, ROOM_TEMP, 32, packet[3]);
            ESP_LOGD(LOG_TEMP_SENSOR_TAG, "packet[3] map --> [Room °C : %f]", receivedStatus.roomTemperature);
        }

        receivedStatus.runtimeHours = float((packet[11] << 16) | (packet[12] << 8) | packet[13]) / 60;

        ESP_LOGD("Decoder", "[Room °C: %f]", receivedStatus.roomTemperature);
        ESP_LOGD("Decoder", "[OAT  °C: %f]", receivedStatus.outsideAirTemperature);

        heatpumpStatus currentStatus = hpState.getCurrentStatus();
        // no change with this packet to currentStatus for operating and compressorFrequency
        receivedStatus.operating = currentStatus.operating;
        receivedStatus.compressorFrequency = currentStatus.compressorFrequency;
        receivedStatus.inputPower = currentStatus.inputPower;
        receivedStatus.kWh = currentStatus.kWh;

        hpState.updateCurrentStatus(receivedStatus);
    }

    void CN105Protocol::parseTimers0x05(uint8_t* packet, CN105State& hpState) {
        heatpumpTimers receivedTimers;
        receivedTimers.mode                = lookupByteMapValue(TIMER_MODE_MAP, TIMER_MODE, 4, packet[3]);
        receivedTimers.onMinutesSet        = packet[4] * TIMER_INCREMENT_MINUTES;
        receivedTimers.onMinutesRemaining  = packet[6] * TIMER_INCREMENT_MINUTES;
        receivedTimers.offMinutesSet       = packet[5] * TIMER_INCREMENT_MINUTES;
        receivedTimers.offMinutesRemaining = packet[7] * TIMER_INCREMENT_MINUTES;

        hpState.setTimers(receivedTimers);
    }

    void CN105Protocol::parseStatus0x06(uint8_t* packet, CN105State& hpState) {
        //FC 62 01 30 10 06 00 00 1A 01 00 00 00 00 00 00 00 00 00 00 00 3C
        //MSZ-RW25VGHZ-SC1 / MUZ-RW25VGHZ-SC1
        //FC 62 01 30 10 06 00 00 00 01 00 08 05 50 00 00 42 00 00 00 00 B7
        //                           OP IP IP EU EU       ??
        //FC 62 01 30 10 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 57
        // OP = operating status (1 = compressor running, 0 = standby)
        // IP = Current input power in Watts (16-bit decimal)
        // EU = energy usage
        //      (used energy in kWh = value/10)
        //      TODO: Currently the maximum size of the counter is not known and
        //            if the counter extends to other bytes.
        // ?? = unknown bytes that appear to have a fixed/constant value

        heatpumpStatus receivedStatus;
        receivedStatus.operating = packet[4];
        receivedStatus.compressorFrequency = packet[3];
        receivedStatus.inputPower = (packet[5] << 8) | packet[6];
        receivedStatus.kWh = float((packet[7] << 8) | packet[8]) / 10.0;

        hpState.setOperating(receivedStatus.operating);
        hpState.setCompressorFrequency(receivedStatus.compressorFrequency);
        hpState.setInputPower(receivedStatus.inputPower);
        hpState.setKWh(receivedStatus.kWh);
    }
    
    void CN105Protocol::parseFunctions0x20(uint8_t* packet, CN105State& hpState) {
        hpState.getFunctions().setData1(&packet[1]);
    }

    void CN105Protocol::parseFunctions0x22(uint8_t* packet, CN105State& hpState) {
        hpState.getFunctions().setData2(&packet[1]);
    }

    void CN105Protocol::getDataFromResponsePacket(const uint8_t* packet, const int dataLength, CN105State& hpState) {
        // D'abord, laissons l'orchestrateur traiter les codes connus
        const uint8_t code = packet[0];
        // Sinon, switch pour les cas non gérés par l'orchestrateur
        switch (code) {

        case 0x04:
            /* unknown */
            ESP_LOGI("Decoder", "[0x04 is unknown : not implemented]");
            //this->last_received_packet_sensor->publish_state("0x62-> 0x04: Data -> Unknown");
            break;

        case 0x05:
            /* timer packet */
            ESP_LOGW("Decoder", "[0x05 is Timer : not implemented]");
            //this->last_received_packet_sensor->publish_state("0x62-> 0x05: Data -> Timer Packet");
            break;

        case 0x06:
            break; // orchestrator
        case 0x09:
            break; // orchestrator

        case 0x10:
            ESP_LOGD("Decoder", "[0x10 is Unknown : not implemented]");
            //this->getAutoModeStateFromResponsePacket();
            break;

        case 0x20: // fallthrough
            if (dataLength == 0x10) {
                //this->parseFunctions0x20(packet, hpState);
                ESP_LOGE(LOG_CYCLE_TAG, "Got functions packet 1, requesting part 2");
                //this->getFunctionsPart2();
            }
            break;
        case 0x22: {
            ESP_LOGD("Decoder", "[Packet Functions 0x20 et 0x22]");
            //this->last_received_packet_sensor->publish_state("0x62-> 0x20/0x22: Data -> Packet functions");
            if (dataLength == 0x10) {
                //this->parseFunctions0x22(packet, hpState);
                ESP_LOGE(LOG_CYCLE_TAG, "Got functions packet 2");
                //this->functionsArrived();
            }
        }
                break;

        case 0x42:
            break; // orchestrator

        default:
            ESP_LOGW("Decoder", "packet type [%02X] <-- unknown and unexpected", packet[0]);
            //this->last_received_packet_sensor->publish_state("0x62-> ?? : Data -> Unknown");
            break;
        }

    }
    // Read Protocol
}
