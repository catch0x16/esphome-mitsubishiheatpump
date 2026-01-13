#include "cn105_controlflow.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105ControlFlow"; // Logging tag

    CN105ControlFlow::CN105ControlFlow(
            CN105Connection* connection,
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
                terminateCallback
            ) {
        
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

    bool CN105ControlFlow::processInput(CN105State& hpState) {
        uint8_t storedInputData[MAX_DATA_BYTES] = {}; // multi-byte data

        bool processed = false;
        while (this->connection_->readNextPacket(storedInputData)) {
            processed = true;
            // TODO: Parse the packet
        }
        return processed;
    }

    void CN105ControlFlow::loop(cycleManagement& loopCycle, CN105State& hpState) {
        // Bootstrap connexion CN105 (UART + CONNECT) depuis loop()
        //this->maybe_start_connection_();

        // Tant que la connexion n'a pas réussi, on ne lance AUCUN cycle/écriture (sinon ça court-circuite le délai).
        // On continue quand même à lire/processer l'input afin de détecter le 0x7A/0x7B (connection success).
        //const bool can_talk_to_hp = this->isHeatpumpConnected_;

        if (!this->processInput(hpState)) {                                            // if we don't get any input: no read op
            //if (!can_talk_to_hp) {
            //    return;
            //}
            if ((hpState.getWantedSettings().hasChanged) && (!loopCycle.isCycleRunning())) {
                //this->checkPendingWantedSettings();
            } else if ((hpState.getWantedRunStates().hasChanged) && (!loopCycle.isCycleRunning())) {
                //this->checkPendingWantedRunStates();
            } else {
                if (loopCycle.isCycleRunning()) {                         // if we are  running an update cycle
                    loopCycle.checkTimeout();
                } else { // we are not running a cycle
                    if (loopCycle.hasUpdateIntervalPassed()) {
                        //this->buildAndSendRequestsInfoPackets();            // initiate an update cycle with this->cycleStarted();
                    }
                }
            }
        }
    }

}
