#include "cn105_controlflow.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105ControlFlow"; // Logging tag

    CN105ControlFlow::CN105ControlFlow(
        RequestScheduler::TimeoutCallback timeoutCallback,
        RequestScheduler::TerminateCallback terminateCallback
    ): scheduler_(
            // send_callback: envoie un paquet via buildAndSendInfoPacket
            [this](uint8_t code) {
                ESP_LOGI(TAG, "scheduled code: %d", code);
                this->buildAndSendInfoPacket(code);
            },
            timeoutCallback,
            terminateCallback
        )  {
        
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

    void CN105ControlFlow::loop(CN105State& hpState) {
        this->scheduler_.send_next_after(0x00); // This will call terminate cycle when complete
    }

}
