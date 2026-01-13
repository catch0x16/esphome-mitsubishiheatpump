#include "cn105_controlflow.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105ControlFlow"; // Logging tag

    CN105ControlFlow::CN105ControlFlow(
            IIODevice* io_device,
            RequestScheduler::TimeoutCallback timeoutCallback,
            RequestScheduler::TerminateCallback terminateCallback
        ): io_device_{io_device},
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

    /**
     * Seek the byte pointer to the beginning of the array
     * Initializes few variables
    */
    void CN105ControlFlow::initBytePointer() {
        this->foundStart = false;
        this->bytesRead = 0;
        this->dataLength = -1;
        this->command = 0;
    }

    void CN105ControlFlow::checkHeader(uint8_t inputData) {
        if (this->bytesRead == 4) {
            if (storedInputData[2] == HEADER[2] && storedInputData[3] == HEADER[3]) {
                ESP_LOGV("Header", "header matches HEADER");
                ESP_LOGV("Header", "[%02X] (%02X) %02X %02X [%02X]<-- header", storedInputData[0], storedInputData[1], storedInputData[2], storedInputData[3], storedInputData[4]);
                ESP_LOGD("Header", "command: (%02X) data length: [%02X]<-- header", storedInputData[1], storedInputData[4]);
                this->command = storedInputData[1];
            }
            this->dataLength = storedInputData[4];
        }
    }

    void CN105ControlFlow::parse(uint8_t inputData) {
        ESP_LOGV("Decoder", "--> %02X [nb: %d]", inputData, this->bytesRead);

        if (!this->foundStart) {                // no packet yet
            if (inputData == HEADER[0]) {
                this->foundStart = true;
                this->bytesRead = 0;
                storedInputData[this->bytesRead++] = inputData;
            } else {
                // unknown bytes
            }
        } else {                                // we are getting a packet
            if (this->bytesRead >= (MAX_DATA_BYTES - 1)) {
                ESP_LOGW("Decoder", "buffer overflow preventive reset (bytesRead=%d)", this->bytesRead);
                this->initBytePointer();
                return;
            }
            storedInputData[this->bytesRead] = inputData;

            checkHeader(inputData);

            if (this->dataLength != -1) {       // is header complete ?
                if ((this->dataLength + 6) > MAX_DATA_BYTES) {
                    ESP_LOGW("Decoder", "declared data length %d too large, resetting parser", this->dataLength);
                    this->initBytePointer();
                    return;
                }

                if ((this->bytesRead) == this->dataLength + 5) {

                    //this->processDataPacket();
                    this->initBytePointer();
                } else {                                        // packet is still filling
                    this->bytesRead++;                          // more data to come
                }
            } else {
                ESP_LOGV("Decoder", "data length toujours pas connu");
                // header is not complete yet
                this->bytesRead++;
            }
        }

    }

    bool CN105ControlFlow::processInput() {
        bool processed = false;
        while (this->io_device_->available()) {
            processed = true;
            uint8_t inputData;
            if (this->io_device_->read(&inputData)) {
                parse(inputData);
            }

        }
        return processed;
    }

}
