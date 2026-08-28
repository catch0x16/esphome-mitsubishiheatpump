#include "cn105_connection.h"

#include "esphome.h"
#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif
#include "logging.h"

#include "cn105_utils.h"
#include "cn105_logging.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105Connection"; // Logging tag

    CN105Connection::CN105Connection(
                IIODevice* io_device,
                TimeoutCallback timeoutCallback,
                ConnectedCallback connectedCallback,
                int update_interval) :
            io_device_{io_device},
            timeoutCallback_{timeoutCallback},
            connectedCallback_{connectedCallback},
            update_interval_{update_interval} {
        this->boot_ms_ = CUSTOM_MILLIS;
        this->initBytePointer();
        this->lastResponseMs = CUSTOM_MILLIS;
    }

    bool CN105Connection::isConnected() {
        return isHeatpumpConnected_;
    }

    void CN105Connection::ensureConnection() {
        if (this->conn_bootstrap_started_) {
            return;
        }

        // Global timeout: after 2 minutes we start even without WiFi
        if (!this->conn_timeout_armed_) {
            ESP_LOGI(TAG, "Connection timeout not armed, arming.");

            this->conn_timeout_armed_ = true;
            timeoutCallback_("cn105_bootstrap_timeout", 120000, [this]() {
                if (this->conn_bootstrap_started_) return;
                    ESP_LOGW(LOG_CONN_TAG, "Bootstrap connection: timeout 120s, starting CN105 anyway");
                    this->conn_bootstrap_started_ = true;
                    this->setupUART();
                    this->sendFirstConnectionPacket();
                });
        }

    #ifdef USE_WIFI
        if (esphome::wifi::global_wifi_component != nullptr && !esphome::wifi::global_wifi_component->is_connected()) {
            if (!this->conn_wait_logged_) {
                this->conn_wait_logged_ = true;
                ESP_LOGI(LOG_CONN_TAG, "Bootstrap connection: waiting for WiFi before init UART/CONNECT");
            }
            return;
        }
    #endif

        // Grace delay to let OTA log stream connect (avoids missing the CONNECT sequence)
        const uint32_t grace_ms = this->conn_bootstrap_delay_ms_;
        const uint32_t elapsed = CUSTOM_MILLIS - this->boot_ms_;
        if (elapsed < grace_ms) {
            if (!this->conn_grace_logged_) {
                this->conn_grace_logged_ = true;
                ESP_LOGI(LOG_CONN_TAG, "Bootstrap connection: grace delay %ums for OTA logs", grace_ms);
            }
            return;
        }

        this->conn_bootstrap_started_ = true;
        ESP_LOGI(LOG_CONN_TAG, "Bootstrap connection: init UART + sending CONNECT (loop)");
        this->setupUART();
        this->sendFirstConnectionPacket();
    }

    bool CN105Connection::ensureActiveConnection() {
        if (this->isConnectionActive() && this->isUARTConnected_) {
            if (CUSTOM_MILLIS - this->lastSend > 300) {        // we don't want to send too many packets
                //this->cycleEnded();   // only if we let the cycle be interrupted to send wented settings
                return true;
            } else {
                ESP_LOGD(TAG, "will send later because we've sent one too recently...");
            }
        } else {
            ESP_LOGW(TAG, "attempting to reconnect...");
            this->reconnectIfConnectionLost();
        }
        return false;
    }

    void CN105Connection::reconnectIfConnectionLost() {
        long reconnectTimeMs = CUSTOM_MILLIS - this->lastReconnectTimeMs;

        if (reconnectTimeMs < this->update_interval_) {
            ESP_LOGW(TAG, "reconnect time to recent...");
            return;
        }

        if (!this->isConnectionActive()) {
            long connectTimeMs = CUSTOM_MILLIS - this->lastConnectRqTimeMs;
            if (connectTimeMs > this->update_interval_) {
                long lrTimeMs = CUSTOM_MILLIS - this->lastResponseMs;
                ESP_LOGW(TAG, "Heatpump has not replied for %ld s", lrTimeMs / 1000);
                ESP_LOGI(TAG, "We think Heatpump is not connected anymore..");
                this->reconnectUART();
            }
        }
    }

    /**
     * Seek the byte pointer to the beginning of the array
     * Initializes few variables
    */
    void CN105Connection::initBytePointer() {
        this->foundStart = false;
        this->bytesRead = 0;
        this->dataLength = -1;
        this->command = 0;
    }

    bool CN105Connection::checkSum() {
        // Bounds check: ensure we don't read past buffer
        if (this->bytesRead >= MAX_DATA_BYTES || this->dataLength < 0 ||
            (this->dataLength + 5) > MAX_DATA_BYTES) {
            ESP_LOGW("chkSum", "Invalid packet dimensions: bytesRead=%d dataLength=%d",
                     this->bytesRead, this->dataLength);
            return false;
        }

        uint8_t packetCheckSum = storedInputData[this->bytesRead];
        uint8_t processedCS = 0;

        ESP_LOGV("chkSum", "controling chkSum should be: %02X ", packetCheckSum);

        for (int i = 0;i < this->dataLength + 5;i++) {
            ESP_LOGV("chkSum", "adding %02X to %03X --> %X", this->storedInputData[i], processedCS, processedCS + this->storedInputData[i]);
            processedCS += this->storedInputData[i];
        }

        processedCS = (0xfc - processedCS) & 0xff;

        if (packetCheckSum == processedCS) {
            ESP_LOGD("chkSum", "OK-> %02X=%02X ", processedCS, packetCheckSum);
        } else {
            ESP_LOGW("chkSum", "KO-> %02X!=%02X ", processedCS, packetCheckSum);
            // Pendant le handshake, une erreur de checksum est un signal utile: logguer la trame sous CN105_CONN
            if (!this->isHeatpumpConnected_) {
                ESP_LOGD(LOG_CONN_TAG, "Checksum KO during handshake (computed=%02X packet=%02X, cmd=0x%02X len=%d)",
                    processedCS, packetCheckSum, this->command, this->dataLength);
                hpPacketDebug(this->storedInputData, this->bytesRead + 1, LOG_CONN_TAG);
            }
        }

        return (packetCheckSum == processedCS);
    }


    // SERIAL_8E1
    void CN105Connection::setupUART() {
        this->setConnectionState(false);
        this->isUARTConnected_ = false;

        if (io_device_->begin()) {
            ESP_LOGI(LOG_CONN_TAG, "UART configured as SERIAL_8E1");
            this->isUARTConnected_ = true;
            this->initBytePointer();
        } else {
            ESP_LOGW(LOG_CONN_TAG, "UART is not configured as SERIAL_8E1");
        }
    }

    void CN105Connection::disconnectUART() {
        ESP_LOGD(TAG, "disconnectUART()");
        this->isUARTConnected_ = false;
        this->firstRun = true;
    }

    void CN105Connection::reconnectUART() {
        ESP_LOGD(TAG, "reconnectUART()");
        this->lastReconnectTimeMs = CUSTOM_MILLIS;
        this->disconnectUART();
        // Disabled: low-level UART fallback (ESP-IDF 5.4.x) can interfere with
        // handshake/fallback tests. We let UARTComponent handle standard reinit.
        //this->force_low_level_uart_reinit();
        this->setupUART();
        this->sendFirstConnectionPacket();
    }

    void CN105Connection::sendFirstConnectionPacket() {
        if (this->isUARTConnected_) {
            this->lastReconnectTimeMs = CUSTOM_MILLIS;          // marker to prevent to many reconnections
            this->setConnectionState(true);
            uint8_t packet[CONNECT_LEN];
            memcpy(packet, CONNECT, CONNECT_LEN);

            // Handshake mode selection: standard (0x5A) or installer (0x5B)
            packet[1] = 0x5A;
            // CONNECT has a pre-calculated checksum in the constant; if we modify the command byte, we must recalculate it.
            packet[CONNECT_LEN - 1] = devicestate::checkSum(packet, CONNECT_LEN - 1);

            // Byte details in DEBUG on the connection tag
            hpPacketDebug(packet, CONNECT_LEN, LOG_CONN_TAG);

            this->writePacket(packet, CONNECT_LEN, false);      // checkIsActive=false because it's the first packet and we don't have any reply yet

            this->lastSend = CUSTOM_MILLIS;
            this->lastConnectRqTimeMs = CUSTOM_MILLIS;
            //this->nbHeatpumpConnections_++;

            // we wait for a 10s timeout to check if the hp has replied to connection packet
            timeoutCallback_("checkFirstConnection", 10000, [this]() {
                if (!this->isHeatpumpConnected_) {
                    ESP_LOGE(LOG_CONN_TAG, "--> Heatpump did not reply: NOT CONNECTED <--");
                    ESP_LOGI(LOG_CONN_TAG, "Reinitializing UART and trying to connect again...");
                    this->reconnectUART();
                }});

        } else {
            ESP_LOGE(LOG_CONN_TAG, "UART doesn't seem to be connected...");
            this->setupUART();
            // this delay to prevent a logging flood should never happen
            CUSTOM_DELAY(750);
        }
    }

    bool CN105Connection::isConnectionActive() {
        long lrTimeMs = CUSTOM_MILLIS - this->lastResponseMs;

        // if (lrTimeMs > MAX_DELAY_RESPONSE_FACTOR * this->update_interval_) {
        //     ESP_LOGV(TAG, "Heatpump has not replied for %ld s", lrTimeMs / 1000);
        //     ESP_LOGV(TAG, "We think Heatpump is not connected anymore..");
        //     this->disconnectUART();
        // }

        return  (lrTimeMs < MAX_DELAY_RESPONSE_FACTOR * this->update_interval_);
    }

    void CN105Connection::try_write_pending_packet() {
        if (!this->has_pending_packet_) return;
        if (!this->isUARTConnected_) {
            this->reconnectUART();
            timeoutCallback_("write", 2000, [this]() { this->try_write_pending_packet(); });
            return;
        }
        this->writePacket(this->pending_packet_, this->pending_packet_len_, this->pending_check_is_active_);
        this->has_pending_packet_ = false;
    }

    void CN105Connection::writePacket(uint8_t* packet, int length, bool checkIsActive) {
        if ((this->isUARTConnected_) &&
            (this->isConnectionActive() || (!checkIsActive))) {

            ESP_LOGD(TAG, "writing packet...");
            hpPacketDebug(packet, length, "WRITE");

            for (int i = 0; i < length; i++) {
                this->io_device_->write((uint8_t)packet[i]);
            }

            // Prevent sending wantedSettings too soon after writing for example the remote temperature update packet
            this->lastSend = CUSTOM_MILLIS;

        } else {
            ESP_LOGW(TAG, "could not write as asked, because UART is not connected");
            this->reconnectUART();
            ESP_LOGW(TAG, "delaying packet writing because we need to reconnect first...");
            if (length > PACKET_LEN) {
                ESP_LOGE(TAG, "Packet length %d exceeds PACKET_LEN %d, dropping.", length, PACKET_LEN);
                return;
            }

            memcpy(this->pending_packet_, packet, static_cast<size_t>(length));
            this->pending_packet_len_ = length;
            this->pending_check_is_active_ = checkIsActive;
            this->has_pending_packet_ = true;

            timeoutCallback_("write", 4000, [this]() {
                this->try_write_pending_packet();
            });
        }
    }

    void CN105Connection::checkHeader(uint8_t inputData) {
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

    void CN105Connection::updateSuccess() {
        ESP_LOGD(LOG_ACK, "Last heatpump data update successful!");
        // nothing can be done here because we have no mean to know wether it is an external temp ack
        // or a wantedSettings update ack
    }

    void CN105Connection::processCommand(PacketCallback packetCallback) {
        switch (this->command) {
        case 0x61:  /* last update was successful */
            hpPacketDebug(this->storedInputData, this->bytesRead + 1, LOG_ACK);
            this->updateSuccess();
            break;

        case 0x62:  /* packet contains data (room °C, settings, timer, status, or functions...)*/
            packetCallback(this->data, this->dataLength);
            break;
        case 0x7a:  // Connection success (User / standard)
        case 0x7b:  // Connection success (Installer / extended)
            // Log at INFO level on dedicated tag, details at DEBUG via hpPacketDebug
            ESP_LOGI(LOG_CONN_TAG, "--> Heatpump did reply: connection success (%s, 0x%02X)! <--",
                (this->command == 0x7b) ? "Installer" : "User",
                this->command);
            hpPacketDebug(this->storedInputData, this->bytesRead + 1, LOG_CONN_TAG);
            
            this->setConnectionState(true);
            break;
        default:
            break;
        }
    }

    void CN105Connection::setConnectionState(bool state) {
        this->isHeatpumpConnected_ = state;
        connectedCallback_(state);
    }

    void CN105Connection::processDataPacket(PacketCallback packetCallback) {
        ESP_LOGV(TAG, "processing data packet...");

        this->data = &storedInputData[5];

        hpPacketDebug(this->storedInputData, this->bytesRead + 1, "READ");

        // During handshake (while not connected), log all RX frames under CN105_CONN at DEBUG level
        // to facilitate diagnostics (0x7A/0x7B expected, or other unexpected response).
        if (!this->isHeatpumpConnected_) {
            ESP_LOGD(LOG_CONN_TAG, "RX during handshake (cmd=0x%02X len=%d)", this->command, this->dataLength);
            hpPacketDebug(storedInputData, this->bytesRead + 1, LOG_CONN_TAG);
        }

        if (this->checkSum()) {
            // checkPoint of a heatpump response
            this->lastResponseMs = CUSTOM_MILLIS;    //esphome::CUSTOM_MILLIS;

            // processing the specific command
            processCommand(packetCallback);
        }
    }

    void CN105Connection::parse(uint8_t inputData, PacketCallback packetCallback) {
        ESP_LOGV("Decoder", "--> %02X [nb: %d]", inputData, this->bytesRead);

        bool hasNext = false;
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
                    this->processDataPacket(packetCallback);
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

    uint8_t* CN105Connection::getData() {
        return this->data;
    }

    int CN105Connection::getDataLength() {
        return this->dataLength;
    }

    bool CN105Connection::processInput(PacketCallback packetCallback) {
        bool processed = false;
        while (this->io_device_->available()) {
            processed = true;
            uint8_t inputData;
            if (this->io_device_->read(&inputData)) {
                parse(inputData, packetCallback);
            }
        }
        return processed;
    }

}