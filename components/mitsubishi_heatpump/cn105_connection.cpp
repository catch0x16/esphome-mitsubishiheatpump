#include "cn105_connection.h"

#include "esphome.h"
#include "logging.h"

#include "cn105_utils.h"
#include "cn105_logging.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105Connection"; // Logging tag

    CN105Connection::CN105Connection(IIODevice* io_device, TimeoutCallback timeoutCallback, int update_interval) :
        io_device_{io_device},
        timeoutCallback_{timeoutCallback},
        update_interval_{update_interval} {
    }

    bool CN105Connection::isConnected() {
        return isHeatpumpConnected_;
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
        // TODO: use the CN105Climate::checkSum(byte bytes[], int len) function

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
        this->isHeatpumpConnected_ = false;
        this->isUARTConnected_ = false;

        if (io_device_->begin()) {
            ESP_LOGI(LOG_CONN_TAG, "UART configuré en SERIAL_8E1");
            this->isUARTConnected_ = true;
            this->initBytePointer();
        } else {
            ESP_LOGW(LOG_CONN_TAG, "UART n'est pas configuré en SERIAL_8E1");
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
        // Désactivé: le fallback UART bas-niveau (ESP-IDF 5.4.x) peut interférer avec les
        // tests de handshake/fallback. On laisse UARTComponent gérer la réinit standard.
        //this->force_low_level_uart_reinit();
        this->setupUART();
        this->sendFirstConnectionPacket();
    }

    void CN105Connection::sendFirstConnectionPacket() {
        if (this->isUARTConnected_) {
            this->lastReconnectTimeMs = CUSTOM_MILLIS;          // marker to prevent to many reconnections
            this->isHeatpumpConnected_ = false;
            uint8_t packet[CONNECT_LEN];
            memcpy(packet, CONNECT, CONNECT_LEN);

            // Choix du mode de handshake: standard (0x5A) ou installateur (0x5B)
            packet[1] = 0x5A;
            // CONNECT a un checksum pré-calculé dans la constante; si on modifie l'octet commande, on doit le recalculer.
            packet[CONNECT_LEN - 1] = devicestate::checkSum(packet, CONNECT_LEN - 1);

            // Détails des octets en DEBUG sur le tag de connexion
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

    bool CN105Connection::isHeatpumpConnectionActive() {
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
            (this->isHeatpumpConnectionActive() || (!checkIsActive))) {

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

    void CN105Connection::processCommand(CommandCallback commandCallback) {
        switch (this->command) {
        case 0x61:  /* last update was successful */
            hpPacketDebug(this->storedInputData, this->bytesRead + 1, LOG_ACK);
            this->updateSuccess();
            break;

        case 0x62:  /* packet contains data (room °C, settings, timer, status, or functions...)*/
            this->getDataFromResponsePacket();
            break;
        case 0x7a:  // Connection success (User / standard)
        case 0x7b:  // Connection success (Installer / extended)
            // Log en INFO sur le tag dédié, détails en DEBUG via hpPacketDebug
            ESP_LOGI(LOG_CONN_TAG, "--> Heatpump did reply: connection success (%s, 0x%02X)! <--",
                (this->command == 0x7b) ? "Installer" : "User",
                this->command);
            hpPacketDebug(this->storedInputData, this->bytesRead + 1, LOG_CONN_TAG);
            this->isHeatpumpConnected_ = true;

            commandCallback(this->command);
            break;
        default:
            break;
        }
    }

    void CN105Connection::processDataPacket(CommandCallback commandCallback) {
        ESP_LOGV(TAG, "processing data packet...");

        this->data = &storedInputData[5];

        hpPacketDebug(this->storedInputData, this->bytesRead + 1, "READ");

        // Pendant le handshake (tant que non connecté), logguer toute trame RX sous CN105_CONN en DEBUG
        // afin de faciliter le diagnostic (0x7A/0x7B attendus, ou autre réponse inattendue).
        if (!this->isHeatpumpConnected_) {
            ESP_LOGD(LOG_CONN_TAG, "RX during handshake (cmd=0x%02X len=%d)", this->command, this->dataLength);
            hpPacketDebug(storedInputData, this->bytesRead + 1, LOG_CONN_TAG);
        }

        if (this->checkSum()) {
            // checkPoint of a heatpump response
            this->lastResponseMs = CUSTOM_MILLIS;    //esphome::CUSTOM_MILLIS;

            // processing the specific command
            processCommand(commandCallback);
        }
    }

    void CN105Connection::parse(uint8_t inputData, CommandCallback commandCallback) {
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
                    this->processDataPacket(commandCallback);
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

    bool CN105Connection::processInput(CommandCallback commandCallback) {
        bool processed = false;
        while (this->io_device_->available()) {
            processed = true;
            uint8_t inputData;
            if (this->io_device_->read(&inputData)) {
                parse(inputData, commandCallback);
            }
        }
        return processed;
    }

}