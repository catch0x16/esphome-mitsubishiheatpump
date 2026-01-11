#include "cn105_protocol.h"

namespace devicestate {

    CN105Protocol::CN105Protocol(devicestate::IIODevice* io_device) :
    io_device_{io_device} {
        this->isUARTConnected_ = false;
    }

    // SERIAL_8E1
    void CN105Protocol::setupUART() {
        log_info_uint32(TAG, "setupUART() with baudrate ", this->parent_->get_baud_rate());
        ESP_LOGI(LOG_CONN_TAG, "setupUART(): baud=%d tx=%d rx=%d (UART port=%d)", this->parent_->get_baud_rate(), this->tx_pin_, this->rx_pin_, this->uart_port_);
        this->setHeatpumpConnected(false);
        this->isUARTConnected_ = false;

        // just for debugging purpose, a way to use a button i, yaml to trigger a reconnect
        this->uart_setup_switch = true;

        if (this->io_device_->connect();) {
            ESP_LOGI(LOG_CONN_TAG, "UART configuré en SERIAL_8E1");
            this->isUARTConnected_ = true;
            this->initBytePointer();
        } else {
            ESP_LOGW(LOG_CONN_TAG, "UART n'est pas configuré en SERIAL_8E1");
        }
    }

    bool CN105Protocol::connect() {
        setupUART();
    }

    bool CN105Protocol::isConnected() {
        long lrTimeMs = CUSTOM_MILLIS() - this->lastResponseMs;

        // if (lrTimeMs > MAX_DELAY_RESPONSE_FACTOR * this->update_interval_) {
        //     ESP_LOGV(TAG, "Heatpump has not replied for %ld s", lrTimeMs / 1000);
        //     ESP_LOGV(TAG, "We think Heatpump is not connected anymore..");
        //     this->disconnectUART();
        // }

        return  (lrTimeMs < MAX_DELAY_RESPONSE_FACTOR * this->update_interval_);
    }

}
