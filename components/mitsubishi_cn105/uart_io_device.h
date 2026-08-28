#pragma once

#include "io_device.h"
#include <stdint.h>

#include "esphome.h"
#include "esphome/components/uart/uart.h"

namespace devicestate {

    class UARTIODevice : public IIODevice {
    private:
        const char* TAG = "UARTIODevice"; // Logging tag

    public:
        esphome::uart::UARTComponent* uart_;

        UARTIODevice(
            esphome::uart::UARTComponent* uart
        ) {
            uart_ = uart;
        }

        bool begin() override {
            ESP_LOGD(TAG, "setupUART() with baudrate %d", uart_->get_baud_rate());

            if (uart_->get_data_bits() == 8 &&
                    uart_->get_parity() == esphome::uart::UART_CONFIG_PARITY_EVEN &&
                    uart_->get_stop_bits() == 1) {
                ESP_LOGD(TAG, "UART is configured in SERIAL_8E1");
                return true;
            } else {
                ESP_LOGW(TAG, "UART not configured in SERIAL_8E1");
                return false;
            }
        }

        void write(uint8_t bytes) override {
            uart_->write_byte(bytes);
        }

        int available() override {
            return uart_->available();
        }

        bool read(uint8_t *data) override {
            return uart_->read_byte(data);
        }
    };

}