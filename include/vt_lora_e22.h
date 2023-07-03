#ifndef VT_LORA_E22_H
#define VT_LORA_E22_H

#include "vt_lora_base.h"

namespace vt {
    class lora_e22 : public detail::lora_base {
    private:
        struct lora_e22_param params_ = {};

    public:
        lora_e22(HardwareSerial &hardware_serial,
                 pin_t digital_pin_M0,
                 pin_t digital_pin_M1)
                : detail::lora_base(hardware_serial,
                                    digital_pin_M0,
                                    digital_pin_M1) {}

        void config() {
            set_mode(POWER_SAVING);
            delay(POWER_DELAY);
            hw_serial_.end();
            delay(10);
            hw_serial_.begin(BAUD_CONFIG);
        }

        void set_param(uint16_t addr,
                       uint8_t net_id,
                       uint32_t baud,
                       enum LoRaParity parity,
                       uint32_t data_rate,
                       uint8_t channel,
                       uint8_t packet_length,
                       bool save_params) {
            params_ = {};
            params_.head = save_params ? 0xc0 : 0xc2;
            params_.addh = static_cast<uint8_t>(addr >> 8);
            params_.addl = addr & 0xff;
            params_.netid = net_id;
            params_.reg0 |= (baud_to_lora(baud) << 5);
            params_.reg0 |= (parity << 3);
            params_.reg0 |= (rate_to_lora(data_rate));
            params_.reg1 |= (len_to_lora(packet_length) << 6);
            params_.reg2 |= (channel > 83 ? 0 : channel);

            write_to_module(params_.head, 0x00, 9);  // Start address

            hw_serial_.write(params_.addh);
            hw_serial_.write(params_.addl);
            hw_serial_.write(params_.netid);

            hw_serial_.write(params_.reg0);
            hw_serial_.write(params_.reg1);
            hw_serial_.write(params_.reg2);

            hw_serial_.write(params_.reg3);
            hw_serial_.write(params_.crypt_h);
            hw_serial_.write(params_.crypt_l);

            delay(POWER_DELAY);
        }

        const lora_e22_param &get_param() const { return params_; }

        void query_param() {
            uint8_t config[16] = {};
            write_to_module(0xc1, 0x00, 7);
            while (!hw_serial_.available());
            delay(POWER_DELAY);
            uint8_t i = 0;
            while (hw_serial_.available()) {
                config[i] = hw_serial_.read();
                print_base16(config[i]);
                Serial.print(": (");
                print_base2(config[i]);
                Serial.print(") ");
                ++i;
            }
        }

    private:
        static uint8_t rate_to_lora(uint32_t rate) {
            switch (rate) {
                case static_cast<uint32_t>(300UL):
                    return 0b000;
                case static_cast<uint32_t>(1200UL):
                    return 0b001;
                case static_cast<uint32_t>(2400UL):
                    return 0b010;
                case static_cast<uint32_t>(4800UL):
                    return 0b011;
                case static_cast<uint32_t>(9600UL):
                    return 0b100;
                case static_cast<uint32_t>(19200UL):
                    return 0b101;
                case static_cast<uint32_t>(38400UL):
                    return 0b110;
                case static_cast<uint32_t>(62500UL):
                    return 0b111;
                default:
                    return 0b010;
            }
        }

        static uint8_t baud_to_lora(uint32_t baud) {
            switch (baud) {
                case static_cast<uint32_t>(1200):
                    return 0b000;
                case static_cast<uint32_t>(2400):
                    return 0b001;
                case static_cast<uint32_t>(4800):
                    return 0b010;
                case static_cast<uint32_t>(9600):
                    return 0b011;
                case static_cast<uint32_t>(19200):
                    return 0b100;
                case static_cast<uint32_t>(38400):
                    return 0b101;
                case static_cast<uint32_t>(57600):
                    return 0b110;
                case static_cast<uint32_t>(115200):
                    return 0b111;
                default:
                    return 0b011;
            }
        }

        static uint8_t len_to_lora(uint8_t packet_length) {
            switch (packet_length) {
                case static_cast<uint8_t>(240):
                    return 0b00;
                case static_cast<uint8_t>(128):
                    return 0b01;
                case static_cast<uint8_t>(64):
                    return 0b10;
                case static_cast<uint8_t>(32):
                    return 0b11;
                default:
                    return 0b00;
            }
        }
    };
}

#endif //VT_LORA_E22_H
