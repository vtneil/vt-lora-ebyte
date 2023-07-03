#ifndef VT_LORA_BASE_H
#define VT_LORA_BASE_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

namespace vt {
    using pin_t = uint8_t;

    struct lora_e22_param {
        uint8_t head;
        uint8_t addh;
        uint8_t addl;
        uint8_t netid;
        uint8_t reg0;
        uint8_t reg1;
        uint8_t reg2;
        uint8_t reg3;
        uint8_t crypt_h;
        uint8_t crypt_l;
    };

    struct lora_e3x_param {
        uint8_t head;
        uint8_t addh;
        uint8_t addl;
        uint8_t sped;
        uint8_t chan;
        uint8_t option;
    };

    enum LoRaMode {
        NORMAL = 0,
        WOR,
        POWER_SAVING,
        SLEEP
    };

    enum LoRaTxPower {
        TX_MAX = 0b00,
        TX_HIGH,
        TX_LOW,
        TX_MIN
    };

    enum LoRaParity {
        PARITY_8N1 = 0b00,
        PARITY_8O1,
        PARITY_8E1
    };

    namespace detail {
        class lora_base {
        protected:
            static constexpr uint32_t BAUD_CONFIG = 9600U;
            static constexpr uint32_t POWER_DELAY = 50U;

            HardwareSerial &hw_serial_;
            uint32_t baud_op_ = BAUD_CONFIG;
            uint8_t pin_op_ = 0;

            pin_t PIN_M0 = 0;
            pin_t PIN_M1 = 0;

            uint8_t MASK_M0 = 0;
            volatile uint8_t *PORT_M0 = nullptr;
            volatile uint8_t *DDR_M0 = nullptr;

            uint8_t MASK_M1 = 0;
            volatile uint8_t *PORT_M1 = nullptr;
            volatile uint8_t *DDR_M1 = nullptr;

        public:
            lora_base(HardwareSerial &hardware_serial,
                      pin_t digital_pin_M0,
                      pin_t digital_pin_M1)
                    : hw_serial_{hardware_serial} {
                pin_op_ = 1;
                PIN_M0 = digital_pin_M0;
                PIN_M1 = digital_pin_M1;
                pinMode(PIN_M0, OUTPUT);
                pinMode(PIN_M1, OUTPUT);
            }

            lora_base(HardwareSerial &hardware_serial,
                      uint8_t mask_M0,
                      volatile uint8_t *ddr_M0,
                      volatile uint8_t *port_M0,
                      uint8_t mask_M1,
                      volatile uint8_t *ddr_M1,
                      volatile uint8_t *port_M1)
                    : hw_serial_{hardware_serial} {
                pin_op_ = 0;
                MASK_M0 = mask_M0;
                PORT_M0 = port_M0;
                DDR_M0 = ddr_M0;
                MASK_M1 = mask_M1;
                PORT_M1 = port_M1;
                DDR_M1 = ddr_M1;
                *DDR_M0 |= ((MASK_M0));      // Set M0 to OUTPUT mode
                *DDR_M1 |= ((MASK_M1));      // Set M1 to OUTPUT mode
            }

            lora_base() = delete;

            lora_base(const lora_base &other) = delete;

            lora_base(lora_base &&other) noexcept = delete;

            void begin(uint32_t baud_rate) {
                baud_op_ = baud_rate;
                set_mode(NORMAL);
                delay(POWER_DELAY);
                hw_serial_.end();
                delay(10);
                hw_serial_.begin(baud_op_);
            }

            void end() {
                hw_serial_.end();
                delay(10);
            }

            void config() {
                set_mode(SLEEP);
                delay(POWER_DELAY);
                hw_serial_.end();
                delay(10);
                hw_serial_.begin(BAUD_CONFIG);
            }

            HardwareSerial &serial() { return hw_serial_; }

        protected:
            void set_mode(enum LoRaMode mode) {
                if (pin_op_) {
                    switch (mode) {  // (M1, M0)
                        case NORMAL:  // (0, 0)
                            digitalWrite(PIN_M1, 0);
                            digitalWrite(PIN_M0, 0);
                            return;
                        case WOR:  // (0, 1)
                            digitalWrite(PIN_M1, 0);
                            digitalWrite(PIN_M0, 1);
                            return;
                        case POWER_SAVING:  // (1, 0)
                            digitalWrite(PIN_M1, 1);
                            digitalWrite(PIN_M0, 0);
                            return;
                        case SLEEP:  // (1, 1)
                            digitalWrite(PIN_M1, 1);
                            digitalWrite(PIN_M0, 1);
                            return;
                        default:
                            return;
                    }
                } else {
                    switch (mode) {  // (M1, M0)
                        case NORMAL:  // (0, 0)
                            *PORT_M1 &= ~((MASK_M1));
                            *PORT_M0 &= ~((MASK_M0));
                            return;
                        case WOR:  // (0, 1)
                            *PORT_M1 &= ~((MASK_M1));
                            *PORT_M0 |= ((MASK_M0));
                            return;
                        case POWER_SAVING:  // (1, 0)
                            *PORT_M1 |= ((MASK_M1));
                            *PORT_M0 &= ~((MASK_M0));
                            return;
                        case SLEEP:  // (1, 1)
                            *PORT_M0 |= ((MASK_M0));
                            *PORT_M1 |= ((MASK_M1));
                            return;
                        default:
                            return;
                    }
                }
            }

            inline void write_triple(uint8_t b) { write_to_module(b, b, b); }

            template<typename T = uint8_t>
            inline void write_to_module(T last) { hw_serial_.write(last); }

            template<typename T = uint8_t, typename ...Args>
            inline void write_to_module(T first, Args... args) {
                hw_serial_.write(first);
                write_to_module(args...);
            }

            static void print_base16(uint8_t b) {
                Serial.print(b >> 4, 16);
                Serial.print(b & 0b00001111, 16);
            }

            static void print_base2(uint8_t b) {
                for (uint8_t i = 0; i < 8; ++i) {
                    if (b & (1 << 7)) Serial.print(1);
                    else Serial.print(0);
                    b <<= 1;
                }
            }
        };
    }
}

#endif //VT_LORA_BASE_H
