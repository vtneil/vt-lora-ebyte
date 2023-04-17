#include <Arduino.h>
#include "vnet_lora.h"

//#define CFG_E32
#define CFG_E34

void setup() {
    Serial.begin(115200);

#ifdef CFG_E32
    Serial.print("Beginning config");

    for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    LoRa_E32 lora(&Serial3, 115200, 10, 11);
//    LoRa_E32 lora(&Serial3, 115200, DDRB, DDRB, PORTB, PORTB, PB6, PB7);

    Serial.println("Initialized LoRa.");

    lora.begin_cfg();
    Serial.println("Began config.");

    lora.cmd_get_params();
    Serial.println("Param got.");

    lora.print_params();
    Serial.println("----------");
    lora.cmd_set_params(0, LoRa_E32::LORA_BAUD_9600, LoRa_E32::LORA_8N1,
                        LoRa_E32::LORA_RATE_2400, LoRa_E32::LORA_CHANNEL_12,
                        LoRa_E32::LORA_TX_MAX, true, true);
    lora.cmd_write_params();
    delay(500);
    lora.cmd_get_params();
    lora.print_params();
    Serial.println("===== End of Configuration =====");
#elif defined(CFG_E34)
    Serial.print("Beginning config");

    for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    LoRa_E34 lora1(&Serial3, 115200, 10, 11);
    LoRa_E34 lora2(&Serial2, 115200, 22, 23);

    Serial.println("Initialized LoRa.");

    lora1.begin_cfg();
    Serial.println("Began config.");

    lora1.cmd_get_params();
    Serial.println("Param got.");

    lora1.print_params();
    Serial.println("----------");
    lora1.cmd_set_params(0, LoRa_E34::LORA_BAUD_115200, LoRa_E34::LORA_8N1,
                        LoRa_E34::LORA_RATE_250k, LoRa_E34::LORA_CHANNEL_0,
                        LoRa_E34::LORA_TX_MAX, true);
    lora1.cmd_write_params();
    delay(500);
    lora1.cmd_get_params();
    lora1.print_params();
    Serial.println("===== End of Configuration =====");

    delay(1000);

    Serial.print("Beginning config");

    for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    Serial.println("Initialized LoRa.");

    lora2.begin_cfg();
    Serial.println("Began config.");

    lora2.cmd_get_params();
    Serial.println("Param got.");

    lora2.print_params();
    Serial.println("----------");
    lora2.cmd_set_params(0, LoRa_E34::LORA_BAUD_115200, LoRa_E34::LORA_8N1,
                        LoRa_E34::LORA_RATE_250k, LoRa_E34::LORA_CHANNEL_0,
                        LoRa_E34::LORA_TX_MAX, true);
    lora2.cmd_write_params();
    delay(500);
    lora2.cmd_get_params();
    lora2.print_params();
    Serial.println("===== End of Configuration =====");
#else
    Serial.println("BEGIN");

    LoRa_E34 lora1(&Serial3, 115200, 10, 11);
//    LoRa_E34 lora1(&Serial3, 115200, &DDRB, &DDRB, &PORTB, &PORTB, PB4, PB5);
    LoRa_E34 lora2(&Serial2, 115200, 22, 23);
//    LoRa_E34 lora2(&Serial2, 115200, &DDRA, &DDRA, &PORTA, &PORTA, PA0, PA1);

    lora1.begin_normal(115200);
    lora2.begin_normal(115200);

    delay(100);
#endif
}

void loop() {
#if !defined(CFG_E32) && !defined(CFG_E34)
    static uint32_t tim = millis();
    static uint32_t cnt_tx = 0;
    static uint32_t cnt_rx = 0;

    while (Serial2.available()) {
        Serial2.read();
        ++cnt_rx;
        Serial.print("RX ");
        Serial.println(cnt_rx);
    }

    if (millis() - tim >= 50) {
        for (uint32_t i = 0; i < 64; ++i) {
            Serial3.write((uint8_t) 0xaa);
            ++cnt_tx;
        }
        Serial.print("++++++++TX ");
        Serial.println(cnt_tx);
        tim = millis();
    }
#endif
}
