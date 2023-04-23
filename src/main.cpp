#include <Arduino.h>
#include "vnet_lora.h"

#define CFG_E32
//#define CFG_E34

void setup() {
    Serial.begin(115200);

#ifdef CFG_E32
    Serial.print("Beginning config");

    for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    LoRa_E32<> lora1(&Serial3, 9600, 10, 11);
//    LoRa_E32<> lora2(&Serial2, 115200, 22, 23);
//    LoRa_E32<> lora3(&Serial2, 9600U, &DDRH, &DDRH, &PORTH, &PORTH, (1 << PH2), (1 << PH3));

//    LoRa_E32<> *arr[] = {&lora1, &lora2, &lora3};
    LoRa_E32<> *arr[] = {&lora1};

    for (const LoRa_E32<> *lora_ptr: arr) {
        LoRa_E32<> lora = *lora_ptr;

        Serial.println("Initialized LoRa.");

        lora.begin_cfg();
        Serial.println("Began config.");

        lora.cmd_get_params();
        Serial.println("Param got.");

        lora.print_params();
        Serial.println("----------");
        lora.cmd_set_params(0, LoRaCFG::LORA_BAUD_9600, LoRaCFG::LORA_8N1,
                            LoRa_E32<>::LORA_RATE_2400, 53,
                            LoRaCFG::LORA_TX_MAX, false, true);
        lora.cmd_write_params();
        delay(500);
        lora.cmd_get_params();
        lora.print_params();
        Serial.println("=-=-=-=-=-=");

        lora.end_cfg();
    }

    Serial.println("===== End of Configuration =====");

#elif defined(CFG_E34)
    Serial.print("Beginning config");

    for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    LoRa_E34 lora1(&Serial3, 38400, 10, 11);
    LoRa_E34 lora2(&Serial2, 38400, 22, 23);

    Serial.println("Initialized LoRa.");

    lora1.begin_cfg();
    Serial.println("Began config.");

    lora1.cmd_get_params();
    Serial.println("Param got.");

    lora1.print_params();
    Serial.println("----------");
    lora1.cmd_set_params(0, LoRa_E34::LORA_BAUD_38400, LoRa_E34::LORA_8N1,
                        LoRa_E34::LORA_RATE_250k, LoRa_E34::LORA_CHANNEL_0,
                        LoRa_E34::LORA_TX_MAX, true);
    lora1.cmd_write_params();
    delay(500);
    lora1.cmd_get_params();
    lora1.print_params();
    lora1.end_cfg();
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
    lora2.cmd_set_params(0, LoRa_E34::LORA_BAUD_38400, LoRa_E34::LORA_8N1,
                        LoRa_E34::LORA_RATE_250k, LoRa_E34::LORA_CHANNEL_0,
                        LoRa_E34::LORA_TX_MAX, true);
    lora2.cmd_write_params();
    delay(500);
    lora2.cmd_get_params();
    lora2.print_params();
    lora2.end_cfg();
    Serial.println("===== End of Configuration =====");
#else
    LoRa_E34 lora1(&Serial3, 115200, 10, 11);
    LoRa_E34 lora2(&Serial2, 115200, 22, 23);

    lora1.begin_normal();
    lora2.begin_normal();

    pinMode(30, OUTPUT);
    digitalWrite(30, 1);

    delay(100);
#endif
}

void loop() {
#if !defined(CFG_E32) && !defined(CFG_E34)
//    static HardwareSerial &Tx = Serial3;
    static HardwareSerial &Rx = Serial2;

    while (Rx.available()) {
        Serial.write(Rx.read());
    }

//    static unsigned long tim = millis();
//
//    uint32_t x = 0;
//
//    if (millis() - tim > 1000) {
//        for (uint8_t i = 0; i < 128; ++i)
//            Tx.print(x++ % 10);
//        Tx.println();
//        tim = millis();
//    }
#endif
}
