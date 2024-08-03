#include <Arduino.h>
//#include "SoftwareSerial.h"
#include "vt_lora"

using namespace vt;
using lora_t = lora_e22;

HardwareSerial Serial2(PA3, PA2);

lora_t lora_tx(Serial1, PA0, PA1);
lora_t lora_rx(Serial2, PA5, PA6);

void setup() {
    delay(5000);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin();

    // Begin configuration (sleep) mode.
    lora_t *loras[] = {&lora_tx, &lora_rx};
    for (auto &lora: loras) {
        Serial.println("Beginning configuration mode...");
        lora->config();
        Serial.println("Entered configuration mode!");
        lora->set_param(0xffff,
                        0,
                        115200,
                        LoRaParity::PARITY_8N1,
                        2400,
                        63,
                        240,
                        true);

        lora->query_param();

        Serial.println("Done config!");
    }

    lora_tx.begin(115200);
    lora_rx.begin(115200);
}

void loop() {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    Serial.println("Sent!");

    // Test tx
    lora_tx.hw_serial().println("Message");

    // Test rx
    while (lora_rx.hw_serial().available()) Serial.write(lora_rx.hw_serial().read());

    delay(1000);
}
