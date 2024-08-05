#include <Arduino.h>
//#include "SoftwareSerial.h"
#include "vt_lora"

using namespace vt;
using lora_t = lora_e32;

lora_t lora_tx(Serial3, 2, 3);
lora_t lora_rx(Serial2, 4, 5);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);

    // Begin configuration (sleep) mode.
    lora_t *loras[] = {&lora_tx, &lora_rx};
    for (auto &lora: loras) {
        Serial.println("Beginning configuration mode...");
        lora->config();
        Serial.println("Entered configuration mode!");
        lora->set_param(0xffff, // Address
                        115200, // Operational baud rate
                        LoRaParity::PARITY_8N1, // Serial parity
                        2400, // Air data rate
                        0, // Frequency channel
                        LoRaTxPower::TX_MAX, // Tx Power
                        false, // Enable Forward Error Correction?
                        true); // Save configuration for next boot?

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
