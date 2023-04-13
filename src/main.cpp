#include <Arduino.h>
#include "vnet_lora.h"

void setup() {
    delay(1000);
    Serial.begin(115200);
    LoRa_E34 lora(&Serial3, 115200, 12, 13);
    lora.begin_cfg();
    lora.cmd_get_params();
    lora.print_params();
    Serial.println("----------");
    lora.cmd_set_params(0, LoRa_E34::LORA_BAUD_115200, LoRa_E34::LORA_8N1,
                        LoRa_E34::LORA_RATE_250k, LoRa_E34::LORA_CHANNEL_0,
                        LoRa_E34::LORA_TX_MAX, true);
    lora.cmd_write_params();
    delay(200);
    lora.cmd_get_params();
    lora.print_params();
}

void loop() {}