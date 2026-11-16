#include "tmc_uart.h"

void setup() {
    constexpr int driverEnable = 25;
    constexpr int driverStep = 32;
    digitalWrite(driverEnable, HIGH);
    pinMode(driverEnable, OUTPUT);
    digitalWrite(driverStep, LOW);
    pinMode(driverStep, OUTPUT);
    Serial.begin(115200);
    Serial.println(tmc_uart::begin() ? "EVENT,BOOT,TMC_UART_PROBE" : "EVENT,TMC_UART_INIT_ERROR");
}

void loop() {
    tmc_uart::configure();
    delay(2500);
}
