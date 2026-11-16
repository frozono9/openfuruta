#pragma once

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <hal/uart_ll.h>
#include <soc/gpio_sig_map.h>
#include <soc/uart_struct.h>

namespace tmc_uart {
constexpr gpio_num_t UART_PIN = GPIO_NUM_12;
constexpr uart_port_t UART_PORT = UART_NUM_2;
constexpr int BAUD = 9600;
constexpr uint8_t IOIN = 0x06;
constexpr uint8_t IFCNT = 0x02;
constexpr uint8_t GCONF = 0x00;
constexpr uint8_t IHOLD_IRUN = 0x10;
constexpr uint8_t CHOPCONF = 0x6C;
constexpr unsigned RUN_CURRENT = 15, HOLD_CURRENT = 4;
constexpr uint32_t CURRENT = (6u << 16) | (RUN_CURRENT << 8) | HOLD_CURRENT;
constexpr uint8_t address = 3;  // MS1 and MS2 are both high.
bool uartReady = false;

uint8_t crc(const uint8_t* data, size_t size) {
    uint8_t result = 0;
    for (size_t i = 0; i < size; ++i) {
        uint8_t value = data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const bool mix = ((result >> 7) ^ value) & 1;
            result <<= 1;
            if (mix) result ^= 0x07;
            value >>= 1;
        }
    }
    return result;
}

bool send(uint8_t* frame, size_t size, unsigned turnaroundUs) {
    frame[size - 1] = crc(frame, size - 1);
    uart_flush_input(UART_PORT);
    gpio_set_direction(UART_PIN, GPIO_MODE_INPUT_OUTPUT);
    esp_rom_gpio_connect_out_signal(UART_PIN, U2TXD_OUT_IDX, false, false);
    delayMicroseconds(1500);
    const int sent = uart_write_bytes(UART_PORT, frame, size);
    const uint32_t started = micros();
    while (!uart_ll_is_tx_idle(&UART2) && micros() - started < 15000) {}
    const bool transmitted = sent == static_cast<int>(size) && uart_ll_is_tx_idle(&UART2);
    if (transmitted) delayMicroseconds(turnaroundUs);
    gpio_set_direction(UART_PIN, GPIO_MODE_INPUT);
    return transmitted;
}

bool readRegister(uint8_t reg, uint32_t& value, unsigned turnaroundUs) {
    uint8_t request[] = {0x05, address, reg, 0};
    if (!send(request, sizeof(request), turnaroundUs)) return false;

    uint8_t reply[24];
    const int received = uart_read_bytes(UART_PORT, reply, sizeof(reply), pdMS_TO_TICKS(30));
    for (int i = 0; i + 8 <= received; ++i) {
        const uint8_t* data = reply + i;
        if (data[0] != 0x05 || data[1] != 0xFF || data[2] != reg || crc(data, 7) != data[7]) continue;
        value = (uint32_t(data[3]) << 24) | (uint32_t(data[4]) << 16) |
                (uint32_t(data[5]) << 8) | data[6];
        return true;
    }
    return false;
}

bool writeRegister(uint8_t reg, uint32_t value) {
    uint8_t frame[] = {0x05, address, uint8_t(reg | 0x80), uint8_t(value >> 24),
                       uint8_t(value >> 16), uint8_t(value >> 8), uint8_t(value), 0};
    const bool sent = send(frame, sizeof(frame), 1500);
    delayMicroseconds(1500);
    return sent;
}

bool readVerified(uint8_t reg, uint32_t& value, unsigned delayUs) {
    for (int offset : {0, -30, 30, -60, 60}) {
        const int candidate = static_cast<int>(delayUs) + offset;
        if (candidate >= 0 && readRegister(reg, value, candidate)) return true;
    }
    return false;
}

bool configure() {
    if (!uartReady) return false;
    uint32_t identity;
    unsigned turnaroundUs = 0;
    // PDN has a pull-down. Keep the stop level high until just before the reply.
    for (const unsigned delayUs : {790u, 760u, 820u, 730u, 0u}) {
        if (readRegister(IOIN, identity, delayUs) && (identity >> 24) == 0x21) {
            turnaroundUs = delayUs;
            Serial.printf("EVENT,TMC_UART_ID,0x%08lx,address=%u,turnaround_us=%u\n",
                          static_cast<unsigned long>(identity), address, turnaroundUs);
            uint32_t config, before, after, actual, chopper;
            if (!readVerified(GCONF, config, turnaroundUs) ||
                !readVerified(IFCNT, before, turnaroundUs) ||
                !readVerified(CHOPCONF, chopper, turnaroundUs)) break;
            // Use cycle-by-cycle current regulation during rapid reversals.
            const uint32_t requested = (config | (1u << 6) | (1u << 2)) & ~3u;
            if (writeRegister(IHOLD_IRUN, CURRENT) && writeRegister(GCONF, requested) &&
                readVerified(IFCNT, after, turnaroundUs) &&
                readVerified(GCONF, actual, turnaroundUs) &&
                uint8_t(after - before) == 2 && actual == requested) {
                Serial.printf("EVENT,TMC_UART_VERIFIED,irun=%u,ihold=%u,gconf=0x%08lx,chopconf=0x%08lx\n",
                              RUN_CURRENT, HOLD_CURRENT, static_cast<unsigned long>(actual), static_cast<unsigned long>(chopper));
                return true;
            }
            break;
        }
    }
    Serial.println("EVENT,TMC_UART_UNVERIFIED");
    return false;
}

bool begin() {
    uart_config_t config{};
    config.baud_rate = BAUD;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_APB;
    uartReady = uart_param_config(UART_PORT, &config) == ESP_OK &&
                uart_driver_install(UART_PORT, 256, 0, 0, nullptr, 0) == ESP_OK &&
                uart_set_pin(UART_PORT, UART_PIN, UART_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_OK &&
                uart_set_rx_full_threshold(UART_PORT, 1) == ESP_OK &&
                uart_set_rx_timeout(UART_PORT, 2) == ESP_OK;
    gpio_set_direction(UART_PIN, GPIO_MODE_INPUT);
    return uartReady;
}
}
