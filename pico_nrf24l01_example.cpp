#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "lib/nrf24l01/nrf24l01.h"

// UART defines

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Packet format defines
#define PACKET_SIZE 7
#define HEADER_INDEX 0
#define COMMAND_INDEX 1
#define CHECKSUM_INDEX 6
#define PACKET_HEADER 0xAA

// NRF24L01 pins
#define SPI_PORT spi0
#define PIN_CE   2
#define PIN_CSN  3
#define PIN_SCK  18
#define PIN_MOSI 19  // TX
#define PIN_MISO 16  // RX
#define PIN_TX_LED  25 // Built-in LED on Pico
#define PIN_RX_LED  15 // External LED for RX indication

// Buffer for UART data
uint8_t tx_packet[PACKET_SIZE];

// Calculate checksum (simple XOR of all bytes)
uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

// Process received UART data and create a packet
bool process_uart_data(uint8_t header, uint8_t command, uint8_t* packet) {
    // Construct packet: [header, command, 0, 0, 0, 0, checksum]
    packet[HEADER_INDEX] = header;
    packet[COMMAND_INDEX] = command;

    // Zero the data bytes
    for (int i = 2; i < CHECKSUM_INDEX; i++) {
        packet[i] = 0;
    }

    // Calculate checksum (excluding the checksum position itself)
    packet[CHECKSUM_INDEX] = calculate_checksum(packet, CHECKSUM_INDEX);

    return true;
}

int main()
{
    stdio_init_all();

    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initialize NRF24L01
    NRF24L01 radio(SPI_PORT, PIN_CE, PIN_CSN, PIN_SCK, PIN_MOSI, PIN_MISO);

    radio.init();

    // Configure NRF24L01
    radio.setChannel(76);

    // Set fixed payload size to match our packet size
    radio.setPayloadSize(PACKET_SIZE);

    // Set destination address
    uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    radio.setAddress(address);

    printf("NRF24L01 UART Bridge initialized\n");
    printf("Waiting for UART data... \n");

    int packet_pos = 0;
    bool in_packet = false;
    // Main loop
    while (true) {
        // Check if any UART data is available
        while (uart_is_readable(UART_ID)) {
            // Read header byte
            uint8_t byte = uart_getc(UART_ID);

            // If byte not in a packet, check for the header
            if (!in_packet) {
                if (byte == PACKET_HEADER) {
                    in_packet = true;
                    packet_pos = 0;
                    tx_packet[packet_pos++] = byte;
                }
            }
            // if byte in a packet, keep adding bytes
            else {
                tx_packet[packet_pos++] = byte;
                if (packet_pos >= PACKET_SIZE) {
                    in_packet = false;
                    printf("packet %d, %d, %d, %d, %d, %d, %d\n", tx_packet[0], tx_packet[1], tx_packet[2], tx_packet[3], tx_packet[4], tx_packet[5], tx_packet[6]);
                    if (process_uart_data(tx_packet[0], tx_packet[1], tx_packet)) {
                        bool success = radio.transmit(tx_packet, PACKET_SIZE);
                        if (success) {
                            printf("Transmitted packet: [0x%02X, 0x%02X, 0, 0, 0, 0, 0x%02X]\n", tx_packet[HEADER_INDEX], tx_packet[COMMAND_INDEX], tx_packet[CHECKSUM_INDEX]);
                        } else {
                            printf("Failed to transmit packet\n");
                        }
                    }
                }
            }
        }

        // Small delay to prevent CPU overload
        sleep_ms(1);
    }

    return 0;
}
