#ifndef NRF24L01_H
#define NRF24L01_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <cstring>

// NRF24L01 Register addresses
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_OBSERVE_TX  0x08
#define NRF_RPD         0x09
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_ADDR_P1  0x0B
#define NRF_RX_ADDR_P2  0x0C
#define NRF_RX_ADDR_P3  0x0D
#define NRF_RX_ADDR_P4  0x0E
#define NRF_RX_ADDR_P5  0x0F
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_RX_PW_P1    0x12
#define NRF_RX_PW_P2    0x13
#define NRF_RX_PW_P3    0x14
#define NRF_RX_PW_P4    0x15
#define NRF_RX_PW_P5    0x16
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD       0x1C
#define NRF_FEATURE     0x1D

// NRF24L01 Commands
#define R_REGISTER      0x00
#define W_REGISTER      0x20
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xA0
#define FLUSH_TX        0xE1
#define FLUSH_RX        0xE2
#define REUSE_TX_PL     0xE3
#define NOP             0xFF

// NRF24L01 Bit Mnemonics
#define MASK_RX_DR      6
#define MASK_TX_DS      5
#define MASK_MAX_RT     4
#define EN_CRC          3
#define CRCO            2
#define PWR_UP          1
#define PRIM_RX         0

// Status bits
#define RX_DR           6
#define TX_DS           5
#define MAX_RT          4

class NRF24L01 {
    public:
        NRF24L01(spi_inst_t* spi_inst, uint8_t ce_pin, uint8_t csn_pin,
                uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin);

        void init();
        void setTxMode();
        void setRxMode();
        bool transmit(const uint8_t* data, uint8_t length);
        bool receive(uint8_t* data, uint8_t* length);
        void setChannel(uint8_t channel);
        void setPayloadSize(uint8_t size);
        void setAddress(const uint8_t* address);
        uint8_t getStatus();
        void powerUp();
        void powerDown();
    
    private:
        spi_inst_t* spi;
        uint8_t ce_pin;
        uint8_t csn_pin;
        uint8_t payload_size;

        void writeRegister(uint8_t reg, uint8_t value);
        void writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len);
        uint8_t readRegister(uint8_t reg);
        void readRegister(uint8_t reg, uint8_t* buf, uint8_t len);
        void csnLow();
        void csnHigh();
        void ceHigh();
        void ceLow();
        uint8_t spiTransfer(uint8_t data);
        void flushRx();
        void flushTx();
    };

    #endif // NRF24L01_H