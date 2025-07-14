#include "nrf24l01.h"

NRF24L01::NRF24L01(spi_inst_t* spi_inst, uint8_t ce_pin, uint8_t csn_pin, 
                 uint8_t sck_pin, uint8_t mosi_pin, uint8_t miso_pin) {
    spi = spi_inst;
    this->ce_pin = ce_pin;
    this->csn_pin = csn_pin;
    this->payload_size = 32;  // Default payload size
    
    // Initialize SPI pins
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);
    
    // Set CE and CSN as outputs
    gpio_init(ce_pin);
    gpio_init(csn_pin);
    gpio_set_dir(ce_pin, GPIO_OUT);
    gpio_set_dir(csn_pin, GPIO_OUT);
    
    // Default states
    ceLow();
    csnHigh();
}

void NRF24L01::init() {
    // Give the module time to settle
    sleep_ms(100);
    
    // Initialize SPI at 4MHz
    spi_init(spi, 4000000);
    
    // Reset all registers
    writeRegister(NRF_CONFIG, 0x08);     // Power down, enable CRC (1 byte)
    writeRegister(NRF_EN_AA, 0x3F);      // Enable auto-ack on all pipes
    writeRegister(NRF_EN_RXADDR, 0x03);  // Enable rx address on pipe 0 and 1
    writeRegister(NRF_SETUP_AW, 0x03);   // 5-byte address width
    writeRegister(NRF_SETUP_RETR, 0x03); // 250μs retransmit delay, 3 retransmits
    writeRegister(NRF_RF_CH, 76);        // Channel 76 (2.476 GHz)
    writeRegister(NRF_RF_SETUP, 0x0E);   // 2Mbps data rate, 0dBm power
    
    // Set payload size for pipe 0
    writeRegister(NRF_RX_PW_P0, payload_size);
    
    // Flush FIFOs
    flushRx();
    flushTx();
    
    // Clear status flags
    writeRegister(NRF_STATUS, 0x70);
    
    // Power up in standby mode
    powerUp();
    
    // Allow the radio to reach power up mode
    sleep_ms(5);
}

void NRF24L01::setTxMode() {
    uint8_t config = readRegister(NRF_CONFIG);
    config &= ~(1 << PRIM_RX);  // Clear PRIM_RX bit
    writeRegister(NRF_CONFIG, config);
    ceLow();  // Ensure CE is low for standby-I mode
    sleep_ms(1);
}

void NRF24L01::setRxMode() {
    uint8_t config = readRegister(NRF_CONFIG);
    config |= (1 << PRIM_RX);  // Set PRIM_RX bit
    writeRegister(NRF_CONFIG, config);
    ceHigh();  // Set CE high to start receiving
    sleep_ms(1);
}

bool NRF24L01::transmit(const uint8_t* data, uint8_t length) {
    // Ensure the length does not exceed the payload size
    if (length > payload_size) {
        length = payload_size;
    }
    
    // Set to TX mode
    setTxMode();
    
    // Flush the TX FIFO
    flushTx();
    
    // Write payload to TX FIFO
    csnLow();
    spiTransfer(W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        spiTransfer(data[i]);
    }
    // Pad remaining bytes with zeros if needed
    for (uint8_t i = length; i < payload_size; i++) {
        spiTransfer(0);
    }
    csnHigh();
    
    // Start transmission
    ceHigh();
    sleep_us(15);  // Minimum CE high time for transmission (10μs)
    ceLow();
    
    // Wait for transmission to complete or timeout
    const uint32_t timeout = to_ms_since_boot(get_absolute_time()) + 100;  // 100ms timeout
    bool success = false;
    
    while (to_ms_since_boot(get_absolute_time()) < timeout) {
        uint8_t status = getStatus();
        
        // Check if transmission successful
        if (status & (1 << TX_DS)) {
            writeRegister(NRF_STATUS, (1 << TX_DS));  // Clear TX_DS flag
            success = true;
            break;
        }
        
        // Check if max retransmits reached
        if (status & (1 << MAX_RT)) {
            writeRegister(NRF_STATUS, (1 << MAX_RT));  // Clear MAX_RT flag
            flushTx();  // Flush TX FIFO
            break;
        }
        
        sleep_ms(1);
    }
    
    return success;
}

bool NRF24L01::receive(uint8_t* data, uint8_t* length) {
    // Set to RX mode
    setRxMode();
    
    // Check if data is available
    if (!(getStatus() & (1 << RX_DR))) {
        return false;
    }
    
    // Get payload length (using fixed payload size)
    *length = payload_size;
    
    // Read payload
    csnLow();
    spiTransfer(R_RX_PAYLOAD);
    for (uint8_t i = 0; i < payload_size; i++) {
        data[i] = spiTransfer(NOP);
    }
    csnHigh();
    
    // Clear RX_DR flag
    writeRegister(NRF_STATUS, (1 << RX_DR));
    
    return true;
}

void NRF24L01::setChannel(uint8_t channel) {
    // Ensure channel is within valid range (0-125)
    if (channel > 125) {
        channel = 125;
    }
    writeRegister(NRF_RF_CH, channel);
}

void NRF24L01::setPayloadSize(uint8_t size) {
    // Limit payload size to 32 bytes (maximum for nRF24L01)
    if (size > 32) {
        size = 32;
    }
    payload_size = size;
    writeRegister(NRF_RX_PW_P0, size);
}

void NRF24L01::setAddress(const uint8_t* address) {
    // Set TX address
    writeRegister(NRF_TX_ADDR, address, 5);
    
    // Set RX address for pipe 0 (for auto-ack)
    writeRegister(NRF_RX_ADDR_P0, address, 5);
}

uint8_t NRF24L01::getStatus() {
    csnLow();
    uint8_t status = spiTransfer(NOP);  // NOP command returns status
    csnHigh();
    return status;
}

void NRF24L01::powerUp() {
    uint8_t config = readRegister(NRF_CONFIG);
    config |= (1 << PWR_UP);
    writeRegister(NRF_CONFIG, config);
    sleep_ms(5);  // Wait for power-up
}

void NRF24L01::powerDown() {
    uint8_t config = readRegister(NRF_CONFIG);
    config &= ~(1 << PWR_UP);
    writeRegister(NRF_CONFIG, config);
}

void NRF24L01::writeRegister(uint8_t reg, uint8_t value) {
    csnLow();
    spiTransfer(W_REGISTER | (reg & 0x1F));
    spiTransfer(value);
    csnHigh();
}

void NRF24L01::writeRegister(uint8_t reg, const uint8_t* buf, uint8_t len) {
    csnLow();
    spiTransfer(W_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) {
        spiTransfer(buf[i]);
    }
    csnHigh();
}

uint8_t NRF24L01::readRegister(uint8_t reg) {
    csnLow();
    spiTransfer(R_REGISTER | (reg & 0x1F));
    uint8_t value = spiTransfer(0xFF);
    csnHigh();
    return value;
}

void NRF24L01::readRegister(uint8_t reg, uint8_t* buf, uint8_t len) {
    csnLow();
    spiTransfer(R_REGISTER | (reg & 0x1F));
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = spiTransfer(0xFF);
    }
    csnHigh();
}

void NRF24L01::csnLow() {
    gpio_put(csn_pin, 0);
}

void NRF24L01::csnHigh() {
    gpio_put(csn_pin, 1);
}

void NRF24L01::ceHigh() {
    gpio_put(ce_pin, 1);
}

void NRF24L01::ceLow() {
    gpio_put(ce_pin, 0);
}

uint8_t NRF24L01::spiTransfer(uint8_t data) {
    uint8_t result;
    spi_write_read_blocking(spi, &data, &result, 1);
    return result;
}

void NRF24L01::flushRx() {
    csnLow();
    spiTransfer(FLUSH_RX);
    csnHigh();
}

void NRF24L01::flushTx() {
    csnLow();
    spiTransfer(FLUSH_TX);
    csnHigh();
}