#include <Arduino.h>
#include <IMU380.h>

/*
TODO:
    getSerialNumber
    selfTest
    readSensor
    setGyroRange
    float getAccelX
    float getAccelY
    float getAccelZ
    float getGyroX
    float getGyroY
    float getGyroZ
    float getTemp
*/

IMU380::IMU380(SPIClass &bus, uint8_t csPin) {
    _spi = &bus;
    _csPin = csPin;
}

int IMU380::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();

    // Check if correct IMU
    readRegister(PRODUCT_ID, _buffer);
    if(_buffer != 0x3810) {
        return -1;
    }

    if(selfTest() < 0) {
        return -2;
    }

    // Set to defaults config
    writeRegister(DATA_READY, 0x04); // Data ready enabled, active low
    writeRegister(OUTPUT_DATA_RATE, 0x01); // 200 Hz output
    writeRegister(SYSTEM_CLOCK, 0x01); // Internetal system clock
    writeRegister(RS_DYNAMIC_RANGE, 0x04); // +- 250 deg/s
    writeRegister(LOW_PASS_FILTER, 0x00); // Unfiltered output

    return 1;
}

// Initiate self test and check status register
int IMU380::selfTest() {
    writeRegister(SELF_TEST, 0b00000100);
    delay(100);
    readRegister(STATUS, _buffer);

    if(bitRead(_buffer, 5)) {
        return 1;
    } else {
        return -1;
    }
}

// Write data to register at address
int IMU380::writeRegister(uint8_t address, uint8_t data) {
    // Begin transaction and select IMU380
    _spi->beginTransaction(settings);
    digitalWrite(_csPin, LOW);

    // Send address in write mode and transfer data
    _spi->transfer(address | SPI_WRITE);
    _spi->transfer(data);

    // End transaction
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    delay(10);

    readRegister(address, _buffer);
    if((uint8_t) _buffer == data) {
        return 1;
    } else {
        return -1;
    }
}

// Reads data from register at address
int IMU380::readRegister(uint8_t address, uint16_t data) {
    // Begin transaction and select IMU380
    _spi->beginTransaction(settings);
    digitalWrite(_csPin, LOW);

    // Send address and read data
    _spi->transfer16((uint16_t) address << 8);
    data = _spi->transfer16(0x0000);

    // End transaction
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    return 1;
}
