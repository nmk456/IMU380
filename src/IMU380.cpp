#include <Arduino.h>
#include <IMU380.h>

/*
TODO:
    setGyroRange
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

    _gyroScale = 1.0f/100.0f; // deg/s/ADU

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
int IMU380::writeRegister(uint8_t address, uint8_t const &data) {
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
int IMU380::readRegister(uint8_t address, uint16_t &data) {
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

// Reads sensor data and update internal buffers
int IMU380::readSensor() {
    // Initiate burst mode
    readRegister(BURST_MODE, _buffer);
    readRegister(0x00, _gxadu);
    readRegister(0x00, _gyadu);
    readRegister(0x00, _gzadu);
    readRegister(0x00, _axadu);
    readRegister(0x00, _ayadu);
    readRegister(0x00, _azadu);
    readRegister(0x00, _tempadu);
    readRegister(0x00, _tempadu);

    _gx = ((float) _gxadu) * _accelScale;
    _gy = ((float) _gyadu) * _accelScale;
    _gz = ((float) _gzadu) * _accelScale;
    _ax = ((float) _axadu) * _accelScale;
    _ay = ((float) _ayadu) * _accelScale;
    _az = ((float) _azadu) * _accelScale;

    _temperature = ((float) _tempadu) * _tempScale + _tempOffset;

    return 1;
}

// Gets temperature in deg C
float IMU380::getTemp() {
    return _temperature;
}

// Gets serial number as uint_16
uint16_t IMU380::getSerialNumber() {
    return _serialNumber;
}

// Return accel X value in m/s^2
float IMU380::getAccelX() {
    return _ax;
}

// Return accel Y value in m/s^2
float IMU380::getAccelY() {
    return _ay;
}

// Return accel Z value in m/s^2
float IMU380::getAccelZ() {
    return _az;
}

// Return gyro X value in deg/s
float IMU380::getGyroX() {
    return _gx;
}

// Return gyro Y value in deg/s
float IMU380::getGyroY() {
    return _gy;
}

// Return gyro Z value in deg/s
float IMU380::getGyroZ() {
    return _gz;
}
