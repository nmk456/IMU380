#include <Arduino.h>
#include <IMU380.h>

IMU380::IMU380(SPIClass &bus, uint8_t csPin) {
    _spi = &bus;
    _csPin = csPin;
}

int IMU380::begin() {
    // Setup SPI
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
    _spi->begin();

    // Check if correct IMU
    readRegister(IMU_ID, _buffer);
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

    if(!bitRead(_buffer, 5)) {
        return 1;
    } else {
        return -1;
    }
}

// Set gyro range
int IMU380::setGyroRange(GyroRange range) {
    switch(range) {
        case GYRO_RANGE_62DPS: {
            if(writeRegister(RS_DYNAMIC_RANGE, 0x01) < 0) {
                return -1;
            }
            _gyroScale = 1.0f/400.0f; // deg/s/ADU
            break;
        }
        case GYRO_RANGE_125DPS: {
            if(writeRegister(RS_DYNAMIC_RANGE, 0x02) < 0) {
                return -1;
            }
            _gyroScale = 1.0f/200.0f; // deg/s/ADU
            break;
        }
        case GYRO_RANGE_250DPS: {
            if(writeRegister(RS_DYNAMIC_RANGE, 0x04) < 0) {
                return -1;
            }
            _gyroScale = 1.0f/100.0f; // deg/s/ADU
            break;
        }
    }
    return 1;
}

// Set filter type
int IMU380::setFilter(Filter filter) {
    switch(filter) {
        case NO_FILTER: {
            if(writeRegister(LOW_PASS_FILTER, 0x00) < 0) {
                return -1;
            }
            _filter = NO_FILTER;
        }
        case BARTLETT_40HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x03) < 0) {
                return -1;
            }
            _filter = BARTLETT_40HZ;
        }
        case BARTLETT_20HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x04) < 0) {
                return -1;
            }
            _filter = BARTLETT_20HZ;
        }
        case BARTLETT_10HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x05) < 0) {
                return -1;
            }
            _filter = BARTLETT_10HZ;
        }
        case BARTLETT_5HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x06) < 0) {
                return -1;
            }
            _filter = BARTLETT_5HZ;
        }
        case BUTTERWORTH_50HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x30) < 0) {
                return -1;
            }
            _filter = BUTTERWORTH_50HZ;
        }
        case BUTTERWORTH_20HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x40) < 0) {
                return -1;
            }
            _filter = BUTTERWORTH_20HZ;
        }
        case BUTTERWORTH_10HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x50) < 0) {
                return -1;
            }
            _filter = BUTTERWORTH_10HZ;
        }
        case BUTTERWORTH_5HZ: {
            if(writeRegister(LOW_PASS_FILTER, 0x60) < 0) {
                return -1;
            }
            _filter = BUTTERWORTH_5HZ;
        }
    }
    return 1;
}

// Set output data rate
int IMU380::setDataRate(ODR odr) {
    return writeRegister(OUTPUT_DATA_RATE, odr);
}

// Configure data ready enable and polarity (high on data ready if true)
int IMU380::setDataReady(bool enable, bool polarity) {
    return writeRegister(DATA_READY, (uint8_t) (enable << 2) | (polarity << 1));
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

    return 1;
}

// Reads data from register at address
int IMU380::readRegister(uint8_t address, uint16_t &data) {
    // Begin transaction and select IMU380
    _spi->beginTransaction(settings);
    digitalWrite(_csPin, LOW);

    // Send address and read data
    _spi->transfer16(((uint16_t) address) << 8);
    data = _spi->transfer16(0x0000);

    // End transaction
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

    return 1;
}

// Reads sensor data and update internal buffers
int IMU380::readSensor() {
    // Begin transaction and select IMU380
    _spi->beginTransaction(settings);
    digitalWrite(_csPin, LOW);

    // Start burst mode
    _spi->transfer16(((uint16_t) BURST_MODE) << 8);
    _status = _spi->transfer16(0x0000);
    _gxadu = _spi->transfer16(0x0000);
    _gyadu = _spi->transfer16(0x0000);
    _gzadu = _spi->transfer16(0x0000);
    _axadu = _spi->transfer16(0x0000);
    _ayadu = _spi->transfer16(0x0000);
    _azadu = _spi->transfer16(0x0000);
    _tempadu = _spi->transfer16(0x0000);

    _gx = ((float) _gxadu) * _gyroScale;
    _gy = ((float) _gyadu) * _gyroScale;
    _gz = ((float) _gzadu) * _gyroScale;
    _ax = ((float) _axadu) * _accelScale;
    _ay = ((float) _ayadu) * _accelScale;
    _az = ((float) _azadu) * _accelScale;

    _temperature = ((float) _tempadu) * _tempScale + _tempOffset;

    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();

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
