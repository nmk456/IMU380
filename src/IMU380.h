#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "SPI.h"     // SPI library

class IMU380 {
    public:
        enum GyroRange {
            GYRO_RANGE_62DPS,
            GYRO_RANGE_125DPS,
            GYRO_RANGE_250DPS
        };
        enum Filter {
            NO_FILTER,
            BARTLETT_40HZ,
            BARTLETT_20HZ,
            BARTLETT_10HZ,
            BARTLETT_5HZ,
            BUTTERWORTH_50HZ,
            BUTTERWORTH_20HZ,
            BUTTERWORTH_10HZ,
            BUTTERWORTH_5HZ
        };
        enum ODR {
            ODR_0HZ = 0x00,
            ODR_200HZ = 0x01,
            ODR_100HZ = 0x02,
            ODR_50HZ = 0x03,
            ODR_25HZ = 0x04,
            ODR_20HZ = 0x05,
            ODR_10HZ = 0x06,
            ODR_5HZ = 0x07,
            ODR_4HZ = 0x08,
            ODR_2HZ = 0x09,
            ODR_1HZ = 0x10
        };

        IMU380(SPIClass &bus, uint8_t csPin);
        int begin();
        int readSensor();
        int selfTest();
        int setGyroRange(GyroRange range);
        int setFilter(Filter filter);
        int setDataRate(ODR odr);
        int setDataReady(bool enable, bool polarity);
        uint16_t getSerialNumber();
        float getAccelX();
        float getAccelY();
        float getAccelZ();
        float getGyroX();
        float getGyroY();
        float getGyroZ();
        float getTemp();
    protected:
        uint16_t _buffer;
        uint16_t _serialNumber;
        Filter _filter;

        // SPI
        SPIClass *_spi;
        uint8_t _csPin;
        const uint32_t SPI_SPEED = 2000000; // 2 MHz
        const uint8_t SPI_WRITE = 0b1000000;
        SPISettings settings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE2);

        // Data scale factors
        float _accelScale = 1.0f/4000.0f; // g/ADU
        float _gyroScale;
        const float _tempScale = 0.7311f; // deg C/ADU
        const float _tempOffset = 31.0f;

        // Data buffer
        uint16_t _axadu, _ayadu, _azadu;
        uint16_t _gxadu, _gyadu, _gzadu;
        uint16_t _tempadu;
        uint16_t _status;
        float _ax, _ay, _az;
        float _gx, _gy, _gz;
        float _temperature;

        // Private Functions
        int writeRegister(uint8_t address, uint8_t const &data);
        int readRegister(uint8_t address, uint16_t &data);

        // Read-only Registers
        const uint8_t X_RATE = 0x04;
        const uint8_t Y_RATE = 0x06;
        const uint8_t Z_RATE = 0x08;
        const uint8_t X_ACCEL = 0x0A;
        const uint8_t Y_ACCEL = 0x0C;
        const uint8_t Z_ACCEL = 0x0E;
        const uint8_t RATE_TEMP = 0x16;
        const uint8_t BOARD_TEMP = 0x18;

        const uint8_t STATUS = 0x3C;
        const uint8_t BURST_MODE = 0x3E;
        const uint8_t MANUF_CODE = 0x52;
        const uint8_t UNIT_CODE = 0x54;
        const uint8_t PRODUCT_ID = 0x56;
        const uint8_t SERIAL_NUMBER = 0x58;

        //Writable Registers
        const uint8_t DATA_READY = 0x34;
        const uint8_t SELF_TEST = 0x35;
        const uint8_t SYSTEM_CLOCK = 0x36;
        const uint8_t OUTPUT_DATA_RATE = 0x37;
        const uint8_t LOW_PASS_FILTER = 0x38;
        const uint8_t RS_DYNAMIC_RANGE = 0x39;
};
#endif