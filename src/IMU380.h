#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "SPI.h"     // SPI library

class IMU380 {
    public:
        IMU380(SPIClass &bus, uint8_t csPin);
        int begin();
        int setGyroRange();
        int readSensor();
        int selfTest();
        float getAccelX();
        float getAccelY();
        float getAccelZ();
        float getGyroX();
        float getGyroY();
        float getGyroZ();
        float getTemp();
    protected:
        uint16_t _buffer;

        // SPI
        SPIClass *_spi;
        uint8_t _csPin;
        const uint32_t SPI_SPEED = 2000000; // 2 MHz
        const uint8_t SPI_WRITE = 0b1000000;
        SPISettings settings = SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE2);

        // Private Functions
        int writeRegister(uint8_t address, uint8_t data);
        int readRegister(uint8_t address, uint16_t data);
        int readBurstMode();

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