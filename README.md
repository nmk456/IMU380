# IMU380

Arduino library for interfacing with MEMSIC IMU380ZA-200 via SPI.

## Overview

The IMU380 is a discountinued near-tactical grade IMU that is intermitently available on the internet for relatively low cost, considering it's specifications. The IMU itself supports communication via SPI or UART, but only SPI is imlemented in this library currently.

## Specifications

All of these specifications come from the datasheet (found in the extras folder).

| Gyro |   |
| ------------ | - |
| Range (deg/s) | +-200 |
| Bias Instability (deg/hr) | < 10 |
| Resolution (deg/s) | < 0.02 |

| Accelerometer| |
| --------------- | ------ |
| Range (g) | +-4 |
| Bias Instability (mg) | <0.02 |
| Resolution (mg) | <0.5 |

| Electrical ||
|-|-|
| Input Voltage (VDC) | 3.0 to 5.5 |
| Power Consumption (mW) | <250 |
| Output Data Rate (Hz) | 1 to 200 |
| Operating Temperature (deg C) | -40 to +85 |

## IMU380 Class

### Object Declaration

**IMU380(SPIClass &bus, uint_8 csPin)**

Declaring the IMU380 object is necessary to use it. The SPI bus and CS pin used must be specified. The example below uses the default SPI bus and pin 10 as the CS pin.

```IMU380 imu(SPI, 10);```

### IMU Initialization

**int begin();**

This function should be called once in your setup function. It initializes the SPI bus with the CS pin specified earlier, checks if the IMU is connected and runs a self test to see if the IMU is operational. If it returns a value other than 1, it may not be connected correctly.

```int status = imu.begin();```

### Read Sensor

**int readSensor();**

This function updates the internal sensor values using the sensor's burst mode over SPI. It does not return any values, the function below is necessary to do that.

```int status = imu.readSensor();```

### Get Sensor Data

**float getAccelX();**

These functions get the accelerometer and gyro data from the internal buffers. The example below is for the X gyro and accel data, but there are similar function for all axes. The function returns a float with the accelerometer data in m/s^2 and the gyro data in deg/s.

```float ax = imu.getAccelX();```
```float gx = imu.getGyroX();```

### Self Test

**int selfTest();**

This function runs the built in self test in the IMU. It will return 1 if the test if successfuly, and -1 if it is not. More elaborate testing functionality will be implemented in the future.

```int status = imu.selfTest();```

### Gyro Range

**int setGyroRange(GyroRange range);**

This function sets the output range and precision of the gyro, which are inversely related. The table below lists the possible gyro ranges.

| Range Name | Range (deg/s) | Precision (ADU/(deg/s)) |
| - | - | - |
| GYRO_RANGE_62DPS  | 62.5 | 400 |
| GYRO_RANGE_125DPS | 125  | 200 |
| GYRO_RANGE_250DPS | 250  | 100 |

```int status = setGyroRange(IMU380::GYRO_RANGE_250DPS);```

### Output Data Rate

**int setDataRate(ODR odr);**

This function sets the rate that the IMU outputs data over SPI. The table below shows the names and corredsponding data rates and timing options.

| ODR Name | Data Rate | Time |
| - | - | - |
| ODR_0HZ | 0 Hz | x ms |
| ODR_200HZ | 200 Hz | x ms |
| ODR_100HZ | 100 Hz | x ms |
| ODR_50HZ | 50 Hz | x ms |
| ODR_25HZ | 25 Hz | x ms |
| ODR_20HZ | 20 Hz | x ms |
| ODR_10HZ | 10 Hz | x ms |
| ODR_5HZ | 5 Hz | x ms |
| ODR_4HZ | 4 Hz | x ms |
| ODR_2HZ | 2 Hz | x ms |
| ODR_1HZ | 1 Hz | x ms |

```imu.setDataRate(IMU380::ODR_200HZ);```

### Set Output Filter

**int setFilter(Filter filter);**

This functions sets which built in filter the IMU uses. More info to come.

```imu.setFilter(IMU380::NO_FILTER);```

### Data Ready Configuration

**int setDataReady(bool enable, bool polarity);**

This function configures the data ready function on pin 7 of the IMU. The first argument enables or disables it, and the second argument changes the polarity. If `polarity` is set to true, the pin will be high when the IMU has data ready and low otherwise, and if `polarity` set to false, the pin will be low when data ready and high otherwise. The frequency at which the IMU will have new data ready is set by the `setDataRate()` function.

```imu.setDataReady(false, false);```

### Get Serial Number

**uint16_t getSerialNumber();**

This function returns the serial number of the IMU. It returns the exact contents of the serial number register as an unsigned 16 bit integer.

```int serialNumber = imu.getSerialNumber();```

## Sensor Orientation

The sensor data is in the axes shown in the image below. This only applies when the sensor is in SPI mode.

<img src="https://raw.githubusercontent.com/nmk456/IMU380/master/extras/IMU380_Orientation.PNG" alt="IMU Axes" width=500>

## Examples

* **BasicPrint**: gets data from IMU and prints it to serial. Also demonstrates configuration functions.

## Electrical Connections

This chart only lists the pins necessary to interface with the IMU using SPI. For information on the other pins, consult the datasheet.

| IMU Pin | Function |
| - | - |
| 3 | SCLK (SPI Clock) |
| 4 | MISO (SPI Data Out) |
| 5 | MOSI (SPI Data In) |
| 6 | SS (SPI Chip Select) |
| 7 | Data Ready |
| 10-12 | VIN (3-5 V) |
| 13-15 | GND |