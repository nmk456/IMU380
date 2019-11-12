#include <Arduino.h>
#include <IMU380.h>

IMU380 imu(SPI, 10);

void setup() {
    int status = imu.begin();
    if(status < 0) {
        Serial.print("IMU Error: ");
        Serial.println(status);
        while(1);
    }
    Serial.println("IMU Initialized");

    // Optional config functions
    imu.setGyroRange(IMU380::GYRO_RANGE_250DPS);
    imu.setFilter(IMU380::NO_FILTER);
    imu.setDataRate(IMU380::ODR_200HZ);
    imu.setDataReady(false, false);
}

void loop() {
    imu.readSensor();
    Serial.print(" gx: "); Serial.print(imu.getGyroX());
    Serial.print(" gy: "); Serial.print(imu.getGyroY());
    Serial.print(" gz: "); Serial.print(imu.getGyroZ());
    Serial.print(" ax: "); Serial.print(imu.getAccelX());
    Serial.print(" ay: "); Serial.print(imu.getAccelY());
    Serial.print(" az: "); Serial.print(imu.getAccelZ());

    Serial.println();
    delay(50);
}
