#include <Arduino.h>
#include <IMU380.h>

IMU380 imu = IMU380(SPI, 10);

void setup() {
    int status = imu.begin();
    if(status < 0) {
        Serial.print("IMU Error: ");
        Serial.println(status);
        while(1);
    }
    Serial.println("IMU Initialized");
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
    delay(5);
}
