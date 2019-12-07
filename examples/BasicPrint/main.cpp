#include <Arduino.h>
#include <IMU380.h>

//#define CALIB

IMU380 imu(SPI, 10);
int num = 1000;

elapsedMillis timer;
unsigned int interval = 100;
int i = 0;

double gx, gy, gz;
double ax, ay, az;

void setup() {
    Serial.begin(115200);
    while(!Serial);

    int status = imu.begin();
    if(status < 0) {
        Serial.print("IMU Error: ");
        Serial.println(status);
        while(1);
    }
    Serial.println("IMU Initialized");

    // Optional config functions
    imu.setGyroRange(IMU380::GYRO_RANGE_250DPS);
    imu.setFilter(IMU380::BUTTERWORTH_20HZ);
    imu.setDataRate(IMU380::ODR_20HZ);
    imu.setDataReady(false, false);

    #ifndef CALIB
    imu.setBias(-0.0744499980, -0.0623099982, 0.2286699940, 0.078577361, -0.003357263, -0.005844155);
    imu.setScale(1.007145919, 0.999380455, 0.998160113);
    #else
    Serial.println("Starting calibration in:");

    for(int i = 5; i > 0; i--) {
        Serial.println(i);
        delay(1000);
    }
    #endif
}

void loop() {
    #ifdef CALIB
    if(timer > interval && i < num) {
        imu.readSensor();

        gx += (double) imu.getGyroX()/num;
        gy += (double) imu.getGyroY()/num;
        gz += (double) imu.getGyroZ()/num;
        ax += (double) imu.getAccelX()/num;
        ay += (double) imu.getAccelY()/num;
        az += (double) imu.getAccelZ()/num;

        i++;
        timer = 0;
    } else if(i >= num) {
        Serial.print("gx: "); Serial.println(gx, 10);
        Serial.print("gy: "); Serial.println(gy, 10);
        Serial.print("gz: "); Serial.println(gz, 10);
        Serial.print("ax: "); Serial.println(ax, 10);
        Serial.print("ay: "); Serial.println(ay, 10);
        Serial.print("az: "); Serial.println(az, 10);
        while(1);
    }
    #else
    if(timer > interval) {
        imu.readSensor();

        Serial.print(" gx: "); Serial.print(imu.getGyroX());
        Serial.print(" gy: "); Serial.print(imu.getGyroY());
        Serial.print(" gz: "); Serial.print(imu.getGyroZ());
        Serial.print(" ax: "); Serial.print(imu.getAccelX());
        Serial.print(" ay: "); Serial.print(imu.getAccelY());
        Serial.print(" az: "); Serial.print(imu.getAccelZ());

        Serial.println();

        timer = 0;
    }
    #endif
}
