#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "sensors/MS5611_Driver.h"
#include "sensors/LSM6DSOX_Driver.h"

LSM6DSOX_Driver imu(I2CAddressList::LSM6DSOX_I2C_ADDR);
MS5611_Driver barometer(I2CAddressList::MS5611_I2C_ADDR);   

void setup(){
    Serial.begin(Timings::MAIN_SERIAL_BAUD);
    
    while(!Serial and millis() < 4000);
    Serial.println("Flight computer is starting...");
    Wire.setSDA(TeensyPins::I2C_SDA_PIN);
    Wire.setSCL(TeensyPins::I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(Timings::TARGET_LOOP_TIME_US);

    if(!imu.begin()){
        Serial.println("IMU initialization failed!");
    }
    if(!barometer.begin()){
        Serial.println("Barometer initialization failed!");
    }
}

void loop(){
    imu.read();
    barometer.read();
    if(barometer.read()){
        const SensorData& baroData = barometer.getData();
        Serial.print("Pressure: ");
        Serial.print(baroData.pressure_hpa);
        Serial.print("hPa | Temperature: ");
        Serial.print(baroData.temperature_c);
        Serial.print(" C");

        const SensorData& imuData = imu.getData();
        Serial.println("Accel X: ");
        Serial.print(imuData.accel.x);
        Serial.println("Accel Y:");
        Serial.print(imuData.accel.y);
        Serial.println("Accel Z: ");
        Serial.print(imuData.accel.z);
        Serial.println("Gyro X: ");
        Serial.print(imuData.gyro.x);
        Serial.println("Gyro Y: ");
        Serial.print(imuData.gyro.y);
        Serial.println("Gyro Z: ");
        Serial.print(imuData.gyro.z);
    }

    delay(10); //100Hz flight loop
}