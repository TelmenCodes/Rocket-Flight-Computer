#include <cmath>
#include "sensors/LSM6DSOX_Driver.h"

LSM6DSOX_Driver::LSM6DSOX_Driver(uint8_t address){
    i2cAddress = address;
    currentData.isValid = false;
}

bool LSM6DSOX_Driver::begin(){ //Initializes LSM6DS and sets the Accelerometer and Gyrometer's range and data rate.
    if (!lsm.begin_I2C(i2cAddress)) {
        return false;
    }
    lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

    lsm.setAccelDataRate(LSM6DS_RATE_416_HZ);
    lsm.setGyroDataRate(LSM6DS_RATE_416_HZ);

    return true;
}

bool LSM6DSOX_Driver::read(){
    currentData.timestamp_us = micros(); //Marks the timestamp of the data provided by the sensors.

    sensors_event_t accel, gyro, temp;
    if (!lsm.getEvent(&accel, &gyro, &temp)){ //Checks if the data changed. (Reduces copied data.)
        currentData.isValid = false;
        return false;
    }

    currentData.accel = Vector3D (accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

    currentData.gyro = Vector3D (gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

    float saturation_threshold = Safety::overheadSafetyG * PhysicsConstants::STANDARD_GRAVITY_M_S2; //Checks if LSM6DS becomes saturated. (overhead safety is 15.5)
    if (std::abs(accel.acceleration.x) > saturation_threshold ||
        std::abs(accel.acceleration.y) > saturation_threshold ||
        std::abs(accel.acceleration.z) > saturation_threshold) {
            currentData.isSaturated = true;
        } else {
            currentData.isSaturated = false;
        }
    currentData.isValid = true;
    return true;
}

const SensorData& LSM6DSOX_Driver::getData() const {
    return currentData;
}

SensorType LSM6DSOX_Driver::getType() const {
    return SensorType::LOW_G_IMU;
}