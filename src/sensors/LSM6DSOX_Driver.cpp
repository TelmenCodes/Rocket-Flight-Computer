#include <cmath>
#include "sensors/LSM6DSOX_Driver.h"

const SPISettings LSM6DSOX_Driver::spiSettings(LSM6DSOX_Driver::MAX_SPEED, MSBFIRST, SPI_MODE3);

LSM6DSOX_Driver::LSM6DSOX_Driver(uint8_t LSM6DS_CS_PIN){
    csPin = LSM6DS_CS_PIN;
    currentData.isValid = false;
}

SensorType LSM6DSOX_Driver::getType() const {
    return SensorType::LOW_G_IMU;
}

bool LSM6DSOX_Driver::begin(){ //Initializes LSM6DS and sets the Accelerometer and Gyrometer's range and data rate.
    pinMode(csPin, OUTPUT); //Sets the Chip Select pin as an output.
    deselectChip(); //Sets the Chip Select pin high, so the chip remains unselected.
    SPI.begin(); //Initializes the SPI Hardware.

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_DEVID | SPI_READ);
    uint8_t dev_ID = SPI.transfer(DUMMY_BYTE);
    SPI.endTransaction();
    deselectChip();

    if(dev_ID != DEVID_EXPECTED){
        return false;
    }

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_ACCEL);
    SPI.transfer(ACCEL_416HZ);
    SPI.endTransaction();
    deselectChip();

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_GYRO);
    SPI.transfer(GYRO_416HZ);
    SPI.endTransaction();
    deselectChip();

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_BDU);
    SPI.transfer(BDU_ENABLED);
    SPI.endTransaction();
    deselectChip();
    return true;
}

bool LSM6DSOX_Driver::read(){
    uint8_t rawBuffer[12];
    burstReadData(rawBuffer);
    gyro_rawX = (int16_t) ((uint16_t) rawBuffer[1] << 8 | rawBuffer[0]); //Stores the data ()
    gyro_rawY = (int16_t) ((uint16_t) rawBuffer[3] << 8 | rawBuffer[2]);
    gyro_rawZ = (int16_t) ((uint16_t) rawBuffer[5] << 8 | rawBuffer[4]);
    accel_rawX = (int16_t) ((uint16_t) rawBuffer[7] << 8 | rawBuffer[6]);
    accel_rawY = (int16_t) ((uint16_t) rawBuffer[9] << 8 | rawBuffer[8]);
    accel_rawZ = (int16_t) ((uint16_t) rawBuffer[11] << 8 | rawBuffer[10]);

    float gyro_x = gyro_rawX * SCALE_FACTOR_DPS * DEG_TO_RAD_MULT;
    float gyro_y = gyro_rawY * SCALE_FACTOR_DPS * DEG_TO_RAD_MULT;
    float gyro_z = gyro_rawZ * SCALE_FACTOR_DPS * DEG_TO_RAD_MULT;
    float accel_x = accel_rawX * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    float accel_y = accel_rawY * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    float accel_z = accel_rawZ * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;

    if(std::abs(accel_x) >= OVERHEAD_SAFETY or //Checks if the accelerometer is saturated.
    std::abs(accel_y) >= OVERHEAD_SAFETY or 
    std::abs(accel_z) >= OVERHEAD_SAFETY){
        currentData.isSaturated = true;
        return false;
    }
    currentData.gyro.x = gyro_x;
    currentData.gyro.y = gyro_y;
    currentData.gyro.z = gyro_z;

    currentData.accel.x = accel_x;
    currentData.accel.y = accel_y;
    currentData.accel.z = accel_z;

    currentData.timestamp_us = micros();
    currentData.isValid = true;
    return true;
}

const SensorData& LSM6DSOX_Driver::getData() const {
    return currentData;
}

void LSM6DSOX_Driver::burstReadData(uint8_t* buffer){
    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_OUT_G | SPI_READ);

    for (int i = 0; i<12; i++){ //Loops through the received data (which are in bytes) and stores them.
        buffer[i] = SPI.transfer(DUMMY_BYTE);
    }
    SPI.endTransaction();
    deselectChip();
}