#include <cmath>
#include "sensors/ADXL375_Driver.h"

const SPISettings ADXL375_Driver::spiSettings(ADXL375_Driver::MAX_SPEED, MSBFIRST, SPI_MODE3);

ADXL375_Driver::ADXL375_Driver(uint8_t ADXL375_CS_PIN){
    csPin = ADXL375_CS_PIN;
    currentData.isValid = false;
}

SensorType ADXL375_Driver::getType() const{
    return SensorType::HIGH_G_ACCEL;
}

bool ADXL375_Driver::begin(){
    pinMode(csPin, OUTPUT); //Sets the CS pin as an output.
    deselectChip(); //Sets the CS pin high so the chip remains unselected.
    SPI.begin(); //Initializes the SPI hardware.

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_DEVID | SPI_READ); //Reads the device ID.
    uint8_t dev_ID = SPI.transfer(DUMMY_BYTE); //Dummy byte to clock in response.
    SPI.endTransaction();
    deselectChip();

    if (dev_ID != DEVID_EXPECTED){ //Checks if the returned Device ID matches the expected Device ID.
        return false;
    }

    selectChip(); //This part sets the Bandwidth and Rate as 400 Hz.
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_BW_RATE);
    SPI.transfer(BW_RATE_400HZ);
    deselectChip();

    selectChip(); //This part wakes up ADXL375 from Standby mode into Active Measurement Mode.
    SPI.transfer(REG_PWR_CTRL);
    SPI.transfer(PWR_CTRL_MEASURE);
    SPI.endTransaction();
    deselectChip();

    return true;
}

bool ADXL375_Driver::read(){
    uint8_t rawData[6];
    burstReadData(rawData);
    rawX = (int16_t) ((uint16_t)rawData[1]<<8 | rawData[0]);
    rawY = (int16_t) ((uint16_t)rawData[3]<<8 | rawData[2]);
    rawZ = (int16_t) ((uint16_t)rawData[5]<<8 | rawData[4]);

    float accel_x = rawX * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    float accel_y = rawY * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    float accel_z = rawZ * SCALE_FACTOR_G * PhysicsConstants::STANDARD_GRAVITY_M_S2;

    if (std::abs(accel_x) >= OVERHEAD_SAFETY or //Checks if the accelerometer is saturated.
    std::abs(accel_y) >= OVERHEAD_SAFETY or
    std::abs(accel_z) >= OVERHEAD_SAFETY){
        currentData.isSaturated = true;
        return false;
    }

    currentData.accel.x = accel_x;
    currentData.accel.y = accel_y;
    currentData.accel.z = accel_z;

    currentData.timestamp_us = micros();
    currentData.isValid = true;
    return true;
}

const SensorData &ADXL375_Driver::getData() const{
    return currentData;
}

void ADXL375_Driver::burstReadData(uint8_t* data_buffer){
    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_DATA_X_LSB | SPI_READ | MULTI_BYTE);

    for (int i = 0; i<6; i++){ //Loops through the received data (which are in bytes) and stores them inside a pointer.
        data_buffer[i] = SPI.transfer(DUMMY_BYTE);
    }

    SPI.endTransaction();
    deselectChip();
}