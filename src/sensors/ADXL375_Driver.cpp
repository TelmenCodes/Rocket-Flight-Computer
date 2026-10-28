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
    burstReadData();
    return true;
}

const SensorData &ADXL375_Driver::getData() const{
    return currentData;
}

void ADXL375_Driver::burstReadData(){
    uint8_t data[6];

    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(REG_DATA_X_LSB | SPI_READ | MULTI_BYTE);

    for (int i = 0; i<6; i++){
        data[i] = SPI.transfer(DUMMY_BYTE);
    }

    SPI.endTransaction();
    deselectChip();
    rawX = (int16_t) ((uint16_t)data[1]<<8 | data[0]);
    rawY = (int16_t) ((uint16_t)data[3]<<8 | data[2]);
    rawZ = (int16_t) ((uint16_t)data[5]<<8 | data[4]);

    float accel_x_g = rawX * SCALE_FACTOR_G;
    float accel_y_g = rawY * SCALE_FACTOR_G;
    float accel_z_g = rawZ * SCALE_FACTOR_G;

    currentData.accel.x = accel_x_g * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    currentData.accel.y = accel_y_g * PhysicsConstants::STANDARD_GRAVITY_M_S2;
    currentData.accel.z = accel_z_g * PhysicsConstants::STANDARD_GRAVITY_M_S2;

    currentData.timestamp_us = micros();
    currentData.isValid = true;
}