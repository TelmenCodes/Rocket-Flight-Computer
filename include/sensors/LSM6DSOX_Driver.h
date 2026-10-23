#pragma once

#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h>
#include "Config.h"
#include "ISensor.h"

class LSM6DSOX_Driver: public ISensor{
    public:
        LSM6DSOX_Driver(uint8_t address = LSM6DS_I2CADDR_DEFAULT);

        bool begin() override;
        bool read() override;
        const SensorData& getData() const override;
        SensorType getType() const override;
    private:
        Adafruit_LSM6DSOX lsm; //Retrieves Adafruit's driver for the IMU.
        SensorData currentData;
        uint8_t i2cAddress;
};