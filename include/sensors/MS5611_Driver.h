#pragma once

#include "Config.h"
#include "ISensor.h"

class MS5611_Driver: public ISensor {
    public:
        MS5611_Driver(uint8_t address = I2CAddressList::MS5611_I2C_ADDR);
        bool begin() override;
        bool read() override;
        const SensorData& getData() const override;
        SensorType getType() const override;
    private:
        enum class State{
            UNINITIALIZED,
            TRIGGER_D1,
            WAIT_D1,
            READ_D1_TRIGGER_D2,
            WAIT_D2,
            READ_D2_CALCULATE
        };
        State currentState = State::UNINITIALIZED;
        uint8_t i2cAddress;
        uint16_t C[6];
        uint32_t b0 = 0;
        uint32_t b1 = 0;
        uint32_t b2 = 0;
        uint32_t D1 = 0;
        uint32_t D2 = 0;
        uint32_t lastConversionStart_us = 0;
        SensorData currentData;
        void calculateCompensation();
};