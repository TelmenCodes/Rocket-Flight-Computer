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
        static constexpr uint8_t PROM_C1 = 0xA2; //(C1) MS5611's PROM Calibration constants' C1 address. Add 2 bytes to get C2-C6 (Note: The last C constant is C6)
        static constexpr uint8_t RESET = 0x1E; //(Reset) MS5611's I2C address for the reset command.
        static constexpr uint8_t PWR_ON_DELAY = 3; //Time needed for MS5611 to power on. (millisecond)
        static constexpr uint8_t ADC = 0x00; //(ADC) MS5611's I2C address for ADC read command.
        static constexpr uint8_t D1_ADDR = 0x48; //(Pressure) MS5611's I2C address for the conversion command for pressure data.
        static constexpr uint8_t D2_ADDR = 0x58; //(Temperature) MS5611's I2C address for the conversion command for temperature data.
        static constexpr uint8_t C_BYTES = 2; //Required bytes for calibration constants for successful read.
        static constexpr uint8_t D_BYTES = 3; //Required bytes for returned measurement for successful read.
        
        uint16_t C[6]; //Hold Calibration constants. C1-C6.
        uint32_t b0 = 0; //b0-b2 hold the 3 bytes for both the D1 (Pressure) and D2 (Temperature).
        uint32_t b1 = 0;
        uint32_t b2 = 0;
        uint32_t D1 = 0; //(Pressure) Uncorrected Pressure data.
        uint32_t D2 = 0; //(Temp) Uncorrected Temperature data.
        uint32_t lastConversionStart_us = 0; //Timestamp of the last conversion.
        SensorData currentData;
        void calculateCompensation();
};