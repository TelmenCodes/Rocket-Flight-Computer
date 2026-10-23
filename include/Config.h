#pragma once

#include <Arduino.h>

namespace TeensyPins{
    constexpr int I2C_SDA_PIN = 18;
    constexpr int I2C_SCL_PIN = 19;
}

//This namespace defines constants that I need for calculations.
namespace PhysicsConstants{
    constexpr float STANDARD_GRAVITY_M_S2 = 9.80665f;
    constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f; 
}

namespace Timings{
    constexpr uint32_t TARGET_LOOP_TIME_US = 10000;
    constexpr uint32_t MAIN_SERIAL_BAUD = 115200;
    constexpr int POWER_ON_RESET = 3;
}

namespace I2CAddressList{
    constexpr uint8_t LSM6DSOX_I2C_ADDR = 0x6A; //This is going to be replaced by the Adafruit's default address variable, but it is pretty much the same thing.
    constexpr uint8_t MS5611_I2C_ADDR = 0x76; //I2C Address for MS5611.
    constexpr uint8_t MS5611_PROM_BASE_ADDR = 0xA2; //(C1) MS5611's PROM Calibration constants' C1 address. Add 2 bytes to get C2-C6 (Note: The last C constant is C6)
    constexpr uint8_t MS5611_RESET_ADDR = 0x1E; //(Reset) MS5611's I2C address for the reset command.
    constexpr uint8_t MS5611_ADC_ADDR = 0x00; //(ADC) MS5611's I2C address for ADC read command.
    constexpr uint8_t MS5611_D1_ADDR = 0x48; //(Pressure) MS5611's I2C address for the conversion command for pressure data.
    constexpr uint8_t MS5611_D2_ADDR = 0x58; //(Temperature) MS5611's I2C address for the conversion command for temperature data
    //constexpr uint8_t ADXL375_I2C_ADDR = ;

}

namespace Required_Bytes{
    constexpr int PROM_C = 2; //PROM calibrations set by the factory's bytes.
    constexpr int D = 3; //D1 (raw pressure) measurement bytes.
    //constexpr int d2_temperature_measurement = ; //D2 (raw temperature) measurement bytes.
}

namespace Safety{
    constexpr float overheadSafetyG = 15.5f;
}