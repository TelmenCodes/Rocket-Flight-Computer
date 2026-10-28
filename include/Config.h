#pragma once

#include <Arduino.h>

namespace TeensyPins{
    constexpr int I2C_SDA_PIN = 18; //LSM6DSOX and MS5611's SDA pin.
    constexpr int I2C_SCL_PIN = 19; //LSM6DSOX and MS5611's SCL pin.
    constexpr int CS_PIN = 10; //ADXL375's Chip Select pin.
    constexpr int MOSI_PIN = 11; //(Pin to send data to the sensor) ADXL375's Master Out Slave In pin.
    constexpr int MISO_PIN = 12; //(Pin to receive data from the sensor) ADXL375's Master In Slave Out pin.
    constexpr int SCK_PIN = 13; //(SPI's Clock Pin) ADXL375's SCK pin.
}

namespace PhysicsConstants{
    static constexpr float STANDARD_GRAVITY_M_S2 = 9.80665f;
    static constexpr float SEA_LEVEL_PRESSURE_HPA = 1013.25f; 
}

namespace Timings{
    constexpr uint32_t TARGET_LOOP_TIME_US = 10000;
    constexpr uint32_t MAIN_SERIAL_BAUD = 115200;
}

namespace I2CAddressList{
    constexpr uint8_t LSM6DSOX_I2C_ADDR = 0x6A; //I2C Address for the LSM6DSOX.
    constexpr uint8_t MS5611_I2C_ADDR = 0x77; //I2C Address for MS5611.
    constexpr uint8_t ADXL375_I2C_ADDR = 0x53; //I2C Address for the ADXL375.

}

namespace Safety{
    constexpr float overheadSafetyG = 15.5f;
}