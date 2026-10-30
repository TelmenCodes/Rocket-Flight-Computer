#pragma once

#include <SPI.h>
#include "Config.h"
#include "ISensor.h"

class LSM6DSOX_Driver: public ISensor{
    public:
        LSM6DSOX_Driver(uint8_t LSM6DS_CS_PIN = TeensyPins::LSM6DSOX_CS_PIN);
        bool begin() override;
        bool read() override;
        const SensorData& getData() const override;
        SensorType getType() const override;
    private:
        uint8_t csPin;
        int16_t gyro_rawX = 0;
        int16_t gyro_rawY = 0;
        int16_t gyro_rawZ = 0;
        int16_t accel_rawX = 0;
        int16_t accel_rawY = 0;
        int16_t accel_rawZ = 0;

        static constexpr uint8_t REG_DEVID = 0x0F; //(Device ID Register) Registers the device ID. (It should match the Device ID Address)
        static constexpr uint8_t DEVID_EXPECTED = 0x6C; //(Expected Device ID)
        
        static constexpr uint8_t REG_ACCEL = 0x10; //(Acceleration Register)
        static constexpr uint8_t ACCEL_416HZ = 0x64; //(Acceleration Oversampling Ratio 416Hz at 16G) Sets the acceleration oversampling rate to 416Hz and the maximum acceleration at 16G.
        
        static constexpr uint8_t REG_GYRO = 0x11; //(Gyroscope Register)
        static constexpr uint8_t GYRO_416HZ = 0x6C; //(Gyroscope Oversampling Ratio 416Hz at 2000DPS) Sets the gyroscope oversampling rate to 416Hz and the maximum DPS at 2000.

        static constexpr uint8_t REG_BDU = 0x12; //(Block Data Update Register)
        static constexpr uint8_t BDU_ENABLED = 0x44; //(Enables Block Data Update) Enabling Block Data Update prevents byte updates during read and keeps register address auto-increment enabled.

        static constexpr uint8_t REG_OUT_G = 0x22; //(Data's first byte out Register) Gyroscope data (X axis and LSB first) is up first, then Accelerometer data (X axis and LSB first).

        static constexpr uint8_t SPI_READ = 0x80;
        static constexpr uint8_t DUMMY_BYTE = 0x00; //(Dummy byte)
        
        static constexpr uint32_t MAX_SPEED = 10000000UL; //(Maximum Clock Speed 10MHz) 
        static const SPISettings spiSettings;

        static constexpr float SCALE_FACTOR_G = 0.000488f; //(Acceleration Scale Factor) Scale factor to correct Acceleration data. (Acceleration data * Scale factor * g)
        static constexpr float SCALE_FACTOR_DPS = 0.070f; //(Gyroscope DPS Scale Factor) Scale factor to correct gyroscope data. (Gyroscope data * Scale factor * Degree to Radians Multiplier)
        static constexpr float DEG_TO_RAD_MULT = 0.0174532925199f; //(Pi / 180) Multiplier to change Degrees into Radians
        static constexpr float OVERHEAD_SAFETY = 15.5f * PhysicsConstants::STANDARD_GRAVITY_M_S2; //(Overhead safety for the LSM6DSOX 15.5G) Overhead Safety to prevent sensor saturation for the LSM6DSOX.
        SensorData currentData;

        inline void selectChip(){ //(Selects Chip) Sets the CS Pin low, so the chip is selected.
            digitalWrite(csPin, LOW);
        }

        inline void deselectChip(){ //(Deselect Chip) Sets the CS Pin high, so the chip is unselected.
            digitalWrite(csPin, HIGH);
        }
        void burstReadData(uint8_t* buffer);
};