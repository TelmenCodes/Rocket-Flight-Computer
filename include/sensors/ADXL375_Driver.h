#pragma once

#include <SPI.h>
#include "Config.h"
#include "ISensor.h"

class ADXL375_Driver: public ISensor{
    public:
        ADXL375_Driver(uint8_t ADXL375_CS_PIN = TeensyPins::ADXL375_CS_PIN);
        bool begin() override;
        bool read() override;
        const SensorData& getData() const override;
        SensorType getType() const override;
    private:
        uint8_t csPin;
        int16_t rawX = 0; //(Raw Acceleration data X) Formula to get actual data is rawAxis * Scale Factor * g.
        int16_t rawY = 0; //(Raw Acceleration data Y) Formula to get actual data is rawAxis * Scale Factor * g.
        int16_t rawZ = 0; //(Raw Acceleration data Z) Formula to get actual data is rawAxis * Scale Factor * g.

        static constexpr uint8_t REG_DEVID = 0x00; //(Device ID Register) Registers the device ID. (It should match the Device ID Address)
        static constexpr uint8_t DUMMY_BYTE = 0x00; //(Dummy byte)
        static constexpr uint8_t DEVID_EXPECTED = 0xE5; //(Expected Device ID)

        static constexpr uint8_t REG_BW_RATE = 0x2C; //Register address for Bandwidth and Rate
        static constexpr uint8_t BW_RATE_400HZ = 0x0C; //(Bandwidth and Rate 400Hz) Value to set the Bandwidth and Rate as 400 Hz.
        static constexpr uint8_t RATE_100HZ = 0x0A;//(Bandwidth and Rate 100Hz) Value to set the Bandwidth and Rate as 100 Hz.
        
        static constexpr uint8_t REG_PWR_CTRL = 0x2D; //Register Address for power control. (Wakes up the ADXL375 from Standby mode into Active Measurement mode.)
        static constexpr uint8_t PWR_CTRL_MEASURE = 0x08;

        static constexpr uint8_t DATA_FORMAT = 0x31;
        static constexpr uint8_t REG_DATA_X_LSB = 0x32; //(LSB of Acceleration's X axis) X,Y and Z are from LSB to MSB on addresses 0x32-0x37
        
        static constexpr uint8_t MULTI_BYTE = 0x40; //(Receives data as Multi-bytes)
        static constexpr uint8_t SPI_READ = 0x80; //(Read command)

        static constexpr uint32_t MAX_SPEED = 5000000; //(Maximum Clock Speed 5MHz) ADXL375's maximum clock speed is 5MHz.
        static const SPISettings spiSettings;

        static constexpr float SCALE_FACTOR_G = 0.049f; //(Acceleration Scale Factor) Scale factor to correct received data. (Acceleration data * Scale factor * g)
        static constexpr float OVERHEAD_SAFETY = 195.00f * PhysicsConstants::STANDARD_GRAVITY_M_S2; //(Saturation Overhead Safety)
        SensorData currentData;
        
        inline void selectChip(){ //(Selects Chip) Sets the CS Pin low, so the chip is selected.
            digitalWrite(csPin, LOW);
        }

        inline void deselectChip(){ //(Deselect Chip) Sets the CS Pin high, so the chip is unselected.
            digitalWrite(csPin, HIGH);
        }
        void burstReadData(uint8_t* data_buffer);
};