#pragma once

#include <SPI.h>
#include "Config.h"
#include "ISensor.h"

class MS5611_Driver: public ISensor {
    public:
        MS5611_Driver(uint8_t MS5611_CS_PIN = TeensyPins::MS5611_CS_PIN);
        bool begin() override;
        bool read() override;
        const SensorData& getData() const override;
        SensorType getType() const override;
    private:
        enum class State{
            UNINITIALIZED,
            TRIGGER_D1,
            WAIT_D1_TRIGGER_D2,
            WAIT_D2_CALCULATE
        };
        State currentState = State::UNINITIALIZED;
        uint8_t csPin;
        static constexpr uint8_t PROM_C1 = 0xA0; //(C1) MS5611's PROM Calibration constants' C1 address. Add 2 bytes to get C2-C6 (Note: The last C constant is C6)
        static constexpr uint8_t RESET = 0x1E; //(Reset) MS5611's SPI address for the reset command.
        
        static constexpr uint8_t D1_ADDR = 0x48; //(Pressure, Oversampling Ratio 4096Hz) MS5611's SPI address for the conversion command for pressure data.
        static constexpr uint8_t D2_ADDR = 0x58; //(Temperature, Oversampling Ratio 4096Hz) MS5611's SPI address for the conversion command for temperature data.
        
        static constexpr uint8_t SPI_READ = 0x80; //(Read command) Must pair this with an address/register using an bitwise OR operator to read data.
        static constexpr uint8_t ADC_READ = 0x00; //(ADC) MS5611's SPI address for ADC read command.
        static constexpr uint8_t DUMMY_BYTE = 0x00; //Dummy byte to send, which gives back data from the sensor.

        static constexpr uint8_t C_BYTES = 2; //Required bytes for calibration constants for successful read.
        static constexpr uint8_t D_BYTES = 3; //Required bytes for returned measurement for successful read.
        static constexpr uint8_t PWR_ON_DELAY = 3; //Time needed for MS5611 to power on. (millisecond)

        static constexpr uint32_t MAX_SPEED = 10000000UL; //(Maximum Clock Speed 10MHz)
        static const SPISettings spiSettings;

        uint16_t C[5]; //Hold Calibration constants C1-C6 and Factory reserved data (Not needed for calculation). C7 is Cyclic Redundency Check, which is used to confirm that the data hasn't corrupted.
        uint32_t D1 = 0; //(Pressure) Uncorrected Pressure data.
        uint32_t D2 = 0; //(Temp) Uncorrected Temperature data.
        uint32_t lastConversionStart_us = 0; //Timestamp of the last conversion.
        SensorData currentData;
        inline void selectChip(){ //(Selects Chip) Sets the CS Pin low, so the chip is selected.
            digitalWrite(csPin, LOW);
        }

        inline void deselectChip(){ //(Deselect Chip) Sets the CS Pin high, so the chip is unselected.
            digitalWrite(csPin, HIGH);
        }
        bool crcCheck(); //Checks if CRC matches calculated CRC (Note: It also sets the Calibration constants automatically IF it matches.)
        void triggerADC(uint8_t d_cmd);
        uint32_t readData();
        void calculateCompensation();
};