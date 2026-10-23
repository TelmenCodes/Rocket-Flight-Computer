#include <Arduino.h>
#include <Wire.h>
#include "sensors/MS5611_Driver.h"


MS5611_Driver::MS5611_Driver(uint8_t address){
    i2cAddress = address;
    currentData.isValid = false;
}

SensorType MS5611_Driver::getType() const {
    return SensorType::BAROMETER;
}

bool MS5611_Driver::begin(){ //Initializes MS5611 and sets the I2C address.
    Wire.begin();
    Wire.beginTransmission(i2cAddress);
    Wire.write(I2CAddressList::MS5611_RESET_ADDR);
    Wire.endTransmission();
    delay(Timings::POWER_ON_RESET);
    for (int i = 0; i < 6; i++){ //Cycles through C1-C6
        uint8_t c_addr = I2CAddressList::MS5611_PROM_BASE_ADDR + (i * 2); //Loops through the PROM addresses. Difference between C constants are 2 bytes.
        Wire.beginTransmission(i2cAddress);
        Wire.write(c_addr);
        Wire.endTransmission();
        Wire.requestFrom(i2cAddress, (uint8_t)Required_Bytes::PROM_C);
        if (Wire.available() >= Required_Bytes::PROM_C){
            uint16_t msb = Wire.read(); //Reads the Most Significant Byte which are the initial bytes.
            uint16_t lsb = Wire.read(); //Reads the Least Significant Byte which are the last bytes.
            C[i] = (msb << 8) | lsb; //Shifts the MSB 8 bits to the left and then adds the LSB (OR operator).
        }
        else if (Wire.available() < Required_Bytes::PROM_C){
            return false;
        }
    }
    currentState = State::TRIGGER_D1;
    return true;
}

bool MS5611_Driver::read(){
    switch(currentState){
        case State::UNINITIALIZED:
            return false;

        case State::TRIGGER_D1:
            Wire.beginTransmission(i2cAddress);
            Wire.write(I2CAddressList::MS5611_D1_ADDR);
            Wire.endTransmission();
            lastConversionStart_us = micros(); //Records the timestamp.
            currentState = State::WAIT_D1;
            return false;
        
        case State::WAIT_D1:
            if(micros() - lastConversionStart_us < 9000){
                return false;
            }
            else{
                currentState = State::READ_D1_TRIGGER_D2;
                return false;
            }
        
        case State::READ_D1_TRIGGER_D2:
            Wire.beginTransmission(i2cAddress);
            Wire.write(I2CAddressList::MS5611_ADC_ADDR);
            Wire.endTransmission();
            Wire.requestFrom(i2cAddress, (uint8_t)Required_Bytes::D);
            if (Wire.available() >= Required_Bytes::D){
                MS5611_Driver::b0 = Wire.read();
                MS5611_Driver::b1 = Wire.read();
                MS5611_Driver::b2 = Wire.read();
                MS5611_Driver::D1 = ((uint32_t)b0 << 16) | 
                                    ((uint32_t)b1 << 8) |
                                    (uint32_t)b2;
                Wire.beginTransmission(i2cAddress);
                Wire.write(I2CAddressList::MS5611_D2_ADDR);
                Wire.endTransmission();
                lastConversionStart_us = micros();
                currentState = State::WAIT_D2;
                return false;
            }
            currentData.isValid = false;
            currentState = State::TRIGGER_D1;
            return false;
        
        case State::WAIT_D2:
            if(micros() - lastConversionStart_us < 9000){
                return false;
            }
            else{
                currentState = State::READ_D2_CALCULATE;
                return false;
            }
        
        case State::READ_D2_CALCULATE:
            Wire.beginTransmission(i2cAddress);
            Wire.write(I2CAddressList::MS5611_ADC_ADDR);
            Wire.endTransmission();

            Wire.requestFrom(i2cAddress, (uint8_t)Required_Bytes::D);

            if(Wire.available() < Required_Bytes::D) {
                currentData.isValid = false;
                currentState = State::TRIGGER_D1;
                return false;
            }

            b0 = Wire.read();
            b1 = Wire.read();
            b2 = Wire.read();

            D2 = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;

            calculateCompensation();

            lastConversionStart_us = micros();
            currentData.isValid = true;
            currentState = State::TRIGGER_D1;
            return true;
    }
    return false;
}

const SensorData& MS5611_Driver::getData() const {
    return currentData;
}

void MS5611_Driver::calculateCompensation(){
        int64_t sens = 0; 
        int64_t off = 0;
        int64_t actualTemp = 0;

        int64_t sens2 = 0;
        int64_t off2 = 0;
        int64_t temp2 = 0;

        int64_t tempDiff = 0;

        int64_t twoP7 = 1ULL << 7; //2^7. Offset denominator
        int64_t twoP8 = 1ULL << 8; //2^8 Sensitivity denominator
        int64_t twoP23 = 1ULL << 23; //2^23 Actual temperature's denominator
        
        int64_t twoP31 = 1ULL << 31; //2^31 Second order Temperature denominator
        int64_t twoP1 = 1ULL << 1; // 2^1 Second order Offset denominator
        int64_t twoP2 = 1ULL << 2; //2^2 Second order Sensitivity denominator
        int64_t secondOrderCoeff5 = 5;
        int64_t secondOrderCoeff7 = 7;
        int64_t secondOrderCoeff11 = 11;

        int64_t pressureDenominator21 = 1ULL << 21; //2^21
        int64_t pressureDenominator15 = 1ULL << 15; //2^15

        tempDiff = D2 - ((int32_t)C[4] << 8);
        actualTemp = 2000 + (((int64_t)tempDiff * C[5]) / twoP23);

        off = ((int64_t)C[1] << 16) + (((int64_t)C[3] * tempDiff ) / twoP7);
        sens = ((int64_t)C[0] << 15) + ( ( ((int64_t)C[2]) * tempDiff) / twoP8);

        if (actualTemp < 2000){
                temp2 = ((int64_t)tempDiff * tempDiff) / twoP31;
                off2 = off2 + (secondOrderCoeff5 * (actualTemp-2000) * (actualTemp-2000)) / twoP1;
                sens2 = sens2 + (secondOrderCoeff5 * (actualTemp-2000) * (actualTemp-2000)) / twoP2;
            if (actualTemp < -1500){
                off2 += (secondOrderCoeff7 * (actualTemp + 1500) * (actualTemp + 1500));
                sens2 += (secondOrderCoeff11 * (actualTemp + 1500) * (actualTemp + 1500)) / twoP1;
            }
        }
        actualTemp = actualTemp - temp2;
        off = off - off2;
        sens = sens - sens2;

        currentData.pressure_hpa = ( (((int64_t)D1 * sens)/pressureDenominator21 - off) / pressureDenominator15) / 100.0f;
        currentData.temperature_c = actualTemp / 100.f;
}