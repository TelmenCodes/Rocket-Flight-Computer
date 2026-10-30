#include "sensors/MS5611_Driver.h"

const SPISettings MS5611_Driver::spiSettings(MS5611_Driver::MAX_SPEED, MSBFIRST, SPI_MODE3);

MS5611_Driver::MS5611_Driver(uint8_t MS5611_CS_PIN){
    csPin = MS5611_CS_PIN;
    currentData.isValid = false;
}

SensorType MS5611_Driver::getType() const {
    return SensorType::BAROMETER;
}

bool MS5611_Driver::begin(){ //Initializes MS5611 and sets the SPI address.
    pinMode(csPin, OUTPUT);
    deselectChip();
    SPI.begin();
    
    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(RESET);
    SPI.endTransaction();
    deselectChip();
    delay(PWR_ON_DELAY); //Waits 3ms for MS5611 to fully power on.
    
    bool crcStatus = MS5611_Driver::crcCheck();
    
    if (crcStatus == false){ //Checks if the CRC check was unsuccessful.
        return false;
    }

    currentState = State::TRIGGER_D1;
    return true;
}

bool MS5611_Driver::read(){
    switch(currentState){
        case State::UNINITIALIZED:
            return false;

        case State::TRIGGER_D1:
            triggerADC(D1_ADDR);
            lastConversionStart_us = micros(); //Records the timestamp.
            currentState = State::WAIT_D1_TRIGGER_D2;
            return false;
        
        case State::WAIT_D1_TRIGGER_D2:
            if(micros() - lastConversionStart_us < 10000){
                return false;
            }
            else{
                D1 = readData();
                triggerADC(D2_ADDR);
                lastConversionStart_us = micros();
                currentState = State::WAIT_D2_CALCULATE;
                return false;
            }
        
        case State::WAIT_D2_CALCULATE:
            if(micros() - lastConversionStart_us < 10000){
                return false;
            }
            else{
                D2 = readData();
                calculateCompensation();
                lastConversionStart_us = micros();
                currentData.isValid = true;
                currentState = State::TRIGGER_D1;
                return true;
            }
    }
    return false;
}

const SensorData& MS5611_Driver::getData() const{
    return currentData;
}

bool MS5611_Driver::crcCheck(){
    uint16_t prom[8];
    for (int i = 0; i < 8; i++){ //Cycles through Factory reserved data and C1-C7.
        uint8_t c_addr = PROM_C1 + (i * 2); //Loops through the PROM to get the Calibration constants. Difference between C constants are 2 bytes.
        
        selectChip();
        SPI.beginTransaction(spiSettings);
        SPI.transfer(c_addr);
        uint8_t msb = SPI.transfer(DUMMY_BYTE);
        uint8_t lsb = SPI.transfer(DUMMY_BYTE);
        SPI.endTransaction();
        deselectChip();
        
        prom[i] = (uint16_t)msb << 8 | (uint16_t) lsb;
        
        if (prom[i] == 0x0000 or prom[i] == 0xFFFF){ //Checks if the Calibration constants (C1-C7) are corrupted.
            return false;
        }
    }
    
    uint16_t n_rem = 0; //CRC Reminder
    uint8_t nth_bit; //Cycles through 8 bits.

    uint16_t crc = prom[7] & 0x000F; //Saves the CRC 4 bits.

    prom[7] &= 0xFF00; //Resets the CRC 4 bits.

    for (int i = 0; i < 16; i++){
        if (i % 2 == 1){
            n_rem ^= (uint8_t) ((prom[i >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t) ((prom[i >> 1]) >> 8);
        }

        for (nth_bit = 8; nth_bit > 0; nth_bit--){
            if (n_rem & (0x8000)){
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = n_rem << 1;
            }
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); //Moves the CRC bits to the right. Last 4 are CRC.
    n_rem ^= 0x00;
    uint16_t calculated_crc = n_rem & 0x000F;

    if (calculated_crc != crc){ //Checks if C7 matches (Indicates corrupted data).
        return false;
    } else {
        for (int n = 0; n < 6; n++){
            C[n] = prom[n+1]; //Stores the Calibration constants. n+1 to skip the factory reserved data.
        }
        return true;
    }
}

void MS5611_Driver::triggerADC(uint8_t d_cmd){
    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(d_cmd);
    SPI.endTransaction();
    deselectChip();
}

uint32_t MS5611_Driver::readData(){
    selectChip();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(ADC_READ);
    uint8_t msb = SPI.transfer(DUMMY_BYTE);
    uint8_t mid = SPI.transfer(DUMMY_BYTE);
    uint8_t lsb = SPI.transfer(DUMMY_BYTE);
    SPI.endTransaction();
    deselectChip();
    uint32_t data = ((uint32_t) msb << 16) | ((uint32_t) mid << 8) | ((uint32_t) lsb);
    return data;
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

        tempDiff = (int64_t)D2 - ((int64_t)C[4] << 8);
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