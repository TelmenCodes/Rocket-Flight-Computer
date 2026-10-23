#pragma once

#include <Arduino.h>
#include "Config.h"

enum class SensorType {
    LOW_G_IMU,
    HIGH_G_ACCEL,
    BAROMETER,
    UNKNOWN
};

struct Vector3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3D() = default;
    Vector3D(float x_val, float y_val, float z_val)
        : x(x_val), y(y_val), z(z_val) {}
};

struct SensorData {
    uint32_t timestamp_us = 0;
    float pressure_hpa = 0.0f; //Pressure data. Unit is hPa (hectopascal).
    float temperature_c = 0.0f; //Temperature data. Unit is C (celsius).
    Vector3D accel; //Acceleration data. Unit is m/s^2
    Vector3D gyro; //Angular velocity data. Unit is rad/s
    bool isValid = false; //This indicates a successful bus read.
    bool isSaturated = false; //This indicates if a physical measurement is saturated (i.e. maxed out).
};

class ISensor {
public:
    virtual ~ISensor() = default;
    virtual bool begin() = 0; //Initializes sensors.
    virtual bool read() = 0; //Checks if we read sensors.
    virtual const SensorData& getData() const = 0; 
    virtual SensorType getType() const = 0;
};