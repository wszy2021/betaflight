#pragma once

#include <stdint.h>

#include "common/axis.h"

#define GYRO_CALIBRATION_VALUE_SCALE 100
#define GYRO_CALIBRATION_CONFIG_VALUE_COUNT (XYZ_AXIS_COUNT + 1)
#define GYRO_CALIBRATION_CONFIG_COUNT 2

typedef struct gyroCalibrationConfigValues_s {
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t calibrationCompleted;
} gyroCalibrationConfigValues_t;

typedef union gyroCalibrationConfig_u {
    int32_t raw[GYRO_CALIBRATION_CONFIG_VALUE_COUNT];
    gyroCalibrationConfigValues_t values;
} gyroCalibrationConfig_t;

void followCliGyroCal(const char *cmdName, char *cmdline);
