#ifndef APP_CAN_IMU_BARO_H
#define APP_CAN_IMU_BARO_H

#include "stm32g4xx_hal.h"
#include "app_bmi330.h"
#include <stdint.h>

#define SENSOR_CAN_ID_FUSED    0x101U
#define SENSOR_ALT_INVALID_CM  ((int32_t)0x7FFFFFFF)

HAL_StatusTypeDef CAN_SENSOR_App_SendCombined(const bmi330_raw_t *imu, int32_t fused_altitude_cm);

#endif
