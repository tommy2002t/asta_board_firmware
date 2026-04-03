#ifndef APP_CAN_IMU_BARO_H
#define APP_CAN_IMU_BARO_H

#include "stm32g4xx_hal.h"
#include "app_bmi330.h"
#include "app_bmp384.h"

#define SENSOR_CAN_ID_FRAME1   0x101U
#define SENSOR_CAN_ID_FRAME2   0x102U

HAL_StatusTypeDef CAN_SENSOR_App_SendCombined(const bmi330_raw_t *imu, const bmp384_sample_t *bmp);

#endif
