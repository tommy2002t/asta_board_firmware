#ifndef APP_BMI330_H
#define APP_BMI330_H

#include "stdint.h"
#include "stm32g4xx_hal.h"
#include "bmi330.h"

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} bmi330_raw_t;

HAL_StatusTypeDef BMI330_App_Init(void);
HAL_StatusTypeDef BMI330_App_ReadRaw(bmi330_raw_t *data);

#endif
