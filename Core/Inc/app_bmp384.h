#ifndef APP_BMP384_H
#define APP_BMP384_H

#include "main.h"
#include "bmp3.h"
#include <stdbool.h>

typedef struct
{
    float pressure_pa;
    float temperature_c;
    bool valid;
} bmp384_sample_t;

HAL_StatusTypeDef BMP384_App_Init(void);
HAL_StatusTypeDef BMP384_App_ReadSample(bmp384_sample_t *sample);

void BMP384_App_DataReadyIRQ(void);
uint8_t BMP384_App_HasNewData(void);
void BMP384_App_ClearNewData(void);

#endif
