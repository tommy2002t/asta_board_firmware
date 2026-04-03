#include "app_bmp384.h"
#include "bmp3_port.h"
#include "app_config.h"
#include <string.h>

extern SPI_HandleTypeDef hspi2;

static struct bmp3_dev bmp_dev;
static bmp3_hal_dev_t bmp_hal;
static volatile uint8_t data_ready = 0U;

HAL_StatusTypeDef BMP384_App_Init(void)
{
    int8_t rslt;
    uint32_t settings_sel;
    struct bmp3_settings settings;

    memset(&bmp_dev, 0, sizeof(bmp_dev));
    memset(&bmp_hal, 0, sizeof(bmp_hal));
    memset(&settings, 0, sizeof(settings));

    data_ready = 0U;

    BMP3_Port_InitDWT();

    HAL_GPIO_WritePin(BMP384_CS_GPIO_Port, BMP384_CS_Pin, GPIO_PIN_SET);

    bmp_hal.hspi = &hspi2;
    bmp_hal.cs_port = BMP384_CS_GPIO_Port;
    bmp_hal.cs_pin = BMP384_CS_Pin;

    bmp_dev.intf = BMP3_SPI_INTF;
    bmp_dev.intf_ptr = &bmp_hal;
    bmp_dev.read = bmp3_hal_read;
    bmp_dev.write = bmp3_hal_write;
    bmp_dev.delay_us = bmp3_hal_delay_us;

    rslt = bmp3_init(&bmp_dev);
    if (rslt != BMP3_OK)
    {
        return HAL_ERROR;
    }

    settings.press_en = BMP3_ENABLE;
    settings.temp_en  = BMP3_ENABLE;

    settings.odr_filter.press_os   = BMP384_DEFAULT_PRESS_OS;
    settings.odr_filter.temp_os    = BMP384_DEFAULT_TEMP_OS;
    settings.odr_filter.iir_filter = BMP384_DEFAULT_IIR_FILTER;
    settings.odr_filter.odr        = BMP384_DEFAULT_ODR;

    settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
    settings.int_settings.level       = BMP3_INT_PIN_ACTIVE_HIGH;
    settings.int_settings.latch       = BMP3_INT_PIN_NON_LATCH;
    settings.int_settings.drdy_en     = BMP3_ENABLE;

    settings_sel =
        BMP3_SEL_PRESS_EN |
        BMP3_SEL_TEMP_EN |
        BMP3_SEL_PRESS_OS |
        BMP3_SEL_TEMP_OS |
        BMP3_SEL_IIR_FILTER |
        BMP3_SEL_ODR |
        BMP3_SEL_OUTPUT_MODE |
        BMP3_SEL_LEVEL |
        BMP3_SEL_LATCH |
        BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &bmp_dev);
    if (rslt != BMP3_OK)
    {
        return HAL_ERROR;
    }

    settings.op_mode = BMP3_MODE_NORMAL;

    rslt = bmp3_set_op_mode(&settings, &bmp_dev);
    if (rslt != BMP3_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BMP384_App_ReadSample(bmp384_sample_t *sample)
{
    struct bmp3_data data;

    if (sample == NULL)
    {
        return HAL_ERROR;
    }

    if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmp_dev) != BMP3_OK)
    {
        return HAL_ERROR;
    }

    sample->temperature_c = (float)data.temperature;
    sample->pressure_pa   = (float)data.pressure;
    sample->valid = true;

    return HAL_OK;
}

void BMP384_App_DataReadyIRQ(void)
{
    data_ready = 1U;
}

uint8_t BMP384_App_HasNewData(void)
{
    return data_ready;
}

void BMP384_App_ClearNewData(void)
{
    data_ready = 0U;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BMP384_INT_Pin)
    {
        BMP384_App_DataReadyIRQ();
    }
}
