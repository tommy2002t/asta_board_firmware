#include "app_bmi330.h"
#include "spi.h"
#include "gpio.h"

#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define BMI330_CS_PORT GPIOB
#define BMI330_CS_PIN  GPIO_PIN_0

static struct bmi3_dev bmi_dev;
static uint8_t bmi330_initialized = 0U;
static uint8_t bmi330_delay_timer_ready = 0U;
static uint32_t bmi330_cycles_per_us = 0U;

/* ================= LOW LEVEL ================= */

static void cs_low(void)
{
    HAL_GPIO_WritePin(BMI330_CS_PORT, BMI330_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(BMI330_CS_PORT, BMI330_CS_PIN, GPIO_PIN_SET);
}

static void BMI330_InitDelayTimer(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0U;

    bmi330_cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
    bmi330_delay_timer_ready = (bmi330_cycles_per_us > 0U) ? 1U : 0U;
}

static BMI3_INTF_RET_TYPE spi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef hal_ret;
    uint8_t tx[256];

    (void)intf_ptr;

    if ((data == NULL) || ((len + 1U) > sizeof(tx)))
    {
        return BMI3_E_COM_FAIL;
    }

    tx[0] = (uint8_t)(reg & 0x7FU);

    for (uint32_t i = 0; i < len; i++)
    {
        tx[i + 1U] = data[i];
    }

    cs_low();
    hal_ret = HAL_SPI_Transmit(&hspi1, tx, (uint16_t)(len + 1U), HAL_MAX_DELAY);
    cs_high();

    if (hal_ret != HAL_OK)
    {
        return BMI3_E_COM_FAIL;
    }

    return BMI3_OK;
}

static BMI3_INTF_RET_TYPE spi_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr)
{
    HAL_StatusTypeDef hal_ret;
    uint8_t addr;
    uint8_t dummy_tx[256];
    uint8_t rx_buf[256];

    (void)intf_ptr;

    if ((data == NULL) || (len > sizeof(dummy_tx)))
    {
        return BMI3_E_COM_FAIL;
    }

    memset(dummy_tx, 0, sizeof(dummy_tx));
    memset(rx_buf, 0, sizeof(rx_buf));

    addr = (uint8_t)(reg | 0x80U);

    cs_low();

    hal_ret = HAL_SPI_Transmit(&hspi1, &addr, 1U, HAL_MAX_DELAY);
    if (hal_ret != HAL_OK)
    {
        cs_high();
        return BMI3_E_COM_FAIL;
    }

    hal_ret = HAL_SPI_TransmitReceive(&hspi1, dummy_tx, rx_buf, (uint16_t)len, HAL_MAX_DELAY);

    cs_high();

    if (hal_ret != HAL_OK)
    {
        return BMI3_E_COM_FAIL;
    }

    memcpy(data, rx_buf, len);
    return BMI3_OK;
}

static void delay_us(uint32_t period, void *intf_ptr)
{
    uint32_t start;
    uint32_t ticks;

    (void)intf_ptr;

    if (period == 0U)
    {
        return;
    }

    if ((bmi330_delay_timer_ready == 0U) || (bmi330_cycles_per_us == 0U))
    {
        HAL_Delay((period + 999U) / 1000U);
        return;
    }

    start = DWT->CYCCNT;
    ticks = period * bmi330_cycles_per_us;

    while ((DWT->CYCCNT - start) < ticks)
    {
        /* wait */
    }
}

/* ================= HIGH LEVEL ================= */

HAL_StatusTypeDef BMI330_App_Init(void)
{
    int8_t rslt;
    struct bmi3_sens_config config[2];

    memset(&bmi_dev, 0, sizeof(bmi_dev));
    memset(config, 0, sizeof(config));

    bmi330_initialized = 0U;
    BMI330_InitDelayTimer();
    cs_high();

    bmi_dev.read = spi_read;
    bmi_dev.write = spi_write;
    bmi_dev.delay_us = delay_us;
    bmi_dev.intf = BMI3_SPI_INTF;
    bmi_dev.intf_ptr = NULL;

    rslt = bmi330_init(&bmi_dev);
    if (rslt != BMI3_OK)
    {
        return HAL_ERROR;
    }

    config[0].type = BMI3_ACCEL;
    config[1].type = BMI3_GYRO;

    rslt = bmi330_get_sensor_config(config, 2, &bmi_dev);
    if (rslt != BMI3_OK)
    {
        return HAL_ERROR;
    }

    config[0].cfg.acc.odr = BMI3_ACC_ODR_100HZ;
    config[0].cfg.acc.range = BMI3_ACC_RANGE_4G;
    config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
    config[0].cfg.acc.avg_num = BMI3_ACC_AVG1;
    config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    config[1].cfg.gyr.odr = BMI3_GYR_ODR_100HZ;
    config[1].cfg.gyr.range = BMI3_GYR_RANGE_500DPS;
    config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_QUARTER;
    config[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;
    config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    rslt = bmi330_set_sensor_config(config, 2, &bmi_dev);
    if (rslt != BMI3_OK)
    {
        return HAL_ERROR;
    }

    bmi330_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef BMI330_App_ReadRaw(bmi330_raw_t *data)
{
    int8_t rslt;
    struct bmi3_sensor_data sensor_data[2];

    if ((data == NULL) || (bmi330_initialized == 0U))
    {
        return HAL_ERROR;
    }

    memset(sensor_data, 0, sizeof(sensor_data));

    sensor_data[0].type = BMI3_ACCEL;
    sensor_data[1].type = BMI3_GYRO;

    rslt = bmi330_get_sensor_data(sensor_data, 2, &bmi_dev);
    if (rslt != BMI3_OK)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 2U; i++)
    {
        if (sensor_data[i].type == BMI3_ACCEL)
        {
            data->ax = sensor_data[i].sens_data.acc.x;
            data->ay = sensor_data[i].sens_data.acc.y;
            data->az = sensor_data[i].sens_data.acc.z;
        }
        else if (sensor_data[i].type == BMI3_GYRO)
        {
            data->gx = sensor_data[i].sens_data.gyr.x;
            data->gy = sensor_data[i].sens_data.gyr.y;
            data->gz = sensor_data[i].sens_data.gyr.z;
        }
    }

    return HAL_OK;
}
