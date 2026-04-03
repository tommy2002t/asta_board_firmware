#include "app_can_IMU_BARO.h"

#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static HAL_StatusTypeDef BMI330_CAN_SendFrame(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef txHeader;

    memset(&txHeader, 0, sizeof(txHeader));

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t *)data) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static uint16_t BMP384_PressureToDeciHpa(float pressure_pa)
{
    float scaled;

    if (pressure_pa <= 0.0f)
    {
        return 0U;
    }

    scaled = (pressure_pa / 10.0f) + 0.5f;

    if (scaled >= 65535.0f)
    {
        return 65535U;
    }

    return (uint16_t)scaled;
}

static int16_t BMP384_TemperatureToCentiDeg(float temperature_c)
{
    float scaled;

    if (temperature_c >= 0.0f)
    {
        scaled = (temperature_c * 100.0f) + 0.5f;
    }
    else
    {
        scaled = (temperature_c * 100.0f) - 0.5f;
    }

    if (scaled > 32767.0f)
    {
        return 32767;
    }

    if (scaled < -32768.0f)
    {
        return -32768;
    }

    return (int16_t)scaled;
}

HAL_StatusTypeDef CAN_SENSOR_App_SendCombined(const bmi330_raw_t *imu, const bmp384_sample_t *bmp)
{
    uint8_t frame1[8];
    uint8_t frame2[8];
    uint16_t pressure_u;
    int16_t temperature_s;

    if ((imu == NULL) || (bmp == NULL) || (bmp->valid == false))
    {
        return HAL_ERROR;
    }

    pressure_u = BMP384_PressureToDeciHpa(bmp->pressure_pa);
    temperature_s = BMP384_TemperatureToCentiDeg(bmp->temperature_c);

    /* Frame 1 (8 bytes): ax, ay, az, gx */
    frame1[0] = (uint8_t)(((uint16_t)imu->ax) >> 8);
    frame1[1] = (uint8_t)((uint16_t)imu->ax);
    frame1[2] = (uint8_t)(((uint16_t)imu->ay) >> 8);
    frame1[3] = (uint8_t)((uint16_t)imu->ay);
    frame1[4] = (uint8_t)(((uint16_t)imu->az) >> 8);
    frame1[5] = (uint8_t)((uint16_t)imu->az);
    frame1[6] = (uint8_t)(((uint16_t)imu->gx) >> 8);
    frame1[7] = (uint8_t)((uint16_t)imu->gx);

    /* Frame 2 (8 bytes): gy, gz, pressure(deci-hPa), temperature(cdegC) */
    frame2[0] = (uint8_t)(((uint16_t)imu->gy) >> 8);
    frame2[1] = (uint8_t)((uint16_t)imu->gy);
    frame2[2] = (uint8_t)(((uint16_t)imu->gz) >> 8);
    frame2[3] = (uint8_t)((uint16_t)imu->gz);
    frame2[4] = (uint8_t)(pressure_u >> 8);
    frame2[5] = (uint8_t)(pressure_u);
    frame2[6] = (uint8_t)(((uint16_t)temperature_s) >> 8);
    frame2[7] = (uint8_t)((uint16_t)temperature_s);

    if (BMI330_CAN_SendFrame(SENSOR_CAN_ID_FRAME1, frame1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (BMI330_CAN_SendFrame(SENSOR_CAN_ID_FRAME2, frame2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
