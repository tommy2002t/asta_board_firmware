#include "app_can_IMU_BARO.h"

#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static HAL_StatusTypeDef BMI330_CAN_SendFrame(uint32_t id, const uint8_t data[16])
{
    FDCAN_TxHeaderTypeDef txHeader;

    memset(&txHeader, 0, sizeof(txHeader));

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_16;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_ON;
    txHeader.FDFormat = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t *)data) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef CAN_SENSOR_App_SendCombined(const bmi330_raw_t *imu, int32_t fused_altitude_cm)
{
    uint8_t frame[16];

    if (imu == NULL)
    {
        return HAL_ERROR;
    }

    /* Packet 2:
     * fused_altitude_cm (int32_t)
     * ax, ay, az, gx, gy, gz (int16_t raw)
     */
    frame[0]  = (uint8_t)(((uint32_t)fused_altitude_cm) >> 24);
    frame[1]  = (uint8_t)(((uint32_t)fused_altitude_cm) >> 16);
    frame[2]  = (uint8_t)(((uint32_t)fused_altitude_cm) >> 8);
    frame[3]  = (uint8_t)((uint32_t)fused_altitude_cm);

    frame[4]  = (uint8_t)(((uint16_t)imu->ax) >> 8);
    frame[5]  = (uint8_t)((uint16_t)imu->ax);
    frame[6]  = (uint8_t)(((uint16_t)imu->ay) >> 8);
    frame[7]  = (uint8_t)((uint16_t)imu->ay);
    frame[8]  = (uint8_t)(((uint16_t)imu->az) >> 8);
    frame[9]  = (uint8_t)((uint16_t)imu->az);
    frame[10] = (uint8_t)(((uint16_t)imu->gx) >> 8);
    frame[11] = (uint8_t)((uint16_t)imu->gx);
    frame[12] = (uint8_t)(((uint16_t)imu->gy) >> 8);
    frame[13] = (uint8_t)((uint16_t)imu->gy);
    frame[14] = (uint8_t)(((uint16_t)imu->gz) >> 8);
    frame[15] = (uint8_t)((uint16_t)imu->gz);

    return BMI330_CAN_SendFrame(SENSOR_CAN_ID_FUSED, frame);
}
