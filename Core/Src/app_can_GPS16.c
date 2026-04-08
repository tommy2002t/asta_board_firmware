#include "app_can_GPS16.h"

#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

static void CAN_PackFloatLE(float value, uint8_t *dst)
{
    union
    {
        float f;
        uint32_t u32;
    } conv;

    conv.f = value;

    dst[0] = (uint8_t)(conv.u32);
    dst[1] = (uint8_t)(conv.u32 >> 8);
    dst[2] = (uint8_t)(conv.u32 >> 16);
    dst[3] = (uint8_t)(conv.u32 >> 24);
}

static HAL_StatusTypeDef NEO6M_CAN_SendFrame(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef txHeader;

    memset(&txHeader, 0, sizeof(txHeader));

    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
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

HAL_StatusTypeDef CAN_NEO6M_App_SendData(const neo6m_data_t *gnss)
{
    uint8_t data_pos[8];
    float latitude_deg;
    float longitude_deg;

    if ((gnss == NULL) || (gnss->valid == 0U))
    {
        return HAL_ERROR;
    }

    latitude_deg = ((float)gnss->latitude_e7) / 10000000.0f;
    longitude_deg = ((float)gnss->longitude_e7) / 10000000.0f;

    /* Packet 1 (little-endian):
     * latitude_deg  (float32)
     * longitude_deg (float32)
     */
    CAN_PackFloatLE(latitude_deg, &data_pos[0]);
    CAN_PackFloatLE(longitude_deg, &data_pos[4]);

    return NEO6M_CAN_SendFrame(NEO6M_CAN_ID_POSITION, data_pos);
}

HAL_StatusTypeDef CAN_NEO6M_App_Send(void)
{
    neo6m_data_t gnss;

    if (NEO6M_App_ReadLatest(&gnss) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return CAN_NEO6M_App_SendData(&gnss);
}
