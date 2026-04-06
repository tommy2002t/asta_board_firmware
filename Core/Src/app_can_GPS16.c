#include "app_can_GPS16.h"

#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;

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

    if ((gnss == NULL) || (gnss->valid == 0U))
    {
        return HAL_ERROR;
    }

    /* Packet 1:
     * latitude_e7  (int32_t)
     * longitude_e7 (int32_t)
     */
    data_pos[0] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 24);
    data_pos[1] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 16);
    data_pos[2] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 8);
    data_pos[3] = (uint8_t)((uint32_t)gnss->latitude_e7);

    data_pos[4] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 24);
    data_pos[5] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 16);
    data_pos[6] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 8);
    data_pos[7] = (uint8_t)((uint32_t)gnss->longitude_e7);

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
