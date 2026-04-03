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

HAL_StatusTypeDef CAN_NEO6M_App_SendData(const neo6m_data_t *gnss)
{
    uint8_t data_pos[8];
    uint8_t data_status[8];
    uint8_t year_offset;
    uint8_t hdop_sat;
    uint8_t fix_quality_sat;

    if ((gnss == NULL) || (gnss->valid == 0U))
    {
        return HAL_ERROR;
    }

    data_pos[0] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 24);
    data_pos[1] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 16);
    data_pos[2] = (uint8_t)(((uint32_t)gnss->latitude_e7) >> 8);
    data_pos[3] = (uint8_t)((uint32_t)gnss->latitude_e7);

    data_pos[4] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 24);
    data_pos[5] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 16);
    data_pos[6] = (uint8_t)(((uint32_t)gnss->longitude_e7) >> 8);
    data_pos[7] = (uint8_t)((uint32_t)gnss->longitude_e7);

    if (gnss->year >= 2000U)
    {
        year_offset = (uint8_t)(gnss->year - 2000U);
    }
    else
    {
        year_offset = 0U;
    }

    fix_quality_sat = (uint8_t)(((gnss->fix_quality & 0x07U) << 5) | (gnss->satellites & 0x1FU));
    hdop_sat = (gnss->hdop_x10 > 127U) ? 127U : (uint8_t)gnss->hdop_x10;
    hdop_sat |= (uint8_t)((gnss->fix_valid & 0x01U) << 7);

    /* Reduced GPS payload (16 bytes total):
     *   Frame 1: latitude, longitude
     *   Frame 2: time/date + packed fix metadata
     * Removed from CAN: altitude, speed, course
     */
    data_status[0] = year_offset;
    data_status[1] = gnss->month;
    data_status[2] = gnss->day;
    data_status[3] = gnss->hour;
    data_status[4] = gnss->minute;
    data_status[5] = gnss->second;
    data_status[6] = fix_quality_sat;
    data_status[7] = hdop_sat;

    if (NEO6M_CAN_SendFrame(NEO6M_CAN_ID_POSITION, data_pos) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (NEO6M_CAN_SendFrame(NEO6M_CAN_ID_STATUS, data_status) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
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
