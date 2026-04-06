#ifndef APP_CAN_GPS16_H
#define APP_CAN_GPS16_H

#include "stm32g4xx_hal.h"
#include "app_neo6m.h"

#define NEO6M_CAN_ID_POSITION   0x103U

HAL_StatusTypeDef CAN_NEO6M_App_Send(void);
HAL_StatusTypeDef CAN_NEO6M_App_SendData(const neo6m_data_t *gnss);

#endif
