#ifndef APP_NEO6M_H
#define APP_NEO6M_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    int32_t latitude_e7;        /* deg * 1e7 */
    int32_t longitude_e7;       /* deg * 1e7 */
    int16_t altitude_dm;        /* decimeters */
    uint16_t speed_centi_kmh;   /* km/h * 100 */
    uint16_t course_cdeg;       /* deg * 100 */
    uint16_t hdop_x10;          /* hdop * 10 */

    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    uint8_t fix_valid;
    uint8_t fix_quality;
    uint8_t satellites;
    uint8_t valid;
} neo6m_data_t;

HAL_StatusTypeDef NEO6M_App_Init(void);
HAL_StatusTypeDef NEO6M_App_ReadLatest(neo6m_data_t *data);

uint8_t NEO6M_App_HasFix(void);
uint8_t NEO6M_App_HasNewData(void);
void NEO6M_App_ClearNewData(void);
void NEO6M_App_Process(void);

#endif
