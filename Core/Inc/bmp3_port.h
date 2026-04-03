#ifndef BMP3_PORT_H
#define BMP3_PORT_H

#include "main.h"
#include "bmp3.h"
#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} bmp3_hal_dev_t;

void BMP3_Port_InitDWT(void);

int8_t bmp3_hal_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t bmp3_hal_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmp3_hal_delay_us(uint32_t period, void *intf_ptr);

#ifdef __cplusplus
}
#endif

#endif
