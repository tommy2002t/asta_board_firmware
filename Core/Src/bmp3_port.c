#include "bmp3_port.h"

static void bmp3_cs_low(bmp3_hal_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void bmp3_cs_high(bmp3_hal_dev_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

void BMP3_Port_InitDWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

int8_t bmp3_hal_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bmp3_hal_dev_t *dev = (bmp3_hal_dev_t *)intf_ptr;
    uint8_t tx_addr = reg_addr | 0x80U;
    uint8_t dummy_tx = 0U;

    bmp3_cs_low(dev);

    if (HAL_SPI_Transmit(dev->hspi, &tx_addr, 1U, BMP3_SPI_TIMEOUT_MS) != HAL_OK)
    {
        goto error;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        if (HAL_SPI_TransmitReceive(dev->hspi, &dummy_tx, &reg_data[i], 1U, BMP3_SPI_TIMEOUT_MS) != HAL_OK)
        {
            goto error;
        }
    }

    bmp3_cs_high(dev);
    return 0;

error:
    bmp3_cs_high(dev);
    return -1;
}

int8_t bmp3_hal_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bmp3_hal_dev_t *dev = (bmp3_hal_dev_t *)intf_ptr;
    reg_addr &= 0x7FU;

    bmp3_cs_low(dev);

    if (HAL_SPI_Transmit(dev->hspi, &reg_addr, 1U, BMP3_SPI_TIMEOUT_MS) != HAL_OK)
    {
        goto error;
    }

    if (HAL_SPI_Transmit(dev->hspi, (uint8_t *)reg_data, len, BMP3_SPI_TIMEOUT_MS) != HAL_OK)
    {
        goto error;
    }

    bmp3_cs_high(dev);
    return 0;

error:
    bmp3_cs_high(dev);
    return -1;
}

void bmp3_hal_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (HAL_RCC_GetHCLKFreq() / 1000000U);

    while ((DWT->CYCCNT - start) < ticks)
    {
        /* wait */
    }
}
