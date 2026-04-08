#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/* BMP384 */
#define BMP384_CS_GPIO_Port          GPIOB
#define BMP384_CS_Pin                GPIO_PIN_10

#define BMP384_INT_GPIO_Port         GPIOB
#define BMP384_INT_Pin               GPIO_PIN_11

/* CAN */
#define CAN_STB_GPIO_Port            GPIOB
#define CAN_STB_Pin                  GPIO_PIN_7

#define BMP3_SPI_TIMEOUT_MS          100U

/*
 * BMP384 IIR "coefficient 2" from the external requirement is interpreted as
 * register setting 0x02, which corresponds to the Bosch driver macro
 * BMP3_IIR_FILTER_COEFF_3.
 */
#define BMP384_DEFAULT_PRESS_OS      BMP3_OVERSAMPLING_2X
#define BMP384_DEFAULT_TEMP_OS       BMP3_NO_OVERSAMPLING
#define BMP384_DEFAULT_IIR_FILTER    BMP3_IIR_FILTER_COEFF_3
#define BMP384_DEFAULT_ODR           BMP3_ODR_25_HZ

#endif
