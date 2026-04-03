/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_bmi330.h"
#include "app_bmp384.h"
#include "app_neo6m.h"
#include "app_can_IMU_BARO.h"
#include "app_can_GPS16.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} app_led_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMI330_TX_PERIOD_MS            10U
#define BMP384_TX_PERIOD_MS            40U
#define NEO6M_TX_PERIOD_MS             200U

#define BMP384_STALE_TIMEOUT_MS        150U

#define HEARTBEAT_PERIOD_MS            1000U
#define HEARTBEAT_PULSE_1_START_MS     0U
#define HEARTBEAT_PULSE_1_END_MS       70U
#define HEARTBEAT_PULSE_2_START_MS     140U
#define HEARTBEAT_PULSE_2_END_MS       210U

#define ERROR_BLINK_ON_MS              140U
#define ERROR_BLINK_OFF_MS             140U
#define ERROR_BLINK_GROUP_PAUSE_MS     900U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t bmi330_last_tick = 0U;
static uint32_t bmp384_last_tick = 0U;
static uint32_t neo6m_last_tick  = 0U;
static uint32_t bmp384_last_valid_tick = 0U;

static bmp384_sample_t latest_bmp384_sample = {0};
static uint8_t latest_bmp384_valid = 0U;
static volatile app_error_code_t g_app_error_code = APP_ERROR_NONE;

static const app_led_t g_led1 = { STATUS_LED_1_GPIO_Port, STATUS_LED_1_Pin };
static const app_led_t g_led2 = { STATUS_LED_2_GPIO_Port, STATUS_LED_2_Pin };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void APP_Init(void);
static void APP_Run(void);
static HAL_StatusTypeDef CAN_App_Init(void);
static void APP_Heartbeat_Update(uint32_t now);
static void APP_LED_On(const app_led_t *led);
static void APP_LED_Off(const app_led_t *led);
static void APP_LED_Set(const app_led_t *led, uint8_t on);
static void APP_LED_AllOff(void);
static void APP_ErrorDelayMs(uint32_t delay_ms);
static void APP_ErrorPatternLoop(app_error_code_t code);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void APP_LED_On(const app_led_t *led)
{
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

static void APP_LED_Off(const app_led_t *led)
{
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

static void APP_LED_Set(const app_led_t *led, uint8_t on)
{
  if (on != 0U)
  {
    APP_LED_On(led);
  }
  else
  {
    APP_LED_Off(led);
  }
}

static void APP_LED_AllOff(void)
{
  APP_LED_Off(&g_led1);
  APP_LED_Off(&g_led2);
}

void Error_Handler_SetCode(app_error_code_t code)
{
  if ((g_app_error_code == APP_ERROR_NONE) && (code != APP_ERROR_NONE))
  {
    g_app_error_code = code;
  }
}

static void APP_ErrorDelayMs(uint32_t delay_ms)
{
  while (delay_ms > 0U)
  {
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0U)
    {
    }
    delay_ms--;
  }
}

static void APP_ErrorPatternLoop(app_error_code_t code)
{
  uint32_t blink_count;
  uint32_t i;

  if (code == APP_ERROR_NONE)
  {
    code = APP_ERROR_ASSERT;
  }

  blink_count = (uint32_t)code;

  while (1)
  {
    for (i = 0U; i < blink_count; i++)
    {
      APP_LED_On(&g_led1);
      APP_LED_On(&g_led2);
      APP_ErrorDelayMs(ERROR_BLINK_ON_MS);

      APP_LED_AllOff();
      APP_ErrorDelayMs(ERROR_BLINK_OFF_MS);
    }

    APP_ErrorDelayMs(ERROR_BLINK_GROUP_PAUSE_MS);
  }
}

static void APP_Heartbeat_Update(uint32_t now)
{
  uint32_t phase = now % HEARTBEAT_PERIOD_MS;
  uint8_t led1_on = 0U;
  uint8_t led2_on = 0U;

  if ((phase >= HEARTBEAT_PULSE_1_START_MS) && (phase < HEARTBEAT_PULSE_1_END_MS))
  {
    led1_on = 1U;
  }

  if ((phase >= HEARTBEAT_PULSE_2_START_MS) && (phase < HEARTBEAT_PULSE_2_END_MS))
  {
    led2_on = 1U;
  }

  APP_LED_Set(&g_led1, led1_on);
  APP_LED_Set(&g_led2, led2_on);
}

static HAL_StatusTypeDef CAN_App_Init(void)
{
  HAL_GPIO_WritePin(CAN_STB_GPIO_Port, CAN_STB_Pin, GPIO_PIN_RESET);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void APP_Init(void)
{
  if (CAN_App_Init() != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_CAN_START);
  }

  if (BMI330_App_Init() != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_BMI330_INIT);
  }

  if (BMP384_App_Init() != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_BMP384_INIT);
  }

  if (NEO6M_App_Init() != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_NEO6M_INIT);
  }

  bmi330_last_tick = HAL_GetTick();
  bmp384_last_tick = HAL_GetTick();
  neo6m_last_tick  = HAL_GetTick();
  bmp384_last_valid_tick = 0U;

  latest_bmp384_sample.pressure_pa = 0.0f;
  latest_bmp384_sample.temperature_c = 0.0f;
  latest_bmp384_sample.valid = false;
  latest_bmp384_valid = 0U;

  APP_LED_AllOff();
}

static void APP_Run(void)
{
  uint32_t now = HAL_GetTick();

  APP_Heartbeat_Update(now);

  NEO6M_App_Process();

  if ((uint32_t)(now - bmp384_last_tick) >= BMP384_TX_PERIOD_MS)
  {
    bmp384_sample_t bmp_sample;

    bmp384_last_tick = now;

    if (BMP384_App_HasNewData() != 0U)
    {
      if (BMP384_App_ReadSample(&bmp_sample) == HAL_OK)
      {
        BMP384_App_ClearNewData();

        if (bmp_sample.valid != false)
        {
          latest_bmp384_sample = bmp_sample;
          latest_bmp384_valid = 1U;
          bmp384_last_valid_tick = now;
        }
      }
    }
  }

  if ((latest_bmp384_valid != 0U) &&
      ((uint32_t)(now - bmp384_last_valid_tick) > BMP384_STALE_TIMEOUT_MS))
  {
    latest_bmp384_valid = 0U;
    latest_bmp384_sample.valid = false;
  }

  if ((uint32_t)(now - bmi330_last_tick) >= BMI330_TX_PERIOD_MS)
  {
    bmi330_raw_t imu_sample;

    bmi330_last_tick = now;

    if ((latest_bmp384_valid != 0U) && (BMI330_App_ReadRaw(&imu_sample) == HAL_OK))
    {
      (void)CAN_SENSOR_App_SendCombined(&imu_sample, &latest_bmp384_sample);
    }
  }

  if ((uint32_t)(now - neo6m_last_tick) >= NEO6M_TX_PERIOD_MS)
  {
    neo6m_last_tick = now;

    if (NEO6M_App_HasNewData() != 0U)
    {
      if (CAN_NEO6M_App_Send() == HAL_OK)
      {
        NEO6M_App_ClearNewData();
      }
    }
  }

  HAL_Delay(1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  APP_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    APP_Run();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_CLOCK_CONFIG);
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_HandlerEx(APP_ERROR_CLOCK_CONFIG);
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  Error_HandlerEx(g_app_error_code);
  /* USER CODE END Error_Handler_Debug */
}

void Error_HandlerEx(app_error_code_t code)
{
  /* USER CODE BEGIN Error_HandlerEx_Debug */
  Error_Handler_SetCode(code);
  __disable_irq();
  APP_LED_AllOff();
  APP_ErrorPatternLoop(g_app_error_code);
  /* USER CODE END Error_HandlerEx_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  Error_HandlerEx(APP_ERROR_ASSERT);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
