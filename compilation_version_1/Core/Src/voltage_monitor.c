/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : voltage_monitor.c
  * @brief          : Stripped-down build: measure the USB-C port voltage on
  *                   ADC1_IN1 (PA0) and stream it over USART1 (PB6=TX, PB7=RX,
  *                   115200 8N1) for the YP-05 USB-serial bridge.
  *
  *                   No motor / STGIPN IPM control, no SPI, no USB device, no
  *                   encoder communication — only ADC -> UART.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ADC reference voltage in millivolts (STM32 VDDA / VREF+). */
#define VREF_MV            3300u

/* 12-bit ADC -> full scale count. */
#define ADC_FULL_SCALE     4095u

/* --------------------------------------------------------------------------
 * Input voltage divider in front of PA0 (ADC1_IN1).
 *
 *      USB-C VBUS ---[ R_TOP ]---+--- PA0 (ADC pin, 0..3.3 V max)
 *                                |
 *                             [ R_BOTTOM ]
 *                                |
 *                               GND
 *
 *   Vusb = Vpin * (R_TOP + R_BOTTOM) / R_BOTTOM
 *
 * SET THESE to your actual resistor values (any consistent unit, e.g. kOhm).
 * Defaults below mean "no divider" (R_TOP = 0) -> Vusb is reported equal to
 * the raw pin voltage. With a 5 V or 20 V USB-C rail you MUST add a divider
 * and enter its resistor values here, otherwise the reported volts are wrong.
 * -------------------------------------------------------------------------- */
#define VDIV_R_TOP         0u    /* high-side resistor */
#define VDIV_R_BOTTOM      1u    /* low-side resistor (must be > 0)            */

/* How often to send a reading, in milliseconds. */
#define SAMPLE_PERIOD_MS   200u

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
static uint32_t ADC_ReadOnce(void);
/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize only the peripherals this build needs. */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  /* Calibrate the ADC once for an accurate reading. */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  {
    const char *banner = "USB-C voltage monitor ready\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)banner, strlen(banner), HAL_MAX_DELAY);
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    uint32_t adc  = ADC_ReadOnce();

    /* Voltage at the ADC pin, in millivolts. */
    uint32_t vpin_mv = (adc * VREF_MV) / ADC_FULL_SCALE;

    /* Scale back up through the input divider to the real USB-C voltage. */
    uint32_t vusb_mv = vpin_mv * (VDIV_R_TOP + VDIV_R_BOTTOM) / VDIV_R_BOTTOM;

    char line[80];
    int n = snprintf(line, sizeof(line),
                     "VUSB=%lu.%03lu V  (adc=%lu, vpin=%lu mV)\r\n",
                     (unsigned long)(vusb_mv / 1000u),
                     (unsigned long)(vusb_mv % 1000u),
                     (unsigned long)adc,
                     (unsigned long)vpin_mv);
    HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, HAL_MAX_DELAY);

    HAL_Delay(SAMPLE_PERIOD_MS);
    /* USER CODE END 3 */
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Trigger a single ADC conversion and return the raw count.
  * @retval 12-bit ADC value (0..4095)
  */
static uint32_t ADC_ReadOnce(void)
{
  uint32_t value = 0;

  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK)
  {
    value = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  return value;
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function (single channel, ADC1_IN1 = PA0)
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;            /* PA0 -> USB-C voltage sense */
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function (PB6=TX, PB7=RX, 115200 8N1)
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  * @note   PA0 (ADC) and PB6/PB7 (USART1) pins are configured by their
  *         respective HAL MspInit callbacks; here we only enable the clocks.
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
