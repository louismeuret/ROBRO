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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ----- Open-loop V/Hz sinusoidal drive (STGIPN3H60T IPM, Mitsubishi
 * HC-KFS13 PMSM servo) -----
 * TIM1 timer clock = 16 MHz, center-aligned: f_pwm = 16e6 / (2 * PWM_ARR).
 * PWM_ARR = 400 -> 20 kHz PWM. */
#define PWM_ARR                  400u

/* HC-KFS13 is a 100 W, 200 Vac-class PMSM (0.71 A / 3000 r/min rated, per
 * Mitsubishi's published specs). Its exact pole-pair count isn't in the
 * generally available manuals; 4 pole pairs (8 poles) is typical for this
 * small low-inertia Mitsubishi servo class and used below purely to relate
 * electrical frequency to an approximate mechanical speed in comments --
 * open-loop drive correctness doesn't depend on it. Verify against the
 * motor nameplate/manual if the exact RPM matters. */
#define MOTOR_POLE_PAIRS         4u

/* One electrical cycle, values in [-1, 1]. 96 so that a 120-degree phase
 * shift is an exact 32-sample index offset (96 / 3), avoiding any
 * interpolation/rounding error between the three phases. */
#define SINE_LUT_SIZE            96u

/* TIM1 runs center-aligned, so the update event fires on both the up- and
 * down-count each PWM period -> 2x the 20 kHz carrier. */
#define PWM_UPDATE_HZ            40000.0f
#define LUT_STEP_PER_HZ          ((float)SINE_LUT_SIZE / PWM_UPDATE_HZ)

/* Open-loop start-up ramp: electrical frequency ramps 0 -> ELEC_FREQ_TARGET_HZ
 * over RAMP_TIME_S seconds; modulation index (PWM duty amplitude) ramps
 * alongside it from a fixed low-speed boost (M_START, to overcome resistive
 * drop/static friction while electrical frequency is near 0) up to
 * M_TARGET. No encoder feedback and no current limiting: this only proves
 * the waveform is a genuine rotating field, it is not closed-loop servo
 * control. Keep M_TARGET conservative until current has been verified on
 * the phase shunts (IU/IV/IW) with a scope. */
#define ELEC_FREQ_TARGET_HZ      20.0f     /* -> ~300 r/min @ 4 pole pairs */
#define RAMP_TIME_S              5.0f
#define M_START                  0.10f
#define M_TARGET                 0.80f
#define RAMP_STEP_PER_TICK       (1.0f / (RAMP_TIME_S * PWM_UPDATE_HZ))

/* ----- RS485 daisy-chain ping/pong (USART2, shared TX/RX bus over both
 * RJ45 jacks) ----- */
#define DC_FRAME_LEN             8u
#define DC_SYNC0                 0xAAu
#define DC_SYNC1                 0x55u
#define DC_TYPE_PING             0x01u
#define DC_TYPE_PONG             0x02u
#define DC_PING_PERIOD_MS        1000u
#define DC_LED_BLINK_MS          80u
#define DC_RX_BUF_LEN            64u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Motor_PWM_Update(void);
static void DaisyChain_Init(void);
static void DaisyChain_Send(uint8_t type);
static void DaisyChain_ProcessRx(void);
static void DaisyChain_Poll(void);
static void DaisyChain_BlinkLed(void);
static uint32_t DaisyChain_Rand(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_PCD_Init();
  MX_SPI1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Sinusoidal open-loop drive through the STGIPN3H60T IPM.
   *   PA8/PB13  = HIN1/LIN1 -> phase U
   *   PA9/PB14  = HIN2/LIN2 -> phase V
   *   PA10/PB15 = HIN3/LIN3 -> phase W
   * TIM1 produces the complementary HIN/LIN pairs with dead-time; the IPM
   * inputs are active-high. Motor_PWM_Update() (called from the TIM1 update
   * interrupt) drives each phase with a 120-degree-shifted sine, ramping
   * frequency and amplitude together -- a genuine rotating field, unlike
   * the previous equal-duty smoke test which produced zero line-to-line
   * voltage and no torque. */

  /* Hold PA5 (NSD_ADC) high at all times to keep the current/voltage-sense
   * signal conditioning out of shutdown. It defaults to a plain input;
   * override it to a push-pull output and drive it high. Set ODR high first
   * to avoid a glitch when the pin switches to output. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  {
    GPIO_InitTypeDef pa5 = {0};
    pa5.Pin   = GPIO_PIN_5;
    pa5.Mode  = GPIO_MODE_OUTPUT_PP;
    pa5.Pull  = GPIO_NOPULL;
    pa5.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &pa5);
  }

  /* PB11 ("RS") is NOT a digital IPM enable pin: it feeds an RC filter
   * (R12/C16) into the IPM's CIN pin, the analog overcurrent-comparator
   * input that gates the U-channel driver specifically. Driving it high
   * holds CIN above its trip threshold, permanently faulting out U while
   * leaving V/W unaffected. Keep it low so CIN stays below threshold.
   * NOTE: this also means the IPM's own overcurrent comparator can never
   * trip (CIN is held below threshold at all times) -- there is no
   * hardware or software current protection on this drive. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim1);

  /* RS485 daisy-chain ping/pong discovery (USART2, shared bus over both
   * RJ45 jacks). */
  DaisyChain_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 15V-supply status housekeeping (preserved from the smoke test). */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
    {
      /* PA15 (Fault_15V) high -> no 15V */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    }

    /* U/V/W duty is updated from the TIM1 update interrupt (Motor_PWM_Update) —
     * nothing to do for the motor drive here. */
    DaisyChain_Poll();
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
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = PWM_ARR;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB11 PB5 PB6
                           PB7 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ===== Open-loop sinusoidal V/Hz motor drive ============================ */

/* One electrical cycle, generated offline: sinf(2*pi*i/SINE_LUT_SIZE). */
static const float sine_lut[SINE_LUT_SIZE] =
{
  0.000000f, 0.065403f, 0.130526f, 0.195090f, 0.258819f, 0.321439f, 0.382683f, 0.442289f,
  0.500000f, 0.555570f, 0.608761f, 0.659346f, 0.707107f, 0.751840f, 0.793353f, 0.831470f,
  0.866025f, 0.896873f, 0.923880f, 0.946930f, 0.965926f, 0.980785f, 0.991445f, 0.997859f,
  1.000000f, 0.997859f, 0.991445f, 0.980785f, 0.965926f, 0.946930f, 0.923880f, 0.896873f,
  0.866025f, 0.831470f, 0.793353f, 0.751840f, 0.707107f, 0.659346f, 0.608761f, 0.555570f,
  0.500000f, 0.442289f, 0.382683f, 0.321439f, 0.258819f, 0.195090f, 0.130526f, 0.065403f,
  0.000000f, -0.065403f, -0.130526f, -0.195090f, -0.258819f, -0.321439f, -0.382683f, -0.442289f,
  -0.500000f, -0.555570f, -0.608761f, -0.659346f, -0.707107f, -0.751840f, -0.793353f, -0.831470f,
  -0.866025f, -0.896873f, -0.923880f, -0.946930f, -0.965926f, -0.980785f, -0.991445f, -0.997859f,
  -1.000000f, -0.997859f, -0.991445f, -0.980785f, -0.965926f, -0.946930f, -0.923880f, -0.896873f,
  -0.866025f, -0.831470f, -0.793353f, -0.751840f, -0.707107f, -0.659346f, -0.608761f, -0.555570f,
  -0.500000f, -0.442289f, -0.382683f, -0.321439f, -0.258819f, -0.195090f, -0.130526f, -0.065403f,
};

static float motor_phase_idx = 0.0f;   /* current position in sine_lut, wraps at SINE_LUT_SIZE */
static float motor_ramp_frac = 0.0f;   /* 0..1 progress through the start-up ramp */

static void Motor_PWM_Update(void)
{
  if (motor_ramp_frac < 1.0f)
  {
    motor_ramp_frac += RAMP_STEP_PER_TICK;
    if (motor_ramp_frac > 1.0f)
    {
      motor_ramp_frac = 1.0f;
    }
  }

  float elec_freq_now = motor_ramp_frac * ELEC_FREQ_TARGET_HZ;
  float m_now = M_START + motor_ramp_frac * (M_TARGET - M_START);

  motor_phase_idx += elec_freq_now * LUT_STEP_PER_HZ;
  if (motor_phase_idx >= (float)SINE_LUT_SIZE)
  {
    motor_phase_idx -= (float)SINE_LUT_SIZE;
  }

  uint32_t idx_u = (uint32_t)motor_phase_idx % SINE_LUT_SIZE;
  uint32_t idx_v = (idx_u + SINE_LUT_SIZE - SINE_LUT_SIZE / 3u) % SINE_LUT_SIZE;
  uint32_t idx_w = (idx_u + SINE_LUT_SIZE / 3u) % SINE_LUT_SIZE;

  uint32_t ccu = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_u]));
  uint32_t ccv = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_v]));
  uint32_t ccw = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_w]));

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccu);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccv);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccw);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    Motor_PWM_Update();
  }
}

/* ===== RS485 daisy-chain ping/pong ======================================= */

static uint32_t board_id;

static uint8_t dc_rx_byte;                /* landing pad for HAL_UART_Receive_IT */
static uint8_t dc_rx_ring[DC_RX_BUF_LEN];
static volatile uint16_t dc_rx_head = 0;  /* written by the RX-complete ISR */
static uint16_t dc_rx_tail = 0;           /* read by the main loop */

static uint32_t dc_rand_state;
static uint32_t dc_next_ping_tick = 0;
static uint32_t dc_led_off_tick = 0;
static uint32_t dc_pong_due_tick = 0;
static uint8_t dc_pong_pending = 0;

static uint32_t DaisyChain_Rand(void)
{
  /* Numerical Recipes LCG: enough randomness to stagger bus access on a
   * shared half-duplex link, not intended to be cryptographically sound. */
  dc_rand_state = dc_rand_state * 1664525u + 1013904223u;
  return dc_rand_state;
}

static void DaisyChain_BlinkLed(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* LED2 */
  dc_led_off_tick = HAL_GetTick() + DC_LED_BLINK_MS;
}

static void DaisyChain_Send(uint8_t type)
{
  uint8_t frame[DC_FRAME_LEN];
  frame[0] = DC_SYNC0;
  frame[1] = DC_SYNC1;
  frame[2] = type;
  frame[3] = (uint8_t)(board_id);
  frame[4] = (uint8_t)(board_id >> 8);
  frame[5] = (uint8_t)(board_id >> 16);
  frame[6] = (uint8_t)(board_id >> 24);
  frame[7] = (uint8_t)(frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]);
  HAL_UART_Transmit(&huart2, frame, DC_FRAME_LEN, 20);
}

static void DaisyChain_ProcessRx(void)
{
  for (;;)
  {
    uint16_t head = dc_rx_head;
    uint16_t available = (uint16_t)((head + DC_RX_BUF_LEN - dc_rx_tail) % DC_RX_BUF_LEN);
    if (available < DC_FRAME_LEN)
    {
      break;
    }

    uint8_t b0 = dc_rx_ring[dc_rx_tail];
    uint8_t b1 = dc_rx_ring[(dc_rx_tail + 1u) % DC_RX_BUF_LEN];
    if (b0 != DC_SYNC0 || b1 != DC_SYNC1)
    {
      dc_rx_tail = (uint16_t)((dc_rx_tail + 1u) % DC_RX_BUF_LEN); /* resync: drop one byte */
      continue;
    }

    uint8_t frame[DC_FRAME_LEN];
    for (uint32_t i = 0; i < DC_FRAME_LEN; i++)
    {
      frame[i] = dc_rx_ring[(dc_rx_tail + i) % DC_RX_BUF_LEN];
    }

    uint8_t checksum = (uint8_t)(frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6]);
    if (checksum != frame[7])
    {
      dc_rx_tail = (uint16_t)((dc_rx_tail + 1u) % DC_RX_BUF_LEN); /* resync: bad checksum */
      continue;
    }

    dc_rx_tail = (uint16_t)((dc_rx_tail + DC_FRAME_LEN) % DC_RX_BUF_LEN);

    uint32_t sender_id = (uint32_t)frame[3] | ((uint32_t)frame[4] << 8) |
                          ((uint32_t)frame[5] << 16) | ((uint32_t)frame[6] << 24);
    if (sender_id == board_id)
    {
      continue; /* ignore any echo of our own transmission */
    }

    DaisyChain_BlinkLed();

    if (frame[2] == DC_TYPE_PING)
    {
      /* Randomized backoff before replying: the TX pair is a shared bus
       * with no collision detection, so every board replying to a
       * broadcast ping at the same instant would collide. */
      dc_pong_due_tick = HAL_GetTick() + 1u + (DaisyChain_Rand() % 15u);
      dc_pong_pending = 1;
    }
  }
}

static void DaisyChain_Init(void)
{
  board_id = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  dc_rand_state = board_id ^ HAL_GetTick() ^ 0xA5A5A5A5u;
  if (dc_rand_state == 0u)
  {
    dc_rand_state = 0xDEADBEEFu; /* LCG must not start at 0 */
  }

  /* Stagger the first ping so boards that power up together don't all
   * transmit in the same instant. */
  dc_next_ping_tick = HAL_GetTick() + (DaisyChain_Rand() % DC_PING_PERIOD_MS);

  HAL_UART_Receive_IT(&huart2, &dc_rx_byte, 1);
}

static void DaisyChain_Poll(void)
{
  DaisyChain_ProcessRx();

  if (dc_pong_pending && (int32_t)(HAL_GetTick() - dc_pong_due_tick) >= 0)
  {
    DaisyChain_Send(DC_TYPE_PONG);
    dc_pong_pending = 0;
  }

  if ((int32_t)(HAL_GetTick() - dc_next_ping_tick) >= 0)
  {
    DaisyChain_Send(DC_TYPE_PING);
    dc_next_ping_tick = HAL_GetTick() + DC_PING_PERIOD_MS;
  }

  if (dc_led_off_tick != 0u && (int32_t)(HAL_GetTick() - dc_led_off_tick) >= 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    dc_led_off_tick = 0u;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    dc_rx_ring[dc_rx_head] = dc_rx_byte;
    dc_rx_head = (uint16_t)((dc_rx_head + 1u) % DC_RX_BUF_LEN);
    HAL_UART_Receive_IT(&huart2, &dc_rx_byte, 1);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
