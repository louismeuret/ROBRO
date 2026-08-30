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
#define DC_MAX_PEERS             4u     /* boards remembered in the peer table */

/* ----- ESP32 telemetry / command link (SPI1, STM32 is master) -----
 * All four signals land on the board's existing header, so nothing has to be
 * soldered to the LED pads:
 *
 *   PB3 (GPIO4, SPI_SCK)  -> ESP32 SCLK
 *   PA7 (GPIO6, SPI_MOSI) -> ESP32 MOSI   (telemetry out)
 *   PB4 (GPIO5, SPI_MISO) <- ESP32 MISO   (commands in)
 *   PB9 (free header pin) -> ESP32 CS     (software NSS, active low)
 *   GND                   -- GND
 *
 * SPI was chosen because it is the only full-duplex link available on the
 * reachable pins: PB3/PB4/PA14 all map to USART2, which is already driving the
 * RS485 daisy chain, and PB9's UART partner (USART3_RX) is PB8-BOOT0.
 *
 * SPI1 was already configured by CubeMX as a full-duplex master with soft NSS
 * and never used. Two changes are needed: 8-bit words instead of 4, and a
 * slower clock -- 8 MHz is more than an ESP32 SPI slave will follow reliably.
 *
 * The exchange is symmetric and fixed length: every transfer clocks a
 * telemetry frame out on MOSI while a command frame arrives on MISO. */
#define ESP_FRAME_LEN            72u
#define ESP_PERIOD_MS            50u    /* 20 exchanges per second */

#define ESP_TX_SYNC0             0xA5u  /* STM32 -> ESP32 (telemetry) */
#define ESP_TX_SYNC1             0x5Au
#define ESP_RX_SYNC0             0xC3u  /* ESP32 -> STM32 (command)   */
#define ESP_RX_SYNC1             0x3Cu

#define ESP_CMD_NOP              0x00u
#define ESP_CMD_SET_FREQ         0x01u  /* arg = centi-Hz      */
#define ESP_CMD_SET_MOD          0x02u  /* arg = milli-units   */
#define ESP_CMD_START            0x03u
#define ESP_CMD_STOP             0x04u
#define ESP_CMD_PING             0x05u
#define ESP_CMD_SET_MOD_START    0x06u  /* arg = milli-units   */
#define ESP_CMD_SET_RAMP_MS      0x07u  /* arg = milliseconds  */
#define ESP_CMD_SET_DIR          0x08u  /* arg = 0 fwd, 1 rev  */
#define ESP_CMD_SET_LED          0x09u  /* arg lo = led index, hi = mode */

/* LED override modes for ESP_CMD_SET_LED. */
#define ESP_LED_AUTO             0u
#define ESP_LED_ON               1u
#define ESP_LED_OFF              2u
#define ESP_LED_COUNT            2u     /* 0 = LED1 (PB6), 1 = LED2 (PB7) */

#define ESP_CS_PORT              GPIOB
#define ESP_CS_PIN               GPIO_PIN_9

/* The ESP32 slave needs CS stable a little before the first clock edge and
 * after the last. ~1 us at 16 MHz; HAL_Delay's 1 ms granularity is far too
 * coarse for this. */
#define ESP_CS_SETTLE()          do { for (volatile uint32_t i_ = 0; i_ < 16u; i_++) { __NOP(); } } while (0)

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
static volatile uint8_t fault_15v = 0;       /* PA15: 1 = no 15V rail */
static volatile uint32_t dc_peer_frames = 0; /* valid frames seen from other boards */
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
static uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
static void Esp_Init(void);
static void Esp_Poll(void);
static void Esp_Exchange(void);
static void Esp_ApplyCommand(const uint8_t *frame);
static void Motor_PWM_Update(void);
static void DaisyChain_Init(void);
static void DaisyChain_Send(uint8_t type);
static void DaisyChain_ProcessRx(void);
static void DaisyChain_Poll(void);
static void DaisyChain_BlinkLed(void);
static void DaisyChain_NotePeer(uint32_t id);
static uint32_t DaisyChain_Rand(void);
static void Leds_Apply(void);
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

  /* ESP32 telemetry/command link on SPI1 (PB3/PB4/PA7 + PB9 as CS). Also
   * calibrates ADC1 and ADC2 -- they were initialised by CubeMX but never
   * calibrated or started. */
  Esp_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 15V-supply status housekeeping (preserved from the smoke test).
     * PA15 (Fault_15V) high -> no 15V. The state is also published in the
     * telemetry frame so the ESP32 dashboard can show it. */
    fault_15v = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) ? 1u : 0u;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, fault_15v ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Leds_Apply();

    /* U/V/W duty is updated from the TIM1 update interrupt (Motor_PWM_Update) —
     * nothing to do for the motor drive here. */
    DaisyChain_Poll();
    Esp_Poll();
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;  /* 16 MHz / 16 = 1 MHz */
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
                           PB7 PB9
    PB9 doubles as the software-NSS chip select for the ESP32 SPI link; it is
    driven high (idle) in Esp_Init(). */
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

/* Setpoints the ESP32 can write at runtime. Read by Motor_PWM_Update() in the
 * TIM1 ISR, written by the main loop: single 32-bit words, so each store is
 * atomic on Cortex-M4 and no critical section is needed. */
static volatile float motor_freq_target = ELEC_FREQ_TARGET_HZ;
static volatile float motor_mod_target  = M_TARGET;
static volatile float motor_mod_start   = M_START;
static volatile float motor_ramp_step   = RAMP_STEP_PER_TICK;
static volatile uint16_t motor_ramp_ms  = (uint16_t)(RAMP_TIME_S * 1000.0f);
static volatile uint8_t motor_reverse   = 0;  /* 1 = swap V/W -> reverse rotation */

/* LED override state. Index 0 = LED1 (PB6), 1 = LED2 (PB7). While a LED is in
 * ESP_LED_AUTO the firmware drives it as before; otherwise the dashboard owns
 * it and the automatic logic leaves it alone. */
static volatile uint8_t led_mode[ESP_LED_COUNT] = { ESP_LED_AUTO, ESP_LED_AUTO };


static void Motor_PWM_Update(void)
{
  if (motor_ramp_frac < 1.0f)
  {
    motor_ramp_frac += motor_ramp_step;
    if (motor_ramp_frac > 1.0f)
    {
      motor_ramp_frac = 1.0f;
    }
  }

  float m_start_now = motor_mod_start;
  float elec_freq_now = motor_ramp_frac * motor_freq_target;
  float m_now = m_start_now + motor_ramp_frac * (motor_mod_target - m_start_now);

  motor_phase_idx += elec_freq_now * LUT_STEP_PER_HZ;
  if (motor_phase_idx >= (float)SINE_LUT_SIZE)
  {
    motor_phase_idx -= (float)SINE_LUT_SIZE;
  }

  uint32_t idx_u = (uint32_t)motor_phase_idx % SINE_LUT_SIZE;
  uint32_t idx_lag  = (idx_u + SINE_LUT_SIZE - SINE_LUT_SIZE / 3u) % SINE_LUT_SIZE;
  uint32_t idx_lead = (idx_u + SINE_LUT_SIZE / 3u) % SINE_LUT_SIZE;
  /* Swapping which of V/W leads reverses the rotating field. */
  uint32_t idx_v = motor_reverse ? idx_lead : idx_lag;
  uint32_t idx_w = motor_reverse ? idx_lag  : idx_lead;

  uint32_t ccu = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_u]));
  uint32_t ccv = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_v]));
  uint32_t ccw = (uint32_t)((float)(PWM_ARR / 2u) * (1.0f + m_now * sine_lut[idx_w]));

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccu);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccv);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccw);
}

/* LED1 (PB6) normally mirrors "15V present"; LED2 (PB7) is blinked by the
 * daisy chain. Either can be taken over from the dashboard. */
static void Leds_Apply(void)
{
  if (led_mode[0] == ESP_LED_AUTO)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, fault_15v ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,
                      (led_mode[0] == ESP_LED_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  if (led_mode[1] != ESP_LED_AUTO)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
                      (led_mode[1] == ESP_LED_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
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

static uint32_t dc_peer_id[DC_MAX_PEERS];
static uint32_t dc_peer_seen[DC_MAX_PEERS];   /* HAL_GetTick() of last frame */
static uint8_t dc_peer_count = 0;

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
  if (led_mode[1] == ESP_LED_AUTO)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); /* LED2 */
    dc_led_off_tick = HAL_GetTick() + DC_LED_BLINK_MS;
  }
  dc_peer_frames++;   /* also reported in the telemetry frame */
}

/* Remember which boards are on the chain. The table is tiny and searched
 * linearly -- DC_MAX_PEERS is 4. When it is full the least recently heard
 * entry is evicted, so a board that drops off eventually makes room. */
static void DaisyChain_NotePeer(uint32_t id)
{
  uint32_t now = HAL_GetTick();

  for (uint8_t i = 0; i < dc_peer_count; i++)
  {
    if (dc_peer_id[i] == id)
    {
      dc_peer_seen[i] = now;
      return;
    }
  }

  if (dc_peer_count < DC_MAX_PEERS)
  {
    dc_peer_id[dc_peer_count] = id;
    dc_peer_seen[dc_peer_count] = now;
    dc_peer_count++;
    return;
  }

  uint8_t oldest = 0;
  for (uint8_t i = 1; i < DC_MAX_PEERS; i++)
  {
    if ((int32_t)(dc_peer_seen[i] - dc_peer_seen[oldest]) < 0)
    {
      oldest = i;
    }
  }
  dc_peer_id[oldest] = id;
  dc_peer_seen[oldest] = now;
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
    DaisyChain_NotePeer(sender_id);

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
    if (led_mode[1] == ESP_LED_AUTO)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    }
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

/* ===== ESP32 telemetry / command link (SPI1 master, PB3/PB4/PA7 + PB9 CS) ==
 *
 * One fixed-length full-duplex transfer per period: the telemetry frame is
 * clocked out on MOSI while the ESP32's command frame arrives on MISO. The
 * ESP32 runs as an SPI slave and stages its reply before the transfer starts,
 * so the command it returns is always the one it prepared last time round --
 * that one-frame lag is why commands carry a sequence number.
 *
 * Telemetry frame (STM32 -> ESP32), 72 bytes, little-endian:
 *    0     0xA5 sync
 *    1     0x5A sync
 *    2-3   VDC        4-5   VU        6-7   VV (always 0)
 *    8-9   VW        10-11  IU       12-13  IV       14-15  IW
 *    16    flags: bit0 = 15V fault, bit1 = outputs enabled,
 *                 bit2 = reverse, bit3 = PB10 input level
 *    17    number of daisy-chain peers in the table
 *    18-21 total RS485 frames seen from other boards
 *    22-23 electrical frequency setpoint, centi-Hz
 *    24-25 modulation index target, milli-units
 *    26-29 this board's id
 *    30-31 modulation index at start of ramp, milli-units
 *    32-33 ramp time, milliseconds
 *    34-35 live ramp progress, milli (0..1000)
 *    36-37 PWM_ARR (carrier = 16 MHz / (2 * ARR))
 *    38    LED1 mode      39   LED2 mode   (0 auto, 1 on, 2 off)
 *    40-63 peer table: 4 entries of { u32 id, u16 age in 10 ms units }
 *    64-69 reserved
 *    70    XOR checksum of bytes 0..69
 *    71    padding
 *
 * Command frame (ESP32 -> STM32), 72 bytes:
 *    0     0xC3 sync
 *    1     0x3C sync
 *    2     command id
 *    3     sequence number -- acted on only when it changes
 *    4-5   argument
 *    6-70  reserved
 *    71    XOR checksum of bytes 0..70
 */

static uint8_t esp_tx_frame[ESP_FRAME_LEN];
static uint8_t esp_rx_frame[ESP_FRAME_LEN];
static uint32_t esp_next_tick = 0;
static uint8_t esp_last_seq = 0;
static uint8_t esp_seq_valid = 0;

/* Single-shot read of one regular channel. ScanConvMode is disabled and
 * NbrOfConversion is 1, so rank 1 is re-pointed at each channel in turn. */
static uint16_t ADC_ReadChannel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  uint16_t value = 0;

  sConfig.Channel      = channel;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  /* 247.5 cycles: the sense pins are fed by high-impedance resistive
   * dividers, which the 2.5-cycle default cannot settle. */
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    return 0;
  }
  if (HAL_ADC_Start(hadc) != HAL_OK)
  {
    return 0;
  }
  if (HAL_ADC_PollForConversion(hadc, 5) == HAL_OK)
  {
    value = (uint16_t)HAL_ADC_GetValue(hadc);
  }
  HAL_ADC_Stop(hadc);
  return value;
}

static void Esp_PutU16(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
}

static void Esp_PutU32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static uint8_t Esp_Checksum(const uint8_t *p, uint32_t n)
{
  uint8_t x = 0;
  for (uint32_t i = 0; i < n; i++)
  {
    x ^= p[i];
  }
  return x;
}

static void Esp_BuildTelemetry(void)
{
  /* Channel map comes from Repository.ioc:
   *   VDC PB12 ADC1_IN11 | VU PB1 ADC1_IN12 | VW PA6 ADC2_IN3
   *   IU  PA0  ADC1_IN1  | IV PA4 ADC2_IN17 | IW PB0 ADC1_IN15
   * VV has no ADC channel: PB2 is configured as a GPIO output in both .ioc
   * files, so the README's "VV = PB2 = ADC2_IN12" was never true for this
   * pinout. It is sent as 0. To get a real VV, reassign PB2 to ADC2_IN12 in
   * CubeMX and read ADC_CHANNEL_12 on hadc2 here. */
  uint16_t vdc = ADC_ReadChannel(&hadc1, ADC_CHANNEL_11);
  uint16_t vu  = ADC_ReadChannel(&hadc1, ADC_CHANNEL_12);
  uint16_t vw  = ADC_ReadChannel(&hadc2, ADC_CHANNEL_3);
  uint16_t iu  = ADC_ReadChannel(&hadc1, ADC_CHANNEL_1);
  uint16_t iv  = ADC_ReadChannel(&hadc2, ADC_CHANNEL_17);
  uint16_t iw  = ADC_ReadChannel(&hadc1, ADC_CHANNEL_15);

  uint8_t flags = 0;
  if (fault_15v)                                          { flags |= 0x01u; }
  if (htim1.Instance->BDTR & TIM_BDTR_MOE)                { flags |= 0x02u; }
  if (motor_reverse)                                      { flags |= 0x04u; }
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) { flags |= 0x08u; }

  for (uint32_t i = 0; i < ESP_FRAME_LEN; i++)
  {
    esp_tx_frame[i] = 0;
  }
  esp_tx_frame[0] = ESP_TX_SYNC0;
  esp_tx_frame[1] = ESP_TX_SYNC1;
  Esp_PutU16(&esp_tx_frame[2],  vdc);
  Esp_PutU16(&esp_tx_frame[4],  vu);
  Esp_PutU16(&esp_tx_frame[6],  0u);   /* VV: no ADC channel on this pinout */
  Esp_PutU16(&esp_tx_frame[8],  vw);
  Esp_PutU16(&esp_tx_frame[10], iu);
  Esp_PutU16(&esp_tx_frame[12], iv);
  Esp_PutU16(&esp_tx_frame[14], iw);
  esp_tx_frame[16] = flags;
  esp_tx_frame[17] = dc_peer_count;
  Esp_PutU32(&esp_tx_frame[18], dc_peer_frames);
  Esp_PutU16(&esp_tx_frame[22], (uint16_t)(motor_freq_target * 100.0f));
  Esp_PutU16(&esp_tx_frame[24], (uint16_t)(motor_mod_target * 1000.0f));
  Esp_PutU32(&esp_tx_frame[26], board_id);
  Esp_PutU16(&esp_tx_frame[30], (uint16_t)(motor_mod_start * 1000.0f));
  Esp_PutU16(&esp_tx_frame[32], motor_ramp_ms);
  Esp_PutU16(&esp_tx_frame[34], (uint16_t)(motor_ramp_frac * 1000.0f));
  Esp_PutU16(&esp_tx_frame[36], (uint16_t)PWM_ARR);
  esp_tx_frame[38] = led_mode[0];
  esp_tx_frame[39] = led_mode[1];

  /* Peer table. Age is in 10 ms units so a 16-bit field covers ~11 minutes;
   * anything older saturates. */
  uint32_t now = HAL_GetTick();
  for (uint8_t i = 0; i < DC_MAX_PEERS; i++)
  {
    uint8_t *slot = &esp_tx_frame[40 + (i * 6u)];
    if (i < dc_peer_count)
    {
      uint32_t age = (now - dc_peer_seen[i]) / 10u;
      if (age > 0xFFFFu) { age = 0xFFFFu; }
      Esp_PutU32(&slot[0], dc_peer_id[i]);
      Esp_PutU16(&slot[4], (uint16_t)age);
    }
    else
    {
      Esp_PutU32(&slot[0], 0u);
      Esp_PutU16(&slot[4], 0u);
    }
  }

  esp_tx_frame[70] = Esp_Checksum(esp_tx_frame, 70);
}

static void Esp_ApplyCommand(const uint8_t *frame)
{
  if (frame[0] != ESP_RX_SYNC0 || frame[1] != ESP_RX_SYNC1)
  {
    return;   /* no ESP32 attached, or it had nothing staged yet */
  }
  if (Esp_Checksum(frame, ESP_FRAME_LEN - 1u) != frame[ESP_FRAME_LEN - 1u])
  {
    return;
  }

  uint8_t seq = frame[3];
  if (esp_seq_valid && seq == esp_last_seq)
  {
    return;   /* same command still sitting in the ESP32's buffer */
  }
  esp_last_seq = seq;
  esp_seq_valid = 1;

  uint16_t arg = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

  switch (frame[2])
  {
    case ESP_CMD_SET_FREQ:
    {
      /* Open loop with no current limiting, so refuse to be driven somewhere
       * the drive cannot follow. */
      float v = (float)arg / 100.0f;
      if (v > 100.0f) { v = 100.0f; }
      motor_freq_target = v;
      break;
    }
    case ESP_CMD_SET_MOD:
    {
      float v = (float)arg / 1000.0f;
      if (v > 0.95f) { v = 0.95f; }
      motor_mod_target = v;
      break;
    }
    case ESP_CMD_START:
      motor_ramp_frac = 0.0f;   /* re-run the alignment/ramp from zero */
      motor_phase_idx = 0.0f;
      __HAL_TIM_MOE_ENABLE(&htim1);
      break;

    case ESP_CMD_STOP:
      /* Clearing MOE tri-states all six IPM inputs immediately. */
      __HAL_TIM_MOE_DISABLE(&htim1);
      break;

    case ESP_CMD_PING:
      DaisyChain_Send(DC_TYPE_PING);
      break;

    case ESP_CMD_SET_MOD_START:
    {
      float v = (float)arg / 1000.0f;
      if (v > 0.95f) { v = 0.95f; }
      motor_mod_start = v;
      break;
    }
    case ESP_CMD_SET_RAMP_MS:
    {
      /* Guard the divide: a zero ramp would make the step infinite. */
      uint16_t ms = arg;
      if (ms < 100u) { ms = 100u; }
      motor_ramp_ms = ms;
      motor_ramp_step = 1.0f / (((float)ms / 1000.0f) * PWM_UPDATE_HZ);
      break;
    }
    case ESP_CMD_SET_DIR:
      motor_reverse = (arg != 0u) ? 1u : 0u;
      break;

    case ESP_CMD_SET_LED:
    {
      uint8_t idx  = (uint8_t)(arg & 0xFFu);
      uint8_t mode = (uint8_t)(arg >> 8);
      if (idx < ESP_LED_COUNT && mode <= ESP_LED_OFF)
      {
        led_mode[idx] = mode;
        /* LED2 is only ever written by the blink path, so handing it back to
         * AUTO while it happens to be lit would strand it on until the next
         * peer frame. Clear it here instead. */
        if (idx == 1u && mode == ESP_LED_AUTO)
        {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
          dc_led_off_tick = 0u;
        }
      }
      break;
    }

    case ESP_CMD_NOP:
    default:
      break;
  }
}

static void Esp_Exchange(void)
{
  Esp_BuildTelemetry();

  /* Software NSS: frame the transfer with PB9. The ESP32's SPI slave uses the
   * CS edges to delimit the transaction, so it must fall before the first
   * clock and rise after the last. */
  HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_RESET);
  ESP_CS_SETTLE();   /* CS setup before the first clock edge */
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, esp_tx_frame,
                                                 esp_rx_frame, ESP_FRAME_LEN, 20);
  ESP_CS_SETTLE();   /* CS hold after the last clock edge */
  HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_SET);

  if (st == HAL_OK)
  {
    Esp_ApplyCommand(esp_rx_frame);
  }
  else
  {
    /* A timeout leaves the peripheral mid-word; reset it so the next transfer
     * starts on a word boundary. */
    HAL_SPI_Abort(&hspi1);
  }
}

static void Esp_Init(void)
{
  /* ADC1/ADC2 were initialised by CubeMX but never calibrated or started.
   * Calibration must run while the ADC is disabled, i.e. before any read. */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }

  /* CS idles high. MX_GPIO_Init() already made PB9 a push-pull output. */
  HAL_GPIO_WritePin(ESP_CS_PORT, ESP_CS_PIN, GPIO_PIN_SET);

  esp_next_tick = HAL_GetTick() + ESP_PERIOD_MS;
}

static void Esp_Poll(void)
{
  if ((int32_t)(HAL_GetTick() - esp_next_tick) >= 0)
  {
    Esp_Exchange();
    esp_next_tick = HAL_GetTick() + ESP_PERIOD_MS;
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
