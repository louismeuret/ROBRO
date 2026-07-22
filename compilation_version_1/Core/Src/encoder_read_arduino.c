/**
  ******************************************************************************
  * @file    encoder_read_arduino.c
  * @brief   Standalone encoder reader that also streams the position to a
  *          second microcontroller (Arduino / ESP32) over a plain UART.
  *
  * Same encoder polling as encoder_read.c, plus a one-way ASCII feed on
  * USART2 so a companion board can forward the position to a PC over USB.
  *
  * Build it INSTEAD of Core/Src/main.c (see HOWTO_BUILD.md / the Makefile's
  * APP variable). The motor main.c is left untouched.
  *
  * --- Encoder link (external MAX485, manual direction) ---------------------
  *   PB6  USART1_TX -> MAX485 DI
  *   PB7  USART1_RX <- MAX485 RO
  *   PB9  GPIO out  -> MAX485 DE + RE (HIGH = transmit, LOW = receive)
  *   MAX485 A / B   <-> encoder RS485 pair        @ 2.5 Mbit/s, 8N1
  *
  * --- Companion-board link (STM32 -> Arduino/ESP32) ------------------------
  *   PB3  USART2_TX -> Arduino/ESP32 RX           @ 115200, 8N1 (one way)
  *   GND  <-> GND   (common ground required)
  *
  *   After every valid encoder frame the STM32 sends one ASCII line:
  *       "<full_mdeg>,<single_mdeg>\r\n"
  *   where the numbers are millidegrees (degrees * 1000, integer):
  *       full_mdeg   = absolute shaft angle  (gear-ratio corrected)
  *       single_mdeg = single-turn angle, range [-180000, 180000)
  *   The companion board just echoes these bytes to its USB serial; on the PC
  *   you read e.g. "123456,-45000" = 123.456 deg absolute, -45.000 deg/turn.
  ******************************************************************************
  */

#include "main.h"
#include <string.h>
#include <stdio.h>

/* Encoder protocol --------------------------------------------------------- */
#define ENC_CMD          0x32u
#define ENC_RESP_LEN     9
#define ENC_GEAR_RATIO   256.0f

/* MAX485 direction line ---------------------------------------------------- */
#define DE_PORT          GPIOB
#define DE_PIN           GPIO_PIN_9

/* Companion-board UART ----------------------------------------------------- */
#define ARDUINO_BAUD     115200u   /* lower to 38400 if the companion board
                                      uses SoftwareSerial (e.g. Arduino Uno) */

UART_HandleTypeDef huart1;   /* encoder (RS485) */
UART_HandleTypeDef huart2;   /* companion board */

/* Referenced by ADC1_2_IRQHandler in stm32g4xx_it.c; the ADC is never used. */
ADC_HandleTypeDef hadc1;

/* Latest decoded sample - also inspectable over SWD. ---------------------- */
volatile float   enc_single_turn_deg = 0.0f;
volatile float   enc_turns           = 0.0f;
volatile float   enc_full_deg        = 0.0f;
volatile uint8_t enc_raw[ENC_RESP_LEN]  = {0};
volatile uint8_t enc_valid           = 0;

static void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void GPIO_Init(void);

/**
  * @brief  One request/response transaction with the encoder.
  * @retval 1 if a valid 0x32 frame was decoded, 0 on timeout / bad echo.
  */
static int Encoder_Read(void)
{
  uint8_t cmd = ENC_CMD;
  uint8_t resp[ENC_RESP_LEN];

  HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_SET);
  HAL_UART_Transmit(&huart1, &cmd, 1, 5);
  HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_RESET);

  if (HAL_UART_Receive(&huart1, resp, ENC_RESP_LEN, 5) != HAL_OK ||
      resp[0] != ENC_CMD)
  {
    enc_valid = 0;
    return 0;
  }

  /* Decode - ported from read_serial.py decode_position(). */
  uint32_t ipos = (uint32_t)(resp[2] & 0x80)
                + ((uint32_t)resp[3] << 8)
                + ((uint32_t)resp[4] << 16);
  float single = ((float)ipos * 360.0f / 16777216.0f) - 180.0f;

  int32_t turns_raw = (int32_t)(resp[5] | (resp[6] << 8));
  if (turns_raw >= 32768) turns_raw -= 65536;          /* signed 16-bit */

  memcpy((void *)enc_raw, resp, ENC_RESP_LEN);
  enc_single_turn_deg = single;
  enc_turns           = (float)turns_raw / ENC_GEAR_RATIO;
  enc_full_deg        = (single + (float)turns_raw * 360.0f) / ENC_GEAR_RATIO;
  enc_valid           = 1;
  return 1;
}

/**
  * @brief  Send the latest position to the companion board (one ASCII line).
  *         Integers only, so no float-printf support is needed.
  */
static void Position_Send(void)
{
  char    line[32];
  int32_t full_mdeg   = (int32_t)(enc_full_deg        * 1000.0f);
  int32_t single_mdeg = (int32_t)(enc_single_turn_deg * 1000.0f);

  int n = snprintf(line, sizeof line, "%ld,%ld\r\n",
                   (long)full_mdeg, (long)single_mdeg);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 10);
}

/**
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  while (1)
  {
    if (Encoder_Read())
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* heartbeat on valid frame */
      Position_Send();
    }
    HAL_Delay(10);                              /* ~100 Hz */
  }
}

/**
  * @brief  System clock: 80 MHz from the HSI via PLL (exact 2.5 MBaud divisor).
  */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState        = RCC_PLL_ON;
  osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;   /* 16 MHz */
  osc.PLL.PLLM            = RCC_PLLM_DIV1;
  osc.PLL.PLLN            = 10;                   /* VCO = 160 MHz */
  osc.PLL.PLLP            = RCC_PLLP_DIV2;
  osc.PLL.PLLQ            = RCC_PLLQ_DIV2;
  osc.PLL.PLLR            = RCC_PLLR_DIV2;        /* SYSCLK = 80 MHz */
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;          /* HCLK  = 80 MHz */
  clk.APB1CLKDivider = RCC_HCLK_DIV1;            /* PCLK1 = 80 MHz -> USART2 */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;            /* PCLK2 = 80 MHz -> USART1 */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  USART1: 2.5 Mbit/s, 8N1 (PB6/PB7 set up by HAL_UART_MspInit).
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance                    = USART1;
  huart1.Init.BaudRate               = 2500000;
  huart1.Init.WordLength             = UART_WORDLENGTH_8B;
  huart1.Init.StopBits               = UART_STOPBITS_1;
  huart1.Init.Parity                 = UART_PARITY_NONE;
  huart1.Init.Mode                   = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
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
  * @brief  USART2: TX-only feed to the companion board on PB3.
  *         The project's HAL_UART_MspInit only handles USART1, so the clock
  *         and the PB3 alternate-function pin are configured here directly.
  */
static void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin       = GPIO_PIN_3;            /* PB3 -> USART2_TX (AF7) */
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOB, &gpio);

  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = ARDUINO_BAUD;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  MAX485 direction pin (PB9) and heartbeat LED (PC13).
  */
static void GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_RESET);
  gpio.Pin   = DE_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_PORT, &gpio);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  gpio.Pin   = GPIO_PIN_13;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

/**
  * @brief  Halt on unrecoverable error.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
