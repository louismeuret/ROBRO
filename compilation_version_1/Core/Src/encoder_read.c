/**
  ******************************************************************************
  * @file    encoder_read.c
  * @brief   Standalone absolute-encoder reader (Tamagawa-style, command 0x32).
  *
  * This is a complete, self-contained application with its own main(). It does
  * nothing but poll the encoder over RS485 and decode its position. Build it
  * INSTEAD of Core/Src/main.c (swap that single line in the Makefile's
  * C_SOURCES list); the motor-control main.c is left untouched.
  *
  * --- Wiring (external MAX485 module, manual direction control) -------------
  *   PB6  USART1_TX  -> MAX485 DI            (data master -> bus)
  *   PB7  USART1_RX  <- MAX485 RO            (bus -> data in)
  *   PB9  GPIO out   -> MAX485 DE *and* RE   (tied together):
  *                        HIGH = transmit (drive the bus)
  *                        LOW  = receive (listen)
  *   MAX485 A / B    <-> encoder RS485 differential pair
  *
  * --- Link -----------------------------------------------------------------
  *   2.5 Mbit/s, 8N1. SYSCLK / PCLK2 is raised to 80 MHz so the baud divisor
  *   is exact: 80 MHz / 2.5 MHz = 32.
  *
  * --- Protocol (matches read_serial.py decode_position) --------------------
  *   TX: one byte 0x32
  *   RX: 9 bytes -> [CF=0x32, SF, ABS0, ABS1, ABS2, ABM0, ABM1, ABM2, CRC]
  *
  * --- Reading the result ---------------------------------------------------
  *   The decoded sample is published in the volatile globals below
  *   (enc_full_deg, enc_single_turn_deg, enc_turns, enc_raw, enc_valid).
  *   Watch them as live expressions over SWD; PC13 toggles on every valid
  *   frame as a heartbeat. (Debug text is deliberately NOT echoed onto the
  *   bus: ASCII '2' == 0x32 would look like a command to the encoder.)
  ******************************************************************************
  */

#include "main.h"
#include <string.h>

/* Encoder protocol --------------------------------------------------------- */
#define ENC_CMD          0x32u   /* request byte (Tamagawa Data-ID 6)         */
#define ENC_RESP_LEN     9       /* CF, SF, ABS0..2, ABM0..2, CRC             */
#define ENC_GEAR_RATIO   256.0f  /* internal turns per physical turn          */

/* MAX485 driver-enable / receiver-enable line ----------------------------- */
#define DE_PORT          GPIOB
#define DE_PIN           GPIO_PIN_9

UART_HandleTypeDef huart1;

/* Referenced by ADC1_2_IRQHandler in stm32g4xx_it.c. The ADC is never
 * initialised or enabled here, so this only exists to satisfy the linker. */
ADC_HandleTypeDef hadc1;

/* Latest decoded sample - inspect these over SWD. ------------------------- */
volatile float   enc_single_turn_deg = 0.0f;            /* one turn, [-180,180) */
volatile float   enc_turns           = 0.0f;            /* shaft turns          */
volatile float   enc_full_deg        = 0.0f;            /* absolute shaft angle */
volatile uint8_t enc_raw[ENC_RESP_LEN]  = {0};          /* last 9 raw bytes     */
volatile uint8_t enc_valid           = 0;               /* 1 = last poll good   */

static void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void GPIO_Init(void);

/**
  * @brief  One request/response transaction with the encoder.
  * @retval 1 if a valid 0x32 frame was decoded, 0 on timeout / bad echo.
  */
static int Encoder_Read(void)
{
  uint8_t cmd = ENC_CMD;
  uint8_t resp[ENC_RESP_LEN];

  /* Drive the bus, send the request, wait for the last bit to leave
   * (HAL_UART_Transmit blocks until TC), then hand the bus back to the
   * receiver so we can hear the reply. */
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
  * @brief  The application entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  MX_USART1_UART_Init();

  while (1)
  {
    if (Encoder_Read())
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);   /* heartbeat on valid frame */
    }
    HAL_Delay(10);                              /* ~100 Hz polling */
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
  osc.PLL.PLLM            = RCC_PLLM_DIV1;        /* 16 MHz to the PLL */
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
  clk.APB1CLKDivider = RCC_HCLK_DIV1;            /* PCLK1 = 80 MHz */
  clk.APB2CLKDivider = RCC_HCLK_DIV1;            /* PCLK2 = 80 MHz -> USART1 */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  USART1: 2.5 Mbit/s, 8N1 (PB6/PB7 configured by HAL_UART_MspInit).
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
  * @brief  Configure the MAX485 direction pin (PB9) and the heartbeat LED (PC13).
  */
static void GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* PB9 = DE/RE, idle in receive (LOW). High speed for a clean turnaround. */
  HAL_GPIO_WritePin(DE_PORT, DE_PIN, GPIO_PIN_RESET);
  gpio.Pin   = DE_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DE_PORT, &gpio);

  /* PC13 = heartbeat LED. */
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
