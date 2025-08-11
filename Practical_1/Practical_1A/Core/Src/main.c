/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
// LED mode: 0 = all off, 1 = mode1 back/forth, 2 = mode2 inverse back/forth, 3 = sparkle
static volatile uint8_t currentLedMode = 0;

// Shared state for back/forth modes
static volatile int8_t ledIndex = 0;      // 0..7
static volatile int8_t ledDirection = 1;  // +1 or -1

// Button previous states for edge detection (pull-up inputs: 1 = released, 0 = pressed)
static volatile uint8_t prevBtn1 = 1; // PA1
static volatile uint8_t prevBtn2 = 1; // PA2
static volatile uint8_t prevBtn3 = 1; // PA3

// Debounce state
static uint8_t dbShiftBtn0 = 0xFFu;
static uint8_t dbShiftBtn1 = 0xFFu;
static uint8_t dbShiftBtn2 = 0xFFu;
static uint8_t dbShiftBtn3 = 0xFFu;
static uint8_t stableBtn0 = 1u;
static uint8_t stableBtn1 = 1u;
static uint8_t stableBtn2 = 1u;
static uint8_t stableBtn3 = 1u;
static uint32_t lastDebounceTickMs = 0u;

// Simple PRNG state for sparkle mode
static volatile uint32_t rngState = 0xA5A5A5A5u;

// Sparkle mode state
static volatile uint8_t sparkleMask = 0;      // current LED pattern for sparkle
static volatile uint8_t sparklePhase = 0;     // 0=new random, 1=hold, 2=decay
static volatile uint8_t sparkleHoldTicks = 0; // coarse hold count in timer ticks
static volatile uint8_t sparkleDecayTickCountdown = 0; // ticks before turning off next LED
static volatile uint8_t sparkleJustEntered = 0; // show all-off first frame when entering mode 3


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  init_LCD();
  lcd_command(CLEAR);
  lcd_putstring("Mode 0");
/* USER CODE END 2 */

  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);

 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Debounce and handle buttons every 10 ms
    uint32_t now = HAL_GetTick();
    if ((now - lastDebounceTickMs) >= 10u) {
      lastDebounceTickMs = now;

      // Sample raw levels (pull-up inputs: 1=released, 0=pressed)
      uint8_t raw0 = LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);
      uint8_t raw1 = LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin);
      uint8_t raw2 = LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin);
      uint8_t raw3 = LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin);

      // Shift-register debounce: keep last 8 samples
      dbShiftBtn0 = (uint8_t)((dbShiftBtn0 << 1) | (raw0 & 0x01u));
      dbShiftBtn1 = (uint8_t)((dbShiftBtn1 << 1) | (raw1 & 0x01u));
      dbShiftBtn2 = (uint8_t)((dbShiftBtn2 << 1) | (raw2 & 0x01u));
      dbShiftBtn3 = (uint8_t)((dbShiftBtn3 << 1) | (raw3 & 0x01u));

      // Update stable states when all 8 samples are 0 or 1
      uint8_t newStable0 = (dbShiftBtn0 == 0x00u) ? 0u : (dbShiftBtn0 == 0xFFu) ? 1u : stableBtn0;
      uint8_t newStable1 = (dbShiftBtn1 == 0x00u) ? 0u : (dbShiftBtn1 == 0xFFu) ? 1u : stableBtn1;
      uint8_t newStable2 = (dbShiftBtn2 == 0x00u) ? 0u : (dbShiftBtn2 == 0xFFu) ? 1u : stableBtn2;
      uint8_t newStable3 = (dbShiftBtn3 == 0x00u) ? 0u : (dbShiftBtn3 == 0xFFu) ? 1u : stableBtn3;

      // Detect falling edges (released->pressed)
      if (stableBtn0 == 1u && newStable0 == 0u) {
        // Toggle ARR between 1000-1 and 500-1
        uint32_t currentArr = __HAL_TIM_GET_AUTORELOAD(&htim16);
        uint32_t newArr = (currentArr > 600u) ? (500u - 1u) : (1000u - 1u);
        __HAL_TIM_SET_AUTORELOAD(&htim16, newArr);
      }
      if (stableBtn1 == 1u && newStable1 == 0u) {
        currentLedMode = 1;
        ledIndex = 0;
        ledDirection = 1;
        lcd_command(CLEAR);
        lcd_putstring("Mode 1");
      }
      if (stableBtn2 == 1u && newStable2 == 0u) {
        currentLedMode = 2;
        ledIndex = 0;
        ledDirection = 1;
        lcd_command(CLEAR);
        lcd_putstring("Mode 2");
      }
      if (stableBtn3 == 1u && newStable3 == 0u) {
        currentLedMode = 3;
        sparklePhase = 0;
        sparkleMask = 0;
        sparkleJustEntered = 1;
        // Immediately blank LEDs so user sees all-off upon switching
        LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
        LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
        LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
        LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
        LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
        LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
        LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
        LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
        lcd_command(CLEAR);
        lcd_putstring("Mode 3");
      }

      // Commit new stable states
      stableBtn0 = newStable0;
      stableBtn1 = newStable1;
      stableBtn2 = newStable2;
      stableBtn3 = newStable3;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{
	// Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

    // TODO: Change LED pattern
    // Remove button handling from ISR; handled in debounced main loop

    uint8_t outMask = 0x00u;

    switch (currentLedMode) {
      case 0:
        outMask = 0x00u; // all off
        break;
      case 1: // back/forth single LED on
        // Output current index, then compute next with no-repeat at ends
        if (ledIndex < 0) ledIndex = 0;
        if (ledIndex > 7) ledIndex = 7;
        outMask = (uint8_t)(1u << (uint8_t)ledIndex);
        if (ledDirection > 0) {
          if (ledIndex >= 7) { ledDirection = -1; ledIndex = 6; } else { ledIndex++; }
        } else {
          if (ledIndex <= 0) { ledDirection = 1; ledIndex = 1; } else { ledIndex--; }
        }
        break;
      case 2: // inverse back/forth (all on except one off)
        if (ledIndex < 0) ledIndex = 0;
        if (ledIndex > 7) ledIndex = 7;
        outMask = (uint8_t)~(1u << (uint8_t)ledIndex); // all ones except index
        if (ledDirection > 0) {
          if (ledIndex >= 7) { ledDirection = -1; ledIndex = 6; } else { ledIndex++; }
        } else {
          if (ledIndex <= 0) { ledDirection = 1; ledIndex = 1; } else { ledIndex--; }
        }
        break;
      case 3: // sparkle
      default:
        // When entering mode 3, show all LEDs off for one tick before starting
        if (sparkleJustEntered) {
          outMask = 0x00u;
          sparkleJustEntered = 0u;
          break;
        }
        // Sparkle state machine synchronized to timer tick
        // Determine current tick period in ms from ARR (PSC makes 1 tick = 1ms)
        {
          uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim16);
          uint32_t tickMs = (arr + 1u); // since PSC = 8000-1 => 1kHz timer clock
          if (tickMs == 0u) tickMs = 1u;

          if (sparklePhase == 0) {
            // New random pattern 0..255
            rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
            sparkleMask = (uint8_t)(rngState & 0xFFu);
            // Hold for random 100..1500 ms
            uint32_t rnd = (rngState >> 8);
            uint32_t holdMs = 100u + (rnd % 1401u);
            uint8_t ticks = (uint8_t)(holdMs / tickMs);
            if (ticks == 0u) ticks = 1u;
            sparkleHoldTicks = ticks;
            sparkleDecayTickCountdown = 0;
            sparklePhase = 1;
            outMask = sparkleMask;
          } else if (sparklePhase == 1) {
            if (sparkleHoldTicks > 0u) {
              sparkleHoldTicks--;
              outMask = sparkleMask;
            } else {
              sparklePhase = 2;
              outMask = sparkleMask;
            }
          } else { // phase 2: turn off LEDs one at a time with ~random(100ms) delay
            if (sparkleDecayTickCountdown > 0u) {
              sparkleDecayTickCountdown--;
            } else {
              if (sparkleMask != 0u) {
                // Turn off the lowest-indexed set bit
                for (uint8_t i = 0u; i < 8u; ++i) {
                  uint8_t bit = (uint8_t)(1u << i);
                  if (sparkleMask & bit) { sparkleMask = (uint8_t)(sparkleMask & (uint8_t)~bit); break; }
                }
                // Next delay random(100) ms
                rngState ^= rngState << 13; rngState ^= rngState >> 17; rngState ^= rngState << 5;
                uint32_t dms = (rngState % 100u);
                uint8_t dticks = (uint8_t)(dms / tickMs);
                if (dticks == 0u) dticks = 1u;
                sparkleDecayTickCountdown = dticks;
              }
            }
            outMask = sparkleMask;
            if (sparkleMask == 0u) {
              sparklePhase = 0; // restart sequence
            }
          }
        }
        break;
    }

    // Drive LEDs on GPIOB pins 0..7 according to outMask (1 = LED ON)
    if (outMask & 0x01u) LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin); else LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    if (outMask & 0x02u) LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin); else LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    if (outMask & 0x04u) LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin); else LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    if (outMask & 0x08u) LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin); else LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    if (outMask & 0x10u) LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin); else LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    if (outMask & 0x20u) LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin); else LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    if (outMask & 0x40u) LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin); else LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    if (outMask & 0x80u) LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin); else LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);



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

#ifdef  USE_FULL_ASSERT
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
