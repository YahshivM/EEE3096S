/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time 
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/
// Image dimensions required for testing
volatile int image_dimensions[5] = {128, 160, 192, 224, 256};

// Index to select image size (modifiable via Live Expressions)
volatile int size_index = 0;

// Current width and height (exposed for Live Expressions)
volatile int width = 128;
volatile int height = 128;

// Toggle to select algorithm: 0 = fixed-point, 1 = double
volatile int use_double = 1;

// Timing variables (milliseconds from HAL ticks)
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile uint32_t execution_time = 0;

// Global checksum value
volatile uint64_t checksum = 0ULL;

// Per-dimension results (index 0..4 correspond to image_dimensions[])
volatile uint32_t execution_time_ms_fixed[5] = {0};
volatile uint32_t execution_time_ms_double[5] = {0};
volatile uint64_t checksum_fixed[5] = {0};
volatile uint64_t checksum_double[5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
static void run_benchmarks_for_all_dimensions(void);


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
  /* USER CODE BEGIN 2 */
  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  // Run benchmarks across all 5 image sizes and store results
  run_benchmarks_for_all_dimensions();

  //TODO: Turn on LED 1 to signify the end of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  //TODO: Hold the LEDs on for a 1s delay
  HAL_Delay(1000);

  //TODO: Turn off the LEDs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void run_benchmarks_for_all_dimensions(void)
{
  for (int i = 0; i < 5; i++) {
    // Update selected size and expose width/height
    size_index = i;
    width = image_dimensions[i];
    height = image_dimensions[i];

    // Record the start time
    start_time = HAL_GetTick();

    // Compute checksum for the selected algorithm
    if (use_double) {
      checksum = calculate_mandelbrot_double(width, height, MAX_ITER);
    } else {
      checksum = calculate_mandelbrot_fixed_point_arithmetic(width, height, MAX_ITER);
    }

    // Record the end time and compute execution time
    end_time = HAL_GetTick();
    execution_time = end_time - start_time;

    // Store into per-dimension arrays
    if (use_double) {
      checksum_double[i] = checksum;
      execution_time_ms_double[i] = execution_time;
    } else {
      checksum_fixed[i] = checksum;
      execution_time_ms_fixed[i] = execution_time;
    }
  }
}

//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    // Fixed-point Q3.29 scaling
    const int64_t SCALE = (int64_t)1 << 29;
    const int64_t FOUR_S = 4 * SCALE;

    for (int y = 0; y < height; y++) {
        // y0 = (y/height)*2.0 - 1.0
        int64_t y0 = ((int64_t)y * 2 * SCALE) / height - SCALE;
        for (int x = 0; x < width; x++) {
            // x0 = (x/width)*3.5 - 2.5
            int64_t x0 = ((int64_t)x * 7 * SCALE) / (2 * width) - (5 * SCALE) / 2;

            int64_t xi = 0;
            int64_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                int64_t xi_sq = (xi * xi) >> 29;
                int64_t yi_sq = (yi * yi) >> 29;
                if (xi_sq + yi_sq > FOUR_S) {
                    break;
                }
                int64_t temp = xi_sq - yi_sq;
                yi = ((2 * xi * yi) >> 29) + y0;
                xi = temp + x0;
                iteration++;
            }
            mandelbrot_sum += (uint64_t)iteration;
        }
    }

    return mandelbrot_sum;

}

//TODO: Mandelbroat using variable type double
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    for (int y = 0; y < height; y++) {
        double y0 = ((double)y / (double)height) * 2.0 - 1.0;
        for (int x = 0; x < width; x++) {
            double x0 = ((double)x / (double)width) * 3.5 - 2.5;
            double xi = 0.0;
            double yi = 0.0;
            int iteration = 0;
            while (iteration < max_iterations) {
                double xi_sq = xi * xi;
                double yi_sq = yi * yi;
                if (xi_sq + yi_sq > 4.0) {
                    break;
                }
                double temp = xi_sq - yi_sq;
                yi = (2.0 * xi * yi) + y0;
                xi = temp + x0;
                iteration++;
            }
            mandelbrot_sum += (uint64_t)iteration;
        }
    }
    return mandelbrot_sum;
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
