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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define IN1_PIN GPIO_PIN_3
#define IN1_PORT GPIOB
#define IN2_PIN GPIO_PIN_10
#define IN2_PORT GPIOA
#define IN3_PIN GPIO_PIN_4
#define IN3_PORT GPIOB
#define IN4_PIN GPIO_PIN_5
#define IN4_PORT GPIOB

#define stepsperrev 4096

void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait
}
// Function to set GPIO pins according to step sequence
void stepper_half_drive (int step)
{
  switch (step){
         case 0:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_RESET);   // IN4
		  break;

	  case 1:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_RESET);   // IN4
		  break;

          case 2:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_RESET);   // IN4
		  break;

	  case 3:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_SET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_RESET);   // IN4
		  break;

	  case 4:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_RESET);   // IN4
		  break;

	  case 5:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_SET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_SET);   // IN4
		  break;

	  case 6:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_RESET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_SET);   // IN4
		  break;

	  case 7:
		  HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, GPIO_PIN_SET);   // IN1
		  HAL_GPIO_WritePin(GPIOA, IN2_PIN, GPIO_PIN_RESET);   // IN2
		  HAL_GPIO_WritePin(GPIOB, IN3_PIN, GPIO_PIN_RESET);   // IN3
		  HAL_GPIO_WritePin(GPIOB, IN4_PIN, GPIO_PIN_SET);   // IN4
		  break;

	}
}
// Function to move the motor in either clockwise or counterclockwise direction
/* USER CODE END 0 */
void rotate_120(int direction) {
    const int steps_per_120_deg = 1365;  // 1024 * (120/90)
    for (int i = 0; i < steps_per_120_deg; i++) {
        int step_index;

        if (direction == 0) {
            // Clockwise
            step_index = i % 8;
        } else {
            // Counter-clockwise (reverse stepping order)
            step_index = (8 - (i % 8)) % 8;
        }

        stepper_half_drive(step_index);
        HAL_Delay(1 ); // Tune this delay to control speed (e.g., 1 = slow, 0 = fast)
    }
}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	  rotate_120(0);
	  rotate_120(1);
	  HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3|LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 LD2_Pin PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|LD2_Pin|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
