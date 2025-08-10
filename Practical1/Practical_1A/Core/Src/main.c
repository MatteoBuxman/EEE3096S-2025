
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
static uint8_t led_pattern = 1;           // Current LED pattern (1-3, start with mode 1)
static uint8_t led_step = 0;              // Current step in pattern
static uint16_t timer_period = 1000;      // Timer period in ms (default 1 second)
static uint8_t button_state[4] = {1,1,1,1}; // Previous button states for debouncing
static uint8_t button_pressed[4] = {0,0,0,0}; // Button press flags


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
/* USER CODE END PFP */
void update_led_pattern(void);
void check_buttons(void);
void set_timer_period(uint16_t period);
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

  // TODO: Start timer TIM16

 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay

	  check_buttons();

	      // Small delay to prevent excessive button checking
	      HAL_Delay(50);
    

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

	update_led_pattern();

}

void update_led_pattern(void)
{
    // Clear all LEDs first
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

    switch(led_pattern)
    {
        case 1: // Mode 1: Running light (single LED moving left to right)
            switch(led_step)
            {
                case 0: LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin); break;
                case 1: LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
                case 2: LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
                case 3: LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
                case 4: LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
                case 5: LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
                case 6: LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
                case 7: LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
            }
            led_step = (led_step + 1) % 8;
            break;

            case 2: // Mode 2: Ping pong (LED bouncing back and forth continuously)
                       {
                           static uint8_t direction = 0; // 0 = moving right, 1 = moving left

                           // Light up the current LED
                           switch(led_step)
                           {
                               case 0: LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin); break;
                               case 1: LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
                               case 2: LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
                               case 3: LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
                               case 4: LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
                               case 5: LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
                               case 6: LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
                               case 7: LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
                           }

                           // Move to next position based on direction
                           if(direction == 0) //
                           {
                               led_step++;
                               if(led_step >= 7) // Reached right end, reverse direction
                               {
                                   direction = 1; // Now move left
                               }
                           }
                           else //
                           {
                               led_step--;
                               if(led_step <= 0) // Reached left end, reverse direction
                               {
                                   direction = 0; // Now move right
                               }
                           }
                       }
                       break;

        case 3: // Mode 3: Alternating pattern (odd/even LEDs alternate)
            if(led_step % 2 == 0)
            {
                LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
                LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
                LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
                LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
            }
            else
            {
                LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
                LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
                LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
                LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
            }
            led_step++;
            break;

        default: // Default to Mode 1 if invalid pattern
            led_pattern = 1;
            led_step = 0;
            break;
    }
}
void check_buttons(void)
{
    uint8_t current_state[4];

    // Read current button states (active low)
    current_state[0] = !LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);
    current_state[1] = !LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin);
    current_state[2] = !LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin);
    current_state[3] = !LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin);

    // Check for button press events (transition from 0 to 1)
    for(int i = 0; i < 4; i++)
    {
        if(current_state[i] && !button_state[i] && !button_pressed[i])
        {
            button_pressed[i] = 1;

            switch(i)
            {
                case 0: // Button 0: Change delay speed (cycle through different speeds)
                    if(timer_period == 250)         // Very fast
                        timer_period = 500;         // Fast
                    else if(timer_period == 500)
                        timer_period = 1000;        // Normal
                    else if(timer_period == 1000)
                        timer_period = 1500;        // Slow
                    else if(timer_period == 1500)
                        timer_period = 2000;        // Very slow
                    else
                        timer_period = 250;         // Back to very fast

                    set_timer_period(timer_period);
                    break;

                case 1: // Button 1: Set LED pattern mode 1
                    led_pattern = 1;
                    led_step = 0; // Reset step counter
                    break;

                case 2: // Button 2: Set LED pattern mode 2
                    led_pattern = 2;
                    led_step = 0; // Reset step counter
                    break;

                case 3: // Button 3: Set LED pattern mode 3
                    led_pattern = 3;
                    led_step = 0; // Reset step counter
                    break;
            }
        }

        // Clear button pressed flag when button is released
        if(!current_state[i])
        {
            button_pressed[i] = 0;
        }

        // Update previous state
        button_state[i] = current_state[i];
    }
}

/**
  * @brief  Set new timer period
  * @param  period: New period in milliseconds
  * @retval None
  */
void set_timer_period(uint16_t period)
{
    // Stop timer
    HAL_TIM_Base_Stop_IT(&htim16);

    // Update period (period in ms, timer runs at 1kHz)
    htim16.Init.Period = period - 1;

    // Reinitialize timer
    HAL_TIM_Base_Init(&htim16);

    // Restart timer
    HAL_TIM_Base_Start_IT(&htim16);
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
