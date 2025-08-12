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
#include <stdlib.h>
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
static uint8_t led_mode = 0;              // Current LED mode (0=off, 1-3=modes)
static uint8_t led_step = 0;              // Current step in pattern
static uint8_t direction = 0;             // Direction for back/forth patterns (0=forward, 1=backward)
static uint8_t button_state[4] = {1,1,1,1};

// Sparkle mode variables
static uint8_t sparkle_pattern = 0;       // Current sparkle pattern
static uint8_t sparkle_state = 0;         // 0=generate pattern, 1=hold, 2=turn off LEDs
static uint8_t sparkle_step = 0;          // Step in sparkle turn-off sequence
static uint8_t leds_to_turn_off[8];       // Array to track which LEDs to turn off in sparkle mode
static uint8_t num_leds_on = 0;           // Number of LEDs currently on in sparkle mode
static uint32_t sparkle_start_time = 0;   // Start time for current sparkle state
static uint32_t sparkle_delay = 0;        // Current delay for sparkle timing
#define LEDS_MASK (LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin| \
                   LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin)
static const uint16_t led_pins[8] = {
    LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin,
    LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin
};
#define LEDS_PORT GPIOB
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
void update_led_pattern(void);
void check_buttons(void);
void set_timer_period(uint16_t period);
void clear_all_leds(void);

void handle_sparkle_mode(void);
void set_led_pattern(uint8_t pattern);
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

  // TODO: Start timer TIM16
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay
    check_buttons();

    // Handle sparkle mode timing independently
    if(led_mode == 3)
    {
        handle_sparkle_mode();
    }

    // Small delay to prevent excessive button checking
    HAL_Delay(10);


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

void clear_all_leds(void)
{
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
}

void set_pin(int pin_number) {
	switch (pin_number) {
	case 0:
		LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
		break;
	case 1:
		LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
		break;
	case 2:
		LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
		break;
	case 3:
		LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
		break;
	case 4:
		LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
		break;
	case 5:
		LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
		break;
	case 6:
		LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
		break;
	case 7:
		LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
		break;
	}
}
void set_led_pattern(uint8_t pattern)
{
    clear_all_leds();

    if(pattern & 0x01) LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
    if(pattern & 0x02) LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
    if(pattern & 0x04) LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
    if(pattern & 0x08) LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin);
    if(pattern & 0x10) LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin);
    if(pattern & 0x20) LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin);
    if(pattern & 0x40) LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin);
    if(pattern & 0x80) LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin);
}
void update_led_pattern(void)
{
    switch(led_mode)
    {
        case 0: // All LEDs off
            clear_all_leds();
            break;

        case 1: // Mode 1: Back/forth with single LED
        {
        		// 0 = right, 1 = left
        		static uint8_t direction = 0;
        		uint8_t pattern = 1 << led_step;
        		// light current LED
        		set_led_pattern(pattern);

        		if (direction == 0) {                  // moving right
        			if (led_step < 7)
        				led_step++;
        			else {
        				direction = 1;
        				led_step--;
        			} // bounce at right edge
        		} else {                                // moving left
        			if (led_step > 0)
        				led_step--;
        			else {
        				direction = 0;
        				led_step++;
        			} // bounce at left edge
        		}
        		break;
        	}

        case 2: // Mode 2: Inverse back/forth (all LEDs on except one)
        {
        	    static uint8_t direction = 0; // 0 = right, 1 = left

        	    // Turn ALL LEDs ON in one shot
        	    LL_GPIO_SetOutputPin(LEDS_PORT, LEDS_MASK);

        	    // Turn the current one OFF
        	    LL_GPIO_ResetOutputPin(LEDS_PORT, led_pins[led_step]);

        	    // Advance the "hole" with bounce at edges
        	    if (direction == 0) {                  // moving right
        	        if (led_step < 7) led_step++;
        	        else { direction = 1; led_step--; } // bounce right
        	    } else {                                // moving left
        	        if (led_step > 0) led_step--;
        	        else { direction = 0; led_step++; } // bounce left
        	    }
        	    break;

        	}

        case 3: // Mode 3: Sparkle mode - handled separately in main loop
            // Do nothing here - sparkle mode has its own timing
            break;

        default:
            led_mode = 0; // Default to off
            break;
    }
}

void check_buttons(void)
{
    uint8_t current_state[4];

    // Read current button states (active low, so invert)
    current_state[0] = !LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);
    current_state[1] = !LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin);
    current_state[2] = !LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin);
    current_state[3] = !LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin);

    // Check for button press events (transition from 0 to 1)
    for(int i = 0; i < 4; i++)
    {
        if(current_state[i] && !button_state[i]) // Button just pressed
        {
            switch(i)
            {
                case 0: // Button PA0: Toggle timer period between 1s and 0.5s
                {
                    if(htim16.Init.Period == 999) // Currently 1 second
                    {
                        htim16.Init.Period = 499; // Change to 0.5 second
                    }
                    else
                    {
                        htim16.Init.Period = 999; // Change to 1 second
                    }

                    // Update the timer
                    HAL_TIM_Base_Stop_IT(&htim16);
                    HAL_TIM_Base_Init(&htim16);
                    HAL_TIM_Base_Start_IT(&htim16);
                    break;
                }

                case 1: // Button PA1: Activate Mode 1
                    led_mode = 1;
                    led_step = 0;
                    direction = 0; // Start moving forward
                    break;

                case 2: // Button PA2: Activate Mode 2
                    led_mode = 2;
                    led_step = 0;
                    direction = 0; // Start moving forward
                    break;

                case 3: // Button PA3: Activate Mode 3 (Sparkle)
                    led_mode = 3;
                    sparkle_state = 0; // Reset sparkle state
                    sparkle_start_time = HAL_GetTick();
                    rand(HAL_GetTick());// Initialize timing
                    break;
            }
        }

        // Update previous state
        button_state[i] = current_state[i];
    }
}

void handle_sparkle_mode(void)
{
    uint32_t current_time = HAL_GetTick();

    switch(sparkle_state)
    {
        case 0: // Generate new random pattern
        {
            sparkle_pattern = rand() & 0xFF; // Random 8-bit pattern (0-255)
            set_led_pattern(sparkle_pattern);

            // Count how many LEDs are on and prepare turn-off sequence
            num_leds_on = 0;
            for(int i = 0; i < 8; i++)
            {
                if(sparkle_pattern & (1 << i))
                {
                    leds_to_turn_off[num_leds_on] = i;
                    num_leds_on++;
                }
            }

            // Shuffle the turn-off order for randomness
            for(int i = num_leds_on - 1; i > 0; i--)
            {
                int j = rand() % (i + 1);
                uint8_t temp = leds_to_turn_off[i];
                leds_to_turn_off[i] = leds_to_turn_off[j];
                leds_to_turn_off[j] = temp;
            }

            // Random hold time between 100-1500ms
            sparkle_delay = (rand() % 1401) + 100;
            sparkle_start_time = current_time;
            sparkle_state = 1; // Move to hold state
            break;
        }

        case 1: // Hold pattern for random time
        {
            if((current_time - sparkle_start_time) >= sparkle_delay)
            {
                sparkle_state = 2; // Move to turn-off state
                sparkle_step = 0;
                sparkle_start_time = current_time;
                // Generate random delay for first LED turn-off (up to 100ms)
                sparkle_delay = rand() % 100 + 1;
            }
            break;
        }

        case 2: // Turn off LEDs one by one with random delays
        {
            if((current_time - sparkle_start_time) >= sparkle_delay)
            {
                if(sparkle_step < num_leds_on)
                {
                    // Turn off the next LED in sequence
                    uint8_t led_to_turn_off = leds_to_turn_off[sparkle_step];
                    sparkle_pattern &= ~(1 << led_to_turn_off);
                    set_led_pattern(sparkle_pattern);
                    sparkle_step++;
                    sparkle_start_time = current_time;
                    // Generate next random delay (up to 100ms)
                    sparkle_delay = rand() % 100 + 1;
                }
                else
                {
                    // All LEDs are off, start over after a brief pause
                    sparkle_state = 0;
                    sparkle_start_time = current_time;
                    sparkle_delay = 50; // Brief pause before next cycle
                }
            }
            break;
        }
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
