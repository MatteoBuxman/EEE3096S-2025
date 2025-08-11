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
#include <string.h>

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
#define INITIAL_TIMER_DELAY 500

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables
static uint8_t led_pattern = 1;  // Current LED pattern (1-3, start with mode 1)
static uint8_t led_step = 0;              // Current step in pattern
static uint16_t timer_period = 1000;    // Timer period in ms (default 1 second)
static uint8_t button_state[4] = { 1, 1, 1, 1 }; // Previous button states for debouncing
static uint8_t button_pressed[4] = { 0, 0, 0, 0 }; // Button press flags
static uint8_t  sparkle_state = 0;          // 0=setup, 1=hold, 2=turn-off
static uint8_t  sparkle_leds[8];            // LED states
static uint8_t  sparkle_active_count = 0;   // How many LEDs are ON
static uint16_t sparkle_hold_timer = 0;     // Timer for hold phase (in milliseconds)
static uint16_t sparkle_hold_duration = 0;  // How long to hold before turning off (in milliseconds)
static uint32_t seed = 12345;

//
#define LEDS_MASK (LED0_Pin|LED1_Pin|LED2_Pin|LED3_Pin| \
                   LED4_Pin|LED5_Pin|LED6_Pin|LED7_Pin)

static const uint16_t led_pins[8] = {
    LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin,
    LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin
};

// All LEDS are on GPIOB
#define LEDS_PORT GPIOB

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
int main(void) {

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
	set_timer_period(INITIAL_TIMER_DELAY);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

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
void SystemClock_Config(void) {
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
	while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
	}
	LL_RCC_HSI_Enable();

	/* Wait till HSI is ready */
	while (LL_RCC_HSI_IsReady() != 1) {

	}
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

	/* Wait till System clock is ready */
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {

	}
	LL_SetSystemCoreClock(8000000);

	/* Update the time base */
	if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 8000 - 1;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 1000 - 1;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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
void TIM16_IRQHandler(void) {
	// Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

	// TODO: Change LED pattern

	update_led_pattern();

}
static inline uint32_t rand_lcg(uint32_t *seed) {
    *seed = (*seed * 1103515245u) + 12345u;
    return *seed;
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

void clear_all_leds() {
	// Clear all LEDs first
	LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
	LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
	LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
	LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
	LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
	LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
	LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
	LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
}


void update_led_pattern(void) {

	clear_all_leds();

	switch (led_pattern) {



	case 1: // Mode 1: Ping-Pong (LED bouncing back and forth continuously){

	{
		// 0 = right, 1 = left
		static uint8_t direction = 0;

		// light current LED
		set_pin(led_step);

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

	case 2:  // Inverted bounce: one OFF LED moves; others ON
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

	case 3: // TODO: Fill in the random pattern.
		switch (sparkle_state)
		        		    {
		        		        case 0: // Setup phase - randomly light LEDs
		        		        {
		        		            memset(sparkle_leds, 0, sizeof(sparkle_leds));
		        		            sparkle_active_count = 0;

		        		            // Randomly turn LEDs on (60% chance each)
		        		            for (int i = 0; i < 8; i++) {
		        		                if ((rand_lcg(&seed) % 100) < 60) {
		        		                    sparkle_leds[i] = 1;
		        		                    sparkle_active_count++;
		        		                }
		        		            }

		        		            // Ensure at least one LED is on
		        		            if (sparkle_active_count == 0) {
		        		                int led_idx = rand_lcg(&seed) % 8;
		        		                sparkle_leds[led_idx] = 1;
		        		                sparkle_active_count = 1;
		        		            }

		        		            // Random hold duration (100-1500ms)
		        		            sparkle_hold_duration = (rand_lcg(&seed) % 1401) + 100;
		        		            sparkle_hold_timer = 0;

		        		            sparkle_state = 1; // Go to hold phase
		        		            break;
		        		        }

		        		        case 1: // Hold phase - maintain pattern for specified milliseconds
		        		        {
		        		            sparkle_hold_timer += timer_period; // Accumulate actual timer period
		        		            if (sparkle_hold_timer >= sparkle_hold_duration) {
		        		                sparkle_state = 2; // Go to turn-off phase
		        		            }
		        		            break;
		        		        }

		        		        case 2: // Turn-off phase - turn off one LED at random
		        		        {
		        		            if (sparkle_active_count > 0) {
		        		                int idx;
		        		                // Pick a random LED that is currently ON
		        		                do {
		        		                    idx = rand_lcg(&seed) % 8;
		        		                } while (!sparkle_leds[idx] && sparkle_active_count > 0);

		        		                if (sparkle_leds[idx]) {
		        		                    sparkle_leds[idx] = 0;
		        		                    sparkle_active_count--;
		        		                }
		        		            } else {
		        		                // All LEDs are off, restart the cycle
		        		                sparkle_state = 0;
		        		            }
		        		            break;
		        		        }
		        		    }

		        		    // Update the actual LED hardware based on sparkle_leds array
		        		    GPIO_TypeDef* ports[] = {
		        		        LED0_GPIO_Port, LED1_GPIO_Port, LED2_GPIO_Port, LED3_GPIO_Port,
		        		        LED4_GPIO_Port, LED5_GPIO_Port, LED6_GPIO_Port, LED7_GPIO_Port
		        		    };
		        		    uint32_t pins[] = {
		        		        LED0_Pin, LED1_Pin, LED2_Pin, LED3_Pin,
		        		        LED4_Pin, LED5_Pin, LED6_Pin, LED7_Pin
		        		    };

		        		    for (int i = 0; i < 8; i++) {
		        		        if (sparkle_leds[i])
		        		            LL_GPIO_SetOutputPin(ports[i], pins[i]);
		        		        else
		        		            LL_GPIO_ResetOutputPin(ports[i], pins[i]);
		        		    }
		        		    break;



		        default: // Default to Mode 1 if invalid pattern
		            led_pattern = 1;
		            led_step = 0;
		            break;
	}
}

void check_buttons(void) {
	uint8_t current_state[4];

	// Read current button states (active low)
	current_state[0] = !LL_GPIO_IsInputPinSet(Button0_GPIO_Port, Button0_Pin);
	current_state[1] = !LL_GPIO_IsInputPinSet(Button1_GPIO_Port, Button1_Pin);
	current_state[2] = !LL_GPIO_IsInputPinSet(Button2_GPIO_Port, Button2_Pin);
	current_state[3] = !LL_GPIO_IsInputPinSet(Button3_GPIO_Port, Button3_Pin);

	// Check for button press events (transition from 0 to 1)
	for (int i = 0; i < 4; i++) {
		if (current_state[i] && !button_state[i] && !button_pressed[i]) {
			button_pressed[i] = 1;

			switch (i) {
			case 0: // Button 0: Change delay speed (cycle through different speeds)
				if(timer_period == 1000)         // Currently 1 second
				                	    timer_period = 500;          // Switch to 0.5 second
				                	else
				                	    timer_period = 1000;         // Switch back to 1 second

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
		if (!current_state[i]) {
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
void set_timer_period(uint16_t period) {
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
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
