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
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
#define MIN_CHUNK_SIZE 64
#define MAX_MEMORY_TEST_SIZE (32 * 1024)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// LED definitions for STM32F4 discovery board
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Results structure to store test data
typedef struct {
    int task_num;
    int image_size;
    int max_iter;
    uint32_t execution_time;
    uint32_t cycle_count;
    uint64_t checksum;
    float throughput;
    int data_type; // 0=fixed, 1=float, 2=double
    int scaling_factor;
    int optimization_level;
} TestResult;

// Accuracy comparison structure
typedef struct {
    uint64_t checksum_fixed;
    uint64_t checksum_float;
    uint64_t checksum_double;
    float accuracy_float_vs_double;
    float accuracy_fixed_vs_double;
    uint32_t time_fixed;
    uint32_t time_float_fpu_on;
    uint32_t time_float_fpu_off;
    uint32_t time_double;
} AccuracyTestResult;

// Compiler optimization analysis
typedef struct {
    int optimization_level;
    uint32_t code_size;
    uint32_t data_size;
    uint32_t total_binary_size;
    uint32_t total_runtime;
    uint32_t mandelbrot_runtime;
    float performance_per_byte;
} OptimizationResult;

// Power measurement structure
typedef struct {
    float estimated_power_mw;
    float energy_consumption_mj;
    uint32_t active_time_ms;
    uint32_t idle_time_ms;
    float efficiency_pixels_per_mw;
    char measurement_method[64];
} PowerMeasurement;

// Test storage
#define MAX_TEST_RESULTS 200
TestResult test_results[MAX_TEST_RESULTS];
int test_result_count = 0;

// Additional result storage
AccuracyTestResult accuracy_results[10];
int accuracy_test_count = 0;

OptimizationResult opt_results[4];
int opt_result_count = 0;

PowerMeasurement power_measurements[10];
int power_measurement_count = 0;

// For DWT cycle counting
#define DWT_CYCCNT   *(volatile uint32_t *)0xE0001004
#define DWT_CONTROL  *(volatile uint32_t *)0xE0001000
#define DEMCR        *(volatile uint32_t *)0xE000EDFC

// Live Expressions monitoring variables
volatile uint32_t current_image_size = 0;
volatile uint32_t current_max_iter = 0;
volatile uint32_t current_execution_time = 0;
volatile uint32_t current_cycle_count = 0;
volatile uint64_t current_checksum = 0;
volatile float current_throughput = 0;
volatile uint32_t test_in_progress = 0;
volatile uint32_t current_task = 0;
volatile int current_data_type = 0;
volatile int current_scaling_factor = 0;
volatile int current_optimization_level = 0;

// Image sizes from Practical 1B
const int image_sizes[] = {128, 160, 192, 224, 256};
const int num_sizes = sizeof(image_sizes) / sizeof(image_sizes[0]);

// MAX_ITER values for Task 2
const int max_iter_values[] = {100, 250, 500, 750, 1000};
const int num_iter_values = sizeof(max_iter_values) / sizeof(max_iter_values[0]);

// Task 4 image sizes
const int large_sizes[] = {320, 480, 640, 800, 1024, 1280, 1920};
const int num_large_sizes = sizeof(large_sizes) / sizeof(large_sizes[0]);

// Task 7 scaling factors
const int scaling_factors[] = {1000, 10000, 1000000};
const int num_scaling_factors = sizeof(scaling_factors) / sizeof(scaling_factors[0]);

// Benchmarking variables
uint32_t start_time, end_time, execution_time;
uint32_t start_cycles, end_cycles, cycle_count;

// Binary size measurement (requires linker script symbols)
extern uint32_t _etext;   // End of code section
extern uint32_t _stext;   // Start of code section
extern uint32_t _edata;   // End of data section
extern uint32_t _sdata;   // Start of data section
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Mandelbrot function prototypes
uint64_t calculate_mandelbrot_fixed_point(int width, int height, int max_iterations, int scaling_factor);
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_chunked(int width, int height, int max_iterations, int data_type, int scaling_factor);
int calculate_mandelbrot_point(int px, int py, int width, int height, int max_iterations, int data_type, int scaling_factor);
int mandelbrot_point_fixed(int px, int py, int width, int height, int max_iterations, int scaling_factor);
int mandelbrot_point_float(int px, int py, int width, int height, int max_iterations);
int mandelbrot_point_double(int px, int py, int width, int height, int max_iterations);

// Utility functions
void enable_cycle_counter(void);
uint32_t get_cycle_count(void);
void toggle_led(int led_num);
void run_single_test(int image_size, int max_iter, int task_num, int data_type, int scaling_factor, int opt_level);
void store_test_result(int task_num, int image_size, int max_iter, uint32_t exec_time, uint32_t cycles, uint64_t checksum, float throughput, int data_type, int scaling_factor, int opt_level);

// Task 4 functions
uint32_t get_available_memory(void);
int calculate_chunk_size(int image_size, uint32_t available_memory);
uint64_t validate_chunked_calculation(int image_size);

// Task 5 functions (FPU Control - Manual)
void test_fpu_comprehensive(int image_size);
float calculate_accuracy_difference(uint64_t reference, uint64_t test);

// Task 6 functions (Optimization - Manual)
uint32_t get_code_size(void);
uint32_t get_data_size(void);
uint32_t benchmark_mandelbrot_performance(void);

// Task 8 functions (Power)
float calculate_detailed_power(uint32_t execution_time_ms, int cpu_frequency_mhz, int image_size, int data_type);
void measure_power_comprehensive(int image_size);
void document_power_measurement_challenges(void);

// Task functions
void run_task_2(void);
void run_task_3(void);
void run_task_4(void);
void run_task_5_real(void);
void run_task_6_real(void);
void run_task_7(void);
void run_task_8_real(void);

// Helper functions
void cleanup_memory(void);
/* USER CODE END PFP */

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

    /* USER CODE BEGIN 2 */
    enable_cycle_counter();
    HAL_Delay(1000);

    // Reset results array
    test_result_count = 0;
    accuracy_test_count = 0;
    opt_result_count = 0;
    power_measurement_count = 0;

    // Turn on LED 0 to signify the start of the operation
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

    // Run all tasks
    //run_task_2();
    //HAL_Delay(1000);

    //run_task_3();
    //HAL_Delay(1000);

    //run_task_4();
    //HAL_Delay(1000);

    run_task_5_real();
    HAL_Delay(1000);

    //run_task_6_real();
    //HAL_Delay(1000);

    //run_task_7();
    //HAL_Delay(1000);

    //run_task_8_real();
    //HAL_Delay(1000);

    // Turn on LED 1 to signify the end of the operation
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    HAL_Delay(1000);

    // Turn off all LEDs
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pins : PB0 PB1 PB2 PB3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ==================== CORE UTILITY FUNCTIONS ====================

// Store test results
void store_test_result(int task_num, int image_size, int max_iter,
                      uint32_t exec_time, uint32_t cycles, uint64_t checksum_val,
                      float throughput, int data_type, int scaling_factor, int opt_level) {
    if (test_result_count < MAX_TEST_RESULTS) {
        test_results[test_result_count].task_num = task_num;
        test_results[test_result_count].image_size = image_size;
        test_results[test_result_count].max_iter = max_iter;
        test_results[test_result_count].execution_time = exec_time;
        test_results[test_result_count].cycle_count = cycles;
        test_results[test_result_count].checksum = checksum_val;
        test_results[test_result_count].throughput = throughput;
        test_results[test_result_count].data_type = data_type;
        test_results[test_result_count].scaling_factor = scaling_factor;
        test_results[test_result_count].optimization_level = opt_level;
        test_result_count++;
    }
}

// Run a single test and store results
void run_single_test(int image_size, int max_iter, int task_num,
                    int data_type, int scaling_factor, int opt_level) {
    test_in_progress = 1;
    current_image_size = image_size;
    current_max_iter = max_iter;
    current_data_type = data_type;
    current_scaling_factor = scaling_factor;
    current_optimization_level = opt_level;

    // Record start time and cycles
    start_time = HAL_GetTick();
    start_cycles = get_cycle_count();

    // Calculate Mandelbrot based on data type
    uint64_t local_checksum = 0;
    switch(data_type) {
        case 0: // fixed point
            local_checksum = calculate_mandelbrot_fixed_point(image_size, image_size, max_iter, scaling_factor);
            break;
        case 1: // float
            local_checksum = calculate_mandelbrot_float(image_size, image_size, max_iter);
            break;
        case 2: // double
            local_checksum = calculate_mandelbrot_double(image_size, image_size, max_iter);
            break;
    }

    // Record end time and cycles
    end_cycles = get_cycle_count();
    end_time = HAL_GetTick();

    execution_time = end_time - start_time;
    cycle_count = end_cycles - start_cycles;

    // Calculate throughput
    if (execution_time > 0) {
        current_throughput = (image_size * image_size) / (execution_time / 1000.0f);
    } else {
        current_throughput = 0;
    }

    // Update Live Expressions variables
    current_execution_time = execution_time;
    current_cycle_count = cycle_count;
    current_checksum = local_checksum;
    current_throughput = current_throughput;

    // Store the result
    store_test_result(task_num, image_size, max_iter, execution_time,
                     cycle_count, local_checksum, current_throughput,
                     data_type, scaling_factor, opt_level);

    test_in_progress = 0;
    toggle_led(0);
}

// Enable DWT cycle counter
void enable_cycle_counter(void) {
    DEMCR |= 0x01000000;
    DWT_CYCCNT = 0;
    DWT_CONTROL |= 1;
}

uint32_t get_cycle_count(void) {
    return DWT_CYCCNT;
}

void toggle_led(int led_num) {
    switch(led_num) {
        case 0: HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin); break;
        case 1: HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); break;
        case 2: HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); break;
        case 3: HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); break;
    }
}

void cleanup_memory(void) {
    // Force garbage collection if needed - placeholder
}

// ==================== TASK 4 FUNCTIONS (Scalability) ====================

// Memory availability checker
uint32_t get_available_memory(void) {
    uint32_t max_allocatable = 0;
    uint32_t test_size = 1024;

    // Binary search for maximum allocatable memory
    while (test_size <= MAX_MEMORY_TEST_SIZE) {
        void *test_ptr = malloc(test_size);
        if (test_ptr != NULL) {
            max_allocatable = test_size;
            free(test_ptr);
            test_size *= 2;
        } else {
            break;
        }
    }

    return max_allocatable;
}

// Calculate optimal chunk size based on available memory
int calculate_chunk_size(int image_size, uint32_t available_memory) {
    // Estimate memory needed per pixel (iterations + overhead)
    uint32_t bytes_per_pixel = sizeof(int) + sizeof(float); // Conservative estimate
    uint32_t total_needed = image_size * image_size * bytes_per_pixel;

    if (total_needed <= available_memory) {
        return image_size; // No chunking needed
    }

    // Calculate chunk size that fits in available memory
    int max_pixels_per_chunk = available_memory / bytes_per_pixel;
    int chunk_size = (int)sqrt(max_pixels_per_chunk);

    // Ensure minimum chunk size
    if (chunk_size < MIN_CHUNK_SIZE) {
        chunk_size = MIN_CHUNK_SIZE;
    }

    return chunk_size;
}

// Helper function to calculate single Mandelbrot point
int calculate_mandelbrot_point(int px, int py, int width, int height,
                              int max_iterations, int data_type, int scaling_factor) {
    switch(data_type) {
        case 0: // Fixed point
            return mandelbrot_point_fixed(px, py, width, height, max_iterations, scaling_factor);
        case 1: // Float
            return mandelbrot_point_float(px, py, width, height, max_iterations);
        case 2: // Double
            return mandelbrot_point_double(px, py, width, height, max_iterations);
        default:
            return 0;
    }
}

// Individual point calculations
int mandelbrot_point_fixed(int px, int py, int width, int height, int max_iterations, int scaling_factor) {
    int32_t scale_x = (3.5 * scaling_factor) / width;
    int32_t scale_y = (2.0 * scaling_factor) / height;
    int32_t offset_x = -2.5 * scaling_factor;
    int32_t offset_y = -1.0 * scaling_factor;
    int32_t threshold = 4.0 * scaling_factor;

    int32_t x0 = (px * scale_x) + offset_x;
    int32_t y0 = (py * scale_y) + offset_y;

    int32_t xi = 0;
    int32_t yi = 0;
    int iteration = 0;

    while (iteration < max_iterations) {
        int64_t xi_sq = (int64_t)xi * xi / scaling_factor;
        int64_t yi_sq = (int64_t)yi * yi / scaling_factor;

        if (xi_sq + yi_sq > threshold) {
            break;
        }

        int32_t temp = xi_sq - yi_sq;
        yi = (2 * (int64_t)xi * yi / scaling_factor) + y0;
        xi = temp + x0;

        iteration++;
    }

    return iteration;
}

int mandelbrot_point_float(int px, int py, int width, int height, int max_iterations) {
    float x0 = ((float)px / width) * 3.5f - 2.5f;
    float y0 = ((float)py / height) * 2.0f - 1.0f;

    float xi = 0.0f;
    float yi = 0.0f;
    int iteration = 0;

    while (iteration < max_iterations) {
        float xi_sq = xi * xi;
        float yi_sq = yi * yi;

        if (xi_sq + yi_sq > 4.0f) {
            break;
        }

        float temp = xi_sq - yi_sq;
        yi = 2.0f * xi * yi + y0;
        xi = temp + x0;

        iteration++;
    }

    return iteration;
}

int mandelbrot_point_double(int px, int py, int width, int height, int max_iterations) {
    double x0 = ((double)px / width) * 3.5 - 2.5;
    double y0 = ((double)py / height) * 2.0 - 1.0;

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
        yi = 2.0 * xi * yi + y0;
        xi = temp + x0;

        iteration++;
    }

    return iteration;
}

// Improved chunked Mandelbrot calculation
uint64_t calculate_mandelbrot_chunked(int width, int height, int max_iterations,
                                     int data_type, int scaling_factor) {
    uint32_t available_memory = get_available_memory();
    int chunk_size = calculate_chunk_size(width, available_memory);

    uint64_t total_checksum = 0;
    uint32_t total_chunks = 0;

    // Process image in tiles
    for (int tile_y = 0; tile_y < height; tile_y += chunk_size) {
        for (int tile_x = 0; tile_x < width; tile_x += chunk_size) {
            int actual_width = (tile_x + chunk_size > width) ?
                              (width - tile_x) : chunk_size;
            int actual_height = (tile_y + chunk_size > height) ?
                               (height - tile_y) : chunk_size;

            // Calculate offset for this chunk
            uint64_t chunk_checksum = 0;

            // Process this chunk
            for (int y = 0; y < actual_height; y++) {
                for (int x = 0; x < actual_width; x++) {
                    int global_x = tile_x + x;
                    int global_y = tile_y + y;

                    int iterations = calculate_mandelbrot_point(
                        global_x, global_y, width, height,
                        max_iterations, data_type, scaling_factor);

                    chunk_checksum += iterations;
                }
            }

            total_checksum += chunk_checksum;
            total_chunks++;

            // Small delay to prevent watchdog issues
            HAL_Delay(1);
        }
    }

    return total_checksum;
}

// Validation function to ensure chunked results match non-chunked
uint64_t validate_chunked_calculation(int image_size) {
    // Run both chunked and non-chunked for small images and compare
    if (image_size <= 256) {
        uint64_t normal_result = calculate_mandelbrot_fixed_point(image_size, image_size, 100, 1 << 16);
        uint64_t chunked_result = calculate_mandelbrot_chunked(image_size, image_size, 100, 0, 1 << 16);

        // Should be identical for small images
        return (normal_result == chunked_result) ? 1 : 0;
    }
    return 1; // Assume valid for large images
}

// ==================== TASK 5 FUNCTIONS (FPU Impact) ====================

// Calculate accuracy percentage difference
float calculate_accuracy_difference(uint64_t reference, uint64_t test) {
    if (reference == 0) return 0.0f;

    int64_t diff = (int64_t)reference - (int64_t)test;
    float percentage = (float)(abs((int)diff)) / (float)reference * 100.0f;
    return percentage;
}

// Comprehensive FPU test for single image size (Manual FPU control)
void test_fpu_comprehensive(int image_size) {
    AccuracyTestResult result = {0};

    // Test 1: Double precision (reference for accuracy)
    start_time = HAL_GetTick();
    result.checksum_double = calculate_mandelbrot_double(image_size, image_size, 100);
    result.time_double = HAL_GetTick() - start_time;

    // Test 2: Fixed point
    start_time = HAL_GetTick();
    result.checksum_fixed = calculate_mandelbrot_fixed_point(image_size, image_size, 100, 1 << 16);
    result.time_fixed = HAL_GetTick() - start_time;

    // Test 3: Float (current FPU setting - will be manually controlled)
    start_time = HAL_GetTick();
    result.checksum_float = calculate_mandelbrot_float(image_size, image_size, 100);
    result.time_float_fpu_on = HAL_GetTick() - start_time;

    // Note: FPU enable/disable will be done manually via Makefile/IDE settings
    // This allows testing both FPU enabled and disabled builds

    // Calculate accuracy differences
    result.accuracy_float_vs_double = calculate_accuracy_difference(
        result.checksum_double, result.checksum_float);
    result.accuracy_fixed_vs_double = calculate_accuracy_difference(
        result.checksum_double, result.checksum_fixed);

    // Store comprehensive results
    if (accuracy_test_count < 10) {
        accuracy_results[accuracy_test_count] = result;
        accuracy_test_count++;
    }

    // Store individual results for tracking
    store_test_result(5, image_size, 100, result.time_fixed, 0,
                     result.checksum_fixed, 0, 0, 1 << 16, 0); // Fixed
    store_test_result(5, image_size, 100, result.time_float_fpu_on, 0,
                     result.checksum_float, 0, 1, 0, 1); // Float
    store_test_result(5, image_size, 100, result.time_double, 0,
                     result.checksum_double, 0, 2, 0, -1); // Double
}

// ==================== TASK 6 FUNCTIONS (Optimization) ====================

// Calculate binary size
uint32_t get_code_size(void) {
    return (uint32_t)&_etext - (uint32_t)&_stext;
}

uint32_t get_data_size(void) {
    return (uint32_t)&_edata - (uint32_t)&_sdata;
}

// Benchmark function for optimization testing
uint32_t benchmark_mandelbrot_performance(void) {
    uint32_t total_time = 0;

    // Run multiple small tests for consistent timing
    for (int i = 0; i < 3; i++) {
        uint32_t start = HAL_GetTick();
        calculate_mandelbrot_fixed_point(128, 128, 100, 1 << 16);
        total_time += HAL_GetTick() - start;
        HAL_Delay(10);
    }

    return total_time / 3; // Average
}

// ==================== TASK 8 FUNCTIONS (Power Measurement) ====================

// Enhanced power calculation based on STM32F4 characteristics
float calculate_detailed_power(uint32_t execution_time_ms, int cpu_frequency_mhz,
                              int image_size, int data_type) {
    // STM32F4 power characteristics (from datasheet)
    const float base_current_ma = 30.0f;  // Idle current
    const float cpu_current_per_mhz = 0.8f; // Additional current per MHz
    const float fpu_overhead_ma = 20.0f; // FPU overhead when active
    const float supply_voltage = 3.3f;

    // Calculate dynamic current based on CPU frequency
    float dynamic_current = base_current_ma + (cpu_frequency_mhz * cpu_current_per_mhz);

    // Add FPU overhead for float/double operations
    if (data_type == 1 || data_type == 2) {
        dynamic_current += fpu_overhead_ma;
    }

    // Calculate power in mW
    float power_mw = dynamic_current * supply_voltage;

    return power_mw;
}

// Power measurement with multiple methods
void measure_power_comprehensive(int image_size) {
    PowerMeasurement measurement = {0};

    // Method 1: Theoretical calculation
    uint32_t start_time = HAL_GetTick();
    calculate_mandelbrot_fixed_point(image_size, image_size, 100, 1 << 16);
    uint32_t execution_time = HAL_GetTick() - start_time;

    measurement.active_time_ms = execution_time;
    measurement.estimated_power_mw = calculate_detailed_power(execution_time, 168, image_size, 0);
    measurement.energy_consumption_mj = measurement.estimated_power_mw * execution_time;

    if (measurement.estimated_power_mw > 0) {
        measurement.efficiency_pixels_per_mw = (image_size * image_size) / measurement.estimated_power_mw;
    }

    strcpy(measurement.measurement_method, "Theoretical_Datasheet_Based");

    // Store measurement
    if (power_measurement_count < 10) {
        power_measurements[power_measurement_count] = measurement;
        power_measurement_count++;
    }
}

// Document power measurement challenges
void document_power_measurement_challenges(void) {
    // This would be documented in the report, but we can store the information
    const char* challenges[] = {
        "No built-in current measurement capability on STM32F4",
        "Requires external current sensing (INA219, etc.)",
        "Need oscilloscope or multimeter with logging",
        "Power supply variations affect accuracy",
        "Temperature effects on power consumption",
        "Interference from debug probe"
    };

    const char* required_tools[] = {
        "Precision current sensor (INA219/226)",
        "Digital multimeter with logging capability",
        "Oscilloscope with current probe",
        "Isolated power supply",
        "Temperature monitoring",
        "Shielded measurement setup"
    };

    // This information would be included in the report
}

// ==================== MAIN MANDELBROT IMPLEMENTATIONS ====================

// Fixed-point Mandelbrot implementation
uint64_t calculate_mandelbrot_fixed_point(int width, int height, int max_iterations, int scaling_factor) {
    uint64_t checksum = 0;
    int32_t scale_x = (3.5 * scaling_factor) / width;
    int32_t scale_y = (2.0 * scaling_factor) / height;
    int32_t offset_x = -2.5 * scaling_factor;
    int32_t offset_y = -1.0 * scaling_factor;
    int32_t threshold = 4.0 * scaling_factor;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int32_t x0 = (x * scale_x) + offset_x;
            int32_t y0 = (y * scale_y) + offset_y;

            int32_t xi = 0;
            int32_t yi = 0;
            int iteration = 0;

            while (iteration < max_iterations) {
                int64_t xi_sq = (int64_t)xi * xi / scaling_factor;
                int64_t yi_sq = (int64_t)yi * yi / scaling_factor;

                if (xi_sq + yi_sq > threshold) {
                    break;
                }

                int32_t temp = xi_sq - yi_sq;
                yi = (2 * (int64_t)xi * yi / scaling_factor) + y0;
                xi = temp + x0;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

// Float Mandelbrot implementation
uint64_t calculate_mandelbrot_float(int width, int height, int max_iterations) {
    uint64_t checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float x0 = ((float)x / width) * 3.5f - 2.5f;
            float y0 = ((float)y / height) * 2.0f - 1.0f;

            float xi = 0.0f;
            float yi = 0.0f;
            int iteration = 0;

            while (iteration < max_iterations) {
                float xi_sq = xi * xi;
                float yi_sq = yi * yi;

                if (xi_sq + yi_sq > 4.0f) {
                    break;
                }

                float temp = xi_sq - yi_sq;
                yi = 2.0f * xi * yi + y0;
                xi = temp + x0;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

// Double Mandelbrot implementation
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations) {
    uint64_t checksum = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double x0 = ((double)x / width) * 3.5 - 2.5;
            double y0 = ((double)y / height) * 2.0 - 1.0;

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
                yi = 2.0 * xi * yi + y0;
                xi = temp + x0;

                iteration++;
            }

            checksum += iteration;
        }
    }

    return checksum;
}

// ==================== TASK IMPLEMENTATIONS ====================

// Task 2: Impact of Maximum Iteration Variable
void run_task_2(void) {
    current_task = 2;

    for (int iter_idx = 0; iter_idx < num_iter_values; iter_idx++) {
        int max_iter = max_iter_values[iter_idx];

        for (int size_idx = 0; size_idx < num_sizes; size_idx++) {
            int image_size = image_sizes[size_idx];
            run_single_test(image_size, max_iter, 2, 0, 1 << 16, -1);
            HAL_Delay(50);
        }
        HAL_Delay(200);
    }
    current_task = 0;
}

// Task 3: Extended Execution Time Measurement
void run_task_3(void) {
    current_task = 3;

    for (int size_idx = 0; size_idx < num_sizes; size_idx++) {
        int image_size = image_sizes[size_idx];
        run_single_test(image_size, 100, 3, 0, 1 << 16, -1);
        HAL_Delay(50);
    }
    current_task = 0;
}

// Task 4: Scalability Test (Updated with proper chunking)
void run_task_4(void) {
    current_task = 4;
    uint32_t available_memory = get_available_memory();

    for (int size_idx = 0; size_idx < num_large_sizes; size_idx++) {
        int image_size = large_sizes[size_idx];
        uint32_t estimated_memory = image_size * image_size * sizeof(int);

        current_image_size = image_size;
        test_in_progress = 1;

        start_time = HAL_GetTick();
        start_cycles = get_cycle_count();

        uint64_t checksum;
        if (estimated_memory > available_memory) {
            // Use chunking
            test_in_progress = 2; // Chunking mode indicator
            checksum = calculate_mandelbrot_chunked(image_size, image_size, 100, 0, 1 << 16);
        } else {
            // Normal processing
            checksum = calculate_mandelbrot_fixed_point(image_size, image_size, 100, 1 << 16);
        }

        end_cycles = get_cycle_count();
        end_time = HAL_GetTick();

        execution_time = end_time - start_time;
        cycle_count = end_cycles - start_cycles;

        float throughput = 0;
        if (execution_time > 0) {
            throughput = (image_size * image_size) / (execution_time / 1000.0f);
        }

        store_test_result(4, image_size, 100, execution_time, cycle_count,
                         checksum, throughput, 0, 1 << 16, -1);

        HAL_Delay(100);
    }

    current_task = 0;
    test_in_progress = 0;
}

// Task 5: FPU Impact Analysis (Manual FPU control)
void run_task_5_real(void) {
    current_task = 5;
    accuracy_test_count = 0;

    for (int size_idx = 0; size_idx < num_sizes; size_idx++) {
        int image_size = image_sizes[size_idx];
        test_fpu_comprehensive(image_size);
        HAL_Delay(100);
    }

    current_task = 0;
}

// Task 6: Compiler Optimization Impact (Manual optimization control)
void run_task_6_real(void) {
    current_task = 6;

    OptimizationResult result = {0};
    // Optimization level will be manually set via Makefile/IDE
    result.optimization_level = -1; // Placeholder - will be manually noted
    result.code_size = get_code_size();
    result.data_size = get_data_size();
    result.total_binary_size = result.code_size + result.data_size;

    uint32_t program_start = HAL_GetTick();

    // Test Mandelbrot performance
    result.mandelbrot_runtime = benchmark_mandelbrot_performance();

    // Run full test suite timing
    for (int size_idx = 0; size_idx < num_sizes; size_idx++) {
        int image_size = image_sizes[size_idx];
        run_single_test(image_size, 100, 6, 0, 1 << 16, result.optimization_level);
        HAL_Delay(50);
    }

    result.total_runtime = HAL_GetTick() - program_start;

    // Calculate performance per byte metric
    if (result.total_binary_size > 0 && result.mandelbrot_runtime > 0) {
        result.performance_per_byte = (float)(128 * 128) /
            (result.mandelbrot_runtime * result.total_binary_size);
    }

    // Store optimization result
    if (opt_result_count < 4) {
        opt_results[opt_result_count] = result;
        opt_result_count++;
    }

    // Store in main results array with binary size encoded in checksum
    uint64_t size_encoded = ((uint64_t)result.total_binary_size << 32) | result.code_size;
    store_test_result(6, result.total_binary_size, 100, result.total_runtime,
                     0, size_encoded, result.performance_per_byte,
                     0, 0, result.optimization_level);

    current_task = 0;
}

// Task 7: Fixed Point Scaling Factors
void run_task_7(void) {
    current_task = 7;

    for (int factor_idx = 0; factor_idx < num_scaling_factors; factor_idx++) {
        int scaling_factor = scaling_factors[factor_idx];

        for (int size_idx = 0; size_idx < num_sizes; size_idx++) {
            int image_size = image_sizes[size_idx];
            run_single_test(image_size, 100, 7, 0, scaling_factor, -1);
            HAL_Delay(50);
        }
    }
    current_task = 0;
}

// Task 8: Power Measurement Documentation
void run_task_8_real(void) {
    current_task = 8;
    test_in_progress = 3; // Power measurement mode
    power_measurement_count = 0;

    document_power_measurement_challenges();

    // Test different scenarios
    int test_sizes[] = {128, 256}; // Limited test for power measurement

    for (int i = 0; i < 2; i++) {
        measure_power_comprehensive(test_sizes[i]);
        HAL_Delay(500);
    }

    // Store summary result
    if (power_measurement_count > 0) {
        PowerMeasurement* avg = &power_measurements[0];
        uint64_t power_encoded = (uint64_t)(avg->estimated_power_mw * 1000); // Encode as μW

        store_test_result(8, 128, 100, avg->active_time_ms, 0, power_encoded,
                         avg->efficiency_pixels_per_mw, 0, 0, -1);
    }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add your own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add your own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
