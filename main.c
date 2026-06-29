/* USER CODE BEGIN Header */
#include "main.h"
#include "stm32_traffic_controller.h"
#include "stm32_7segment.h"
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
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define DEBOUNCE_THRESHOLD 3

static traffic_controller_state_t g_controller_state;
static traffic_signal_outputs_t g_signal_outputs;

/* Extern the MLP function from your stm32_mlp_inference.c file */
extern float traffic_mlp_predict_seconds(const float input[5]);

/* Robust Debouncing for IR Sensors */
static float read_lane_density(GPIO_TypeDef *port, uint16_t pin) {
    static uint8_t db[4] = {0};
    uint8_t idx = 0;

    if (pin == GPIO_PIN_0 && port == GPIOB) idx = 1;
    else if (pin == GPIO_PIN_1 && port == GPIOB) idx = 2;
    else if (pin == GPIO_PIN_4 && port == GPIOB) idx = 3;

    // Active-low logic: LOW (RESET) means vehicle detected
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) {
        if (db[idx] < DEBOUNCE_THRESHOLD) db[idx]++;
    } else {
        if (db[idx] > 0) db[idx]--;
    }

    return (db[idx] >= DEBOUNCE_THRESHOLD) ? 1.0f : 0.0f;
}

/* Atomic Bare-Metal Output Control */
static void apply_signal_outputs(const traffic_signal_outputs_t *out) {
    GPIOA->BSRR = out->lane1_red ? GPIO_PIN_4 : (uint32_t)GPIO_PIN_4 << 16;
    GPIOA->BSRR = out->lane1_green ? GPIO_PIN_6 : (uint32_t)GPIO_PIN_6 << 16;

    GPIOA->BSRR = out->lane2_red ? GPIO_PIN_7 : (uint32_t)GPIO_PIN_7 << 16;
    GPIOB->BSRR = out->lane2_green ? GPIO_PIN_5 : (uint32_t)GPIO_PIN_5 << 16;

    GPIOC->BSRR = out->lane3_red ? GPIO_PIN_1 : (uint32_t)GPIO_PIN_1 << 16;
    GPIOC->BSRR = out->lane3_green ? GPIO_PIN_2 : (uint32_t)GPIO_PIN_2 << 16;

    GPIOC->BSRR = out->lane4_red ? GPIO_PIN_3 : (uint32_t)GPIO_PIN_3 << 16;
    GPIOA->BSRR = out->lane4_green ? GPIO_PIN_8 : (uint32_t)GPIO_PIN_8 << 16; // PA8

    uint8_t any_yellow_active = out->lane1_yellow || out->lane2_yellow || out->lane3_yellow || out->lane4_yellow;

    GPIOC->BSRR = any_yellow_active ? GPIO_PIN_5 : (uint32_t)GPIO_PIN_5 << 16;  
    GPIOA->BSRR = any_yellow_active ? GPIO_PIN_1 : (uint32_t)GPIO_PIN_1 << 16;  
    GPIOB->BSRR = any_yellow_active ? GPIO_PIN_13 : (uint32_t)GPIO_PIN_13 << 16; 
    GPIOA->BSRR = any_yellow_active ? GPIO_PIN_9 : (uint32_t)GPIO_PIN_9 << 16;  // PA9
}

/* --- THE MASTER LOGIC: TINYMLP + HYBRID EMA MEMORY + SAFETY CLAMP --- */
static void traffic_control_tick(void) {
    // 1. Read all 4 IR Sensors
    float l1 = read_lane_density(GPIOA, GPIO_PIN_0);
    float l2 = read_lane_density(GPIOB, GPIO_PIN_0);
    float l3 = read_lane_density(GPIOB, GPIO_PIN_1);
    float l4 = read_lane_density(GPIOB, GPIO_PIN_4);

    static uint8_t active_lane = 1; 
    static uint8_t phase = 2;       // 0 = GREEN, 1 = YELLOW, 2 = ALL RED
    static float timer_seconds = 0.0f;
    static float mlp_green_target = 5.0f; 
    
    // Memory Arrays
    static uint32_t waiting_time[5] = {0, 0, 0, 0, 0}; // Short-term queue memory
    static float historical_priority[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Long-term EMA memory

    // Advance timer by 100ms (0.1s)
    timer_seconds += 0.1f;
    
    // 2. Update Short-Term Waiting Times
    uint8_t car_detected[5] = {0, l1 > 0.5f, l2 > 0.5f, l3 > 0.5f, l4 > 0.5f};
    for(int i = 1; i <= 4; i++) {
        if (car_detected[i]) {
            if (phase == 0 && active_lane == i) {
                waiting_time[i] = 0; // Not waiting if currently green
            } else {
                waiting_time[i]++; // Add 1 tick to short-term waiting score
            }
        } else {
            waiting_time[i] = 0; // Car left, clear short-term memory
        }
    }

    // 3. Calculate Hybrid Demand Scores & Update EMA History
    float demand[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
    for(int i = 1; i <= 4; i++) {
        // Step A: Update the Long-Term History using Exponential Decay
        if (car_detected[i]) {
            // Build up priority smoothly if a car is present
            historical_priority[i] = (historical_priority[i] * 0.999f) + 0.1f; 
        } else {
            // Exponentially decay the priority if the lane is empty
            historical_priority[i] = (historical_priority[i] * 0.999f);
        }
        
        // Clamp the history so it doesn't grow infinitely large
        if (historical_priority[i] > 15.0f) historical_priority[i] = 15.0f;

        // Step B: Calculate the Final Hybrid Demand Score for the AI
        if (phase == 0 && active_lane == i) {
            // GREEN lane: Base score of 5.0 for having a car, plus its long-term importance
            demand[i] = car_detected[i] ? (5.0f + historical_priority[i]) : 0.0f; 
        } else {
            // RED lanes: How long they've been waiting NOW, plus their historical importance
            demand[i] = ((float)waiting_time[i] / 10.0f) + historical_priority[i]; 
        }
    }

    // 4. Feed the Hybrid Demand Scores + Elapsed Time into the TinyMLP Inference Engine
    float mlp_inputs[5] = {demand[1], demand[2], demand[3], demand[4], timer_seconds};
    float raw_mlp_prediction = traffic_mlp_predict_seconds(mlp_inputs);

    // 5. Layer 1 Safety Override Constraint (Clamp between 5s and 30s)
    if (raw_mlp_prediction < 5.0f) mlp_green_target = 5.0f;
    else if (raw_mlp_prediction > 30.0f) mlp_green_target = 30.0f;
    else mlp_green_target = raw_mlp_prediction;

    // Default: Wipe the board, set everything to RED
    g_signal_outputs.lane1_red = 1; g_signal_outputs.lane1_green = 0; g_signal_outputs.lane1_yellow = 0;
    g_signal_outputs.lane2_red = 1; g_signal_outputs.lane2_green = 0; g_signal_outputs.lane2_yellow = 0;
    g_signal_outputs.lane3_red = 1; g_signal_outputs.lane3_green = 0; g_signal_outputs.lane3_yellow = 0;
    g_signal_outputs.lane4_red = 1; g_signal_outputs.lane4_green = 0; g_signal_outputs.lane4_yellow = 0;

    // 6. Phase State Machine Logic
    if (phase == 2) { 
        // State 2: All Red, waiting for a car
        if (l1) active_lane = 1;
        else if (l2) active_lane = 2;
        else if (l3) active_lane = 3;
        else if (l4) active_lane = 4;

        if (l1 || l2 || l3 || l4) {
            phase = 0; // Switch to Green
            timer_seconds = 0.0f;
            waiting_time[active_lane] = 0;
        }
    } 
    else if (phase == 0) { 
        // State 0: Green Phase (Lasts exactly as long as the MLP predicts!)
        if (timer_seconds >= mlp_green_target) { 
            phase = 1; // Switch to Yellow
            timer_seconds = 0.0f;
        }
    } 
    else if (phase == 1) { 
        // State 1: Yellow Phase (3 seconds)
        if (timer_seconds >= 3.0f) { 
            uint8_t next_lane = 0;
            uint32_t max_wait = 0;
            
            // FCFS SELECTION: Find the lane that has been waiting the longest NOW
            for(int i = 1; i <= 4; i++) {
                if (waiting_time[i] > max_wait) {
                    max_wait = waiting_time[i];
                    next_lane = i;
                }
            }
            
            if (next_lane != 0) {
                active_lane = next_lane;
                phase = 0; // Go to green for the longest-waiting lane
                waiting_time[active_lane] = 0; // Reset its wait time
            } else {
                phase = 2; // Back to ALL RED
            }
            
            timer_seconds = 0.0f;
        }
    }

    // 7. Apply phase outputs
    if (phase == 0) {
        if (active_lane == 1) { g_signal_outputs.lane1_green = 1; g_signal_outputs.lane1_red = 0; }
        if (active_lane == 2) { g_signal_outputs.lane2_green = 1; g_signal_outputs.lane2_red = 0; }
        if (active_lane == 3) { g_signal_outputs.lane3_green = 1; g_signal_outputs.lane3_red = 0; }
        if (active_lane == 4) { g_signal_outputs.lane4_green = 1; g_signal_outputs.lane4_red = 0; }
    } else if (phase == 1) {
        g_signal_outputs.lane1_yellow = 1;
        g_signal_outputs.lane2_yellow = 1;
        g_signal_outputs.lane3_yellow = 1;
        g_signal_outputs.lane4_yellow = 1;
    }

    // 8. Update 7-Segment Display Countdown (Dynamically reflects AI prediction)
    int remaining = 0;
    if (phase == 0) {
        remaining = (int)(mlp_green_target - timer_seconds);
        if (remaining < 1) remaining = 1; 
    } else {
        remaining = 0; 
    }
    
    Display_SetTime(remaining);
    apply_signal_outputs(&g_signal_outputs);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  traffic_controller_init(&g_controller_state);

  // Start Hardware Timers
  HAL_TIM_Base_Start_IT(&htim2); // 100ms Traffic Logic
  HAL_TIM_Base_Start_IT(&htim6); // 2ms Screen Multiplexer
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
      // Keep Watchdog from resetting the board
      HAL_IWDG_Refresh(&hiwdg);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Set Outputs to LOW */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /* Configure B1 Button */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIOC */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure IR Sensor 1 (PA0) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Initialize GPIOA (Includes PA8/PA9 for Lane 4 Green/Yellow) */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure IR Sensors 2, 3, 4 (PB0, PB1, PB4) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIOB Outputs */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIOD Output */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* Master Interrupt Router */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        traffic_control_tick();
    } else if (htim->Instance == TIM6) {
        Display_Multiplex_ISR();
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}