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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>  // ✅ Required for bool, true, false

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PWM speed constants

#define OBSTACLE_STOP_DISTANCE_CM      5.0f
#define OBSTACLE_WARNING_DISTANCE_CM   15.0f

#define OBSTACLE_FORWARD_MIN_ANGLE     75
#define OBSTACLE_FORWARD_MAX_ANGLE     105
#define BASE_SPEED 600
#define MAX_SPEED 1000
#define MIN_SPEED 0
#define PID_SAMPLE_TIME 0.01f
#define INTEGRAL_LIMIT 100.0f
#define OBSTACLE_SLOW_DISTANCE_CM      10.0f
#define OBSTACLE_SLOW_SPEED            250
#define OBSTACLE_SLOW_EVENT   (1U << 1)
typedef struct
{
    uint16_t left_speed;
    uint16_t right_speed;
    bool left_forward;
    bool right_forward;
    bool stop;
} MotorCommand_t;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct
{
    float distance_cm;
    int16_t angle_deg;
    bool valid;
} ObstacleMeasurement_t;

typedef enum {
    LINE_NONE,
    LINE_LEFT,
    LINE_CENTER,
    LINE_RIGHT // Remember the last line direction
} LineSensor_State;


#define OBSTACLE_EVENT  (1U << 0)
// Constants (tune these!)
float Kp = 46.7f;
float Ki = 0.7f;
float Kd = 15.6f;

float integral = 0.0f;
float previous_error = 0.0f;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim16;

/* Definitions for UltrasonicTrig */
osThreadId_t UltrasonicTrigHandle;
const osThreadAttr_t UltrasonicTrig_attributes = {
  .name = "UltrasonicTrig",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for LineSensorTask */
osThreadId_t LineSensorTaskHandle;
const osThreadAttr_t LineSensorTask_attributes = {
  .name = "LineSensorTask",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 128 * 4
};
/* Definitions for ObstacleTask */
osThreadId_t ObstacleTaskHandle;
const osThreadAttr_t ObstacleTask_attributes = {
  .name = "ObstacleTask",
  .priority = (osPriority_t) osPriorityAboveNormal1,
  .stack_size = 128 * 4
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

osMessageQueueId_t LineQueueHandle;
osMessageQueueId_t MotorQueueHandle;
osMessageQueueId_t ObstacleQueueHandle;
osEventFlagsId_t ControlEventHandle;

LineSensor_State lastKnown = LINE_CENTER;

volatile uint32_t echo_start = 0;
volatile uint32_t echo_end = 0;
volatile uint32_t echo_time_us = 0;

volatile bool echo_capturing = false;
volatile bool echo_ready = false;

volatile uint16_t ultrasonic_angle_deg = 90;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM16_Init(void);
void StartUltrasonic_Trigger(void *argument);
void StartMotorTask(void *argument);
void StartLineSensorTask(void *argument);
void StartObstacleTask(void *argument);
void StartControlTask(void *argument);

/* USER CODE BEGIN PFP */
void EnablePins_Init(void);
void MotorControl_Init(void);
void MotorControl_Set(uint16_t m1_speed, bool m1_forward, uint16_t m2_speed, bool m2_forward);
void MotorControl_Stop(void);
LineSensor_State LineSensor_GetState(void);
bool ObstacleSensor_Read(void);
MotorCommand_t PID_LineFollow_Update(int8_t error);
void StartControlTask(void *argument);
void PID_Reset(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

static void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000U);

    while ((DWT->CYCCNT - start) < cycles)
    {
    }
}


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
  DWT_Delay_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  MotorControl_Init();
  EnablePins_Init();
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  LineQueueHandle = osMessageQueueNew(1, sizeof(int8_t), NULL);

  MotorQueueHandle = osMessageQueueNew(1, sizeof(MotorCommand_t), NULL);
  ObstacleQueueHandle =
      osMessageQueueNew(3, sizeof(ObstacleMeasurement_t), NULL);

  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UltrasonicTrig */
  UltrasonicTrigHandle = osThreadNew(StartUltrasonic_Trigger, NULL, &UltrasonicTrig_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of LineSensorTask */
  LineSensorTaskHandle = osThreadNew(StartLineSensorTask, NULL, &LineSensorTask_attributes);

  /* creation of ObstacleTask */
  ObstacleTaskHandle = osThreadNew(StartObstacleTask, NULL, &ObstacleTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

      ControlEventHandle = osEventFlagsNew(NULL);

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // 1. Obstacle Check

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 169;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 169;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 169;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 19999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EnablePins_Init(void)
{
    // Enable pins should be set as OUTPUT in CubeMX
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // Enable Motor 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);   // Enable Motor 2
}

void MotorControl_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // PA0 (M1 Forward)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // PA1 (M1 Reverse)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // PB5 (M2 Forward)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // PB4 (M2 Reverse)
}



// Set motor speeds and direction
// m1_speed and m2_speed: 0 (stop) to max PWM (e.g., 1000)
// m1_forward and m2_forward: true for forward, false for reverse
void MotorControl_Set(uint16_t m1_speed, bool m1_forward, uint16_t m2_speed, bool m2_forward)
{
    // Motor 1 (PA0 = Forward, PA1 = Reverse)
    if (m1_forward) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, m1_speed); // Forward PWM
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);        // Reverse OFF
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, m1_speed); // Reverse PWM
    }

    // Motor 2 (PB5 = Forward, PB4 = Reverse)
    if (m2_forward) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, m2_speed); // Forward PWM
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);        // Reverse OFF
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, m2_speed); // Reverse PWM
    }
}



// Stop both motors
void MotorControl_Stop(void)
{
    // All PWM OFF
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

LineSensor_State LineSensor_GetState(void)
{
    GPIO_PinState left   = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
    GPIO_PinState center = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
    GPIO_PinState right  = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);

    if (center == GPIO_PIN_RESET &&
        left == GPIO_PIN_SET &&
        right == GPIO_PIN_SET)
    {
        return LINE_CENTER;
    }
    else if (left == GPIO_PIN_RESET)
    {
        return LINE_LEFT;
    }
    else if (right == GPIO_PIN_RESET)
    {
        return LINE_RIGHT;
    }
    else
    {
        return LINE_NONE;
    }
}

MotorCommand_t PID_LineFollow_Update(int8_t error)
{

    float dt = PID_SAMPLE_TIME;

    integral += error * dt;

    if (integral > INTEGRAL_LIMIT)
        integral = INTEGRAL_LIMIT;

    if (integral < -INTEGRAL_LIMIT)
        integral = -INTEGRAL_LIMIT;

    float derivative = (error - previous_error) / dt;

    float correction =
        Kp * error +
        Ki * integral +
        Kd * derivative;

    previous_error = error;


    int left_speed  = BASE_SPEED + (int)correction;
    int right_speed = BASE_SPEED - (int)correction;

    if (left_speed > MAX_SPEED)
        left_speed = MAX_SPEED;

    if (left_speed < MIN_SPEED)
        left_speed = MIN_SPEED;

    if (right_speed > MAX_SPEED)
        right_speed = MAX_SPEED;

    if (right_speed < MIN_SPEED)
        right_speed = MIN_SPEED;

    MotorCommand_t command;

    command.left_speed = left_speed;
    command.right_speed = right_speed;
    command.left_forward = true;
    command.right_forward = true;
    command.stop = false;

    return command;
}

void PID_Reset(void)
{
    integral = 0.0f;
    previous_error = 0.0f;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 &&
        htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t capture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (!echo_capturing)
        {
            // Rising edge: Echo started
            echo_start = capture;
            echo_capturing = true;

            // Next capture should be falling edge
            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                TIM_CHANNEL_1,
                TIM_INPUTCHANNELPOLARITY_FALLING
            );
        }
        else
        {
            // Falling edge: Echo ended
            echo_end = capture;

            if (echo_end >= echo_start)
            {
                echo_time_us = echo_end - echo_start;
            }
            else
            {
                // Timer overflow
                echo_time_us =
                    (65536U - echo_start) + echo_end;
            }

            echo_capturing = false;
            echo_ready = true;

            // Next capture should be rising edge
            __HAL_TIM_SET_CAPTUREPOLARITY(
                htim,
                TIM_CHANNEL_1,
                TIM_INPUTCHANNELPOLARITY_RISING
            );
        }
    }
}
void Ultrasonic_Trigger(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

    DWT_Delay_us(10);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
}
float Ultrasonic_GetDistanceCm(void)
{
    if (!echo_ready)
        return -1.0f;

    echo_ready = false;

    /* Speed of sound: approximately 58 us per cm round trip */
    float distance_cm = (float)echo_time_us / 58.0f;

    return distance_cm;
}

void Servo_SetAngle(uint16_t angle)
{
    if (angle > 180)
        angle = 180;

    uint16_t pulse = 500 + ((angle * 2000) / 180);

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pulse);

    ultrasonic_angle_deg = angle;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUltrasonic_Trigger */
/**
* @brief Function implementing the UltrasonicTrigg thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUltrasonic_Trigger */
void StartUltrasonic_Trigger(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	const uint16_t angles[] = {60, 75, 90, 105, 120};
	    const uint8_t num_angles = sizeof(angles) / sizeof(angles[0]);

	    for (;;)
	    {
	        for (uint8_t i = 0; i < num_angles; i++)
	        {
	            Servo_SetAngle(angles[i]);

	            // Allow servo to move before taking measurement
	            osDelay(100);

	            Ultrasonic_Trigger();

	            // Wait for echo measurement
	            osDelay(60);
       }
  /* USER CODE END 5 */
    }
}
/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
  /* Infinite loop */
	MotorCommand_t command;

	    for (;;)
	    {
	        if (osMessageQueueGet(MotorQueueHandle,
	                              &command,
	                              NULL,
	                              osWaitForever) == osOK)
	        {
	            if (command.stop)
	            {
	                MotorControl_Stop();
	            }
	            else
	            {
	                MotorControl_Set(command.left_speed,
	                                 command.left_forward,
	                                 command.right_speed,
	                                 command.right_forward);
                }
  /* USER CODE END StartMotorTask */
          }
	    }
}
/* USER CODE BEGIN Header_StartLineSensorTask */
/**
* @brief Function implementing the LineSensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLineSensorTask */
void StartLineSensorTask(void *argument)
{
  /* USER CODE BEGIN StartLineSensorTask */
  /* Infinite loop */
	LineSensor_State state;
	    int8_t error;

	    for (;;)
	    {
	        state = LineSensor_GetState();   // Normal function called by RTOS task

	        switch (state)
	        {
	                    case LINE_CENTER:
	                        error = 0;
	                        lastKnown = LINE_CENTER;
	                        break;

	                    case LINE_LEFT:
	                        error = -1;
	                        lastKnown = LINE_LEFT;
	                        break;

	                    case LINE_RIGHT:
	                        error = 1;
	                        lastKnown = LINE_RIGHT;
	                        break;

	                     case LINE_NONE:
	                     default:
	                        if (lastKnown == LINE_LEFT)
	                                        error = -2;
	                        else if (lastKnown == LINE_RIGHT)
	                                        error = 2;
	                        else
	                                        error = 0;
	                        break;
	        }

	        osMessageQueuePut(LineQueueHandle, &error, 0, osWaitForever);

	        osDelay(10);

  }
  /* USER CODE END StartLineSensorTask */
}

/* USER CODE BEGIN Header_StartObstacleTask */
/**
* @brief Function implementing the ObstacleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartObstacleTask */
void StartObstacleTask(void *argument)
{
  /* USER CODE BEGIN StartObstacleTask */
  /* Infinite loop */
	ObstacleMeasurement_t measurement;

	    for (;;)
	    {
	        float distance = Ultrasonic_GetDistanceCm();

	        if (distance > 0.0f)
	        {
	            measurement.distance_cm = distance;
	            measurement.angle_deg = ultrasonic_angle_deg;
	            measurement.valid = true;

	            /* Keep measurements available for diagnostics */
	            osMessageQueuePut(ObstacleQueueHandle,
	                              &measurement,
	                              0,
	                              0);

	            /*
	             * Only the forward sector can influence vehicle motion.
	             */
	            if (measurement.angle_deg >= OBSTACLE_FORWARD_MIN_ANGLE &&
	                measurement.angle_deg <= OBSTACLE_FORWARD_MAX_ANGLE)
	            {
	                /*
	                 * CRITICAL STOP
	                 *
	                 * Once <= 5 cm is detected, keep STOP active.
	                 * Do not clear it from a later sweep measurement.
	                 */
	                if (distance <= OBSTACLE_STOP_DISTANCE_CM)
	                {
	                    osEventFlagsSet(ControlEventHandle,
	                                    OBSTACLE_EVENT);

	                    osEventFlagsClear(ControlEventHandle,
	                                      OBSTACLE_SLOW_EVENT);
	                }

	                /*
	                 * Slow-down zone: 5-10 cm
	                 */
	                else if (distance <= OBSTACLE_SLOW_DISTANCE_CM)
	                {
	                    /*
	                     * Do not overwrite an already active STOP.
	                     */
	                    if (!(osEventFlagsGet(ControlEventHandle) &
	                          OBSTACLE_EVENT))
	                    {
	                        osEventFlagsSet(ControlEventHandle,
	                                        OBSTACLE_SLOW_EVENT);
	                    }
	                }

	                /*
	                 * Obstacle is farther than slow-down zone.
	                 *
	                 * STOP is intentionally NOT cleared.
	                 */
	                else
	                {
	                    if (!(osEventFlagsGet(ControlEventHandle) &
	                          OBSTACLE_EVENT))
	                    {
	                        osEventFlagsClear(ControlEventHandle,
	                                          OBSTACLE_SLOW_EVENT);
	                    }
	                }
	            }
	        }

	        osDelay(10);

	    }
  /* USER CODE END StartObstacleTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
	int8_t error;
	    MotorCommand_t command;

	    for (;;)
	    {
	        if (osMessageQueueGet(LineQueueHandle,
	                              &error,
	                              NULL,
	                              osWaitForever) == osOK)
	        {
	            uint32_t flags = osEventFlagsGet(ControlEventHandle);

	            /*
	             * ==========================================
	             * PRIORITY 1: OBSTACLE STOP
	             * ==========================================
	             */
	            if (flags & OBSTACLE_EVENT)
	            {
	                PID_Reset();

	                command.left_speed = 0;
	                command.right_speed = 0;

	                command.left_forward = true;
	                command.right_forward = true;

	                command.stop = true;
	            }

	            /*
	             * ==========================================
	             * PRIORITY 2: OBSTACLE SLOW
	             * ==========================================
	             */
	            else if (flags & OBSTACLE_SLOW_EVENT)
	            {
	                /*
	                 * Continue following the line, but at
	                 * reduced speed.
	                 */
	                command = PID_LineFollow_Update(error);

	                if (command.left_speed > OBSTACLE_SLOW_SPEED)
	                    command.left_speed = OBSTACLE_SLOW_SPEED;

	                if (command.right_speed > OBSTACLE_SLOW_SPEED)
	                    command.right_speed = OBSTACLE_SLOW_SPEED;

	                command.stop = false;
	            }

	            /*
	             * ==========================================
	             * PRIORITY 3: LINE LOST - LEFT
	             * ==========================================
	             */
	            else if (error == -2)
	            {
	                PID_Reset();

	                command.left_speed = 300;
	                command.right_speed = 500;

	                command.left_forward = true;
	                command.right_forward = true;

	                command.stop = false;
	            }

	            /*
	             * ==========================================
	             * PRIORITY 4: LINE LOST - RIGHT
	             * ==========================================
	             */
	            else if (error == 2)
	            {
	                PID_Reset();

	                command.left_speed = 500;
	                command.right_speed = 300;

	                command.left_forward = true;
	                command.right_forward = true;

	                command.stop = false;
	            }

	            /*
	             * ==========================================
	             * PRIORITY 5: NORMAL LINE FOLLOWING
	             * ==========================================
	             */
	            else
	            {
	                command = PID_LineFollow_Update(error);
	            }

	            osMessageQueuePut(MotorQueueHandle,
	                              &command,
	                              0,
	                              osWaitForever);
	        }
  }
  /* USER CODE END StartControlTask */
}

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

  /* USER CODE END Error_Handler_Debug */
     }
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
