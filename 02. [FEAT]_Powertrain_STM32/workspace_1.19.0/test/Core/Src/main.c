/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FreeRTOS, motor OK)
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  * This software is licensed as-is.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  EVT_BTN_DOWN = 1,
  EVT_BTN_UP,
  EVT_GEAR_EDGE
} evt_type_t;

typedef struct {
  evt_type_t type;
  uint32_t   tick;
  uint16_t   pin;
  uint8_t    level; // 0=Low, 1=High
} evt_t;

typedef enum { GEAR_NEUTRAL = 0, GEAR_FWD = +1, GEAR_BACK = -1 } gear_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX              900    // ARR=999 기준 90%
#define START_DUTY           200
#define PWM_STEP             8
#define STEP_PERIOD_MS       35

#define DEBOUNCE_MS_PRESS    60
#define DEBOUNCE_MS_RELEASE  40
#define RELEASE_POLL_MS      10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static QueueHandle_t  qEvt;          // 단일 이벤트 큐
static TimerHandle_t  tBtnRelease;   // 버튼 해제 감시 타이머(one-shot)
static SemaphoreHandle_t xMotorMutex; // 공유 자원 보호용 뮤텍스

static uint8_t  g_pressed      = 0;   // 버튼 눌림 상태
static int16_t  g_pwm          = 0;   // 현재 듀티 (자료형 변경: uint16_t -> int16_t)
static uint32_t g_last_step_ms = 0;   // 램프업/다운 타이밍
static gear_t   g_gear         = GEAR_NEUTRAL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static inline void Force_NoRemap_TIM3(void);
static inline void Motor_SetDuty(uint16_t d);
static inline void Motor_Fwd(void);
static inline void Motor_Bck(void);
static inline void Motor_Stop(void);
static inline gear_t ReadGear_HoldPrev(gear_t prev);

static void BtnReleaseTimerCb(TimerHandle_t xTimer);
static void ControlTask(void *arg);
static void MotorTask(void *arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void Force_NoRemap_TIM3(void){
  __HAL_RCC_AFIO_CLK_ENABLE();
  AFIO->MAPR &= ~(AFIO_MAPR_TIM3_REMAP); // 00 = No Remap → CH3=PB0
}

/* ENA=PWM(TIM3_CH3), IN1/IN2=방향 GPIO */
static inline void Motor_SetDuty(uint16_t d){
  if (d > PWM_MAX) d = PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, d);
  g_pwm = d;
}
static inline void Motor_Fwd(void){  // IN1=1, IN2=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
static inline void Motor_Bck(void){  // IN1=0, IN2=1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}
static inline void Motor_Stop(void){ // 프리휠 + 듀티 0
  Motor_SetDuty(0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_11, GPIO_PIN_RESET);
}

/* 기어 입력: PC8=FWD(LOW), PC6=BACK(LOW) */
static inline gear_t ReadGear_HoldPrev(gear_t prev)
{
  GPIO_PinState p8 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8); // FWD (LOW=선택)
  GPIO_PinState p6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // BACK(LOW=선택)

  if (p8 == GPIO_PIN_RESET && p6 == GPIO_PIN_SET)  return GEAR_FWD;
  if (p6 == GPIO_PIN_RESET && p8 == GPIO_PIN_SET)  return GEAR_BACK;
  if (p8 == GPIO_PIN_SET   && p6 == GPIO_PIN_SET)  return GEAR_NEUTRAL;
  return prev;
}

/* 버튼 해제(one-shot) 폴링 타이머 */
static void BtnReleaseTimerCb(TimerHandle_t xTimer) {
  (void)xTimer;
  static uint32_t highStart = 0;
  TickType_t now = xTaskGetTickCount();

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
    if (highStart == 0) highStart = now;
    if ((now - highStart) >= pdMS_TO_TICKS(DEBOUNCE_MS_RELEASE)) {
      evt_t e = { .type = EVT_BTN_UP, .tick = now, .pin = GPIO_PIN_5, .level = 1 };
      xQueueSend(qEvt, &e, 0);
      highStart = 0;
      return; // one-shot
    }
  } else {
    highStart = 0;
  }
  xTimerStart(tBtnRelease, 0);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* TIM3 No-Remap 강제 (CH3=PB0) */
  Force_NoRemap_TIM3();

  /* PB0를 AF_PP로 한 번 더 보증(보드/생성옵션에 따라 안전용) */
  {
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gi.Pin   = GPIO_PIN_0;
    gi.Mode  = GPIO_MODE_AF_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);
  }

  /* 초기 기어 반영 */
  g_gear = ReadGear_HoldPrev(GEAR_NEUTRAL);

  /* PWM 시작(CH3) & 모터 정지 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  Motor_Stop();
  g_last_step_ms = HAL_GetTick();

  /* RTOS 객체 생성 */
  qEvt = xQueueCreate(16, sizeof(evt_t));
  configASSERT(qEvt != NULL);

  tBtnRelease = xTimerCreate("btnRel",
                             pdMS_TO_TICKS(RELEASE_POLL_MS),
                             pdFALSE, NULL, BtnReleaseTimerCb);
  configASSERT(tBtnRelease != NULL);

  // 뮤텍스 생성
  xMotorMutex = xSemaphoreCreateMutex();
  configASSERT(xMotorMutex != NULL);

  /* 태스크 생성 */
  xTaskCreate(ControlTask, "ctrl", 320, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(MotorTask,   "motor",256, NULL, tskIDLE_PRIORITY + 2, NULL);

  /* 스케줄러 시작 */
  vTaskStartScheduler();
  /* USER CODE END 2 */

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* EXTI 콜백: 버튼/기어 이벤트를 큐로 보냄 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  BaseType_t hpw = pdFALSE;
  TickType_t now = xTaskGetTickCountFromISR();

  if (pin == GPIO_PIN_5) {  // 버튼: 눌림(FALLING)
    static TickType_t lastPress = 0;
    if ((now - lastPress) < pdMS_TO_TICKS(DEBOUNCE_MS_PRESS)) return;
    lastPress = now;

    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
      evt_t e = { .type = EVT_BTN_DOWN, .tick = now, .pin = GPIO_PIN_5, .level = 0 };
      xQueueSendFromISR(qEvt, &e, &hpw);
      xTimerStartFromISR(tBtnRelease, &hpw); // 해제 감시 시작
    }
  }
  else if (pin == GPIO_PIN_6 || pin == GPIO_PIN_8) { // 기어 엣지
    evt_t e = { .type = EVT_GEAR_EDGE, .tick = now, .pin = pin,
                .level = (HAL_GPIO_ReadPin(GPIOC, pin)==GPIO_PIN_SET) };
    xQueueSendFromISR(qEvt, &e, &hpw);
  }
  portYIELD_FROM_ISR(hpw);
}

/* 컨트롤 태스크 */
static void ControlTask(void *arg)
{
  (void)arg;
  evt_t e;

  for (;;) {
    if (xQueueReceive(qEvt, &e, portMAX_DELAY) != pdTRUE) continue;

    switch (e.type) {
      case EVT_BTN_DOWN:
        xSemaphoreTake(xMotorMutex, portMAX_DELAY);
        if (!g_pressed) {
          g_pressed = 1;
          g_last_step_ms = HAL_GetTick();
#ifdef LD2_Pin
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
#endif
          if (g_gear != GEAR_NEUTRAL) {
            if (g_gear == GEAR_FWD) Motor_Fwd(); else Motor_Bck();
          }
        }
        xSemaphoreGive(xMotorMutex);
        break;

      case EVT_BTN_UP:
        xSemaphoreTake(xMotorMutex, portMAX_DELAY);
        if (g_pressed) {
          g_pressed = 0;
          g_last_step_ms = HAL_GetTick(); // 타이밍 변수 초기화 추가
#ifdef LD2_Pin
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
#endif
        }
        xSemaphoreGive(xMotorMutex);
        break;

      case EVT_GEAR_EDGE: {
        gear_t newg = ReadGear_HoldPrev(g_gear);
        xSemaphoreTake(xMotorMutex, portMAX_DELAY);
        if (newg != g_gear) {
          g_gear = newg;
          // 주행 중 기어 변경 시, 안전을 위해 모터 정지
          if (g_pressed) {
            g_pressed = 0;
            g_last_step_ms = HAL_GetTick(); // 타이밍 변수 초기화 추가
          }
        }
        xSemaphoreGive(xMotorMutex);
        break;
      }
      default: break;
    }
  }
}

/* 모터 램프업/다운 태스크 */
static void MotorTask(void *arg)
{
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last = xTaskGetTickCount();

  for(;;) {
    xSemaphoreTake(xMotorMutex, portMAX_DELAY);

    // 버튼이 눌려있지 않거나 기어가 중립일 때 -> 감속 또는 정지
    if (!g_pressed || g_gear == GEAR_NEUTRAL) {
        if (g_pwm > 0) {
            uint32_t now = HAL_GetTick();
            if ((now - g_last_step_ms) >= STEP_PERIOD_MS) {
                g_last_step_ms = now;
                g_pwm -= PWM_STEP;
                if (g_pwm < 0) g_pwm = 0;
                Motor_SetDuty(g_pwm);
            }
        } else {
            Motor_Stop();
        }
    }
    // 버튼이 눌려있고 기어가 중립이 아닐 때 -> 가속
    else {
      if (g_gear == GEAR_FWD) Motor_Fwd(); else Motor_Bck();

      uint32_t now = HAL_GetTick();
      if ((now - g_last_step_ms) >= STEP_PERIOD_MS) {
        g_last_step_ms = now;
        if (g_pwm < START_DUTY)      Motor_SetDuty(START_DUTY);
        else if (g_pwm < PWM_MAX)    Motor_SetDuty(g_pwm + PWM_STEP);
      }
    }

    xSemaphoreGive(xMotorMutex);
    vTaskDelayUntil(&last, period);
  }
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
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
