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

/* ===== FreeRTOS ===== */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* ===== Types / Events ===== */
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

/* ===== Peripherals ===== */
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* ===== RTOS objects ===== */
static QueueHandle_t  qEvt;          // 단일 이벤트 큐
static TimerHandle_t  tBtnRelease;   // 버튼 해제 감시 타이머(one-shot)

/* ===== Prototypes ===== */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

/* ===== Parameters ===== */
#define PWM_MAX              900    // ARR=999 기준 90%
#define START_DUTY           200
#define PWM_STEP             8
#define STEP_PERIOD_MS       35

#define DEBOUNCE_MS_PRESS    60
#define DEBOUNCE_MS_RELEASE  40
#define RELEASE_POLL_MS      10

/* ===== State ===== */
static volatile uint8_t  g_pressed      = 0;   // 버튼 눌림 상태
static volatile uint16_t g_pwm          = 0;   // 현재 듀티
static uint32_t          g_last_step_ms = 0;   // 램프업 타이밍

typedef enum { GEAR_NEUTRAL = 0, GEAR_FWD = +1, GEAR_BACK = -1 } gear_t;
static volatile gear_t g_gear = GEAR_NEUTRAL;

/* ===== Utils: TIM3 remap 강제 해제 ===== */
static inline void Force_NoRemap_TIM3(void){
  __HAL_RCC_AFIO_CLK_ENABLE();
  AFIO->MAPR &= ~(AFIO_MAPR_TIM3_REMAP); // 00 = No Remap → CH3=PB0
}

/* ===== Motor utils ===== */
static inline void Motor_SetDuty(uint16_t d){
  if (d > PWM_MAX) d = PWM_MAX;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, d);
  g_pwm = d;
}
static inline void Motor_Fwd(void){  // IN1=1, IN2=0
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}
static inline void Motor_Bck(void){  // IN1=0, IN2=1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}
static inline void Motor_Stop(void){ // 프리휠 + 듀티 0
  Motor_SetDuty(0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET);
}

/* ===== Gear read =====
 * 배선 기준:
 *  - PC8 = FWD  (LOW = 선택)
 *  - PC6 = BACK (LOW = 선택)
 *  - 둘 다 HIGH = 중립
 *  - 둘 다 LOW  = 통과구간/노이즈 → 이전 유지
 */
static inline gear_t ReadGear_HoldPrev(gear_t prev)
{
  GPIO_PinState p8 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8); // FWD (LOW=선택)
  GPIO_PinState p6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // BACK(LOW=선택)

  if (p6 == GPIO_PIN_RESET && p8 == GPIO_PIN_SET)  return GEAR_FWD;
  if (p8 == GPIO_PIN_RESET && p6 == GPIO_PIN_SET)  return GEAR_BACK;
  if (p8 == GPIO_PIN_SET   && p6 == GPIO_PIN_SET)  return GEAR_NEUTRAL;
  return prev;
}

/* ===== Button release timer callback ===== */
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
  // 아직 안정 아님 → 다시 돌림
  xTimerStart(tBtnRelease, 0);
}

/* ===== Tasks ===== */
static void ControlTask(void *arg)
{
  (void)arg;
  evt_t e;

  for (;;) {
    if (xQueueReceive(qEvt, &e, portMAX_DELAY) != pdTRUE) continue;

    switch (e.type) {
      case EVT_BTN_DOWN:
        if (!g_pressed) {
          g_pressed = 1;
          g_last_step_ms = HAL_GetTick();
#ifdef LD2_Pin
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
#endif
          // 기어가 들어가 있으면 스타트 듀티로 킥
          if (g_gear != GEAR_NEUTRAL) {
            if (g_gear == GEAR_FWD) Motor_Fwd(); else Motor_Bck();
            Motor_SetDuty(START_DUTY);
          }
        }
        break;

      case EVT_BTN_UP:
        if (g_pressed) {
          g_pressed = 0;
          Motor_Stop();
#ifdef LD2_Pin
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
#endif
        }
        break;

      case EVT_GEAR_EDGE: {
        gear_t newg = ReadGear_HoldPrev(g_gear);
        if (newg != g_gear) {
          g_gear = newg;
          if (g_gear == GEAR_NEUTRAL) {
            g_pressed = 0;
            Motor_Stop();
          } else {
            // 버튼이 눌린 상태라면 즉시 방향 반영
            if (g_pressed) {
              if (g_gear == GEAR_FWD) Motor_Fwd(); else Motor_Bck();
              if (g_pwm == 0) Motor_SetDuty(START_DUTY);
            }
          }
        }
        break;
      }

      default: break;
    }
  }
}

static void MotorTask(void *arg)
{
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t last = xTaskGetTickCount();

  for(;;) {
    // 엣지 놓쳐도 안전하게 주기 샘플링
    g_gear = ReadGear_HoldPrev(g_gear);

    if (!g_pressed || g_gear == GEAR_NEUTRAL) {
      if (g_pwm != 0) Motor_Stop();
    } else {
      if (g_gear == GEAR_FWD) Motor_Fwd(); else Motor_Bck();

      uint32_t now = HAL_GetTick();
      if ((now - g_last_step_ms) >= STEP_PERIOD_MS) {
        g_last_step_ms = now;
        if (g_pwm < START_DUTY)      Motor_SetDuty(START_DUTY);
        else if (g_pwm < PWM_MAX)    Motor_SetDuty(g_pwm + PWM_STEP);
      }
    }
    vTaskDelayUntil(&last, period);
  }
}

/* ===== main ===== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* ★ TIM3 No-Remap 강제 (CH3=PB0), PB0 AF_PP 재보장 */
  Force_NoRemap_TIM3();
  // PB0 강제 재설정(안전)
  GPIO_InitTypeDef gi = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gi.Pin   = GPIO_PIN_0;
  gi.Mode  = GPIO_MODE_AF_PP;
  gi.Pull  = GPIO_NOPULL;
  gi.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gi);

  /* 부팅 직후 현재 기어 반영 */
  g_gear = ReadGear_HoldPrev(GEAR_NEUTRAL);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  Motor_Stop();
  g_last_step_ms = HAL_GetTick();

  /* RTOS objects */
  qEvt = xQueueCreate(16, sizeof(evt_t));
  configASSERT(qEvt != NULL);

  tBtnRelease = xTimerCreate("btnRel",
                             pdMS_TO_TICKS(RELEASE_POLL_MS),
                             pdFALSE, NULL, BtnReleaseTimerCb);
  configASSERT(tBtnRelease != NULL);

  /* Tasks */
  xTaskCreate(ControlTask, "ctrl", 320, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate(MotorTask,   "motor",256, NULL, tskIDLE_PRIORITY + 2, NULL);

  vTaskStartScheduler(); // 절대 반환 안됨

  while (1) { } // 여길 오면 힙 부족
}

/* ===== TIM3 init ===== */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler         = 71;     // 72MHz/72 = 1MHz
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 999;    // 1kHz PWM
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH; // EN이 Active-Low면 LOW로 바꿔보기
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim3);
}

/* ===== UART2 init ===== */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance        = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/* ===== GPIO init ===== */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* LD2 */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = LD2_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Motor IN1/IN2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Button PC5: 눌림(FALLING)만 */
  GPIO_InitStruct.Pin  = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Gear PC6/PC8: 양엣지 */
  GPIO_InitStruct.Pin  = GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* NVIC (FromISR 안전 우선순위) */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ===== Clock ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ===== EXTI ISR ===== */
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

/* ===== Error ===== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
