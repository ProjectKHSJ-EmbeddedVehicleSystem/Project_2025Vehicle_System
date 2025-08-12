/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body  (CMSIS-OS v1)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "gps.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int16_t ax, ay, az;
  uint32_t ts_ms;
} AccelSample_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COLLISION_THRESHOLD_RAW   30000.0f
#define SENSOR_PERIOD_MS          50U
#define DETECT_SIGNAL_FLAG        (1U << 0)
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart1;   // GPS (9600, RX)
UART_HandleTypeDef huart2;   // PC Putty (115200, TX/RX)

/* RTOS */
osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
static osThreadId sensorTaskHandle;
static osThreadId detectTaskHandle;
static osMutexId  i2cMutexHandle;

static AccelSample_t g_sample;
static uint8_t gps_rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);  // PC
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);  // GPS
void StartDefaultTask(void const * argument);
/* USER CODE BEGIN PFP */
static void SensorTask(void const * argument);
static void DetectTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // GPS는 USART1로 수신
  if (huart->Instance == USART1) {
    GPS_OnByte(gps_rx_byte);
    HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1); // 다음 바이트 계속 수신
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();   // PC 출력(115200)
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();   // GPS 입력(9600)

  /* USER CODE BEGIN 2 */
  // GPS 수신 시작 (USART1)
  GPS_Init();
  HAL_UART_Receive_IT(&huart1, &gps_rx_byte, 1);

  // I2C 뮤텍스
  osMutexDef(I2C_MTX);
  i2cMutexHandle = osMutexCreate(osMutex(I2C_MTX));

  // MPU6050 초기화 (I2C 보호)
  if (i2cMutexHandle) {
    osMutexWait(i2cMutexHandle, 100);
    MPU6050_Init(&hi2c1);
    osMutexRelease(i2cMutexHandle);
  }
  /* USER CODE END 2 */

  /* defaultTask (자동 생성) */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(Sensor, SensorTask, osPriorityAboveNormal, 0, 256);
  sensorTaskHandle = osThreadCreate(osThread(Sensor), NULL);

  osThreadDef(Detect, DetectTask, osPriorityNormal, 0, 256);
  detectTaskHandle = osThreadCreate(osThread(Detect), NULL);
  /* USER CODE END RTOS_THREADS */

  osKernelStart();

  while (1) { }
}

/* ===================== System / Periph Init ===================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;      // F1 계열은 RCC_HSI_ON 매크로 사용
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) { Error_Handler(); }
}

static void MX_USART1_UART_Init(void)   // GPS (9600, RX only)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate   = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits   = UART_STOPBITS_1;
  huart1.Init.Parity     = UART_PARITY_NONE;
  huart1.Init.Mode       = UART_MODE_RX;        // GPS는 RX만
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)   // PC Putty (115200)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate   = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* Default task (unused) */
void StartDefaultTask(void const * argument)
{
  for(;;) { osDelay(1000); }
}

/* ===================== App Tasks ===================== */
static void SensorTask(void const * argument)
{
  (void)argument;
  int16_t raw[3];
  uint32_t last = osKernelSysTick();

  for(;;) {
    uint32_t now = osKernelSysTick();
    uint32_t elapsed = now - last;
    if (elapsed < SENSOR_PERIOD_MS) {
      osDelay(SENSOR_PERIOD_MS - elapsed);
    }
    last = osKernelSysTick();

    if (i2cMutexHandle) osMutexWait(i2cMutexHandle, 20);
    MPU6050_Read_Accel(&hi2c1, raw);
    if (i2cMutexHandle) osMutexRelease(i2cMutexHandle);

    g_sample.ax = raw[0];
    g_sample.ay = raw[1];
    g_sample.az = raw[2];
    g_sample.ts_ms = osKernelSysTick();

    if (detectTaskHandle) osSignalSet(detectTaskHandle, DETECT_SIGNAL_FLAG);
  }
}

static void DetectTask(void const * argument)
{
  (void)argument;
  for(;;) {
    osEvent ev = osSignalWait(DETECT_SIGNAL_FLAG, osWaitForever);
    if (ev.status != osEventSignal) continue;

    float ax = (float)g_sample.ax;
    float ay = (float)g_sample.ay;
    float az = (float)g_sample.az;
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    if (mag > COLLISION_THRESHOLD_RAW) {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

      GPSFix fix;
      if (GPS_GetLatestFix(&fix)) {
        char out[96];
        // 정수 포맷(printf float 링크 안해도 됨)
        int32_t lat1e6 = (int32_t)(fix.latitude  * 1000000.0);
        int32_t lon1e6 = (int32_t)(fix.longitude * 1000000.0);
        int32_t lat_abs = (lat1e6 < 0) ? -lat1e6 : lat1e6;
        int32_t lon_abs = (lon1e6 < 0) ? -lon1e6 : lon1e6;

        int n = snprintf(out, sizeof(out),
                         "GPS: lat=%c%ld.%06ld lon=%c%ld.%06ld UTC=%02u:%02u:%02u\r\n",
                         (lat1e6<0)?'-':'+', (long)(lat_abs/1000000), (long)(lat_abs%1000000),
                         (lon1e6<0)?'-':'+', (long)(lon_abs/1000000), (long)(lon_abs%1000000),
                         fix.hh, fix.mm, fix.ss);
        HAL_UART_Transmit(&huart2, (uint8_t*)out, n, 50);  // PC로 출력 (USART2)
      } else {
        const char *no = "GPS: no valid fix yet\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)no, strlen(no), 20); // PC로 출력
      }
    } else {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
  }
}

/* Error Handler */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
