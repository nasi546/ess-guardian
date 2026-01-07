/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : SGP30 x3 (3개의 독립 I2C 비트뱅) 테스트
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************  *
  * - SGP30는 I2C 주소가 모두 0x58로 동일
  *  - 한 버스에 여러개 불가 → GPIO I2C 라인을 3세트로 분리해서 각 1개씩 연결
  *
  *  UART2로 출력(PC Serial)
  *
  * 핀맵(사용자 지정)
  *  SGP30 #1 : SCL=PB6,  SDA=PB7
  *  SGP30 #2 : SCL=PB10, SDA=PB8
  *  SGP30 #3 : SCL=PA8,  SDA=PC9
  *
  * CubeMX 주의:
  *  - PB3는 SWO랑 충돌 가능 → SYS->Debug는 Serial Wire만(Trace 끄기)
  ******************************************************************************  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "sensirion_common.h"
#include "sgp30.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef* scl_port;
  uint16_t      scl_pin;
  GPIO_TypeDef* sda_port;
  uint16_t      sda_pin;
} SoftI2C;

/* ===== 팬이 켜진 이유 표시용 ===== */
typedef enum {
  FAN_REASON_NONE = 0,
  FAN_REASON_TEMP,
  FAN_REASON_HUM,
  FAN_REASON_FAILSAFE
} FanReason;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef STATUS_OK
#define STATUS_OK 0
#endif

#ifndef STATUS_FAIL
#define STATUS_FAIL (-1)
#endif

/* SGP30 7-bit 주소 */
#define SGP30_ADDR_7BIT  (0x58)

/* I2C 속도(대략) 조절 */
#define I2C_DELAY_US     (5)   // 5us 정도면 대략 100kHz 근처(환경에 따라 다름)
/* USER CODE BEGIN PD */
// [SGP30 주기]
#define SGP_INTERVAL 5000 // 1초

// [DHT11 / Fan 핀 설정 (사용자 코드 그대로)]
#define DHT_PORT        GPIOA
#define DHT_PIN         GPIO_PIN_1

// L9110 B채널 입력핀: B-IA=PB1, B-IB=PB2
#define L9110_BIA_PORT  GPIOB
#define L9110_BIA_PIN   GPIO_PIN_1
#define L9110_BIB_PORT  GPIOB
#define L9110_BIB_PIN   GPIO_PIN_2

/* =========================
 * 임계치(히스테리시스)
 * ========================= */
#define T_ON            30.0f
#define T_OFF           26.0f
#define H_ON            30.0f
#define H_OFF           25.0f

/* =========================
 * 주기 설정
 * ========================= */
#define DHT_INTERVAL_MS 5000    // DHT11 읽기 주기(5초)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* 3개의 버스 */
//static SoftI2C g_i2c1; // PB6/PB7
//static SoftI2C g_i2c2; // PB10/PB3
//static SoftI2C g_i2c3; // PA8/PC9
/* [Fan/DHT 변수 (사용자 코드 그대로)] */
static uint8_t  g_fan_on = 0;
static uint32_t g_last_dht_ms = 0;

static FanReason g_fan_reason = FAN_REASON_NONE;
static uint32_t g_dht_fail_cnt = 0;

/* [SGP30 변수] */
static uint32_t last_sgp_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
extern void i2c_select_bus(uint8_t bus_id);
static void esp_send_line(const char *s);
extern int8_t sensirion_i2c_init(void);

// [DHT/Fan 함수]
static void delay_us(uint16_t us);
static void DHT_Pin_Output(void); // (Note: implemented as set_pin_output locally)
static void DHT_Pin_Input(void);  // (Note: implemented as set_pin_input locally)
static uint8_t DHT11_Read(float *t, float *h); // (Note: implemented as DHT_Read)
static void Fan_Stop(void);
static void Fan_On_ForwardOnly(void);
static void FanControl_Update(float t, float h, uint8_t ok);
static void FanControl_Task(void);
static const char* FanReasonToStr(FanReason r);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// [SGP용 ESP 전송 헬퍼]
static void send_sgp_to_esp(int zone, uint16_t tvoc, uint16_t eco2) {
    char payload[128];
    // Board #2: Zone 1, 2, 3
    sprintf(payload, "{\"type\":\"sgp\",\"zone\":\"zone_%d\",\"tvoc\":%u,\"eco2\":%u}\n",
            zone, tvoc, eco2);
    HAL_UART_Transmit(&huart1, (uint8_t *)payload, strlen(payload), 100);
}

// -------------------- DHT/Fan 구현 (변수명/로직 유지) --------------------
static void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);            // 타이머1 카운터 0으로 초기화
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // us 만큼 대기
}

void set_pin_output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;                // DHT 핀
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;   // ✅ 오픈드레인 (안정적)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

void set_pin_input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 모듈이면 보통 외부 풀업 있음
    HAL_GPIO_Init(DHT_PORT, &GPIO_InitStruct);
}

uint8_t DHT_Read(uint8_t *temp, uint8_t *hum)
{
    uint8_t bits[5] = {0};
    uint32_t time;

    // MCU -> 센서 Start 신호
    set_pin_output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);  // 18~20ms
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    delay_us(40);   // 20~40us
    set_pin_input();

    // 센서 응답 대기
    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
        if (++time > 200) return 1;
        delay_us(1);
    }

    // LOW(80us)
    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) {
        if (++time > 200) return 2;
        delay_us(1);
    }

    // HIGH(80us)
    time = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
        if (++time > 200) return 3;
        delay_us(1);
    }

    // 40bit 수신
    for (int i = 0; i < 40; i++) {
        // LOW 시작(50us) 끝날 때까지 대기
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET);

        // HIGH 중간에서 샘플링
        delay_us(40);
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) {
            bits[i / 8] |= (1 << (7 - (i % 8)));
        }

        // HIGH 끝날 때까지 대기
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET);
    }

    // checksum
    uint8_t sum = bits[0] + bits[1] + bits[2] + bits[3];
    if (sum != bits[4]) return 4;

    *hum  = bits[0];
    *temp = bits[2];
    return 0;
}

static void Fan_On_ForwardOnly(void)
{
  HAL_GPIO_WritePin(L9110_BIA_PORT, L9110_BIA_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L9110_BIB_PORT, L9110_BIB_PIN, GPIO_PIN_RESET);
  g_fan_on = 1;
}

static void Fan_Stop(void)
{
  HAL_GPIO_WritePin(L9110_BIA_PORT, L9110_BIA_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L9110_BIB_PORT, L9110_BIB_PIN, GPIO_PIN_RESET);
  g_fan_on = 0;
  g_fan_reason = FAN_REASON_NONE;
}

static void FanControl_Update(float t, float h, uint8_t ok)
{
  if (!ok) {
	if (g_dht_fail_cnt < 255) g_dht_fail_cnt++;

	if (g_dht_fail_cnt >= 3) {
	  if (!g_fan_on) {
		Fan_On_ForwardOnly();
		g_fan_reason = FAN_REASON_FAILSAFE;
	  }
	}
	return; // 실패 시 여기서 끝
  }
  g_dht_fail_cnt = 0; // 성공하면 fail count 리셋

  if (!g_fan_on)
  {
	/* OFF -> ON 조건 */
	if (t >= T_ON) {
	  Fan_On_ForwardOnly();
	  g_fan_reason = FAN_REASON_TEMP;
	}
	else if (h >= H_ON) {
	  Fan_On_ForwardOnly();
	  g_fan_reason = FAN_REASON_HUM;
	}
  }
  else
  {
	/* ON -> OFF 조건 */
	if (t <= T_OFF && h <= H_OFF) {
	  Fan_Stop();
	}
	else {
	  if (g_fan_reason != FAN_REASON_FAILSAFE) {
		if (t >= T_ON)      g_fan_reason = FAN_REASON_TEMP;
		else if (h >= H_ON) g_fan_reason = FAN_REASON_HUM;
	  }
	}
  }
}

static const char* FanReasonToStr(FanReason r)
{
  switch (r) {
    case FAN_REASON_TEMP:     return "TEMP";
    case FAN_REASON_HUM:      return "HUM";
    case FAN_REASON_FAILSAFE: return "FAIL";
    default:                  return "NONE";
  }
}

static void FanControl_Task(void)
{
  uint32_t now = HAL_GetTick();
  if (now - g_last_dht_ms < DHT_INTERVAL_MS) return;
  g_last_dht_ms = now;

  uint8_t temperature = 0, humidity = 0;
  uint8_t status = DHT_Read(&temperature, &humidity);
  uint8_t ok = (status == 0) ? 1 : 0;

  float t = (float)temperature;
  float h = (float)humidity;

  // 임계치 자동 제어
  FanControl_Update(t, h, ok);

  if (ok) {
	printf("[DHT] t=%.1fC h=%.1f%% fan=%s reason=%s\r\n", t, h, (g_fan_on ? "ON" : "OFF"),FanReasonToStr(g_fan_reason));

    // [ESP 전송 (DHT)]
    char json[128];
    const char* reason_str = FanReasonToStr(g_fan_reason);
    const char* fan_str = g_fan_on ? "ON" : "OFF";

    int n = snprintf(json, sizeof(json), "{\"t\":%.0f,\"h\":%.0f,\"fan\":\"%s\",\"reason\":\"%s\"}\n", t, h, fan_str, reason_str);
    HAL_UART_Transmit(&huart1, (uint8_t*)json, n, HAL_MAX_DELAY);
  } else {
	printf("[DHT] read fail cnt=%d fan=%s reason=%s\r\n",g_dht_fail_cnt,(g_fan_on ? "ON" : "OFF"),FanReasonToStr(g_fan_reason));
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  printf("\r\n=== Board #1 Start (DHT/Fan + SGP30) ===\r\n");

  Fan_Stop();
  HAL_Delay(500);
  /* 1. I2C 초기화 (GPIO 클럭 등) */
  sensirion_i2c_init();

  /* 2. 센서 3대 초기화 (Probe & IAQ Init) */
  for(uint8_t i=1; i<=3; i++) {
    printf("Initializing SGP30 #%d ... ", i);

      // 버스 변경 (핵심!)
      i2c_select_bus(i);
      HAL_Delay(100); // 스위칭 안정화

      if (sgp_probe() == STATUS_OK) {
          // 초기화 명령
          if (sgp_iaq_init() == STATUS_OK) {
              printf("OK!\r\n");
          } else {
              printf("Init Fail\r\n");
          }
      } else {
        printf("Not Found (Check Wiring)\r\n");
      }
      HAL_Delay(500);   }

  last_sgp_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE BEGIN 3 */

	  // ==========================================
	  // TASK 1: DHT11 & Fan Control (5초 주기)
	  // ==========================================
	  // (내부 타이머 로직 사용)
	  FanControl_Task();

	  // ==========================================
	  // TASK 2: SGP30 가스 센서 (1초 주기)
	  // ==========================================
	  if (HAL_GetTick() - last_sgp_time >= SGP_INTERVAL) {
		  last_sgp_time = HAL_GetTick();

		  u16 tvoc, eco2;
		  s16 err;

		  // 센서 1,2,3 순차 측정 (Zone 1, 2, 3)
		  for(uint8_t i=1; i<=3; i++) {
			  i2c_select_bus(i);
			  err = sgp_measure_iaq_blocking_read(&tvoc, &eco2);

			  if (err == STATUS_OK) {
				  printf("[SGP #%d] TVOC: %d, eCO2: %d\r\n", i, tvoc, eco2);
				  send_sgp_to_esp(i, tvoc, eco2);
			  }
			  HAL_Delay(20);
		  }
	  }
	  HAL_Delay(10);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* [추가] printf 출력을 UART2로 보내주는 함수 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 100);
  return len;
}

static void esp_send_line(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 100);
    uint8_t nl = '\n';
    HAL_UART_Transmit(&huart1, &nl, 1, 10);
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
