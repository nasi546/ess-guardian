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
  *  SGP30 #2 : SCL=PB10, SDA=PB3
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
#include <stdio.h>
#include <string.h>
#include "sensirion_common.h"
#include "sgp30.h"
#include "RC522.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  GPIO_TypeDef* scl_port;
  uint16_t      scl_pin;
  GPIO_TypeDef* sda_port;
  uint16_t      sda_pin;
} SoftI2C;

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
#define SGP_INTERVAL     1000

/* [릴레이 핀 설정] */
#define RELAY_GPIO_PORT GPIOB
#define RELAY_GPIO_PIN  GPIO_PIN_0
#define RELAY_ACTIVE_LOW 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* RFID용 변수 (사용자 정의 유지) */
uint8_t status;
uint8_t str[16]; // MAX_LEN
uint8_t sNum[5];

/* SGP30 타이머 변수 */
static uint32_t last_sgp_time = 0;

/* [릴레이/알람용 변수 추가] */
typedef enum { ALARM_OFF=0, ALARM_WARNING, ALARM_CRITICAL } AlarmLevel;
static volatile AlarmLevel g_alarm = ALARM_OFF;
static volatile uint32_t g_hold_until = 0;

/* [UART 수신 버퍼 추가] */
static uint8_t rx_ch;
static char rx_line[256];
static volatile uint16_t rx_idx = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
extern void i2c_select_bus(uint8_t bus_id);
static void esp_send_line(const char *s);
extern int8_t sensirion_i2c_init(void);

/* [릴레이 함수 선언] */
static void relay_on(void);
static void relay_off(void);
static void alarm_set(AlarmLevel lv, uint32_t hold_ms);
static void alarm_update(void);
static void handle_alert_json_line(const char* line);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void send_access_request_uart1(uint8_t *uid)
{
    char payload[128];
    sprintf(payload, "{\"uid\":[%d,%d,%d,%d],\"access_point\":\"ew1\"}\n",
            uid[0], uid[1], uid[2], uid[3]);
    HAL_UART_Transmit(&huart1, (uint8_t *)payload, strlen(payload), 100);
}

static void send_env_data_uart1(int zone, uint16_t tvoc, uint16_t eco2)
{
    char payload[128];
    sprintf(payload, "{\"type\":\"sgp\",\"zone\":\"zone_%d\",\"tvoc\":%u,\"eco2\":%u}\n",
            zone, tvoc, eco2);
    HAL_UART_Transmit(&huart1, (uint8_t *)payload, strlen(payload), 100);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // 1. [릴레이 초기화 및 부팅 테스트]
    relay_off();
    // UART 수신 인터럽트 시작 (ESP8266 -> STM32 알람 수신용)
    HAL_UART_Receive_IT(&huart1, &rx_ch, 1);

    // 부팅 확인용 깜빡임 (0.5초)
    relay_on();
    HAL_Delay(500);
    relay_off();

//  uint8_t ver = Read_MFRC522(0x37);
//    printf("RC522 Version: 0x%02X (Expected: 0x92 or 0x91)\r\n", ver);
  printf("\r\n=== Board #2 Start ===\r\n");

  MFRC522_Init();
  printf("RC522 Initialized.\r\n");


  sensirion_i2c_init();
  for(uint8_t i=1; i<=3; i++) {
        printf("Init SGP30 #%d ... ", i);
        i2c_select_bus(i);
        HAL_Delay(50);

        if (sgp_probe() == STATUS_OK) {
            if (sgp_iaq_init() == STATUS_OK) printf("OK!\r\n");
            else printf("Init Fail\r\n");
        } else {
            printf("Not Found\r\n");
        }
        HAL_Delay(500);
  }
  last_sgp_time = HAL_GetTick();
  char buf[64];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // ==========================================
	      // TASK 0: 릴레이(경광등) 패턴 업데이트 [추가됨]
	      // ==========================================
	      alarm_update();

	  // ========================================================
		// [PART 1] RFID 카드 인식 (기존 로직 유지)
		// ========================================================
		// 1) 카드 요청
		status = MFRC522_Request(PICC_REQIDL, str);
		if (status == MI_OK) {
			// 2) 충돌 방지 및 UID 추출
			status = MFRC522_Anticoll(str);
			if (status == MI_OK) {

				// UID 가져오기
				uint8_t u0 = str[0], u1 = str[1], u2 = str[2], u3 = str[3];

				// 3) 같은 카드 연속 인식 방지 (쿨다운 1.5초)
				static uint8_t last_uid4[4] = {0};
				static uint32_t last_rfid_tick = 0;
				uint32_t now = HAL_GetTick();

				int same = (memcmp(str, last_uid4, 4) == 0);
				if (!same || (now - last_rfid_tick) > 1500) {

					// 새로운 카드거나 쿨다운 지남 -> 전송
					memcpy(last_uid4, str, 4);
					last_rfid_tick = now;

					// PC 출력
					printf("[RFID] UID: %d %d %d %d\r\n", u0, u1, u2, u3);

					// ESP 전송 (JSON)
					char payload[128];
					sprintf(payload, "{\"uid\":[%d,%d,%d,%d],\"access_point\":\"ew1\"}\n", u0, u1, u2, u3);
					esp_send_line(payload);
				}
			}
		}

		// ========================================================
		// [PART 2] SGP30 가스 센서 (1초 주기 실행)
		// ========================================================
		if (HAL_GetTick() - last_sgp_time >= SGP_INTERVAL) {
			last_sgp_time = HAL_GetTick();

			u16 tvoc, eco2;
			s16 err;
			char json[128];

			// 센서 1, 2, 3 순차 측정 (4~6이 아니라 1~3이 맞음)
			for(uint8_t i=1; i<=3; i++) {
				i2c_select_bus(i);

				err = sgp_measure_iaq_blocking_read(&tvoc, &eco2);

				if (err == STATUS_OK) {
					// PC 로그
					int zone_num = i+3;
					printf("[SGP #%d] TVOC: %d, eCO2: %d\r\n", zone_num, tvoc, eco2);

					// ESP 전송
					snprintf(json, sizeof(json),
							 "{\"type\":\"sgp\",\"zone\":\"zone_%d\",\"tvoc\":%u,\"eco2\":%u}",
							 zone_num, tvoc, eco2);
					esp_send_line(json);
				} else {
					printf("[SGP #%d] Read Error\r\n", i);
				}
				// 센서 스위칭 안정화 딜레이 (너무 길면 RFID 둔해짐)
				HAL_Delay(50);
			}
		}

		// 루프 전체 속도 조절 (너무 빠르면 과부하)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13;
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
// [릴레이 제어 함수]
static void relay_on(void) {
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_SET);
}

static void relay_off(void) {
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, GPIO_PIN_RESET);
}

// [알람 설정]
static void alarm_set(AlarmLevel lv, uint32_t hold_ms)
{
    uint32_t now = HAL_GetTick();
    if (lv >= g_alarm) {
        g_alarm = lv;
        g_hold_until = now + hold_ms;
    }
}

// [JSON 파싱]
static void handle_alert_json_line(const char* line)
{
    if (strstr(line, "critical")) {
        alarm_set(ALARM_CRITICAL, 10000);
    }
    else if (strstr(line, "warning")) {
        alarm_set(ALARM_WARNING, 10000);
    }
}

// [알람 업데이트]
static void alarm_update(void)
{
    uint32_t now = HAL_GetTick();

    // 시간 종료
    if (g_alarm != ALARM_OFF && (int32_t)(now - g_hold_until) > 0) {
        g_alarm = ALARM_OFF;
        relay_off();
        return;
    }

    // 알람 없음
    if (g_alarm == ALARM_OFF) {
        relay_off();
        return;
    }

    // 패턴 깜빡임
    if (g_alarm == ALARM_WARNING) {
        // 500ms 주기 (느리게)
        if ((now / 500) % 2 == 0) relay_on();
        else relay_off();
    } else { // CRITICAL
        // 100ms 주기 (빠르게)
        if ((now / 100) % 2 == 0) relay_on();
        else relay_off();
    }
}

// [UART 수신 인터럽트 콜백]
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) return;

  if (rx_ch == '\n') {
    rx_line[rx_idx] = '\0';
    rx_idx = 0;

    if (strlen(rx_line) > 2) {
      handle_alert_json_line(rx_line);
    }
  } else {
    if (rx_idx < sizeof(rx_line) - 1) {
      rx_line[rx_idx++] = (char)rx_ch;
    } else {
      rx_idx = 0;
    }
  }

  HAL_UART_Receive_IT(&huart1, &rx_ch, 1);
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
