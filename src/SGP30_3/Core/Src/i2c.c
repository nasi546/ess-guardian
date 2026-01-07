/*
 * i2c.c
 *
 *  Created on: Jan 2, 2026
 *      Author: KCCISTC
 */

#include "main.h"
#include <stdint.h>

/* ================================
 * [핀맵 설정] (사용자 손그림 기준)
 * ================================ */
// SGP #1
#define SGP1_SCL_PORT GPIOB
#define SGP1_SCL_PIN  GPIO_PIN_6
#define SGP1_SDA_PORT GPIOB
#define SGP1_SDA_PIN  GPIO_PIN_7

// SGP #2
#define SGP2_SCL_PORT GPIOB
#define SGP2_SCL_PIN  GPIO_PIN_10
#define SGP2_SDA_PORT GPIOB
#define SGP2_SDA_PIN  GPIO_PIN_9

// SGP #3
#define SGP3_SCL_PORT GPIOA
#define SGP3_SCL_PIN  GPIO_PIN_8
#define SGP3_SDA_PORT GPIOC
#define SGP3_SDA_PIN  GPIO_PIN_9

/* I2C 타이밍 (비트뱅잉 속도 조절) */
#define I2C_DELAY_US  70

typedef struct {
    GPIO_TypeDef* scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef* sda_port;
    uint16_t      sda_pin;
} soft_i2c_bus_t;

static soft_i2c_bus_t g_bus;

/* DWT Delay */
static void dwt_enable_if_needed(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void sensirion_sleep_usec(uint32_t useconds) {
    dwt_enable_if_needed();
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = useconds * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks) { }
}

static void i2c_delay(void) {
    sensirion_sleep_usec(I2C_DELAY_US);
}

/* GPIO 초기화 (Open-Drain) */
static void gpio_od_init(GPIO_TypeDef* port, uint16_t pin) {
    GPIO_InitTypeDef g = {0};
    g.Pin   = pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &g);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET); // Idle High
}

/* 핀 제어 매크로 */
static inline void SDA_H(void) { HAL_GPIO_WritePin(g_bus.sda_port, g_bus.sda_pin, GPIO_PIN_SET); }
static inline void SDA_L(void) { HAL_GPIO_WritePin(g_bus.sda_port, g_bus.sda_pin, GPIO_PIN_RESET); }
static inline void SCL_H(void) { HAL_GPIO_WritePin(g_bus.scl_port, g_bus.scl_pin, GPIO_PIN_SET); }
static inline void SCL_L(void) { HAL_GPIO_WritePin(g_bus.scl_port, g_bus.scl_pin, GPIO_PIN_RESET); }
static inline uint8_t SDA_READ(void) { return (HAL_GPIO_ReadPin(g_bus.sda_port, g_bus.sda_pin) == GPIO_PIN_SET) ? 1 : 0; }

/* ====== 버스 선택 (핵심) ====== */
void i2c_select_bus(uint8_t bus_id) {
    if (bus_id == 1) {
        g_bus.scl_port = SGP1_SCL_PORT; g_bus.scl_pin = SGP1_SCL_PIN;
        g_bus.sda_port = SGP1_SDA_PORT; g_bus.sda_pin = SGP1_SDA_PIN;
    } else if (bus_id == 2) {
        g_bus.scl_port = SGP2_SCL_PORT; g_bus.scl_pin = SGP2_SCL_PIN;
        g_bus.sda_port = SGP2_SDA_PORT; g_bus.sda_pin = SGP2_SDA_PIN;
    } else { // 3
        g_bus.scl_port = SGP3_SCL_PORT; g_bus.scl_pin = SGP3_SCL_PIN;
        g_bus.sda_port = SGP3_SDA_PORT; g_bus.sda_pin = SGP3_SDA_PIN;
    }

    // 선택된 핀을 I2C용(OD)으로 재설정
    gpio_od_init(g_bus.scl_port, g_bus.scl_pin);
    gpio_od_init(g_bus.sda_port, g_bus.sda_pin);

    SDA_H(); SCL_H();
    i2c_delay();
}

/* ====== I2C 기본 동작 ====== */
static void i2c_start(void) {
    SDA_H(); SCL_H(); i2c_delay();
    SDA_L(); i2c_delay();
    SCL_L(); i2c_delay();
}

static void i2c_stop(void) {
    SDA_L(); i2c_delay();
    SCL_H(); i2c_delay();
    SDA_H(); i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H(); else SDA_L();
        i2c_delay();
        SCL_H(); i2c_delay();
        SCL_L(); i2c_delay();
        byte <<= 1;
    }
    SDA_H(); i2c_delay();
    SCL_H(); i2c_delay();
    uint8_t ack = (SDA_READ() == 0) ? 1 : 0;
    SCL_L(); i2c_delay();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t byte = 0;
    SDA_H();
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        SCL_H(); i2c_delay();
        if (SDA_READ()) byte |= 1;
        SCL_L(); i2c_delay();
    }
    if (ack) SDA_L(); else SDA_H();
    i2c_delay();
    SCL_H(); i2c_delay();
    SCL_L(); i2c_delay();
    SDA_H(); i2c_delay();
    return byte;
}

/* ====== Sensirion Driver Interface ====== */
int8_t sensirion_i2c_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
   // i2c_select_bus(1);
    return 0;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count) {
    i2c_start();
    if (!i2c_write_byte((address << 1) | 0x00)) { i2c_stop(); return -1; }
    for (uint16_t i = 0; i < count; i++) {
        if (!i2c_write_byte(data[i])) { i2c_stop(); return -2; }
    }
    i2c_stop();
    return 0;
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count) {
    i2c_start();
    if (!i2c_write_byte((address << 1) | 0x01)) { i2c_stop(); return -1; }
    for (uint16_t i = 0; i < count; i++) {
        uint8_t ack = (i < (count - 1)) ? 1 : 0;
        data[i] = i2c_read_byte(ack);
    }
    i2c_stop();
    return 0;
}
