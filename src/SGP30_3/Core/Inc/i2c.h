/*
 * Copyright (c) 2017, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __I2C_SOFT_MULTI_H__
#define __I2C_SOFT_MULTI_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* 소프트 I2C 버스 정의 */
typedef struct
{
    GPIO_TypeDef* scl_port;
    uint16_t      scl_pin;
    GPIO_TypeDef* sda_port;
    uint16_t      sda_pin;
} SoftI2C_t;

/* 현재 사용할 버스 선택 */
void SoftI2C_SelectBus(SoftI2C_t* bus);

/* Sensirion SGP30 드라이버가 호출하는 함수들 (이 이름 그대로 필요) */
int8_t  sensirion_i2c_init(void);
int8_t  sensirion_i2c_read(uint8_t address, uint8_t* data, uint16_t count);
int8_t  sensirion_i2c_write(uint8_t address, const uint8_t* data, uint16_t count);

#endif /* I2C_SOFT_MULTI_H */

