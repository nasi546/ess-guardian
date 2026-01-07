/* Core/Inc/sensirion_common.h */

#ifndef INC_SENSIRION_COMMON_H_
#define INC_SENSIRION_COMMON_H_

#include "main.h"   /* HAL 드라이버 포함 */
#include <stdint.h> /* uint8_t 등을 쓰기 위해 필수 */

/* 1. Sensirion 자료형 정의 */
typedef uint8_t  u8;
typedef int8_t   s8;
typedef uint16_t u16;
typedef int16_t  s16;
typedef uint32_t u32;
typedef int32_t  s32;
typedef uint64_t u64;
typedef int64_t  s64;

/* 2. 에러 코드 정의 */
#define STATUS_OK 0
#define STATUS_FAIL (-1)

/* 3. CRC 계산 관련 상수 */
#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1
#define SGP_WORD_LEN 2

/* 4. [핵심] 데이터 변환 및 배열 크기 매크로 */
#define be16_to_cpu(s) (((u16)(s) << 8) | ((0xff00 & (u16)(s)) >> 8))
#define be32_to_cpu(s) (((u32)be16_to_cpu(s) << 16) | \
                        (0xffff & (be16_to_cpu((s) >> 16))))
#define be64_to_cpu(s) (((u64)be32_to_cpu(s) << 32) | \
                        (0xffffffff & (be32_to_cpu((s) >> 32))))

/* ★★★ 여기가 추가된 부분입니다 ★★★ */
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* 5. 공통 함수 프로토타입 */
u8 sensirion_common_generate_crc(u8 *data, u16 count);
s8 sensirion_common_check_crc(u8 *data, u16 count, u8 checksum);

/* 6. 드라이버 인터페이스 */
void sensirion_sleep_usec(u32 useconds);
s8 sensirion_i2c_read(u8 address, u8* data, u16 count);
s8 sensirion_i2c_write(u8 address, const u8* data, u16 count);

#endif /* INC_SENSIRION_COMMON_H_ */
