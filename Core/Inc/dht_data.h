#ifndef DHT_DATA_H
#define DHT_DATA_H

#include "stm32f767xx.h"
#include <stdint.h>
//=== 5x7 Font ===//

#define DHT_PORT  GPIOA
#define DHT_PIN   3   // PA3

typedef struct {
    uint8_t humidity;     // DHT11 integer
    uint8_t temperature;  // DHT11 integer
    uint8_t checksum;
} DHT_Data;

void    DHT_Init(void);
uint8_t DHT_Read(DHT_Data *out);   // 1=ok, 0=fail

#endif
