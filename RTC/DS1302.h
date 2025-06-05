#ifndef __DS1302_H
#define __DS1302_H


#ifdef __cplusplus
extern "C" {
#endif
  
#include "main.h"

#define DS1302_SCLK_PIN	        GPIO_PIN_9
#define DS1302_SCLK_PORT	GPIOD
#define DS1302_SDA_PIN	        GPIO_PIN_10
#define DS1302_SDA_PORT	        GPIOD
#define DS1302_RST_PIN	        GPIO_PIN_11
#define DS1302_RST_PORT	        GPIOD

void rtc_Init(void);
void rtc_getTime(uint8_t *buf);
void rtc_setTime(uint8_t *buf);


#ifdef __cplusplus
}
#endif

#endif
