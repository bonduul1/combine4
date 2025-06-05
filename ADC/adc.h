/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
/* Defines -------------------------------------------------------------------*/
#define ADC0_Pin                        GPIO_PIN_0
#define ADC0_GPIO_Port                  GPIOC
#define ADC1_Pin                        GPIO_PIN_1
#define ADC1_GPIO_Port                  GPIOC
#define ADC2_Pin                        GPIO_PIN_2
#define ADC2_GPIO_Port                  GPIOC
#define ADC3_Pin                        GPIO_PIN_3
#define ADC3_GPIO_Port                  GPIOC
  
#define NUMBER_OF_ADC_CHANNELS          5
#define NUMBER_OF_ADC_AVERAGE           100

#define ADC_BACKWARD_PRESSURE           0
#define ADC_FORWARD_PRESSURE            1
#define ADC_SHUTTLE_LEVER               2
#define ADC_CLUTCH_LEVEL                3
#define ADC_OIL_TEMPERATURE             4

/* Functions -----------------------------------------------------------------*/
void adc_init(void);
void adc_dma_init(void);

uint8_t  startConversation();
uint8_t  updateADC();
uint8_t  updateLastAverageADC();

uint16_t getCurrentADCValue(uint8_t channel);
uint16_t getAverageADCValue(uint8_t channel);
uint8_t  getCurrentADCValues(uint16_t *channels_data);
uint8_t  getAverageADCValues(uint16_t *channels_data);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H */
