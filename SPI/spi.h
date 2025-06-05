/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Defines  ------------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/  
void spi_init();
void spi2_enable();

HAL_StatusTypeDef SPI_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_Receive(uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef SPI_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
