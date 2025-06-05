/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
/* Defines -------------------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
void timer_init(void);
void update_timer(uint16_t _duty);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
