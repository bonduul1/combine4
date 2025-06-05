/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *
  *
  *
  *
  *     2024.08.07
  *                     1. LCD Control unit is using PWM signal of the TIM1
  *                     2. Easy scale protocol is removed when control the LCD brightness, sometimes the display is turned OFF.
  *     2024.08.14
  *                     1. LCD brightness test with different frequency
  *     2024.08.19
  *                     1. Fuel percentage is matched with OLD LCD controller, R1 = 180 / 2 = 90Om, R2 is fuel sensor.
  *                     2. 
  *     2024.10.16
  *                     1. Speed sensor calculation is wrong (LSB and MSB bug is fixed)
  *                     2. Camera initialization function is removed (it takes almost 300 ms)
  *                     3. Engine desox errors has a same position.
  *                        if both errors are occurred same time, the timer is using to switch the error images.
  *                        one error image is shown 3000ms and then another image is shown 3000ms, the cycle is going on until both errors are cleared.
  *     2024.10.21
  *                     1. For the new combine (MODEL_NEXT), 2 CAN message error is NOT checked.
  *                     2. For the new combine (MODEL_NEXT), Axel controller is used.
  *                     3. Axel pedal MAX and MIN default value are changed
  *     2024.10.23
  *                     1. Charge warning checking method is changed.
  *     2024.11.06
  *                     1. CAN Reinittialize problem is solved (Bug: CAN Interrupt enable is NOT called)
  *     2024.01.16
  *                     1. Fuel test
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system.h"
#include "input.h"
#include "output.h"
#include "spi.h"
#include "i2c.h"
#include "adc.h"
#include "can.h"
#include "button.h"
#include "J1939.h"
#include "Yvc1_HAL.h"
#include "tw9990.h"
#include "usart.h"
#include "timer.h"
#include "RPM.h"     // tseveen 20250423
#include "stm32f4xx_hal.h"  // tseveen 20250423
// Disable on 15 April 2021 #include "DS1302.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
flagTimer_t flagTimer;
uint32_t localEngineHourTimer;
uint32_t localJobHourTimer;
uint8_t transmitBegin;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
void timer_process();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  transmitBegin = FALSE;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //__enable_irq();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  watchdog_init();
//  button_init(); tseveen 20250421
  input_init();
  output_init();
  tw9990_init();
  ygv643_init();
  lcd_init();
  eeprom_init();
  
  adc_dma_init();
  usart_init();
  spi_init();
  i2c_init();
  can_init();
  adc_init();
  timer_init();
  
#if defined(USER_DEBUG)
  printf("Starting algorithm init ...\r\n");
#endif

  startConversation();
  control_init();
  
#if defined(USER_DEBUG)
  printf("Done init ...\r\n");
#endif
  
  can_start();
  
  while (1)
  {
    if(flagTimer.oneMs == TRUE)
    {
      timer_process();
      watchdog_trigger();
      
      if(flagTimer.twoMs == TRUE)
      {
        control_process();
      }
      transmitBegin = TRUE;
      flagTimer.data = 0;
    }
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
  flagTimer.oneMs = TRUE;

  if(transmitBegin == TRUE)
  {
    //can_transmit_process_engine_line();                                           // CAN Transmission is moved from main process into here, because of time problem
//    can_transmit_process();   tseveen 20250423
  }
  
  update_hours();
}


void timer_process()
{
  static uint8_t tm1ms;
  static uint8_t tm2ms;
  static uint8_t tm10ms;
  static uint8_t tm100ms;
  
  update_bam_timer();

  tm1ms++;                                                                      // every 1ms it is added by one

  if(tm1ms < 2) return;
  tm1ms = 0;
  flagTimer.twoMs = TRUE;

  tm2ms++;                                                                      // every 2ms it is added by one
  if(tm2ms < 5) return;
  tm2ms = 0;
  flagTimer.tenMs = TRUE;

  tm10ms++;                                                                     // every 10 ms it is added by one, so 10*10 = 100ms
  if(tm10ms < 10) return;
  tm10ms = 0;
  flagTimer.hundredMs = TRUE;

  tm100ms++;                                                                    // every 100 ms it is added by one, so 100*10 = 1000ms = 1 second
  if(tm100ms < 10) return;
  tm100ms = 0;
  flagTimer.oneSecond = TRUE;
}

void user_delay(uint32_t delay)
{
  uint32_t i;
  
  for(i = 0; i < delay; i++)
  {
    watchdog_trigger();
    HAL_Delay(1);
  }
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

#ifdef  USE_FULL_ASSERT
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
