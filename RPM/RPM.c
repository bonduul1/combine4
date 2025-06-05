#include "RPM.h"
#include "J1939.h"
// Define constants
#define ALPHA 0.5f

uint32_t rpmTimerCaptured;
uint32_t rpmTimerCapturedCAN;
float rpmNew, rpmOld, rpm, rpmPrevious = 0;
uint16_t rpmCAN = 0;


void rpm_process(void)
{
  // every 100ms calculate rpm
  // 1h = 60 rpm
  // 1s = 100 * 10
  
  if(flagTimer.hundredMs == FALSE)
    return;

  rpmTimerCapturedCAN = rpmTimerCaptured;
  rpmTimerCaptured = 0;

  // rpmNew = ((float)rpmTimerCapturedCAN * 10.0f) * 60 / 10;
  // rpmNew = ((float)rpmTimerCapturedCAN * (ms to s)) * (hz to rpm) / (per rotation counter);
  rpmNew = (float)rpmTimerCapturedCAN * 60.0f;
  // rpmNew = (float)rpmTimerCapturedCAN * 0.14.0f + 18.542; // tseveen 20250519
  rpm = rpmOld * (1 - ALPHA) + rpmNew * ALPHA;
  rpmOld = rpm;

  if (rpmPrevious >= rpm)
  {
      if ((rpmPrevious - rpm) >= 100) // tseveen 20250423 change 100
          rpmPrevious -= 100;            // tseveen 20250423 change 100  
  }
  else
  {
      if ((rpm - rpmPrevious) >= 100)    // tseveen 20250423 change 100
          rpmPrevious += 100;            // tseveen 20250423 change 100
  }

  rpm = rpmPrevious;

  if (rpm >= 500)
  {
      if (rpm > 3000)
          rpm = 3000;
      rpmCAN = (uint16_t)(rpm * 8);
  }
  else
  {
      rpmCAN = 0;
  }
  
  engine_speed = rpmCAN;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)   //tseveen 20250422
{
  if (GPIO_Pin == GPIO_PIN_7)  //tseveen 20250422
  {
    rpmTimerCaptured++;   //tseveen 20250422                                        // 65535 => 
  }
}
