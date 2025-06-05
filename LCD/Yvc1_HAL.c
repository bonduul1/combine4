//add SKLEE 171030


#include <math.h>
#include "Yvc643.h"
#include "Yvc1Ddrv.h"
#include "HD800G48070MT_VC1W.h"
#include "Yvc1Lyr.h"
#include "layoutData.h"
#include "Yvc1Fontctrl.h"
#include "Yvc1Vin.h"

#include "Yvc1_HAL.h"
#include "dwt_stm32_delay.h"
#include "tw9990.h"
#include "sound.h"
#include "J1939.h"
#include "can.h"
#include "spi.h"
#include "AT93C56.h"
#include "adc.h"
#include "input.h"
#include "output.h"
#include "button.h" 
#include "timer.h"
#include "RPM.h"
/*----------------------------------------------- Algorihtm -----------------------------------------------*/
typedef struct {
  uint8_t       percentage;
  uint8_t       resistance;
  uint8_t       r1;                     // 90Om --> Our hardware
  uint8_t       voltage;
  uint16_t      raw_data;
} t_fuel_type;
#define NUMBER_OF_STEP_IN_FUEL          21

const t_fuel_type FUEL_ADC_PERCENTAGES[NUMBER_OF_STEP_IN_FUEL] = 
{   
  // percentage, Om, R1, Voltage, raw_data
  { 0,    105,    90,     5,      3341 }, 
  { 5,    96,     90,     5,      3202 }, 
  { 10,   90,     90,     5,      3102 }, 
  { 14,   83,     90,     5,      2977 }, 
  { 20,   78,     90,     5,      2877 }, 
  { 26,   70,     90,     5,      2714 }, 
  { 30,   65,     90,     5,      2606 }, 
  { 35,   60,     90,     5,      2490 }, 
  { 40,   55,     90,     5,      2365 }, 
  { 44,   50,     90,     5,      2230 }, 
  { 50,   44,     90,     5,      2059 }, 
  { 55,   40,     90,     5,      1933 }, 
  { 59,   36,     90,     5,      1800 }, 
  { 65,   32,     90,     5,      1661 }, 
  { 70,   28,     90,     5,      1511 }, 
  { 74,   25,     90,     5,      1391 }, 
  { 79,   21,     90,     5,      1223 }, 
  { 86,   16,     90,     5,      991 }, 
  { 91,   13,     90,     5,      847 }, 
  { 95,   10,     90,     5,      690 }, 
  { 100,  3,      90,     5,      286 }
};
/*
  Full ( 100% ) - liter = 90 - 10 (sensor starting) = 80L / 20 = 4L = 5%
  Empty ( 0 % ) - 2.74V = 0x0D4D
  Sensor absent - 3.01V = 0xFFF = 4095
  
*/

const t_fuel_type FUEL_ADC_PERCENTAGES_NEXT[NUMBER_OF_STEP_IN_FUEL] = 
{   
  // percentage, Om, R1, Voltage, raw_data
  { 0,    105,    90,     5,      3405 },                                       // 0D4D = 3405 
  { 5,    96,     90,     5,      3293 },                                       // 0CDD = 3293
  { 10,   90,     90,     5,      3168 },                                       // 0C60 = 3168
  { 15,   83,     90,     5,      3037 },                                       // 0BDD = 3037
  { 20,   78,     90,     5,      2892 },                                       // 0B4C = 2892
  { 25,   70,     90,     5,      2742 },                                       // 0AB6 = 2742
  { 30,   65,     90,     5,      2567 },                                       // 0A07 = 2567
  { 35,   60,     90,     5,      2364 },                                       // 093C = 2364
  { 40,   55,     90,     5,      2148 },                                       // 0864 = 2148
  { 45,   50,     90,     5,      1904 },                                       // 0770 = 1904 
  { 50,   44,     90,     5,      1632 },                                       // 0660 = 1632
  { 55,   40,     90,     5,      1532 },                                       // 05FC = 1532
  { 60,   36,     90,     5,      1532 },                                       // 05FC = 1532
  { 65,   32,     90,     5,      1416 },                                       // 0588 = 1416
  { 70,   28,     90,     5,      1292 },                                       // 050C = 1292
  { 75,   25,     90,     5,      1163 },                                       // 048B = 1163
  { 80,   21,     90,     5,      1043 },                                       // 0413 = 1043
  { 85,   16,     90,     5,      848  },                                       // 0350 = 848
  { 90,   13,     90,     5,      640  },                                       // 0280 = 640
  { 95,   10,     90,     5,      409  },                                       // 0199 = 409
  { 100,  3,      90,     5,      182  }                                        // 00B6 = 182
};

flag_t flag;

uint8_t tFuelGage;
uint8_t tFuelPercent;
float tPowerVoltage;
float tFuelVoltage;
float tTemperature;
float tEngineSpeed;

uint8_t acceleratorPedalPosition;
//#define ADC_AVERAGE_FIRST_TIME  5
//#define ADC_AVERAGE_NUMBER      100                                            // 100 * 100ms = 10000ms = 10s
/*----------------------------------------------- Algorihtm -----------------------------------------------*/

/*----------------------------------------------- LCD -----------------------------------------------*/
pageState_t currentPageState;
pageState_t previousPageState;

int16_t index = 0;

int16_t new_setup_data_received_to_update;
int16_t new_setup_data;
int16_t new_setup_data_2;

uint8_t setup_mode;
uint16_t setup_mode_address;
uint8_t setup_mode_type;
uint8_t setup_mode_rw;


uint8_t row_selection;

uint16_t time_update;
uint8_t move_next_setup_page = 0;

int8_t number_index;
int8_t number_index_blink;

int8_t settings_pass[10] = {0,0,0,0,0,0,0,0,0,0};
int8_t settings_time[5] = {0,0,0,0,0};

//int8_t PASSWORD[10] = {2,0,0,1,0,8,0,0,2,5};
int8_t PASSWORD[10] = {2,0,0,0,0,0,0,0,0,0};
/*----------------------------------------------- LCD -----------------------------------------------*/

/*----------------------------------------------- Memory -----------------------------------------------*/

float jobHour;
float engineHour;
float engineOilHour;
float missionOilHour;

uint16_t numberOfEngineOilExchange;
uint16_t numberOfMissionOilExchange;

uint16_t lcdBrigthnessDay;
uint16_t lcdBrigthnessNight;


uint16_t axel_app_sensor1_position_max;
uint16_t axel_app_sensor1_position_min;
uint16_t axel_app_sensor2_position_max;
uint16_t axel_app_sensor2_position_min;

uint8_t axel_auger_delay_temp;
uint8_t axel_threshing_delay_temp;
uint8_t axel_yeache_ku_time_temp;
uint8_t axel_auger_auto_delay_temp;
uint8_t axel_auger_ku_time_temp;
uint8_t axel_tbs_ku_time_temp;
uint8_t axel_c_speed_ku_time_temp;
uint16_t axel_threshing_rpm_temp;

uint8_t modelSelection;
/*----------------------------------------------- Memory -----------------------------------------------*/


/*----------------------------------------------- Warning -----------------------------------------------*/
flagWarning_t flagWarning;

typedef struct                          // Do not change the sequence of the structure
{
  wIndex_t      index;
  wFlag_t       flag;
  wState_t      state;
  wLevel_t      level;
  wPage_t       page;
  
  uint8_t       checkRPM;
  uint8_t       checkTalkuk;
    
  t_soundType   soundType;
  uint16_t      soundPeriodNumber;
  uint16_t      soundPeriod;
  uint16_t      soundOnDuration;
  uint16_t      soundOffDuration;
  uint16_t      timerWarning;
} warning_t;

warning_t warnings[TOTAL_NUMBER_OF_WARNINGS] = 
{
  /*
    Note: if "soundPeriodNumber" variable is higher than 255 (8 bit data), the output is always going on
  */
  {
    W_FUEL_EMPTY,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    5,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_ENGINE_OIL_EXCHANGE,      // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    5,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_MISSION_OIL_EXCHANGE,     // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    5,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CHUHEN_MOTOR,             // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_STARTER_SAFETY,           // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    500,                        // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_INTEGRATED_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_CUTTING_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_AUGER_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_TALKUK_HEIGHT_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_GENERAL_SWITCH,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_AUGER_SWITCH,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
    
  {
    W_CAN_TIMEOUT_HST_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_AXEL_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CAN_TIMEOUT_SONBYOL_CONTROLLER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    3,                          // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CHUHYAN_MOTOR,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_AUGER_MOTOR,              // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    1,                          // soundPeriodNumber
    500,                        // soundPeriod
    500,                        // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TALKUK_HEIGHT,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    1,                          // soundPeriodNumber
    500,                        // soundPeriod
    500,                        // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  // Added on 2024.10.23
  {
    W_CHUHEN_LEVER,             // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CHUHYAN_LEVER,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_BATTERY,                  // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CHUHEN_CONTROLLER,        // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_AUGER_ROTATION_SENSOR,    // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  {
    W_LSA_MOTOR_POSITION_SENSOR,// index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },

  {
    W_TBS_SLOPE_SENSOR,         // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TBS_RIGHT_SENSOR,         // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TBS_LEFT_SENSOR,          // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TBS_MANUAL_SWITCH,        // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_AUGER_MANAUL_SWITCH,      // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_AUGER_SETTING_SWITCH,     // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_LSA_MH_SENSOR,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_LSA_MANUAL_SWITCH,        // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
    
  {
    W_YEACHE_LIFT_SENSOR,       // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_PREVIEW_SENSOR,           // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_YEACHE_MANAUL_SWITCH,     // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TALKUK_CLUTCH_CONNECTION_BLOCK,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TALKUK_CLUTCH_CONNECTION_BLOCK_LSA_MOTOR,            // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_ONETOUCH_UP_DOWN,         // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },

  // Two pages error
  {
    W_ENGINE_STOP_SWITCH,       // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },

  {
    W_CHARGE,                   // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_ENGINE_OIL_PRESSURE,      // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_ENGINE_COOLING_TEMPERATURE,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    FALSE,                      // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_AIR_FILTER,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_TALKUK_BIN,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration                              // 10000 ms ON
    20000,                      // soundOffDuration                             // 10000 ms ON
    0,                          // timerWarning
  },
  
  {
    W_CHORI_BIN,                // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },

  {
    W_2_SENSOR,                 // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_CHIPBECHUL,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_GUGMUL_MANYANG,           // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_MULBUNRIGI,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    FALSE,                      // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_GUGMUL_MANYANG_ENGINE_STOP,               // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    1000,                       // soundPeriod
    1000,                       // soundOnDuration
    0,                          // soundOffDuration
    0,                          // timerWarning
  },

  {
    W_ENGINE_STOP_CUTTING_SAFETY, // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_ENGINE_STOP_YEACHE_SAFETY,// index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },

  {
    W_2_NASON,                  // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_YANGUG_NASON,             // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ACTIVE,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
  
  {
    W_2_NASON_BLOCK_SWITCH,     // index
    W_FLAG_PASSIVE,             // flag
    W_STATE_NO_ERROR,           // state
    W_LEVEL_ON_TOP,             // level
    W_NO_PAGE,                  // page
    TRUE,                       // checkRPM
    TRUE,                       // checkTalkuk
    WARNING_SOUND,              // soundType
    256,                        // soundPeriodNumber
    500,                        // soundPeriod
    10000,                      // soundOnDuration
    20000,                      // soundOffDuration
    0,                          // timerWarning
  },
};
/*----------------------------------------------- Warning -----------------------------------------------*/

/*----------------------------------------------- Main -----------------------------------------------*/
// Disabled on 2024.09.27 static uint8_t diesel_particulate_filter_lamp_command_blink;                    // Added on 12 April 2021
static uint8_t engine_check_lamp_bam_blink;
static uint8_t grain_sensor_4_blink;
static int16_t Angle_Object_prev;

#define NUMBER_OF_LOAD_RATE_AVERAGE             4
uint8_t load_rates[NUMBER_OF_LOAD_RATE_AVERAGE];
static uint8_t prev_load_rate;


#define COOLEANT_UPDATE_COUNTER                 100                             // 100 * 100ms = 10000 ms = 10 s
#define NUMBER_OF_COOLEANT_IMAGES               10
static uint8_t engine_coolant_temperature_gage;
static uint8_t engine_coolant_temperature_counter = 0;
static float engine_coolant_temperature_total[COOLEANT_UPDATE_COUNTER];

#define UREA_UPDATE_COUNTER                     100                             // 100 * 100ms = 10000 ms = 10 s
#define NUMBER_OF_UREA_IMAGES                   8
//static uint8_t urea_level;                                                      // For display images // tseveen 20250522
//static uint8_t urea_level_counter = 0;                                          // counter
//static float urea_level_total[UREA_UPDATE_COUNTER];                             // tseveen 20250522

static uint16_t clockActionNumber = 0;
static uint16_t timeClockAction = 0;
static uint16_t timeECUError = 0;  

#define NUMBER_OF_FUEL_IMAGES                   10
static uint8_t isFuelLower = 0;

#define IC_GARAGE_LEVEL_STEP            10
#define NUMBER_OF_IC_GARAGE_LEVEL       4
const uint16_t IC_GARAGE_LEVEL[NUMBER_OF_IC_GARAGE_LEVEL] = { 250, 450, 615, 820 };

// static uint8_t IC_garage_right_sensor_level = 0;        // tseveen 20250522
//static uint16_t IC_garage_right_sensor_prev = 0;         // tseveen 20250522
// static uint8_t IC_garage_left_sensor_level = 0;         // tseveen 20250522
// static uint16_t IC_garage_left_sensor_prev = 0;         // tseveen 20250522

#define INC_PIC_MINUS_LEVEL     -230
#define INC_PIC_PLUS_LEVEL      230
#define INC_PIC_STEP            10
static int16_t inc_pic = 0;
static int8_t inc_pic_level = 0;

#define CC_PITCH_CYLINDER_LEVEL_STEP            10
#define NUMBER_OF_CC_PITCH_CYLINDER_LEVEL       8
const uint16_t CC_PITCH_CYLINDER_LEVEL[NUMBER_OF_CC_PITCH_CYLINDER_LEVEL] = { 350, 438, 526, 614, 654, 742, 830, 918 };
static uint16_t CC_pitching_cylinder_val_prev = 0;
static int8_t CC_pitching_cylinder_val_level = 0;
/*----------------------------------------------- Main -----------------------------------------------*/

/*----------------------------------------------- Memory -----------------------------------------------*/
uint16_t nvJobHourMsb;
uint16_t nvJobHourLsb;
uint16_t nvEngineHourMsb;
uint16_t nvEngineHourLsb;
uint16_t nvEngineOilHourMsb;
uint16_t nvEngineOilHourLsb;
uint16_t nvMissionOilHourMsb;
uint16_t nvMissionOilHourLsb;
uint16_t nvNumberOfEngineOilExchange;
uint16_t nvNumberOfMissionOilExchange;
uint16_t nvLcdBrigthnessDay;
uint16_t nvLcdBrigthnessNight;
uint16_t nvModelSelection;

uint16_t nvAxelThreshingDelay;
uint16_t nvAxelAugerDelay;
uint16_t nvAxelAugerAutoDelay;

uint16_t nvAxelYeacheKuTime;
uint16_t nvAxelTbsKuTime;
uint16_t nvAxelAugerKuTime;
uint16_t nvAxelCSpeedKuTime;
uint16_t nvAxelThreshingRpm;
uint16_t nvAxelKeepTimeMode;
uint16_t nvAxelAppSensorTotalError;
uint16_t nvAxelDelayMode;

uint16_t nvAxelAppSensor1PositionMax;
uint16_t nvAxelAppSensor1PositionMin;
uint16_t nvAxelAppSensor2PositionMax;
uint16_t nvAxelAppSensor2PositionMin;

nvData_t settingsData[NUMBER_OF_CONFIGURATION] = 
{  
  //min,    max,            default,        value
  { 0,      0xFFFF,         0,              &nvJobHourMsb                   },      // 0 --> CONFIGURE_JOB_HOUR_MSB
  { 0,      0xFFFF,         0,              &nvJobHourLsb                   },      // 1 --> CONFIGURE_JOB_HOUR_LSB
  { 0,      0xFFFF,         0,              &nvEngineHourMsb                },      // 2 --> CONFIGURE_ENGINE_HOUR_MSB
  { 0,      0xFFFF,         0,              &nvEngineHourLsb                },      // 3 --> CONFIGURE_ENGINE_HOUR_LSB
  { 0,      0xFFFF,         0,              &nvEngineOilHourMsb             },      // 4 --> CONFIGURE_ENGINE_OIL_HOUR_MSB
  { 0,      0xFFFF,         0,              &nvEngineOilHourLsb             },      // 5 --> CONFIGURE_ENGINE_OIL_HOUR_LSB
  { 0,      0xFFFF,         0,              &nvMissionOilHourMsb            },      // 6 --> CONFIGURE_MISSION_OIL_HOUR_MSB
  { 0,      0xFFFF,         0,              &nvMissionOilHourLsb            },      // 7 --> CONFIGURE_MISSION_OIL_HOUR_LSB
  { 0,      0xFFFF,         0,              &nvNumberOfEngineOilExchange    },      // 8 --> CONFIGURE_ENGINE_OIL_EXCHANGE_COUNTER
  { 0,      0xFFFF,         0,              &nvNumberOfMissionOilExchange   },      // 9 --> CONFIGURE_MISSION_OIL_EXCHANGE_COUNTER
  { 1,      1000,           950,            &nvLcdBrigthnessDay             },      // 10 --> CONFIGURE_LCD_BRIGHTNESS_DAY
  { 1,      1000,           5,              &nvLcdBrigthnessNight           },      // 11 --> CONFIGURE_LCD_BRIGHTNESS_NIGHT
  { 0,      2,              1,              &nvModelSelection               },      // 12 --> CONFIGURE_MODEL_SELECTION
  { 0,      5,              3,              &nvAxelThreshingDelay           },      // 13 --> CONFIGURE_AXEL_THRESHING_DELAY
  { 0,      5,              5,              &nvAxelAugerDelay               },      // 14 --> CONFIGURE_AXEL_AUGER_DELAY
  { 0,      5,              1,              &nvAxelAugerAutoDelay           },      // 15 --> CONFIGURE_AXEL_AUGER_AUTO_DELAY
  { 0,      5,              0,              &nvAxelYeacheKuTime             },      // 16 --> CONFIGURE_AXEL_YEACHE_KU_TIME
  { 0,      5,              0,              &nvAxelTbsKuTime                },      // 17 --> CONFIGURE_AXEL_TBS_KU_TIME
  { 0,      5,              0,              &nvAxelAugerKuTime              },      // 18 --> CONFIGURE_AXEL_AUGER_KU_TIME
  { 0,      5,              3,              &nvAxelCSpeedKuTime             },      // 19 --> CONFIGURE_AXEL_C_SPEED_KU_TIME
  { 2400,   2700,           2600,           &nvAxelThreshingRpm             },      // 20 --> CONFIGURE_AXEL_THRESHING_RPM
  { 0,      0,              0,              &nvAxelKeepTimeMode             },      // 21 --> CONFIGURE_AXEL_KEEP_TIME_MODE
  { 0,      15,             0,              &nvAxelAppSensorTotalError      },      // 22 --> CONFIGURE_AXEL_APP_SENSOR_TOTAL_ERROR
  { 0,      15,             0,              &nvAxelDelayMode                },      // 23 --> CONFIGURE_AXEL_DELAY_MODE
  { 0,      4095,           6,              &nvAxelAppSensor1PositionMax    },      // 24 --> CONFIGURE_AXEL_APP_SENSOR1_POSITION_MAX
  { 0,      4095,           1083,           &nvAxelAppSensor1PositionMin    },      // 25 --> CONFIGURE_AXEL_APP_SENSOR1_POSITION_MIN
  { 0,      4095,           6,              &nvAxelAppSensor2PositionMax    },      // 26 --> CONFIGURE_AXEL_APP_SENSOR2_POSITION_MAX
  { 0,      4095,           1083,           &nvAxelAppSensor2PositionMin    },      // 27 --> CONFIGURE_AXEL_APP_SENSOR2_POSITION_MIN
};

uint8_t check_setting_data(uint8_t _isSave, uint8_t sourceAddress, uint16_t data)
{      
  uint8_t isSave = FALSE;
  
  if(sourceAddress >= NUMBER_OF_CONFIGURATION)
    return FALSE;
  
  if(_isSave == TRUE)
  {
    if(*(settingsData[sourceAddress].value) == data)
    {
      // there no update is needed because the saved data is NOT changed
      return TRUE;
    }
    
    isSave = TRUE;
    
    if((data <= settingsData[sourceAddress].maxValue) && 
       (data >= settingsData[sourceAddress].minValue))
    {
      *(settingsData[sourceAddress].value) = data;
    }
    else {
      isSave = FALSE;
    }
  } 
  else {
    if((*(settingsData[sourceAddress].value) > settingsData[sourceAddress].maxValue) || 
       (*(settingsData[sourceAddress].value) < settingsData[sourceAddress].minValue))
    {
      *(settingsData[sourceAddress].value) = settingsData[sourceAddress].defaultValue;
      isSave = TRUE;
    }
  }
  
  if(isSave == TRUE)
  {
    write_enable_93c56();
    write_93c56(sourceAddress * 2,  *(settingsData[sourceAddress].value));
    write_disable_93c56();
    return TRUE;
  }
  
  return FALSE;
}

float getEngineHour()
{
  return engineHour;
}

uint16_t get_memory(uint8_t* data)
{
  uint16_t temp = 0;
  
  if(data[1] < NUMBER_OF_CONFIGURATION)
  {
    temp = *(settingsData[data[1]].value);
  }
  
  return temp;
}

uint16_t set_memory(uint8_t* data)
{
  uint16_t temp = 0;
  
  if(data[1] < NUMBER_OF_CONFIGURATION)
  {
    temp = (data[7] << 8) | data[6];
    check_setting_data(TRUE, data[1], temp);
    
    memory_update();
    temp = *(settingsData[data[1]].value);
    
    if((data[1] == CONFIGURE_LCD_BRIGHTNESS_DAY) || (data[1] == CONFIGURE_LCD_BRIGHTNESS_NIGHT))
    {
      if(flagInput.tailLamp == ON)
      {
        update_timer(lcdBrigthnessNight);
      }
      else
      {
        update_timer(lcdBrigthnessDay);
      }
    }
  }
  return temp;
}


void default_memory_update()
{
  uint8_t i;
  
  for(i = 0; i < NUMBER_OF_CONFIGURATION; i++)
  {
    check_setting_data(TRUE, i, settingsData[i].defaultValue);
  }
  memory_update();
}

void memory_update()
{
  uint8_t i;
  
  for(i = 0; i < NUMBER_OF_CONFIGURATION; i++)
  {
    *(settingsData[i].value) = read_93c56(i * 2);

    check_setting_data(FALSE, i, *(settingsData[i].value));
  }
  
  jobHour = (float)(((uint32_t)nvJobHourMsb << 16) + nvJobHourLsb) * 0.01;
  engineHour = (float)(((uint32_t)nvEngineHourMsb << 16) + nvEngineHourLsb) * 0.01;
  engineOilHour = (float)(((uint32_t)nvEngineOilHourMsb << 16) + nvEngineOilHourLsb) * 0.01;
  missionOilHour = (float)(((uint32_t)nvMissionOilHourMsb << 16) + nvMissionOilHourLsb) * 0.01;
  numberOfEngineOilExchange  = nvNumberOfEngineOilExchange;
  numberOfMissionOilExchange = nvNumberOfMissionOilExchange;
  lcdBrigthnessDay = nvLcdBrigthnessDay;
  lcdBrigthnessNight = nvLcdBrigthnessNight;
  
  // Read
  modelSelection = nvModelSelection;
  if((modelSelection == MODEL_NEW_WITH_AXEL) || (modelSelection == MODEL_NEXT))
  {
    axelControlFunctionEnabled = TRUE;
  }
  else
  {
    axelControlFunctionEnabled = FALSE;
  }

  if(axelControlFunctionEnabled == TRUE)
  {
    // read eeprom
    axel_progVer                        = VAC_AXEL_PROGRAM_VERSION;
    axel_threshing_delay                = nvAxelThreshingDelay;
    axel_auger_delay                    = nvAxelAugerDelay;
    axel_auger_auto_delay               = nvAxelAugerAutoDelay;
      
    axel_yeache_ku_time                 = nvAxelYeacheKuTime;
    axel_tbs_ku_time                    = nvAxelTbsKuTime;
    axel_auger_ku_time                  = nvAxelAugerKuTime;
    axel_c_speed_ku_time                = nvAxelCSpeedKuTime;
    axel_threshing_rpm                  = nvAxelThreshingRpm;
    
    axel_keepTimeMode                   = nvAxelKeepTimeMode;
    axel_app_sensor_totalError          = nvAxelAppSensorTotalError;
    axel_delay_mode                     = nvAxelDelayMode;
    
    axel_app_sensor1_position_max       = nvAxelAppSensor1PositionMax;
    axel_app_sensor1_position_min       = nvAxelAppSensor1PositionMin;
    axel_app_sensor2_position_max       = nvAxelAppSensor2PositionMax;
    axel_app_sensor2_position_min       = nvAxelAppSensor2PositionMin;
  }
}

/*----------------------------------------------- Memory -----------------------------------------------*/

// ------------------------ Implementation of Functions ------------------------ 

uint8_t YGV643_SPI_TXRX(uint8_t x)
{
  uint8_t data;
  SPI_TransmitReceive(&x, &data, 1, 100);
  return data;
}

void Y643_CS_Control(uint8_t x)
{
  if(x == 0)
  {
    YGV643_ENABLE_LOW;
    //28 ns needed
    __asm volatile("NOP");                      // (1/168MHz)*5 =~ 29.76
    __asm volatile("NOP");
    __asm volatile("NOP");
    __asm volatile("NOP");
    __asm volatile("NOP");
  }
  else
  {
    __asm volatile("NOP");                      // (1/168MHz)*5 =~ 29.76
    __asm volatile("NOP");
    __asm volatile("NOP");
    __asm volatile("NOP");
    __asm volatile("NOP");
    YGV643_ENABLE_HIGH;
  }
}

/***********************************************************************************************************************
* Function Name: YVC_Reset
* Description  : VC1H Hardware reset
* Arguments    : void
* Return Value : None
***********************************************************************************************************************/
void YVC_Reset(void)
{
  //TIMER_WAIT_MSEC(10);
  WaitMSec(10);
}

/***********************************************************************************************************************
* Function Name: WaitMSec
* Description  : waiting ms 
* Arguments    : UINT16 d 
* Return Value : void
***********************************************************************************************************************/

void WaitMSec(uint16_t cnt)
{
  user_delay(cnt);
}

/***********************************************************************************************************************
* Function Name: Y643_WRITE_bytes
* Description  : Burst write function. matching to user spi driver
* Arguments    : None
* Return Value : TRUE or FALSE        
***********************************************************************************************************************/
uint8_t YVC_BurstWritePort(uint8_t pPort_num, const uint8_t *uWr_data, uint16_t num)
{
  static uint8_t wdt_counter_local = 0;
  uint8_t cmd = pPort_num;
  Y643_CS_Control(0);
  
  SPI_Transmit(&cmd, 1, 100);
  SPI_Transmit((uint8_t*) uWr_data, num, 100);
  
  Y643_CS_Control(1);
  wdt_counter_local++;
  if(wdt_counter_local > 5)
  {
    wdt_counter_local = 0;
    watchdog_trigger();
  }
  
  return TRUE;
}
/***********************************************************************************************************************
* Function Name: Y643_WRITE_bytes
* Description  : Burst write function. matching to user spi driver
* Arguments    : None
* Return Value : TRUE or FALSE        
***********************************************************************************************************************/
uint8_t YVC_BurstReadPort(uint8_t pPort_num,  uint8_t *uRd_data, uint16_t num)
{
  uint8_t ReadCmd[1] = {pPort_num | 0x08};
  
  Y643_CS_Control(0);
  SPI_Transmit(ReadCmd, 1, 100);
  SPI_Receive(uRd_data, num, 100);
  
  Y643_CS_Control(1);
  return TRUE;
}

/***********************************************************************************************************************
* Function Name: Y643_WRITE
* Description  : Single write function. matching to user spi driver
* Arguments    : None
* Return Value : TRUE or FALSE   
***********************************************************************************************************************/
uint8_t YVC_WritePort(uint8_t pPort_num, uint8_t uWr_data)
{
  uint8_t tx[2] = {pPort_num, uWr_data};
  Y643_CS_Control(0);
  
  SPI_Transmit(tx, 2, 100);
  
  Y643_CS_Control(1);
  
  return TRUE;  
}

/***********************************************************************************************************************
* Function Name: Y643_READ
* Description  : Single read function. matching to user spi driver
* Arguments    : None
* Return Value : uint8_t read value 1byte 
***********************************************************************************************************************/
uint8_t YVC_ReadPort(uint8_t pPort_num)
{
  uint8_t readData = 0;
  uint8_t tx[1] = {pPort_num | 0x08};

  Y643_CS_Control(0);
  SPI_Transmit(tx, 1, 100);
  SPI_Receive(&readData, 1, 100);
  Y643_CS_Control(1);
  
  return readData;
}

// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{
  int i = 0, j = len - 1, temp; 
  while(i < j)
  {
    temp = str[i]; 
    str[i] = str[j]; 
    str[j] = temp; 
    i++; 
    j--; 
  } 
}

// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0; 
    while(x)
    {
      str[i++] = (x % 10) + '0'; 
      x = x / 10; 
    } 

    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d)
    {
      str[i++] = '0'; 
    }
    
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 

// Converts a floating-point/double number to a string. 
void ftoa(float _n, char* res, int afterpoint) 
{ 
  float n = _n;
  int s = 0;
  
  if(n < 0)
  {
    n = n*(-1);
    s = 1;
    res[0] = '-';
  }
  // Extract integer part 
  int ipart = (int)(n);
  

  // Extract floating part 
  float fpart = n - (float)ipart; 

  // convert integer part to string 
  int i = intToStr(ipart, res + s, 0); 

  // check for display option after point 
  if(afterpoint != 0)
  {
      res[i] = '.'; // add dot 

      // Get the value of fraction part upto given no. 
      // of points after dot. The third parameter  
      // is needed to handle cases like 233.007 
      fpart = fpart * pow(10, afterpoint); 

      intToStr((int)fpart, res + i + 1, afterpoint); 
  }
  // added enkhbat this condition
  if((res[0] == 0) || ((res[0] == '.') && (res[1] == '0')))
  {
    res[0] = '0';
    res[1] = '\0';
  }
  else if((res[0] == '.') && (res[1] != '0'))
  {
    res[2] = res[1];
    res[1] = '.';
    res[0] = '0';
  }
}


void External_Video_Input_Init(void)
{
  T_YVC1_VIN_DIGITAL_IN        tYvc1VinDigitalIn;
  T_YVC1_VIN_DIGITAL_SYNC      tYvc1VinDigitalSync;
  T_YVC1_VIN_BCD_AREA          tYvc1VinBcdArea;
  T_YVC1_VIN_BCD_SCALE         tYvc1VinBcdScale;

  tYvc1VinDigitalIn.DVIF = 1;                          // 0 - RGB, 1 - 8bit YCrCb, 2 - 16bit YCrCb
  tYvc1VinDigitalIn.DVINTL = 0;                        // 0 - Progressive, 1 - Interlaced
  tYvc1VinDigitalIn.DVPAL = 0;                         // 0 - NTSC, 1 - PAL
  tYvc1VinDigitalIn.DVSP = 0;                          // 0 - L Active,1 - H Active
  tYvc1VinDigitalIn.CRCBS = 0;                         // 0 - Cb-Cr, 1 - Cr-Cb
  YVC1_VinDigitalIn(&tYvc1VinDigitalIn);
 
  tYvc1VinDigitalSync.DVCSE = 1;                       // 0 - DHSIN_N pin to feed horizontal and DVSIN_N pin to feed vertical sync,    1 - DHSIN_N pin feed composite sync signal
  tYvc1VinDigitalSync.DVCSM = 0;                       // 0 - Uses the typical values of automatically-detect horizontal period and the horizonta sync pulse width, DVHTL and DVHSW are used
  tYvc1VinDigitalSync.DVHTL = 0;                       // 0 to 2047
  tYvc1VinDigitalSync.DVHSW = 0;                       // 0 to 511
  YVC1_VinDigitalSync(&tYvc1VinDigitalSync);

#if defined (LCD_AT070TN94) || defined(LCD_MD070NF04_60ID_18A_AM)
  YVC1_VinBcdXPos(145);                                // set the starting dot to display the external video: 8-2047.75 (0.25 steps)
  YVC1_VinBcdVSA(-20);                                 // 

  tYvc1VinBcdArea.Sx =    80;                          // sx : 1-2047
  tYvc1VinBcdArea.Sy =    25;                          // sy : 1-1023      // changed on 6 April 2021
  tYvc1VinBcdArea.Width = 805;                         // width : 1-2047
  tYvc1VinBcdArea.Height = 480;                        // height : 1-1023
   
  YVC1_VinBcdArea(&tYvc1VinBcdArea);
 
  tYvc1VinBcdScale.Enable = 1;                         // 0 (FALSE) - Scaling function OFF, 1 - Scaling function ON
  tYvc1VinBcdScale.Mode = 0;                           // 0 - Bilinear method, 1 - Nearest neighbor method
  tYvc1VinBcdScale.Fx = 1.2f;                          // 0.25 to 4096
  tYvc1VinBcdScale.Fy = 2.0f;                          // 0.25 to 4096
#elif defined(LCD_RFF700A9_AWH_DNN)
  YVC1_VinBcdXPos(140);                                // set the starting dot to display the external video: 8-2047.75 (0.25 steps)
  YVC1_VinBcdVSA(20);                                  // 
 
  tYvc1VinBcdArea.Sx =    100;                         // sx : 1-2047
  tYvc1VinBcdArea.Sy =    100;                         // sy : 1-1023
  tYvc1VinBcdArea.Width = 800;                         // width : 1-2047
  tYvc1VinBcdArea.Height = 480;                        // height : 1-1023
   
  YVC1_VinBcdArea(&tYvc1VinBcdArea);
 
  tYvc1VinBcdScale.Enable = 1;                         // 0 (FALSE) - Scaling function OFF, 1 - Scaling function ON
  tYvc1VinBcdScale.Mode = 0;                           // 0 - Bilinear method, 1 - Nearest neighbor method
  tYvc1VinBcdScale.Fx = 1.2f;                          // 0.25 to 4096
  tYvc1VinBcdScale.Fy = 2.0f;                          // 0.25 to 4096
#endif
  YVC1_VinBcdScale(&tYvc1VinBcdScale);                   
  YVC1_VinBcdExtSync(1);                               // 1 (TRUE): External sync mode, 0 (FALSE): Self-propelled mode

  YVC1_VinBcdDisp(1, 0);                                // First arg: 0 (FALSE) - Does not display the backdrop plane, 1 (TRUE) - Displays the backdrop plane
}

void ygv643_initialize()
{
  uint8_t i;
  YGV643_RESET_LOW;      
  LCD_RST_LOW;               // LCD Reset
  user_delay(10);

  YGV643_RESET_HIGH;       // CHIP RESET... 
  LCD_RST_HIGH;

  user_delay(10);

  if(read_tail_light() == TRUE)
  {
    update_timer(lcdBrigthnessNight);
  }
  else
  {
    update_timer(lcdBrigthnessDay);
  }
  
  if(YVC1_Init(&tYvc1Data) == FALSE)
  {
    user_delay(1);
  }
  //YVC1_WriteReg
  YVC1_WritePlts(PaletteData_ADDR, PaletteData_LYT, PaletteData_SIZE );
  //YVC1_WriteTbls

  YVC1_SetFontTypAttr(0,(const T_Y643_FONT_TYPATTR *) Time_FC.tYgfontData);                     // 48
  YVC1_SetFontTypAttr(1,(const T_Y643_FONT_TYPATTR *) can_raw_data_1_FC.tYgfontData);           // 30
  YVC1_SetFontTypAttr(2,(const T_Y643_FONT_TYPATTR *) can_data_eg1_FC.tYgfontData);             // 28
  YVC1_SetFontTypAttr(3,(const T_Y643_FONT_TYPATTR *) Text_FC.tYgfontData);                     // 24
  YVC1_FlipTbl();
  YVC1_SetFontTypAttr(0,(const T_Y643_FONT_TYPATTR *) Time_FC.tYgfontData);                     // 48
  YVC1_SetFontTypAttr(1,(const T_Y643_FONT_TYPATTR *) can_raw_data_1_FC.tYgfontData);           // 30
  YVC1_SetFontTypAttr(2,(const T_Y643_FONT_TYPATTR *) can_data_eg1_FC.tYgfontData);             // 28
  YVC1_SetFontTypAttr(3,(const T_Y643_FONT_TYPATTR *) Text_FC.tYgfontData);                     // 24

  YVC1_CpuSetAllLyrDisp(TRUE);
  
  for(i = 0; i < UREA_UPDATE_COUNTER; i++)
  {
//    urea_level_total[i] = 0; // tseveen 20250522
  }
  for(i = 0; i < COOLEANT_UPDATE_COUNTER; i++)
  {
    engine_coolant_temperature_total[i] = 0;
  }
}

void ygb643_tw9990_initialize()
{
  spi2_enable();
  ygv643_initialize();
// Removed on 2024.10.16 Because camera function is NOT in this program tw9990_initialize(1);
  
  YVC1_CpuSetAllLyrDisp(TRUE);
  
  row_selection = 1;
  currentPageState = LOGO_PAGE;
}

void update_pages_back()
{
  row_selection = 0;
  index = 0;
  flag.isBrigthnessSetting = FALSE;
  setup_mode = SETUP_MODE_NONE;
  setup_mode_rw = SETUP_READ;
  new_setup_data_received_flag = 0;
  new_setup_data = -1;

  switch(currentPageState)
  {
    case MAIN_PAGE:
    case SETTINGS_PAGE:
      row_selection = 1;
      currentPageState = MAIN_PAGE;
      previousPageState = MAIN_PAGE;
      break;
    
    case SETTINGS_60_PAGE:       row_selection++;
    case SETTINGS_50_PAGE:       row_selection++;
    case SETTINGS_40_PAGE:       row_selection++;
    case SETTINGS_30_PAGE:       row_selection++;
    case SETTINGS_20_PAGE:       row_selection++;
    case SETTINGS_10_PAGE:       row_selection++;
      currentPageState = SETTINGS_PAGE;
      previousPageState = MAIN_PAGE;
      break;
      
    case SETTINGS_16_PAGE:       row_selection++;
    case SETTINGS_15_PAGE:       row_selection++;
    case SETTINGS_14_PAGE:       row_selection++;
    case SETTINGS_13_PAGE:       row_selection++;
    case SETTINGS_12_PAGE:       row_selection++;
    case SETTINGS_11_PAGE:       row_selection++;
      currentPageState = SETTINGS_10_PAGE;
      previousPageState = SETTINGS_PAGE;
      break;
      
    case SETTINGS_111_PAGE:
    case SETTINGS_112_PAGE:
    case SETTINGS_113_PAGE:
      currentPageState = SETTINGS_11_PAGE;
      previousPageState = SETTINGS_10_PAGE;
      break;
    
    case SETTINGS_121_PAGE:
    case SETTINGS_122_PAGE:
      currentPageState = SETTINGS_12_PAGE;
      previousPageState = SETTINGS_10_PAGE;
      break;

    case SETTINGS_131_PAGE:
    case SETTINGS_132_PAGE:
      currentPageState = SETTINGS_13_PAGE;
      previousPageState = SETTINGS_10_PAGE;
      break;

    case SETTINGS_141_PAGE:
      currentPageState = SETTINGS_14_PAGE;
      previousPageState = SETTINGS_10_PAGE;
      break;

    case SETTINGS_151_PAGE:
    case SETTINGS_152_PAGE:
    case SETTINGS_153_PAGE:
    case SETTINGS_154_PAGE:
    case SETTINGS_155_PAGE:
    case SETTINGS_156_PAGE:
    case SETTINGS_157_PAGE:
    case SETTINGS_158_PAGE:
    case SETTINGS_159_PAGE:
    case SETTINGS_1510_PAGE:
    case SETTINGS_1511_PAGE:
    case SETTINGS_1512_PAGE:
      row_selection = 5;
      currentPageState = SETTINGS_10_PAGE;
      previousPageState = SETTINGS_PAGE;
      break;
      
    case SETTINGS_161_PAGE:
    case SETTINGS_162_PAGE:
    case SETTINGS_163_PAGE:
      row_selection = 6;
      currentPageState = SETTINGS_16_PAGE;
      previousPageState = SETTINGS_10_PAGE;
      break;
  
    case SETTINGS_26x_PAGE:       row_selection++;       row_selection++;
    case SETTINGS_24x_PAGE:       row_selection++;
    case SETTINGS_23x_PAGE:       row_selection++;
    case SETTINGS_22x_PAGE:       row_selection++;
    case SETTINGS_21x_PAGE:       row_selection++;
      currentPageState = SETTINGS_20_PAGE;
      previousPageState = SETTINGS_PAGE;
      
      setup_mode = SETUP_MODE_NONE;
      new_setup_data_received_flag = 0;
      new_setup_data = -1;
      break;
  
    case SETTINGS_218_PAGE:       row_selection++;
    case SETTINGS_217_PAGE:       row_selection++;
    case SETTINGS_216_PAGE:       row_selection++;
    case SETTINGS_215_PAGE:       row_selection++;
    case SETTINGS_214_PAGE:       row_selection++;
    case SETTINGS_213_PAGE:       row_selection++;
    case SETTINGS_212_PAGE:       row_selection++;
    case SETTINGS_211_PAGE:       row_selection++;
      currentPageState = SETTINGS_21x_PAGE;
      previousPageState = SETTINGS_20_PAGE;
      break;
    
    case SETTINGS_227_PAGE:       row_selection++;
    case SETTINGS_226_PAGE:       row_selection++;
    case SETTINGS_225_PAGE:       row_selection++;
    case SETTINGS_224_PAGE:       row_selection++;
    case SETTINGS_223_PAGE:       row_selection++;
    case SETTINGS_222_PAGE:       row_selection++;
    case SETTINGS_221_PAGE:       row_selection++;
      currentPageState = SETTINGS_22x_PAGE;
      previousPageState = SETTINGS_20_PAGE;
      break;
    
    case SETTINGS_238_PAGE:       row_selection++;
    case SETTINGS_237_PAGE:       row_selection++;
    case SETTINGS_236_PAGE:       row_selection++;
    case SETTINGS_235_PAGE:       row_selection++;
    case SETTINGS_234_PAGE:       row_selection++;
    case SETTINGS_233_PAGE:       row_selection++;
    case SETTINGS_232_PAGE:       row_selection++;
    case SETTINGS_231_PAGE:       row_selection++;
      currentPageState = SETTINGS_23x_PAGE;
      previousPageState = SETTINGS_20_PAGE;
      break;
    
    case SETTINGS_241_PAGE:
      row_selection = 1;
      currentPageState = SETTINGS_24x_PAGE;
      previousPageState = SETTINGS_20_PAGE;
      break;
  
    case SETTINGS_2511_PAGE:
    case SETTINGS_2510_PAGE:
    case SETTINGS_259_PAGE:
    case SETTINGS_258_PAGE:
    case SETTINGS_257_PAGE:
    case SETTINGS_256_PAGE:
    case SETTINGS_255_PAGE:
    case SETTINGS_254_PAGE:
    case SETTINGS_253_PAGE:
    case SETTINGS_252_PAGE:
    case SETTINGS_251_PAGE:
      row_selection = 5;
      currentPageState = SETTINGS_20_PAGE;
      previousPageState = SETTINGS_PAGE;
      break;
      
    case SETTINGS_262_PAGE:       row_selection++;
    case SETTINGS_261_PAGE:       row_selection++;
      currentPageState = SETTINGS_26x_PAGE;
      previousPageState = SETTINGS_20_PAGE;
      break;
    
    case SETTINGS_2xxF_PAGE:
      currentPageState = SETTINGS_20_PAGE;
      previousPageState = SETTINGS_PAGE;
      break;

    case SETTINGS_36x_PAGE:       row_selection++;
    case SETTINGS_35x_PAGE:       row_selection++;
    case SETTINGS_34x_PAGE:       row_selection++;
    case SETTINGS_33x_PAGE:       row_selection++;
    case SETTINGS_32x_PAGE:       row_selection++;
    case SETTINGS_31x_PAGE:       row_selection++;
      currentPageState = SETTINGS_30_PAGE;
      previousPageState = SETTINGS_PAGE;
      break;
      
    case SETTINGS_315_PAGE:       row_selection++;
    case SETTINGS_314_PAGE:       row_selection++;
    case SETTINGS_313_PAGE:       row_selection++;
    case SETTINGS_312_PAGE:       row_selection++;
    case SETTINGS_311_PAGE:       row_selection++;
      currentPageState = SETTINGS_31x_PAGE;
      previousPageState = SETTINGS_30_PAGE;
      break;
      
    case SETTINGS_3113_PAGE:    row_selection++;
    case SETTINGS_3112_PAGE:    row_selection++;
    case SETTINGS_3111_PAGE:    row_selection++;
      currentPageState = SETTINGS_311_PAGE;
      previousPageState = SETTINGS_31x_PAGE;
      break;
    
    case SETTINGS_3143_PAGE:    row_selection++;
    case SETTINGS_3142_PAGE:    row_selection++;
    case SETTINGS_3141_PAGE:    row_selection++;
      currentPageState = SETTINGS_314_PAGE;
      previousPageState = SETTINGS_31x_PAGE;
      break;
    
    case SETTINGS_331_PAGE:
    case SETTINGS_332_PAGE:
      row_selection++;
      currentPageState = SETTINGS_33x_PAGE;
      previousPageState = SETTINGS_30_PAGE;
      break;
      
  case WARNING_PAGE:
    row_selection = 1;
    currentPageState = previousPageState;
    break;
  }
}

uint16_t draw_logo_page(uint16_t _imgCnt)
{
  static uint16_t logoIndex = 0;
  static uint16_t logoTimer = 0;
  
  uint16_t imgCnt = _imgCnt;
  
  YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[Layer_1 + (logoIndex * 2)], -25, 0, 0);
  YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[Layer_2 + (logoIndex * 2)], -25, 0, 0);
  
  logoIndex++;
  if(logoIndex >= (NUMBER_OF_LOGO_IMAGE - 1))
  {
    currentPageState = MAIN_PAGE;
    logoIndex = NUMBER_OF_LOGO_IMAGE - 1;
    logoTimer += 100;
    if(logoTimer >= LOGO_TIME)
    {
      logoTimer = LOGO_TIME;
      currentPageState = MAIN_PAGE;
    }
  }
  
  return imgCnt;
}

/*------------------------------------- Main page -------------------------------------*/
uint16_t timer_inducement_severity_for_DEF_tank_level = 0;
uint16_t timer_operator_inducement_active_for_SCR_system_failure = 0;
uint16_t timer_pending_scr_inducement_severity = 0;
uint8_t load_rates_counter = 0;      

uint8_t imageSwitchOne = FALSE;
uint8_t imageSwitchTwo = FALSE;
uint16_t timerImageSwitchOne = 0;
uint16_t timerImageSwitchTwo = 0;

uint16_t draw_main_page(uint16_t _imgCnt)
{
  static uint8_t firstTime = TRUE;
  uint16_t imgCnt = _imgCnt;
  uint8_t i;
  
  int16_t Angle_Object = 0;
  C08 hour_string[8];
  uint8_t load_rate = 0;
  uint8_t load_rate_img = 0;
  float temp;
 
  
  // ----------------------------- Background ----------------------------- 
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[BG_001]);
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[BG_002]);
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[BG_kukmul]);        // tseveen sub 20250522
//  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusuu]); // tseveen 20250522 added ysusu
  
  // ----------------------------- WarningBox ----------------------------- 
    if(flagWarning.index == W_ENGINE_COOLING_TEMPERATURE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningyusu]);      // tseveen 20250522   added yusu
  }
    else if(flagWarning.index == W_CHARGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningchunjon]);      // tseveen 20250529   added chunjon
  } 
    else if(flagWarning.index == W_ENGINE_OIL_PRESSURE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningchunjon]);      // tseveen 20250529   added engine_oil
  } 
    else if(flagWarning.index == W_CHORI_BIN)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningtalkuk]);      // tseveen 20250529   added choiri
  }
    else if(flagWarning.index == W_TALKUK_BIN)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningtalkuk]);      // tseveen 20250529   added talkuk
  }  
    else if(flagWarning.index == W_2_SENSOR)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warning2bon]);      // tseveen 20250529   added 2bon
  }
    else if(flagWarning.index == W_GUGMUL_MANYANG)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[warningkukmul]);      // tseveen 20250529   added kukmul
  }     

  if(flag.engineStarted == FALSE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[clock1]);
    clockActionNumber = 0;
    timeClockAction = 0;
  }
  else
  {
    timeClockAction += 100;
    if(timeClockAction >= CLOCK_ACTION_TIME)
    {
      timeClockAction = 0;
      clockActionNumber++;
      if(clockActionNumber >= CLOCK_ACTION_NUMBER)
        clockActionNumber = 0;
    }
    
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[clock1 + clockActionNumber]);
  }
  
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[clock_box]);
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[error_box]);
  
  // ----------------------------- Top status lights based on digital input --------------------------
  if(flagInput.leftLamp) 
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[turn_left]);
  }
  if(flagInput.tailLamp)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[front_light]);
  }
  if(flagInput.rightLamp)           {YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[turn_right]);}
  if(flagInput.charge)              {YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[charge_lamp]);}
  if(flagWarning.oilPressure)       {YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[oil_lamp]);}
  
  // ----------------------------- Top status lights based on CAN data --------------------------
  //inducement_severity_for_DEF_tank_level
  
  if(((pending_scr_inducement_severity == 1) || (pending_scr_inducement_severity == 5)) && 
      ((digital_output2_status & 0x04) == 0x04))
  {
    // Both image should be shown
    imageSwitchOne = TRUE;
  }
  else if(((pending_scr_inducement_severity != 1) && (pending_scr_inducement_severity != 5)) && 
          ((digital_output2_status & 0x04) == 0x00))
  {
    timerImageSwitchOne = 0;
    imageSwitchOne = FALSE;
  }
  
  if(imageSwitchOne == TRUE)
  {
    timerImageSwitchOne += 100;
    if(timerImageSwitchOne > 6000)
      timerImageSwitchOne = 0;
  }
  
  if((imageSwitchOne == FALSE) || ((imageSwitchOne == TRUE) && (timerImageSwitchOne <= 3000)))
  {  
    if(pending_scr_inducement_severity == 1)
    {
      timer_pending_scr_inducement_severity = 0;
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Emission_failure_indicator]);   // removed
    }
    else if(pending_scr_inducement_severity == 5)
    {
      timer_pending_scr_inducement_severity += 100;
      if(timer_pending_scr_inducement_severity <= 1000)
      {
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Emission_failure_indicator]); // removed
      }
      else if(timer_pending_scr_inducement_severity > 2000)
      {
        timer_pending_scr_inducement_severity = 0;
      }
    }
    else
    {
      timer_pending_scr_inducement_severity = 0;
    }
  }
  
  if((imageSwitchOne == FALSE) || ((imageSwitchOne == TRUE) && (timerImageSwitchOne > 3000)))
  {
    if(digital_output2_status & 0x04)
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check002DeSox_active]); // removed
    }
  }
  
  if((digital_output2_status & 0x03) == 0x03)
  {
    // Both image should be shown
    imageSwitchTwo = TRUE;
  }
  else if((digital_output2_status & 0x03) == 0x00)
  {
    timerImageSwitchOne = 0;
    imageSwitchOne = FALSE;
  }
  
  if(imageSwitchTwo == TRUE)
  {
    timerImageSwitchTwo += 100;
    if(timerImageSwitchTwo > 6000)
      timerImageSwitchTwo = 0;
  }
   
  if((imageSwitchTwo == FALSE) || ((imageSwitchTwo == TRUE) && (timerImageSwitchTwo <= 3000)))
  {
    if(digital_output2_status & 0x02)
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Desox_inhibit_by_switch]);      // removed
    }
  }
  
  if((imageSwitchTwo == FALSE) || ((imageSwitchTwo == TRUE) && (timerImageSwitchTwo > 3000)))
  {
    if(digital_output2_status & 0x01)
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check002forced_Desox]); // removed
    }
  }

  /* Disabled on 2024.09.27
  if(exhaust_system_high_temperature_lamp_command) YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check002DeSox_active]);
  
  if(diesel_particulate_filter_lamp_command == 1) YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check002forced_Desox]);
  else if(diesel_particulate_filter_lamp_command == 4){
    diesel_particulate_filter_lamp_command_blink++;
    if(diesel_particulate_filter_lamp_command_blink < 10) YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check002forced_Desox]);
    else if(diesel_particulate_filter_lamp_command_blink >= 20) diesel_particulate_filter_lamp_command_blink = 0;
  }
  if(diesel_particulate_filter_active_regeneration_inhibited_inhibit_switch)    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Desox_inhibit_by_switch]);
  */
  
  if(operator_inducement_active_for_SCR_system_failure != 0)
  {
    timer_operator_inducement_active_for_SCR_system_failure += 100;
    if(timer_operator_inducement_active_for_SCR_system_failure <= 500)
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Engine_Warning]);       // removed
    }
    else if(timer_operator_inducement_active_for_SCR_system_failure > 1000)
    {
      timer_operator_inducement_active_for_SCR_system_failure = 0;
    }
  }
  else if(error_message_ecu[0] != ' ')
  {
//    if(engine_check_lamp_bam_blink < 10) YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[check001Engine_Warning]);    
    timer_operator_inducement_active_for_SCR_system_failure = 0;
  }
  else
  {
    timer_operator_inducement_active_for_SCR_system_failure = 0;
  }
/*  
  if(inducement_severity_for_DEF_tank_level == 1)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);
    timer_inducement_severity_for_DEF_tank_level = 0;
  }
  else if(inducement_severity_for_DEF_tank_level == 3)
  {
    timer_inducement_severity_for_DEF_tank_level += 100;
    if(timer_inducement_severity_for_DEF_tank_level <= 1000)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);
    }
    else if(timer_inducement_severity_for_DEF_tank_level > 2000)
    {
      timer_inducement_severity_for_DEF_tank_level = 0;
    }
  }
  else if(inducement_severity_for_DEF_tank_level == 5)
  {
    timer_inducement_severity_for_DEF_tank_level += 100;
    if(timer_inducement_severity_for_DEF_tank_level <= 500)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);
    }
    else if(timer_inducement_severity_for_DEF_tank_level > 1000)
    {
      timer_inducement_severity_for_DEF_tank_level = 0;
    }
  }
  else
  {
    timer_inducement_severity_for_DEF_tank_level = 0;      
  }
  
  temp = 0;
  urea_level_counter++;
  if(urea_level_counter >= UREA_UPDATE_COUNTER)
  {
    urea_level_counter = 0;
  }
  urea_level_total[urea_level_counter] = (float)aftertreatment1_diesel_exhaust_fluid_tank_level * 8 / 250; // max value is 100 and resolution is 0.4 => 1/100 * 0.4 = 1/250
  for(i = 0; i < UREA_UPDATE_COUNTER; i++)
  {
    temp += urea_level_total[i];
  }
  
  if(firstTime == TRUE)
  {
    for(i = 0; i < UREA_UPDATE_COUNTER; i++)
    {
      urea_level_total[i] = urea_level_total[urea_level_counter];
    }
    urea_level = (uint8_t) urea_level_total[urea_level_counter];
  }
  else
  {
    urea_level = (uint8_t)((float)temp / (float)UREA_UPDATE_COUNTER);
  }
  // ----------------------------- Draw Urea ------------------------------
  if(urea_level > NUMBER_OF_UREA_IMAGES)
  {
    urea_level = NUMBER_OF_UREA_IMAGES;
  }
  for(i = 0 ; i < urea_level; i++)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[ureaorange001+i]);
  }
*/ //tseveen 20250522  
  // ----------------------------- Draw Fuel ------------------------------
  if(isFuelLower == 0)
  {
    if(tFuelPercent < FUEL_RED_IMAGE_ON_LEVEL)
    {
      isFuelLower = 1;
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[fuel_red]);
    }
  }
  else
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[fuel_red]);
    if(tFuelPercent > FUEL_RED_IMAGE_OFF_LEVEL)
    {
      isFuelLower = 0;      
    }
  }
  
  tFuelGage = (uint8_t)((float)tFuelPercent / 10.0) + 1;
  if(tFuelGage > NUMBER_OF_FUEL_IMAGES)
  {
    tFuelGage = NUMBER_OF_FUEL_IMAGES;
  }
  for(i = 0 ; i < tFuelGage ; i++)                                                                                    // the "tfuelGage" calculation is little bit modified
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[fuelorange001+i]);
  }
  
  // ----------------------------- Draw Temp ------------------------------
  temp= 0;
  engine_coolant_temperature_counter++;
  if(engine_coolant_temperature_counter >= COOLEANT_UPDATE_COUNTER)
  {
    engine_coolant_temperature_counter = 0;
  }
  engine_coolant_temperature_total[engine_coolant_temperature_counter] = (float)engine_coolant_temperature / 25;
  for(i = 0; i < COOLEANT_UPDATE_COUNTER; i++)
  {
    temp += engine_coolant_temperature_total[i];
  }
  
  if(firstTime == TRUE)
  {
    for(i = 0; i < COOLEANT_UPDATE_COUNTER; i++)
    {
      engine_coolant_temperature_total[i] = engine_coolant_temperature_total[engine_coolant_temperature_counter];
    }
    engine_coolant_temperature_gage = (uint8_t) engine_coolant_temperature_total[engine_coolant_temperature_counter];
  }
  else
  {
    engine_coolant_temperature_gage = (uint8_t)((float)temp / (float)COOLEANT_UPDATE_COUNTER);
  }
  
  if(engine_coolant_temperature_gage > NUMBER_OF_COOLEANT_IMAGES)
  {
    engine_coolant_temperature_gage = NUMBER_OF_COOLEANT_IMAGES;
  }
  for( i = 0 ; i < engine_coolant_temperature_gage; i ++)
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[water_tempwhite001 + i]); tseveen 20250522 removed img
  }
/*  
  // ----------------------------- Draw Height ------------------------------
  for(i = 0; i < NUMBER_OF_IC_GARAGE_LEVEL; i++)
  {
    if(IC_garage_right_sensor < IC_GARAGE_LEVEL[i])
    {
      if(IC_garage_right_sensor_prev >= IC_GARAGE_LEVEL[i])
      {
        if(IC_garage_right_sensor < (IC_GARAGE_LEVEL[i] - IC_GARAGE_LEVEL_STEP))
        {
          IC_garage_right_sensor_level = i;
          IC_garage_right_sensor_prev = IC_garage_right_sensor;
        }
        else
        {
          IC_garage_right_sensor_level = i + 1;
        }
      }
      else
      {
        IC_garage_right_sensor_level = i;
        IC_garage_right_sensor_prev = IC_garage_right_sensor;
      }
      break;
    }
  }
  if(i == NUMBER_OF_IC_GARAGE_LEVEL)
  {
    IC_garage_right_sensor_level = 4;
    IC_garage_right_sensor_prev = IC_garage_right_sensor;
  }
    
  if(IC_garage_right_sensor_level > 0)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height007]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel002]);  
  }
  if(IC_garage_right_sensor_level > 1)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height006]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel004]); 
  }
  if(IC_garage_right_sensor_level > 2)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height004]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel006]); 
  }
  if(IC_garage_right_sensor_level > 3)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height002]);
  }
  
  for(i = 0; i < NUMBER_OF_IC_GARAGE_LEVEL; i++)
  {
    if(IC_garage_left_sensor < IC_GARAGE_LEVEL[i])
    {
      if(IC_garage_left_sensor_prev >= IC_GARAGE_LEVEL[i])
      {
        if(IC_garage_left_sensor < (IC_GARAGE_LEVEL[i] - IC_GARAGE_LEVEL_STEP))
        {
          IC_garage_left_sensor_level = i;
          IC_garage_left_sensor_prev = IC_garage_left_sensor;
        }
        else
        {
          IC_garage_left_sensor_level = i + 1;
        }
      }
      else
      {
        IC_garage_left_sensor_level = i;
        IC_garage_left_sensor_prev = IC_garage_left_sensor;
      }
      break;
    }
  }
  if(i == NUMBER_OF_IC_GARAGE_LEVEL)
  {
    IC_garage_left_sensor_level = 4;
    IC_garage_left_sensor_prev = IC_garage_left_sensor;
  }
  
  if(IC_garage_left_sensor_level > 0)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height008]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel001]); 
  }
  if(IC_garage_left_sensor_level > 1)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height005]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel003]); 
  }
  if(IC_garage_left_sensor_level > 2)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height003]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglelevel005]); 
  }
  if(IC_garage_left_sensor_level > 3)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[height001]);
  } */ //tseveen 20250522
  // ----------------------------- Draw Roll angle ------------------------------
  
  inc_pic = IC_garage_left_sensor - IC_garage_right_sensor;                                             
  
  if(inc_pic < INC_PIC_MINUS_LEVEL)
  {
    inc_pic_level = -1;
  }
  else if(inc_pic > INC_PIC_PLUS_LEVEL)
  {
    inc_pic_level = 1;
  }
  else
  {
    if((inc_pic_level == 1) && (inc_pic > (INC_PIC_PLUS_LEVEL - INC_PIC_STEP)))
    {
      inc_pic_level = 1;
    }
    else if((inc_pic_level == -1) && (inc_pic < (INC_PIC_MINUS_LEVEL + INC_PIC_STEP)))
    {
      inc_pic_level = -1;
    }
    else
    {
      inc_pic_level = 0;
    }
  }
    
  if(inc_pic_level == -1)
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglereft]);                // check left or right tseveen removed img
  }
  else if(inc_pic_level == 1)
  {                                                                                                     // Need Interval
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_angleright]);               // check left or right tseveen removed img
  }
  else
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[rolling_anglenormal]); tseveen removed img
  }
  
  // ----------------------------- Draw pitch angle ------------------------------
  
  
  for(i = 0; i < NUMBER_OF_CC_PITCH_CYLINDER_LEVEL; i++)
  {
    if(CC_pitching_cylinder_val < CC_PITCH_CYLINDER_LEVEL[i])
    {
      if(CC_pitching_cylinder_val_prev >= CC_PITCH_CYLINDER_LEVEL[i])
      {
        if(CC_pitching_cylinder_val < (CC_PITCH_CYLINDER_LEVEL[i] - CC_PITCH_CYLINDER_LEVEL_STEP))
        {
          CC_pitching_cylinder_val_level = i - 4;
          CC_pitching_cylinder_val_prev = CC_pitching_cylinder_val;
        }
        else
        {
          CC_pitching_cylinder_val_level = i - 3;
        }
      }
      else
      {
        CC_pitching_cylinder_val_level = i - 4;
        CC_pitching_cylinder_val_prev = CC_pitching_cylinder_val;
      }
      break;
    }
  }
  if(i == NUMBER_OF_CC_PITCH_CYLINDER_LEVEL)
  {
    CC_pitching_cylinder_val_level = i - 4;
    CC_pitching_cylinder_val_prev = CC_pitching_cylinder_val;
  }
  
  if(CC_pitching_cylinder_val_level == 0)
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglenormal]); tseveen removed img
  }
/*  else
 {
     if(CC_pitching_cylinder_val_level < 0)
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglefront]); 

      if(CC_pitching_cylinder_val_level < 0)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel001]);    
      if(CC_pitching_cylinder_val_level < -1)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel002]); 
      if(CC_pitching_cylinder_val_level < -2)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel003]);
      if(CC_pitching_cylinder_val_level < -3)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel004]);
    }
    else
    {
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglerear]); 
      
      if(CC_pitching_cylinder_val_level > 0)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel008]);
      if(CC_pitching_cylinder_val_level > 1)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel007]); 
      if(CC_pitching_cylinder_val_level > 2)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel006]);
      if(CC_pitching_cylinder_val_level > 3)
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[pitching_anglelevel005]); 
    }
  } */ //tseveen removed img 20250520
  // ----------------------------- Load Rate -----------------------------
  if(engine_gross_load_ratio_1 == 0xFE)
  {
    // SENSOR ERROR
    prev_load_rate = 0;
  }
  else if(engine_gross_load_ratio_1 == 0xFF)
  {
    // NOT AVAILABLE
    prev_load_rate = 0;
  }
  else
  {
    load_rates_counter++;
    if(load_rates_counter >= NUMBER_OF_LOAD_RATE_AVERAGE)
      load_rates_counter = 0;
    
    load_rates[load_rates_counter] = engine_gross_load_ratio_1;
    temp = 0;
    for(i = 0; i < NUMBER_OF_LOAD_RATE_AVERAGE; i++)
    {
      temp += load_rates[i];
    }
    load_rate = (uint8_t)(temp / (float)NUMBER_OF_LOAD_RATE_AVERAGE);
  /*
  }
    
  if(load_rate <= 2)  { load_rate_img = 0; }
  else if(load_rate <= 10) { load_rate_img = 1; }
  else if(load_rate <= 20) { load_rate_img = 2; }
  else if(load_rate <= 30) { load_rate_img = 3; }
  else if(load_rate <= 40) { load_rate_img = 4; }
  else if(load_rate <= 50) { load_rate_img = 5; }

  else if(load_rate <= 54) { load_rate_img = 6; }
  else if(load_rate <= 58) { load_rate_img = 7; }
  else if(load_rate <= 62) { load_rate_img = 8; }
  else if(load_rate <= 68) { load_rate_img = 9; }
  else if(load_rate <= 70) { load_rate_img = 10; }
  
  else if(load_rate <= 72) { load_rate_img = 11; }
  else if(load_rate <= 74) { load_rate_img = 12; }
  else if(load_rate <= 76) { load_rate_img = 13; }
  else if(load_rate <= 78) { load_rate_img = 14; }
  else if(load_rate <= 80) { load_rate_img = 15; }
  
  else if(load_rate <= 82) { load_rate_img = 16; }
  else if(load_rate <= 84) { load_rate_img = 17; }
  else if(load_rate <= 86) { load_rate_img = 18; }
  else if(load_rate <= 88) { load_rate_img = 19; }
  else if(load_rate <= 90) { load_rate_img = 20; }
  
  else if(load_rate <= 92) { load_rate_img = 21; }
  else if(load_rate <= 94) { load_rate_img = 22; }
  else if(load_rate <= 96) { load_rate_img = 23; }
  else if(load_rate <= 98) { load_rate_img = 24; }
  else                     { load_rate_img = 25; }
  */

    if(((load_rate + 4) > prev_load_rate) && ((load_rate - 4) < prev_load_rate)){
      prev_load_rate = load_rate;
    }
    else if(load_rate > prev_load_rate){
      prev_load_rate += 4;
    }
    else if(load_rate < prev_load_rate){
      prev_load_rate -= 4;
      if(prev_load_rate > 200) prev_load_rate = 0;
    }
  }
  // 50/6 = 8
       if(prev_load_rate <= 2)  { load_rate_img = 0; }
  else if(prev_load_rate <= 10) { load_rate_img = 1; }
  else if(prev_load_rate <= 20) { load_rate_img = 2; }
  else if(prev_load_rate <= 30) { load_rate_img = 3; }
  else if(prev_load_rate <= 40) { load_rate_img = 4; }
  else if(prev_load_rate <= 50) { load_rate_img = 5; }

  else if(prev_load_rate <= 54) { load_rate_img = 6; }
  else if(prev_load_rate <= 58) { load_rate_img = 7; }
  else if(prev_load_rate <= 62) { load_rate_img = 8; }
  else if(prev_load_rate <= 68) { load_rate_img = 9; }
  else if(prev_load_rate <= 70) { load_rate_img = 10; }
   
  else if(prev_load_rate <= 72) { load_rate_img = 11; }
  else if(prev_load_rate <= 74) { load_rate_img = 12; }
  else if(prev_load_rate <= 76) { load_rate_img = 13; }
  else if(prev_load_rate <= 78) { load_rate_img = 14; }
  else if(prev_load_rate <= 80) { load_rate_img = 15; }
  
  else if(prev_load_rate <= 82) { load_rate_img = 16; }
  else if(prev_load_rate <= 84) { load_rate_img = 17; }
  else if(prev_load_rate <= 86) { load_rate_img = 18; }
  else if(prev_load_rate <= 88) { load_rate_img = 19; }
  else if(prev_load_rate <= 90) { load_rate_img = 20; }
  
  else if(prev_load_rate <= 92) { load_rate_img = 21; }
  else if(prev_load_rate <= 94) { load_rate_img = 22; }
  else if(prev_load_rate <= 96) { load_rate_img = 23; }
  else if(prev_load_rate <= 98) { load_rate_img = 24; }
  else                          { load_rate_img = 25; }
  /*
  else if(prev_load_rate > 70)
  {
    // (70 - 50)/ 4 = 5
    if(prev_load_rate >= 100)
    {
      prev_load_rate = 100; 
    }
    
    load_rate_img = 10 + (100 - prev_load_rate) / 2;      
  }
  */
  if(load_rate_img > 25)
  { 
    load_rate_img = 25;
  }
  
  for(i = 0 ; i < load_rate_img; i++)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[load_rate001 + i]);
  }
  // ----------------------------- Draw RPM ------------------------------
  if((engine_speed & 0xFF00) == 0xFE00)
  {
    // SENSOR ERROR
  }
  else if((engine_speed & 0xFF00) == 0xFF00)
  {
    // NOT AVAILABLE
  }
  else
  {
    if(tEngineSpeed >= 3000)                                                    // 3000 to RPM
      Angle_Object = 198;  
    else if(tEngineSpeed >= 1000)                                               // 1000 to 3000 RPM
      Angle_Object = (uint16_t)(((float)(tEngineSpeed - 1000) * 0.09) + 19);    // 020 - 199 = 180/2000 = 0.09
    else                                                                        // 0 to 1000 RPM
      Angle_Object = (uint16_t)((float)tEngineSpeed * 0.019);                   // 001 - 019 = 19
  }

  if((Angle_Object - Angle_Object_prev) == 0)
  {
    Angle_Object_prev = Angle_Object;
  }
  else if((Angle_Object - Angle_Object_prev) > 0)
  {
    // RPM increasing
    if((Angle_Object - Angle_Object_prev) > 7)
    {
      Angle_Object_prev += 7;
    }
    else
    {
      if((Angle_Object - Angle_Object_prev) > 2)
      {
        Angle_Object_prev = Angle_Object;
      }
    }
  }
  else
  {
    // RPM decreasing
    if((Angle_Object_prev - Angle_Object) > 7)
    {
      Angle_Object_prev -= 7;
    }
    else
    {
      if((Angle_Object_prev - Angle_Object) > 2)
      {
        Angle_Object_prev = Angle_Object;
      }
      //Angle_Object_prev = Angle_Object;
    }
  }
  if ( Angle_Object_prev >= 198 )
    Angle_Object_prev = 198;
  else if(Angle_Object_prev < 0)
    Angle_Object_prev = 0;
  
  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_RPM_GT[Angle_Object_prev]);
  
  // ----------------------------- Draw Gokmul ------------------------------

  if(flagInput.grain_4)
  {
    grain_sensor_4_blink++;
    if(grain_sensor_4_blink < 5)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul004]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul003]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul002]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul001]);
    }
    else if(grain_sensor_4_blink >= 10) 
    {
      grain_sensor_4_blink = 0;
    }
  }
  else if(flagInput.grain_3)
  {    
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul003]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul002]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul001]);
  }
  else if(flagInput.grain_2)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul002]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul001]);
  }
  else if(flagInput.grain_1)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[gokmul001]);
  }
  // ----------------------------- Draw Texts ------------------------------
  
  memset(hour_string, 0, 8);
  ftoa(engineHour, hour_string, 1);
  hour_string[7] = 0;
     
  YVC1_SetChar(Text_FC, hour_string);
  YVC1_SetFontAttr(imgCnt++,(const T_Y643_LYR_FONTATTR *)MAIN_GT[Text]);
  
  timeECUError += 100;
  if(timeECUError < PCODE_SWITCHING_TIME)
  {
    YVC1_SetChar(Error_message_1_FC, error_message_ecu);
    YVC1_SetFontAttr(imgCnt++,(const T_Y643_LYR_FONTATTR *)MAIN_GT[Error_message_1]);
  }
  else if(timeECUError < (PCODE_SWITCHING_TIME * 2))
  {
  }
  else 
  {
    timeECUError = 0;
  }

  firstTime = FALSE;
  
  return imgCnt;
}

/*------------------------------------- Warning pages -------------------------------------*/
uint16_t DrawWarningCombineCheckMessage(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  char _string[5] = {'1','2','.','0',0};
  char _string1[5] = {'0','.','0','0',0};;
  float   f_voltage;
  uint16_t voltage;
        
//  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_001]); tseveen 20250522  sub 
//  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_002]); tseveen 20250522  sub 

  if((flagWarning.index >= W_AUGER_ROTATION_SENSOR) && (flagWarning.index <= W_ONETOUCH_UP_DOWN))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_TITLE1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_TITLE2]);
  }
  
  if(flagWarning.index == W_AUGER_ROTATION_SENSOR)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH9_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH9_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH9_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage]);

    f_voltage = (float) AG_auger_potentiometer_value * 5.0 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100.0);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;  

    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
  }
  else if(flagWarning.index == W_LSA_MOTOR_POSITION_SENSOR)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH10_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH10_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH10_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_percent]);
    
//    f_voltage = (float) LSA_multiturn_position * 5.0 / 1023.0;
    voltage = (uint16_t) (dataCAN350.LSA_lsa_motor_output_duty_ratio);
    
    _string[0] = 0;
    _string[1] = 0;
    _string[2] = 0;
    
    ftoa(voltage, _string, 0);
    
    _string[3] = 0;
    _string[4] = 0;  
    
    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);    
  }
  
  else if(flagWarning.index == W_TBS_SLOPE_SENSOR)
  {
    // TBS 경사 샌서 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH1_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH1_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH1_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage]);
    
    f_voltage = (float) IC_inclination * 5.0 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100.0);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;  

    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
  }
  else if(flagWarning.index == W_TBS_RIGHT_SENSOR)
  {
    // TBS 차고 좌특 센서 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH3_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH3_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH3_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage]);
    
    f_voltage = (float) IC_garage_right_sensor * 5.0 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100.0);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;

    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
  }
  else if(flagWarning.index == W_TBS_LEFT_SENSOR)
  {
    // TBS 차고 좌특 센서 에러 
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH2_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH2_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH2_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage]);
    
    f_voltage = (float) IC_garage_left_sensor * 5.0 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100.0);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;
    
    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
  }
  else if(flagWarning.index == W_TBS_MANUAL_SWITCH)
  {
    // TBS 수동 스위치 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_message_2]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_message_3]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH4_message_4]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 100, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 150, 0);
    
    if(dataCAN310.IC_tbs_manual_down_sw == ON)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN310.IC_tbs_manual_up_sw == ON){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);

    if(dataCAN310.IC_tbs_manual_right_down_sw == ON){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 100 , 0);
      YVC1_SetChar(Data3_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 100 , 0);
      YVC1_SetChar(Data3_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data3_value]);
    
    if(dataCAN310.IC_tbs_manual_right_up_sw == ON){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 150 , 0);
      YVC1_SetChar(Data4_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 150 , 0);
      YVC1_SetChar(Data4_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data4_value]);    
  }
  else if(flagWarning.index == W_AUGER_MANAUL_SWITCH)
  {
    // 오거 수동 스위치 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_message_2]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_message_3]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH5_message_4]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 100, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 150, 0);

    if(dataCAN310.IC_auger_manual_right_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN310.IC_auger_manual_left_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);
    
    if(dataCAN310.IC_auger_manual_up_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 100 , 0);
      YVC1_SetChar(Data3_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 100 , 0);
      YVC1_SetChar(Data3_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data3_value]);
    
    if(dataCAN310.IC_auger_manual_down_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 150 , 0);
      YVC1_SetChar(Data4_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 150 , 0);
      YVC1_SetChar(Data4_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data4_value]);
  }
  else if(flagWarning.index == W_AUGER_SETTING_SWITCH)
  {
    // 오거 설정 북귀 스위치 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH6_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH6_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH6_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH6_message_2]);
    
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    
    if(dataCAN310.IC_auger_setting_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);

    if(dataCAN310.IC_auger_return_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);
  }
  else if(flagWarning.index == W_LSA_MH_SENSOR)
  {
    // LSA M/H 센서 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH7_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH7_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH7_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH7_message_2]);
    
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);

    if(dataCAN310.IC_lsa_manual_rise_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN310.IC_lsa_manual_descent_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);    
  }
  else if(flagWarning.index == W_LSA_MANUAL_SWITCH)
  {
    // LSA 수동 스위치 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH8_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH8_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH8_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH8_message_2]);
    
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    
    if(dataCAN310.IC_lsa_manual_descent_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN310.IC_lsa_manual_rise_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);
  }

  else if(flagWarning.index == W_YEACHE_LIFT_SENSOR)
  {
    // 예취 리프트 센서 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH11_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH11_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH11_message_1]);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    
    f_voltage = (float) CC_lift_sensor_val * 5.0 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100.0);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;  

    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);    
  }
  else if(flagWarning.index == W_PREVIEW_SENSOR)
  {
    // 예고 센서 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH12_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH12_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH12_message_1]);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
  
    f_voltage = (float) CC_power_clutch_sensor * 5 / 1023.0;
    voltage = (uint16_t) (f_voltage * 100);
    
    _string[0] = ((voltage%1000)/100) + 0x30;
    _string[1] = '.';
    _string[2] = ((voltage%100)/10) + 0x30;
    _string[3] = (voltage%10) + 0x30;
    _string[4] = 0;  
    YVC1_SetChar(Data1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);

  }
  else if(flagWarning.index == W_YEACHE_MANAUL_SWITCH)
  {
    // 예취 수동 스위치 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH13_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH13_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH13_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH13_message_2]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    
    if(dataCAN330.CC_harvesting_up_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN330.CC_harvesting_down_sw){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);

  }
  else if(flagWarning.index == W_TALKUK_CLUTCH_CONNECTION_BLOCK)
  {
    // 탈곡클러치 연결 차단 에러
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH14_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH14_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH14_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH14_message_2]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);

    if(dataCAN330.CC_threshing_clutch_blocking_sw == ON)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN330.CC_threshing_clutch_connection_sw == ON)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);
  }
  else if(flagWarning.index == W_TALKUK_CLUTCH_CONNECTION_BLOCK_LSA_MOTOR)
  {
    // 예취클러치 연결 차단 에러 --> LSA모터
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH15_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH15_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH15_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH15_message_2]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);

    if(dataCAN330.CC_yeache_clutch_blocking_sw  == ON)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN330.CC_yeache_clutch_connection_sw == ON){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);

  }
  else if(flagWarning.index == W_ONETOUCH_UP_DOWN)
  {
    // 원터치 상승/하강
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH16_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH16_sub_title]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH16_message_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[MH16_message_2]);

    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 0, 0);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_voltage], 0, 50, 0);
    
    if(dataCAN330.CC_one_touch_down){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 0 , 0);
      YVC1_SetChar(Data1_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data1_value]);
    
    if(dataCAN330.CC_one_touch_up){
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string);
    }
    else{
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 0, 50 , 0);
      YVC1_SetChar(Data2_value_FC, _string1);
    }
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)WARNING_3_GT[Data2_value]);
  }
  
  return imgCnt;
}

uint16_t DrawWarningCombineCheck(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;

    if(flagWarning.index == W_ENGINE_COOLING_TEMPERATURE)
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);      // tseveen 20250522   added yusu
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_arrow]);   // tseveen 20250522     // removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_logo]);    // tseveen 20250522     // removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_message]);  // tseveen 20250522 // removed
  }
    // warning message handler
  if(flagWarning.index <= W_CHUHEN_CONTROLLER)
  {
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_001]);  // tseveen 20250522 bg sub
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_002]);  // tseveen 20250522 bg sub
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[flagWarning.index + 2]); // tseveen 20250522 bg sub
  }
  else if((flagWarning.index >= W_ENGINE_STOP_SWITCH) && (flagWarning.index <= W_2_NASON_BLOCK_SWITCH))
  {
    if(warnings[flagWarning.index].page == W_PAGE_1)    
    {
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[bg_new_1]);      // tseveen removed 20250523 bg
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[bg_new_2]);      // tseveen removed 20250523 bg
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[combine_pic]);   // tseveen removed 20250522 bg
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[combine_pic_info]);      // tseveen removed 20250522 bg
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);                    // tseveen 20250523
  
        if(flagWarning.index == W_ENGINE_STOP_SWITCH)
        {
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_stop_arrow]); // tseveen 20250522 bg removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_stop_logo]);   // tseveen 20250522 bg removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_stop_message]);        // tseveen 20250522 bg removed
        }
/*        else if(flagWarning.index == W_CHARGE)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_battery_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_battery_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_battery_message]);
        } */ //tseveen 20250529 chunjon
/*        else if(flagWarning.index == W_ENGINE_OIL_PRESSURE)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_oil_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_oil_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_oil_message]);
        } */ //tseveen 20250529 engine oil
/*        else if(flagWarning.index == W_ENGINE_COOLING_TEMPERATURE)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusu]);      // tseveen 20250522   added yusu
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_arrow]);   // tseveen 20250522     // removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_logo]);    // tseveen 20250522     // removed
//          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_message]);  // tseveen 20250522 // removed
        }*/ //tseveen removed
        else if(flagWarning.index == W_AIR_FILTER)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_flow_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_flow_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_flow_message]);
        }
/*        else if(flagWarning.index == W_TALKUK_BIN)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf1_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf1_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf1_message]);
        }
        else if(flagWarning.index == W_CHORI_BIN)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf2_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf2_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf2_message]);
        } */ // tseveen 20250529 talkuk choiri 
/*        else if(flagWarning.index == W_2_SENSOR)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_rotation_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_rotation_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_rotation_message]);
        } */ // tseveen 20250529 2bon
        else if(flagWarning.index == W_CHIPBECHUL)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gear_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gear_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gear_message]);
        }
/*        else if(flagWarning.index == W_GUGMUL_MANYANG)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_message]);
        } */ // tseveen 20250529 kukmul
        else if(flagWarning.index == W_MULBUNRIGI)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_drop_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_drop_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_drop_message]);
        }
        else if(flagWarning.index == W_GUGMUL_MANYANG_ENGINE_STOP)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gugmuli_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gugmuli_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gugmuli_message]);
        }
        else if(flagWarning.index == W_ENGINE_STOP_CUTTING_SAFETY)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_cut_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_cut_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_cut_message]);
        }
        else if(flagWarning.index == W_ENGINE_STOP_YEACHE_SAFETY)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_eh_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_eh_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_eh_message]);
        }
        else if(flagWarning.index == W_2_NASON)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_error_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_error_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_error_message]);
        }
        else if(flagWarning.index == W_YANGUG_NASON)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_message]);
        }
        else if(flagWarning.index == W_2_NASON_BLOCK_SWITCH)
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_sw_arrow]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_sw_logo]);
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_sw_message]);
        }
    }
    else if(warnings[flagWarning.index].page == W_PAGE_2)
    {  
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_001]); // tseveen 20250522 bg sub
//      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_1_GT[warning_1_bg_002]); // tseveen 20250522 bg sub
      if(flagWarning.index == W_ENGINE_STOP_SWITCH)
      {
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_stop_title]);      // tseveen 20250522 bg removed
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_stop_body]);     // tseveen 20250522 bg removed
      }
      else if(flagWarning.index == W_CHARGE)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_battery_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_battery_body]);
      }
      else if(flagWarning.index == W_ENGINE_OIL_PRESSURE)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_oil_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_oil_body]);
      }
      else if(flagWarning.index == W_ENGINE_COOLING_TEMPERATURE)
      {
//        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MAIN_GT[yusuu]); // tseveen 20250522
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_title]);     // tseveen 20250522 8
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_temperature_body]);      // tseveen 20250522 8
      }
      else if(flagWarning.index == W_AIR_FILTER)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_flow_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_flow_body]);
      }
      else if(flagWarning.index == W_TALKUK_BIN)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf1_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf1_body]);
      }
      else if(flagWarning.index == W_CHORI_BIN)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf2_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf2_body]);
      }
      else if(flagWarning.index == W_2_SENSOR)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_rotation_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_rotation_body]);
      }
      else if(flagWarning.index == W_CHIPBECHUL)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gear_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_gear_body]);
      }
      else if(flagWarning.index == W_GUGMUL_MANYANG)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_body]);
        
      }
      else if(flagWarning.index == W_MULBUNRIGI)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_drop_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_drop_body]);
      }
      else if(flagWarning.index == W_GUGMUL_MANYANG_ENGINE_STOP)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_leaf3_body]);
      }
      else if(flagWarning.index == W_ENGINE_STOP_CUTTING_SAFETY)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_cut_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_cut_body]);
      }
      else if(flagWarning.index == W_ENGINE_STOP_YEACHE_SAFETY)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_eh_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_eh_body]);
      }
      else if(flagWarning.index == W_2_NASON)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_error_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_error_body]);
      }
      else if(flagWarning.index == W_YANGUG_NASON)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_yangug_error_body]);
      }
      else if(flagWarning.index == W_2_NASON_BLOCK_SWITCH)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_sw_title]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_2_GT[warning_2nd_sw_body]);
      }
    }
  }
  else
  {
    imgCnt = DrawWarningCombineCheckMessage(imgCnt);
  }
  return imgCnt;
}

/*------------------------------------- Setting -------------------------------------*/
uint16_t draw_settings_menu(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  if((currentPageState == SETTINGS_PAGE) || (currentPageState == SETTINGS_10_PAGE) || (currentPageState == SETTINGS_20_PAGE)
              || (currentPageState == SETTINGS_30_PAGE) || (currentPageState == SETTINGS_31x_PAGE) 
              || (currentPageState == SETTINGS_311_PAGE) || (currentPageState == SETTINGS_314_PAGE)
              || (currentPageState == SETTINGS_512_PAGE)){

    if(row_selection < 1)
    {
      row_selection = 0;
    }
    
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 55, 103 + (55 * (row_selection - 1)), 0); // BALJAAA

    if(currentPageState == SETTINGS_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_4]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_5]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[Main_1_1_6]);                  // Added on 8 April 2021 enable 20240326
    }
    else if(currentPageState == SETTINGS_10_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_4]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_5]);           // Added on 2021 March 9
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_11_6]);           // Added on 2021 March 9
    }
    else if(currentPageState == SETTINGS_20_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_4]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_5]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_12_6]);
    }
    else if(currentPageState == SETTINGS_30_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_4]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_5]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE3xx_GT[page_13_6]);
    }
    else if(currentPageState == SETTINGS_31x_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_4]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE31x_GT[page_131_5]);
    }
    else if(currentPageState == SETTINGS_311_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_1311_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_1311_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_1311_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_1311_3]);
    }
    else if(currentPageState == SETTINGS_314_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_1314_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_1314_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_1314_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_1314_3]);
    }
  }
  return imgCnt;
}

/*------------------------------------- Setting 1 -------------------------------------*/

uint16_t draw_sub_settings_1(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};
  C08 _string_cur[5] = {0,0,0,0,0};
  
  if((currentPageState == SETTINGS_11_PAGE) || (currentPageState == SETTINGS_111_PAGE) || 
     (currentPageState == SETTINGS_112_PAGE) || (currentPageState == SETTINGS_113_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1111_GT[page_111_0]);
    setup_mode = SETUP_MODE_ONE;
    
    if(currentPageState == SETTINGS_11_PAGE)
    {
      setup_mode_type = 0x04;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 04                               // XX does not matter
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 04                               // XX does not matter, 04 HONE OFF, 0C - Hone ON
      if(dataCAN311.data[7] == 0x0C)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN311.data[7] == 0x04)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1111_GT[page_111_1]);
    }
    
    else if(currentPageState == SETTINGS_111_PAGE)
    {
      setup_mode_type = 0x14;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 14                               // XX does not matter
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 14                               // XX does not matter, 14 HONE OFF, 1C - Hone ON
      if(dataCAN311.data[7] == 0x1C)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN311.data[7] == 0x14)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1111_GT[page_111_2]);
    }
    else if(currentPageState == SETTINGS_112_PAGE)
    {
      setup_mode_type = 0x34;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 34                               // XX does not matter
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 34                               // XX does not matter, 34 HONE OFF, 3C - Hone ON
      if(dataCAN311.data[7] == 0x3C)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN311.data[7] == 0x34)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1111_GT[page_111_3]);
    }
    else if(currentPageState == SETTINGS_113_PAGE)
    {
      if(((dataCAN311.data[7] == 0xA0) || (dataCAN311.data[7] == 0x00)) && (move_next_setup_page == 1))
      {
        currentPageState = SETTINGS_10_PAGE; 
        previousPageState = currentPageState;
        row_selection = 1;
        move_next_setup_page = 0;
        setup_mode = SETUP_MODE_NONE;
        new_setup_data_received_flag = 0;
        new_setup_data = -1;
      }
      else if(dataCAN311.data[7] == 0xF4)
      {
        setup_mode_type = 0x00;
        move_next_setup_page = 1;
      }
      else if(move_next_setup_page == 0)
      {                                                     // Changed this condition on 2021.05.13
        setup_mode_type = 0x74;
        move_next_setup_page = 0;
      }
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 74                               // XX does not matter
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX F4
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX A0
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_normal_text]);
    }
  }
  else if((currentPageState == SETTINGS_12_PAGE) || (currentPageState == SETTINGS_121_PAGE) || (currentPageState == SETTINGS_122_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1112_GT[page_112_0]);
    setup_mode = SETUP_MODE_ONE;
    
    if(currentPageState == SETTINGS_12_PAGE)
    {
      setup_mode_type = 0x06;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 06                               // XX does not matter
      // receive data from 350 (ID) --> XX XX XX XX XX XX XX 06                               // XX does not matter, 06 Maintain
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 06                               // XX does not matter, 06 HONE OFF, 0E - Hone ON
      if(dataCAN311.data[7] == 0x0E)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if((dataCAN311.data[7] == 0x06) && (dataCAN350.data[7] == 0x06))
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1112_GT[page_112_1]);
    }
    else if(currentPageState == SETTINGS_121_PAGE)
    {
      setup_mode_type = 0x16;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 16                               // XX does not matter
      // receive data from 350 (ID) --> XX XX XX XX XX XX XX 16                               // XX does not matter, 16 Maintain
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 16                               // XX does not matter, 16 HONE OFF, 1E - Hone ON
      if(dataCAN311.data[7] == 0x1E)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if((dataCAN311.data[7] == 0x16) && (dataCAN350.data[7] == 0x16))
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1112_GT[page_112_2]);
    }
    else if(currentPageState == SETTINGS_122_PAGE)
    {
      if((dataCAN311.data[7] == 0xA0) && (dataCAN350.data[7] == 0x00) && (move_next_setup_page == 1))
      {
        currentPageState = SETTINGS_10_PAGE; 
        previousPageState = currentPageState;
        row_selection = 1;
        move_next_setup_page = 0;
        setup_mode = SETUP_MODE_NONE;
        new_setup_data_received_flag = 0;
        new_setup_data = -1;
      }
      else if((dataCAN311.data[7] == 0xF6) && (dataCAN350.data[7] == 0xF6))
      {
        setup_mode_type = 0x00;
        move_next_setup_page = 1;
      }
      else if(move_next_setup_page == 0)
      {                                                      // Added this condition on 2021.05.13
        move_next_setup_page = 0;
        setup_mode_type = 0x26;
      }
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 26                               // XX does not matter
      // receive data from 350 (ID) --> XX XX XX XX XX XX XX F6
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX F6
      // receive data from 350 (ID) --> XX XX XX XX XX XX XX 00
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX A0
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_normal_text]);
    }
  }
  else if((currentPageState == SETTINGS_13_PAGE) || (currentPageState == SETTINGS_131_PAGE) || 
          (currentPageState == SETTINGS_132_PAGE) || (currentPageState == SETTINGS_133_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1113_GT[page_113_0]);
    setup_mode = SETUP_MODE_ONE;
    
    if(currentPageState == SETTINGS_13_PAGE)
    {
      setup_mode_type = 0x01;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 01                               // XX does not matter
      // receive data from 330 (ID) --> XX XX XX XX XX XX XX 01                                 // XX does not matter, 01-Hone OFF, 09-Hone ON
      if(dataCAN330.data[7] == 0x09)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN330.data[7] == 0x01)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1113_GT[page_113_1]);
    }
    else if(currentPageState == SETTINGS_131_PAGE)
    {
      setup_mode_type = 0x11;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 11                               // XX does not matter
      // receive data from 330 (ID) --> XX XX XX XX XX XX XX 11                                 // XX does not matter, 11-Hone OFF, 19-Hone ON
      if(dataCAN330.data[7] == 0x19)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN330.data[7] == 0x11)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1113_GT[page_113_2]);
    }
    else if(currentPageState == SETTINGS_132_PAGE)
    {
      setup_mode_type = 0x31;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter
      // receive data from 330 (ID) --> XX XX XX XX XX XX XX 31                                 // XX does not matter, 31-Hone OFF, 39-Hone ON
      if(dataCAN330.data[7] == 0x39)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN330.data[7] == 0x31)
      {
        move_next_setup_page = 1;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1113_GT[page_113_3]);
    }
    else if(currentPageState == SETTINGS_133_PAGE)
    {
      if((dataCAN330.data[7] == 0x00) && (move_next_setup_page == 1))
      {
        currentPageState = SETTINGS_10_PAGE; 
        previousPageState = currentPageState;
        row_selection = 1;
        move_next_setup_page = 0;
        setup_mode = SETUP_MODE_NONE;
        new_setup_data_received_flag = 0;
        new_setup_data = -1;
      }
      else if(dataCAN330.data[7] == 0xF1)
      {
        setup_mode_type = 0x00;
        move_next_setup_page = 1;
      }
      else if(move_next_setup_page == 0)
      {                                                      // Changed this condition on 2021.05.13
        setup_mode_type = 0x71;
        move_next_setup_page = 0;
      }
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 71                               // XX does not matter
      // receive data from 330 (ID) --> XX XX XX XX XX XX XX F1
      // receive data from 330 (ID) --> XX XX XX XX XX XX XX 00
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_normal_text]);
    }
  }
  else if((currentPageState == SETTINGS_14_PAGE) || (currentPageState == SETTINGS_141_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1114_GT[page_114_0]);
    setup_mode = SETUP_MODE_ONE;
    
    if(currentPageState == SETTINGS_14_PAGE)
    {
      setup_mode_type = 0x02;
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 02                               // XX does not matter
      // receive data from 340 (ID) --> XX XX XX XX XX XX XX 02                                 // Maintain
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX 02                                 // XX does not matter, 02-Hone OFF, 2A-Hone SHORT ON, other - Hone LONG ON
      if(dataCAN311.data[7] == 0x2A)
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      else if(dataCAN311.data[7] == 0x02)
      {
        if(dataCAN340.data[7] == 0x02)
        {
          move_next_setup_page = 1;
        }
      }
      else
      {
        set_setting_sound();
        move_next_setup_page = 0;
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1114_GT[page_114_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1114_GT[page_114_2]);
      
      ftoa((float) AG_auger_potentiometer_value * 5.0 / 1023.0, _string, 2);                                   // updated on 2021.06.29
      _string[4] = 0;
      YVC1_SetChar(page114_text_FC, _string);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE1114_GT[page114_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], 560, 380, 0);
      
    }
    else if(currentPageState == SETTINGS_141_PAGE)
    {
      if((dataCAN311.data[7] == 0xA0) && (dataCAN340.data[7] == 0x00) && (move_next_setup_page == 1))
      {
        currentPageState = SETTINGS_10_PAGE; 
        previousPageState = currentPageState;
        row_selection = 1;
        move_next_setup_page = 0;
        setup_mode = SETUP_MODE_NONE;
        new_setup_data_received_flag = 0;
        new_setup_data = -1;
      }
      else if((dataCAN311.data[7] == 0xF2) && (dataCAN340.data[7] == 0xF2))
      {
        setup_mode_type = 0x00;
        move_next_setup_page = 1;
      } 
      else if(move_next_setup_page == 0)
      {                                                    // Added this condition on 2021.05.13
        setup_mode_type = 0x12;
        move_next_setup_page = 0;
      }
      // send command from 320 (ID) --> XX XX XX XX XX XX XX 12                               // XX does not matter
      // receive data from 340 (ID) --> XX XX XX XX XX XX XX F2
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX F2
      // receive data from 340 (ID) --> XX XX XX XX XX XX XX 00
      // receive data from 311 (ID) --> XX XX XX XX XX XX XX A0
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1xx_GT[page_normal_text]);
    }
  }
  else if((currentPageState == SETTINGS_15_PAGE) || (currentPageState == SETTINGS_151_PAGE) || (currentPageState == SETTINGS_152_PAGE) ||
          (currentPageState == SETTINGS_153_PAGE) || (currentPageState == SETTINGS_154_PAGE) || (currentPageState == SETTINGS_155_PAGE) ||
          (currentPageState == SETTINGS_156_PAGE) || (currentPageState == SETTINGS_157_PAGE) || (currentPageState == SETTINGS_158_PAGE) ||
          (currentPageState == SETTINGS_159_PAGE) || (currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE) ||
          (currentPageState == SETTINGS_1512_PAGE))
  {
            
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_0]);
    if(new_setup_data_received_flag != 0)
    {
      if(new_setup_data_received_flag == 1)
      {
        new_setup_data = new_setup_data_received;
        new_setup_data_received_to_update = new_setup_data_received;                    // Added on 2021.06.29
      }
      new_setup_data_received_flag = 2;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      //new_setup_data = -1;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
      {
        new_setup_data = 0;
      }
      setup_mode_address = 0x399;
    }
    ftoa(new_setup_data_received, _string, 0);
    _string[4] = 0;
      
    switch(currentPageState)
    {
      case SETTINGS_15_PAGE:
        setup_mode_type = 0x00;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_2]);
        break;

      case SETTINGS_151_PAGE:
        setup_mode_type = 0x01;
        //ftoa(saved_lr_lever_neutral, _string, 0);
        //_string[4] = 0;
        ftoa(MC_steering_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_lr_lever_neutral;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_1_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_1_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_1_3]);
        break;

      case SETTINGS_152_PAGE:
        setup_mode_type = 0x02;
        //ftoa(saved_lr_lever_left, _string, 0);
        //_string[4] = 0;
        ftoa(MC_steering_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_lr_lever_left;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_2_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_2_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_2_3]);
        break;

      case SETTINGS_153_PAGE:
        setup_mode_type = 0x03;
        //ftoa(saved_lr_lever_right, _string, 0);
        //_string[4] = 0;
        ftoa(MC_steering_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_lr_lever_right;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_3_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_3_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_3_3]);
        break;

      case SETTINGS_154_PAGE:
        setup_mode_type = 0x04;
        //ftoa(saved_fr_lever_neutral, _string, 0);
        //_string[4] = 0;
        ftoa(MC_driveing_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_fr_lever_neutral;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_4_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_4_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_4_3]);
        break;

      case SETTINGS_155_PAGE:
        setup_mode_type = 0x05;
        //ftoa(saved_fr_lever_forward, _string, 0);
        //_string[4] = 0;
        ftoa(MC_driveing_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_fr_lever_forward;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_5_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_5_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_5_3]);
        break;

      case SETTINGS_156_PAGE:
        setup_mode_type = 0x06;
        //ftoa(saved_fr_lever_reward, _string, 0);
        //_string[4] = 0;
        ftoa(MC_driveing_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_fr_lever_reward;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_6_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_6_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_6_3]);
        break;

      case SETTINGS_157_PAGE:
        setup_mode_type = 0x07;
        //ftoa(saved_asc_lever_run, _string, 0);
        //_string[4] = 0;
        ftoa(MC_subshift_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_asc_lever_run;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_7_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_7_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_7_3]);
        break;

      case SETTINGS_158_PAGE:
        setup_mode_type = 0x08;
        //ftoa(saved_asc_lever_stand, _string, 0);
        //_string[4] = 0;
        ftoa(MC_subshift_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = MC_subshift_lever;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_8_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_8_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_8_3]);
        break;

      case SETTINGS_159_PAGE:
        setup_mode_type = 0x09;
        //ftoa(saved_asc_lever_low, _string, 0);
        //_string[4] = 0;
        ftoa(MC_subshift_lever, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_asc_lever_low;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_9_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_9_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_9_3]);
        break;

      case SETTINGS_1510_PAGE:
        setup_mode_type = 0x0A;
        ftoa(new_setup_data_received_to_update, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_fine_right_tunning;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_10_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_10_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_10_3]);
        break;

      case SETTINGS_1511_PAGE:
        setup_mode_type = 0x0B;
        ftoa(new_setup_data_received_to_update, _string_cur, 0);
        _string_cur[4] = 0;
        //new_setup_data = saved_fine_left_tunning;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_11_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_11_2]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_11_3]);
        break;

      case SETTINGS_1512_PAGE:
        setup_mode_type = 0x0D;
        setup_mode_rw = SETUP_WRITE;
        new_setup_data = 0;
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1115_GT[page_115_12_1]);
        break;
    }
    
    
    if(currentPageState != SETTINGS_1512_PAGE)
    {
      YVC1_SetChar(page115_text1_FC, _string);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE1115_GT[page115_text1]);
    }
    
    if((currentPageState != SETTINGS_15_PAGE) && (currentPageState != SETTINGS_1512_PAGE))
    {
      YVC1_SetChar(page115_text2_FC, _string_cur);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE1115_GT[page115_text2]);
    }
  }
  else if((currentPageState == SETTINGS_16_PAGE) || (currentPageState == SETTINGS_161_PAGE) || 
          (currentPageState == SETTINGS_162_PAGE) || (currentPageState == SETTINGS_163_PAGE))
  {
            
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1116_GT[page_116_0]);
    setup_mode = SETUP_MODE_ONE;
    
    switch(currentPageState)
    {
      case SETTINGS_16_PAGE:
        setup_mode_type = 0x03;
        // send command from 320 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter
        // receive data from 330 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter, 03-Hone OFF, 0B-Hone ON
        if(dataCAN330.data[7] == 0x0B)
        {
          set_setting_sound();
          move_next_setup_page = 0;
        }
        else if(dataCAN330.data[7] == 0x03)
        {
          move_next_setup_page = 1;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1116_GT[page_116_1]);
        break;

      case SETTINGS_161_PAGE:
        setup_mode_type = 0x13;
        // send command from 320 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter
        // receive data from 330 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter, 13-Hone OFF, 1B-Hone ON
        if(dataCAN330.data[7] == 0x1B)
        {
          set_setting_sound();
          move_next_setup_page = 0;
        }
        else if(dataCAN330.data[7] == 0x13)
        {
          move_next_setup_page = 1;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1116_GT[page_116_1_1]);
        break;

      case SETTINGS_162_PAGE:
        setup_mode_type = 0x33;
        // send command from 320 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter
        // receive data from 330 (ID) --> XX XX XX XX XX XX XX 31                               // XX does not matter, 33-Hone OFF, 3B-Hone ON
        if(dataCAN330.data[7] == 0x3B)
        {
          set_setting_sound();
          move_next_setup_page = 0;
        }
        else if(dataCAN330.data[7] == 0x33)
        {
          move_next_setup_page = 1;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1116_GT[page_116_2_1]);
        break;

      case SETTINGS_163_PAGE:
        
        if((dataCAN330.data[7] == 0x00) && (move_next_setup_page == 1))
        {
          currentPageState = SETTINGS_10_PAGE; 
          previousPageState = currentPageState;
          row_selection = 1;
          move_next_setup_page = 0;
          setup_mode = SETUP_MODE_NONE;
          new_setup_data_received_flag = 0;
          new_setup_data = -1;
        }
        else if(dataCAN330.data[7] == 0xF3)
        {
          setup_mode_type = 0x00;
          move_next_setup_page = 1;
        } 
        else if(move_next_setup_page == 0)
        {
          setup_mode_type = 0x73;
          move_next_setup_page = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE1116_GT[page_116_3_1]);
        break;
    }
  }
  return imgCnt;
}

/*------------------------------------- Setting 2 -------------------------------------*/
#define SETTING_2_UNIT_X                420
#define SETTING_2_UNIT_Y                335


uint16_t draw_sub_settings_21(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};

  if(currentPageState == SETTINGS_21x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_0]);
         if(row_selection == 1){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_1_text]); }
    else if(row_selection == 2){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_2_text]); }
    else if(row_selection == 3){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_3_text]); }
    else if(row_selection == 4){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_4_text]); }
    else if(row_selection == 5){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_5_text]); }
    else if(row_selection == 6){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_6_text]); }
    else if(row_selection == 7){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_7_text]); }
    else if(row_selection == 8){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_8_text]); }
  }
  else if((currentPageState >= SETTINGS_211_PAGE) && (currentPageState <= SETTINGS_218_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_0]);
    if(new_setup_data_received_flag != 0)
    {
      new_setup_data_received_flag = 1;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
        new_setup_data = 0;
      setup_mode_address = 0x319;
    }
    
    if(currentPageState == SETTINGS_211_PAGE)
    {
      setup_mode_type = 0xA1;
      // send command to 319 (ID) from 329 (ID) --> A1 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A1 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_1_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_sec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_212_PAGE)
    {
      setup_mode_type = 0xA2;
      // send command to 319 (ID) from 329 (ID) --> A2 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A2 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_2_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_sec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_213_PAGE)
    {
      setup_mode_type = 0xA3;
      // send command to 319 (ID) from 329 (ID) --> A3 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A3 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_3_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_gradius], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_214_PAGE)
    {
      setup_mode_type = 0xA8;
      // send command to 319 (ID) from 329 (ID) --> A8 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A8 00 XX                              // XX is 01 or 00
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_4_text]);
      if(new_setup_data_received == 0x0001)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_4_2]);
      else if(new_setup_data_received == 0x0000)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_4_3]);
    }
      
    else if(currentPageState == SETTINGS_215_PAGE)
    {
      setup_mode_type = 0xA9;
      // send command to 319 (ID) from 329 (ID) --> A9 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A9 55 55 or A9 AA AA
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_5_text]);
      if(new_setup_data != -1){
        if(new_setup_data_received == 0x5555)
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_5_2]);
        else if((uint16_t)new_setup_data_received == 0xAAAA)
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_5_3]);
      }
    }
    else if(currentPageState == SETTINGS_216_PAGE)
    {
      setup_mode_type = 0xAA;
      // send command to 319 (ID) from 329 (ID) --> AA 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> AA 55 55 or AA AA AA
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_6_text]);
      if(new_setup_data_received == 0x5555)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_6_2]);
      else if((uint16_t)new_setup_data_received == 0xAAAA)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_6_3]);
    }
    else if(currentPageState == SETTINGS_217_PAGE)
    {
      setup_mode_type = 0xAB;
      // send command to 319 (ID) from 329 (ID) --> AB 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> AB 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_7_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_218_PAGE)
    {
      setup_mode_type = 0xAC;
      // send command to 319 (ID) from 329 (ID) --> AC 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> AC 55 55 or AC AA AA
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_8_text]);
      if(new_setup_data_received == 0x5555)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_8_2]);
      else if((uint16_t)new_setup_data_received == 0xAAAA)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_121_8_3]);
    }
    
    if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE))
    {
      if(new_setup_data_received >= 100)
      {
        if(new_setup_data_received >= 1000){
          _string[0] = ((new_setup_data_received % 1000) / 100) + 0x30;
        }else{
          _string[0] = (new_setup_data_received / 100) + 0x30;
        }
        _string[1] = ((new_setup_data_received % 100) / 10) + 0x30;
        _string[2] = '.';
        _string[3] = (new_setup_data_received % 10) + 0x30;
        _string[4] = 0;
      }
      else
      {      
        _string[0] = (new_setup_data_received / 10) + 0x30;
        _string[1] = '.';
        _string[2] = (new_setup_data_received % 10) + 0x30;
        _string[3] = 0;
        _string[4] = 0;
      }
      YVC1_SetChar(value_FC, _string);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
    }
    else if((currentPageState == SETTINGS_213_PAGE) || (currentPageState == SETTINGS_217_PAGE))
    {
      ftoa(new_setup_data_received, _string, 0);
      _string[4] = 0;
      YVC1_SetChar(value_FC, _string);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
    }    
  }
  return imgCnt;
}

uint16_t draw_sub_settings_22(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};
  
  if(currentPageState == SETTINGS_22x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_0]);
         if(row_selection == 1){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_1_text]); }
    else if(row_selection == 2){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_2_text]); }
    else if(row_selection == 3){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_3_text]); }
    else if(row_selection == 4){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_4_text]); }
    else if(row_selection == 5){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_5_text]); }
    else if(row_selection == 6){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_6_text]); }
    else if(row_selection == 7){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_7_text]); }      
  }
  else if((currentPageState >= SETTINGS_221_PAGE) && (currentPageState <= SETTINGS_227_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_0]);
    if(new_setup_data_received_flag != 0)
    {
      new_setup_data_received_flag = 1;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
        new_setup_data = 0;
      setup_mode_address = 0x339;
    }
    
    if(currentPageState == SETTINGS_221_PAGE)
    {
      setup_mode_type = 0xA1;
      // send command to 339 (ID) from 329 (ID) --> A1 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A1 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_1_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_cm], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_222_PAGE)
    {
      setup_mode_type = 0xA2;
      // send command to 339 (ID) from 329 (ID) --> A2 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A2 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_2_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_cm], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_223_PAGE)
    {
      setup_mode_type = 0xA3;
      // send command to 339 (ID) from 329 (ID) --> A3 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A3 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_3_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_sec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_224_PAGE)
    {
      setup_mode_type = 0xA4;
      // send command to 339 (ID) from 329 (ID) --> A4 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A4 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_4_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_sec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_225_PAGE)
    {
      setup_mode_type = 0xA5;
      // send command to 339 (ID) from 329 (ID) --> A5 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A5 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_5_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_rpm], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_226_PAGE)
    {
      setup_mode_type = 0xA6;
      // send command to 339 (ID) from 329 (ID) --> A6 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A6 55 55 or A6 AA AA
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_6_text]);
      if(new_setup_data_received == 0x5555)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_6_2]);
      else if((uint16_t)new_setup_data_received == 0xAAAA)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_6_3]);
    }
    else if(currentPageState == SETTINGS_227_PAGE)
    {
      setup_mode_type = 0xA7;
      // send command to 339 (ID) from 329 (ID) --> A7 00 00 55
      // receive data to 329 (ID) from 339 (ID) --> A7 55 55 or A6 AA AA
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_7_text]);
      if(new_setup_data_received == 0x5555)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_7_2]);
      else if((uint16_t)new_setup_data_received == 0xAAAA)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_122_7_3]);
    }
    
    if((currentPageState >= SETTINGS_221_PAGE) && (currentPageState <= SETTINGS_225_PAGE))
    {
      ftoa(new_setup_data_received, _string, 0);
      _string[4] = 0;
      YVC1_SetChar(value_FC, _string);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
    }
  }
  return imgCnt;
}

uint16_t draw_sub_settings_23(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};
  
  if(currentPageState == SETTINGS_23x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_0]);
         if(row_selection == 1){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_1_text]); }
    else if(row_selection == 2){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_2_text]); }
    else if(row_selection == 3){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_3_text]); }
    else if(row_selection == 4){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_4_text]); }
    else if(row_selection == 5){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_5_text]); }
    else if(row_selection == 6){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_6_text]); }
    else if(row_selection == 7){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_7_text]); }
    else if(row_selection == 8){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_8_text]); }
  }
  else if((currentPageState >= SETTINGS_231_PAGE) && (currentPageState <= SETTINGS_238_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_0]);
    if(new_setup_data_received_flag != 0)
    {
      new_setup_data_received_flag = 1;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
        new_setup_data = 0;
    }
    if((currentPageState >= SETTINGS_231_PAGE) && (currentPageState <= SETTINGS_234_PAGE))
    {
      setup_mode_address = 0x349;
    }
    else if((currentPageState >= SETTINGS_235_PAGE) && (currentPageState <= SETTINGS_238_PAGE))
    {
      setup_mode_address = 0x319;
    }
        
    if(currentPageState == SETTINGS_231_PAGE)
    {
      setup_mode_type = 0xA1;
      // send command to 349 (ID) from 329 (ID) --> A1 00 00 55
      // receive data to 329 (ID) from 349 (ID) --> A1 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_1_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_a], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_232_PAGE)
    {
      setup_mode_type = 0xA2;
      // send command to 349 (ID) from 329 (ID) --> A2 00 00 55
      // receive data to 329 (ID) from 349 (ID) --> A2 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_2_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_percent], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_233_PAGE)
    {
      setup_mode_type = 0xA3;
      // send command to 349 (ID) from 329 (ID) --> A3 00 00 55
      // receive data to 329 (ID) from 349 (ID) --> A3 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_3_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_percent], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_234_PAGE)
    {
      setup_mode_type = 0xA4;
      // send command to 349 (ID) from 329 (ID) --> A4 00 00 55
      // receive data to 329 (ID) from 349 (ID) --> A4 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_4_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    
    // Added the following 4 cases on 22 March 2021
    else if(currentPageState == SETTINGS_235_PAGE)
    {
      setup_mode_type = 0xA4;
      // send command to 319 (ID) from 329 (ID) --> A4 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A4 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_5_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_236_PAGE)
    {
      setup_mode_type = 0xA5;
      // send command to 319 (ID) from 329 (ID) --> A5 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A5 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_6_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_237_PAGE)
    {
      setup_mode_type = 0xA6;
      // send command to 319 (ID) from 329 (ID) --> A6 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A6 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_7_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    else if(currentPageState == SETTINGS_238_PAGE)
    {
      setup_mode_type = 0xA7;
      // send command to 319 (ID) from 329 (ID) --> A7 00 00 55
      // receive data to 329 (ID) from 319 (ID) --> A7 00 XX
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_123_8_text]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_msec], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    }
    
    ftoa(new_setup_data_received, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
  }
  return imgCnt;
}

uint16_t draw_sub_settings_24(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};
  
  if(currentPageState == SETTINGS_24x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_124_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_124_1_text]);
  }
  else if(currentPageState == SETTINGS_241_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_124_0]);
    if(new_setup_data_received_flag != 0)
    {
      new_setup_data_received_flag = 1;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
        new_setup_data = 0;
      setup_mode_address = 0x359;
      setup_mode_type = 0xA1;
    }
    // send command to 359 (ID) from 329 (ID) --> A1 00 00 55
    // receive data to 329 (ID) from 359 (ID) --> A1 00 XX
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_124_1_text]);
    YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_a], SETTING_2_UNIT_X, SETTING_2_UNIT_Y, 0);
    ftoa(new_setup_data_received, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
  }
  return imgCnt;
}

uint16_t draw_sub_settings_25(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string_1[5] = {0,0,0,0,0};
  C08 _string_2[5] = {0,0,0,0,0};
  C08 _string_3[5] = {0,0,0,0,0};
  C08 _string_4[5] = {0,0,0,0,0};
/*  
  if(currentPageState == SETTINGS_25x_PAGE){
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_0]);
  }
  else 
*/
  if((currentPageState >= SETTINGS_251_PAGE) && (currentPageState <= SETTINGS_2511_PAGE))
  {
    if(axelControlFunctionEnabled == FALSE)
    {
      if(new_setup_data_received_flag != 0)
      {
        if(new_setup_data_received_flag == 1)
        {
          new_setup_data_received_to_update = new_setup_data_received;                    // Added on 2021.06.29
        }        
        new_setup_data_received_flag = 2;                                                 // Changed on 2021.06.29
        setup_mode = SETUP_MODE_NONE;
      }
      else
      {
        new_setup_data_received_flag = 0;
        setup_mode = SETUP_MODE_TWO;
        if(setup_mode_rw == SETUP_READ)
          new_setup_data = 0;
        setup_mode_address = 0x369;
      }
    }
    
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_0]);
    
    if((currentPageState == SETTINGS_251_PAGE) || (currentPageState == SETTINGS_252_PAGE))
    {
      if(currentPageState == SETTINGS_251_PAGE)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_2_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_2_2]);        
      }
      else
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_1]);
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_3]);
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_4]);
      
      if(currentPageState == SETTINGS_251_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x00;
          //ftoa(saved_axel_app_sensor1_position_max, _string_1, 0);
          ftoa(new_setup_data_received, _string_3, 0);                                    // changed on 2021.06.29    2024.05.22
          _string_3[4] = 0;
          //ftoa(saved_axel_app_sensor2_position_max, _string_3, 0);
          ftoa(new_setup_data_2_received, _string_1, 0);                                  // changed on 2021.06.29    2024.05.22
          _string_1[4] = 0;
        }
        else
        {
          ftoa(axel_app_sensor1_position_min, _string_1, 0);                                    // changed on 2021.06.29    2024.05.22
          _string_1[4] = 0;
          ftoa(axel_app_sensor2_position_min, _string_3, 0);                                  // changed on 2021.06.29    2024.05.22
          _string_3[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1_0]);
      }
      else if(currentPageState == SETTINGS_252_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x01;
          //ftoa(saved_axel_app_sensor1_position_min, _string_1, 0);
          ftoa(new_setup_data_received, _string_1, 0);                                    // changed on 2021.06.29
          _string_1[4] = 0;
          //ftoa(saved_axel_app_sensor2_position_min, _string_3, 0);
          ftoa(new_setup_data_2_received, _string_3, 0);                                  // changed on 2021.06.29 --> The values are changed each other on the display vairables.
          _string_3[4] = 0;
        }
        else
        {
          ftoa(axel_app_sensor1_position_max, _string_1, 0);                                    // changed on 2021.06.29    2024.05.22
          _string_1[4] = 0;
          ftoa(axel_app_sensor2_position_max, _string_3, 0);                                  // changed on 2021.06.29    2024.05.22
          _string_3[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_2_0]);
      }
      
      if(axelControlFunctionEnabled == FALSE)
      {
        ftoa(axel_app_sensor1_position, _string_2, 0);
        _string_2[4] = 0;
        ftoa(axel_app_sensor2_position, _string_4, 0);
        _string_4[4] = 0;
      }
      else
      {
        ftoa(axel_app_sensor1_position, _string_2, 0);
        _string_2[4] = 0;
        ftoa(axel_app_sensor2_position, _string_4, 0);
        _string_4[4] = 0;
      }
    }
    
    else if((currentPageState == SETTINGS_253_PAGE) || (currentPageState == SETTINGS_254_PAGE) || (currentPageState == SETTINGS_255_PAGE))
    {
      if(axelControlFunctionEnabled == FALSE)
      {
        ftoa(new_setup_data_received, _string_1, 0);
        _string_1[4] = 0;
        ftoa(new_setup_data_received_to_update, _string_2, 0);
        _string_2[4] = 0;
      }
      
      if(currentPageState == SETTINGS_253_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x02;
        }
        else
        {
          ftoa(axel_threshing_delay, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_threshing_delay_temp, _string_2, 0);
          _string_2[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_3_0]);
      }
      else if(currentPageState == SETTINGS_254_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x03;
        }
        else
        {
          ftoa(axel_auger_delay, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_auger_delay_temp, _string_2, 0);
          _string_2[4] = 0; 
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_4_0]);
      }
      else if(currentPageState == SETTINGS_255_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x04;
        }
        else
        {
          ftoa(axel_auger_auto_delay, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_auger_auto_delay_temp, _string_2, 0);
          _string_2[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_5_0]);
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_3_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_3_2]);
    }
    
    else if((currentPageState == SETTINGS_256_PAGE) || (currentPageState == SETTINGS_257_PAGE) || 
       (currentPageState == SETTINGS_258_PAGE) || (currentPageState == SETTINGS_259_PAGE))
    {
      if(axelControlFunctionEnabled == FALSE)
      {
        ftoa(new_setup_data_received, _string_1, 0);
        _string_1[4] = 0;
        ftoa(new_setup_data_received_to_update, _string_2, 0);
        _string_2[4] = 0;
      }
      
      if(currentPageState == SETTINGS_256_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x05;
        }
        else
        {
          ftoa(axel_yeache_ku_time, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_yeache_ku_time_temp, _string_2, 0);
          _string_2[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_6_0]);
      }
      else if(currentPageState == SETTINGS_257_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x06;
        }
        else
        {
          ftoa(axel_tbs_ku_time, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_tbs_ku_time_temp, _string_2, 0);
          _string_2[4] = 0; 
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_7_0]);
      }
      else if(currentPageState == SETTINGS_258_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x07;
        }
        else
        {
          ftoa(axel_auger_ku_time, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_auger_ku_time_temp, _string_2, 0);
          _string_2[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_8_0]);
      }
      else if(currentPageState == SETTINGS_259_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          setup_mode_type = 0x08;
        }
        else
        {
          ftoa(axel_c_speed_ku_time, _string_1, 0);
          _string_1[4] = 0;
          ftoa(axel_c_speed_ku_time_temp, _string_2, 0);
          _string_2[4] = 0;
        }
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_9_0]);
      }
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_6_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_6_2]);
    }
    else if(currentPageState == SETTINGS_2510_PAGE)
    {
      if(axelControlFunctionEnabled == FALSE)
      {
        ftoa(new_setup_data_received, _string_1, 0);
        _string_1[4] = 0;
        ftoa(new_setup_data_received_to_update, _string_2, 0);
        _string_2[4] = 0;
        setup_mode_type = 0x09;
      }
      else
      {
        // implement
        ftoa(axel_threshing_rpm, _string_1, 0);
        _string_1[4] = 0;
        ftoa(axel_threshing_rpm_temp, _string_2, 0);
        _string_2[4] = 0;
      }
      
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_10_0]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_10_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_10_2]);
    }
    else if(currentPageState == SETTINGS_2511_PAGE)
    {
      if(axelControlFunctionEnabled == FALSE)
        setup_mode_type = 0x0A;
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_125_1]);
    }
    
    if(currentPageState != SETTINGS_2511_PAGE)
    {
      YVC1_SetChar(can_raw_data_5_FC, _string_1);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE31x_GT[can_raw_data_5]);
      
      YVC1_SetChar(can_raw_data_7_FC, _string_2);
      YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE31x_GT[can_raw_data_7]);
      
      if((currentPageState == SETTINGS_251_PAGE) || (currentPageState == SETTINGS_252_PAGE))
      {
        YVC1_SetChar(can_raw_data_9_FC, _string_3);
        YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE31x_GT[can_raw_data_9]);
        
        YVC1_SetChar(can_raw_data_11_FC, _string_4);
        YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE31x_GT[can_raw_data_11]);
      }
    }
  }
  return imgCnt;
}

uint16_t draw_sub_settings_26(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[5] = {0,0,0,0,0};
  
  if(currentPageState == SETTINGS_26x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_0]);
    
         if(row_selection == 1){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_1_text]); }
    else if(row_selection == 2){      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_2_text]); }
  }
  else if((currentPageState == SETTINGS_261_PAGE) || (currentPageState == SETTINGS_262_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_0]);   
    if(new_setup_data_received_flag != 0)
    {
      new_setup_data_received_flag = 1;
      setup_mode = SETUP_MODE_NONE;
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      if(setup_mode_rw == SETUP_READ)
        new_setup_data = 0;
      setup_mode_address = 0x389;
    }
    
    if(currentPageState == SETTINGS_261_PAGE)
    {
      setup_mode_type = 0xB0;
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_1_text]);
    }
    else if(currentPageState == SETTINGS_262_PAGE)
    {
      setup_mode_type = 0xB1;
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[page_126_2_text]);     // it was page_126_1_text too, fixed on 2021.06.29
    }
    ftoa(new_setup_data_received, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(value_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE2xx_GT[value]);
  }
  return imgCnt;
}

uint16_t draw_sub_settings_2F(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  if(currentPageState == SETTINGS_2xxF_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE2xx_GT[Setting_value_change_completed]);
    if(setup_mode == SETUP_MODE_TWO)
    {
      if(new_setup_data_received_flag == 1)
      {
        if(new_setup_data == new_setup_data_received)
        {
          new_setup_data_received_flag = 0;
          setup_mode = SETUP_MODE_NONE;
          currentPageState = SETTINGS_20_PAGE;
          previousPageState = currentPageState;
          row_selection = 1;
          new_setup_data = -1;                                   // Added on 22 March 2021
        }
      }
    }
    else
    {
      new_setup_data_received_flag = 0;
      setup_mode = SETUP_MODE_TWO;
      setup_mode_rw = SETUP_WRITE;
           if(previousPageState == SETTINGS_211_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA1;}
      else if(previousPageState == SETTINGS_212_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA2;}
      else if(previousPageState == SETTINGS_213_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA3;}
      else if(previousPageState == SETTINGS_214_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA8;}
      else if(previousPageState == SETTINGS_215_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA9;}
      else if(previousPageState == SETTINGS_216_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xAA;}
      else if(previousPageState == SETTINGS_217_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xAB;}
      else if(previousPageState == SETTINGS_218_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xAC;}
      else if(previousPageState == SETTINGS_221_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA1;}
      else if(previousPageState == SETTINGS_222_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA2;}
      else if(previousPageState == SETTINGS_223_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA3;}
      else if(previousPageState == SETTINGS_224_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA4;}
      else if(previousPageState == SETTINGS_225_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA5;}
      else if(previousPageState == SETTINGS_226_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA6;}
      else if(previousPageState == SETTINGS_227_PAGE){     setup_mode_address = 0x339;          setup_mode_type = 0xA7;}
      else if(previousPageState == SETTINGS_231_PAGE){     setup_mode_address = 0x349;          setup_mode_type = 0xA1;}
      else if(previousPageState == SETTINGS_232_PAGE){     setup_mode_address = 0x349;          setup_mode_type = 0xA2;}
      else if(previousPageState == SETTINGS_233_PAGE){     setup_mode_address = 0x349;          setup_mode_type = 0xA3;}
      else if(previousPageState == SETTINGS_234_PAGE){     setup_mode_address = 0x349;          setup_mode_type = 0xA4;}
      else if(previousPageState == SETTINGS_235_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA4;}
      else if(previousPageState == SETTINGS_236_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA5;}
      else if(previousPageState == SETTINGS_237_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA6;}
      else if(previousPageState == SETTINGS_238_PAGE){     setup_mode_address = 0x319;          setup_mode_type = 0xA7;}
      else if(previousPageState == SETTINGS_241_PAGE){     setup_mode_address = 0x359;          setup_mode_type = 0xA1;}
      
      else if(previousPageState == SETTINGS_261_PAGE){     setup_mode_address = 0x389;          setup_mode_type = 0xB0;}
      else if(previousPageState == SETTINGS_262_PAGE){     setup_mode_address = 0x389;          setup_mode_type = 0xB1;}
    }
  }
  return imgCnt;
}

uint16_t draw_sub_settings_2(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  
  imgCnt = draw_sub_settings_21(imgCnt);
  imgCnt = draw_sub_settings_22(imgCnt);
  imgCnt = draw_sub_settings_23(imgCnt);
  imgCnt = draw_sub_settings_24(imgCnt);
  imgCnt = draw_sub_settings_25(imgCnt);
  imgCnt = draw_sub_settings_26(imgCnt);
  imgCnt = draw_sub_settings_2F(imgCnt);
  
  return imgCnt;
}

/*------------------------------------- Setting 3 -------------------------------------*/
#define SETTING_3_STATE_X               530
#define SETTING_3_UNIT_X                700
#define SETTING_3_UNIT_Y_START          85
#define SETTING_3_UNIT_Y_STEP           30
#define SETTING_3_UINT_Y_POS(x)         SETTING_3_UNIT_Y_START + x * SETTING_3_UNIT_Y_STEP
#define SETTING_3_NAME_Y_POS(x)         x * SETTING_3_UNIT_Y_STEP

uint16_t draw_string(uint16_t _imgCnt, float _data, uint8_t _index, uint8_t _fp)
{
  uint16_t imgCnt = _imgCnt;
  float _dataTemp = _data;
  C08 _string[5] = {0,0,0,0,0};
  
  ftoa(_dataTemp, _string, _fp);
  switch(_index)
  {
    case 0: {  YVC1_SetChar(can_raw_data_1_FC, _string);        break; }
    case 1: {  YVC1_SetChar(can_raw_data_2_FC, _string);        break; }
    case 2: {  YVC1_SetChar(can_raw_data_3_FC, _string);        break; }
    case 3: {  YVC1_SetChar(can_raw_data_4_FC, _string);        break; }
    case 4: {  YVC1_SetChar(can_raw_data_5_FC, _string);        break; }
    case 5: {  YVC1_SetChar(can_raw_data_6_FC, _string);        break; }
    case 6: {  YVC1_SetChar(can_raw_data_7_FC, _string);        break; }
    case 7: {  YVC1_SetChar(can_raw_data_8_FC, _string);        break; }
    case 8: {  YVC1_SetChar(can_raw_data_9_FC, _string);        break; }
    case 9: {  YVC1_SetChar(can_raw_data_10_FC, _string);       break; }
    case 10: { YVC1_SetChar(can_raw_data_11_FC, _string);       break; }
    case 11: { YVC1_SetChar(can_raw_data_12_FC, _string);       break; }
  }
  YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE31x_GT[can_raw_data_1 + _index]);
  return imgCnt;
}

uint16_t draw_sub_settings_31(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  uint8_t i;
  float tempData[12];
  uint8_t fp;
  
  if(currentPageState == SETTINGS_3111_PAGE)                                                  // Integrated controller - 310
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13111_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13111_total]);
    
    for(i = 0; i < 12; i++)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13111_1 + i + index], 0, SETTING_3_NAME_Y_POS(i), 0);
      tempData[i] = can_3111_data[index + i];
      if(tempData[i] == 0)
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else
      {
        tempData[i] = 12;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      fp = 0;
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  
  else if(currentPageState == SETTINGS_3112_PAGE)                                                  // Integrated controller - 311
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13112_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13112_total]);
    for(i = 0; i < 12; i++)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13112_1 + i + index], 0, SETTING_3_NAME_Y_POS(i), 0);
      tempData[i] = can_3112_data[index + i];
      if(index + i > 3)
      {
        if(tempData[i] == 0)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else
        {
          tempData[i] = 12;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        fp = 0;
      }
      if((index + i == 1) || (index + i == 2) || (index + i == 3))
      {
        // RPM Conversion is needed tempData[i]
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_rpm], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        fp = 0;
      }
      else
      {
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        fp = 2;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  
  else if(currentPageState == SETTINGS_3113_PAGE)                                                  // Integrated controller - 312
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13113_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13113_total]);
    for(i = 0; i < 6; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE311_GT[page_13113_1 + i]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      tempData[i] = can_3113_data[i];
      tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
      fp = 2;
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  else if(currentPageState == SETTINGS_312_PAGE)                                                    // ugo drive controller - 
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE312_GT[page_1312_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE312_GT[page_1312_total]);
    for(i = 0; i < 9; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE312_GT[page_1312_1 + i]);
      tempData[i] = can_312_data[i];
      if(i < 7)
      {
        if(tempData[i] == 0)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else
        {
          tempData[i] = 12;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        fp = 0;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else if(i == 8)
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_percent], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        fp = 0;
      }
      else 
      {                                                                    // 7
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;                    // VAL_10BIT_TO_VOLTAGE_5V = 0.004882
        fp = 2;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  else if(currentPageState == SETTINGS_313_PAGE)                                                     // LSA drive
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE313_GT[page_1313_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE313_GT[page_1313_total]);
    for(i = 0; i < 8; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE313_GT[page_1313_1 + i]);
      tempData[i] = can_313_data[i];
      if(i < 6){
        if(tempData[i] == 0)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else
        {
          tempData[i] = 12;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        fp = 0;
      }
      if(i == 7)
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_percent], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        fp = 0;
      }
      else
      {
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        fp = 2;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  
  else if(currentPageState == SETTINGS_3141_PAGE)                                                    // yeache controller - 330
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13141_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13141_total]);
    fp = 0;
    for(i = 0; i < 12; i++)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13141_1 + i + index], 0, SETTING_3_NAME_Y_POS(i), 0);
      tempData[i] = can_3141_data[index + i];
      if(tempData[i] == 0)
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else
      {
        tempData[i] = 12;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  
  else if(currentPageState == SETTINGS_3142_PAGE)                                                    // yeache controller - 331
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13142_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13142_total]);
    for(i = 0; i < 4; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13142_1 + i]);
      tempData[i] = can_3142_data[i];
      if(i == 0)
      {
        fp = 0;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_hz], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else
      {
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        fp = 2;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  else if(currentPageState == SETTINGS_3143_PAGE)                                                    // yeache controller - 332
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13143_0]);
    for(i = 0; i < 4; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE314_GT[page_13143_1 + i]);
      tempData[i] = can_3143_data[i];
      if((i == 2) || (i == 3))
      {
        fp = 2;
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        imgCnt = draw_string(imgCnt, tempData[i], i, fp);
      }
      else
      {
        if(tempData[i] == 0)
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        else
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
    }
  }
  
  else if(currentPageState == SETTINGS_315_PAGE)                                                     // All data
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE315_GT[page_1315_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE315_GT[page_1315_total]);
    for(i = 0; i < 12; i++)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE315_GT[page_1315_1 + i + index], 0, SETTING_3_NAME_Y_POS(i), 0);
      tempData[i] = can_315_data[index + i];
      if(((index + i) == 56) || ((index + i) == 57) || ((index + i) == 58) || ((index + i) == 59) || 
         ((index + i) == 88) || ((index + i) == 89) || ((index + i) == 96) || ((index + i) == 97) || 
         (((index + i) >= 75) && ((index + i) <= 80)) ||
         (((index + i) >= 133) && ((index + i) <= 140)))
      {
        if(((index + i) == 57) || ((index + i) == 58) || ((index + i) == 59))
        {
          fp = 0;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_rpm], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else if(((index + i) == 89) || ((index + i) == 97))
        {
          fp = 0;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_percent], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else if(((index + i) == 133))
        {
          fp = 0;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_hz], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else if(((index + i) == 137) || ((index + i) == 138))
        {
          if(tempData[i] == 0)
          {
            YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
          }
          else
          {
            tempData[i] = 12;
            YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
          }
        }
        else
        {
          fp = 2;
          tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
        }
      }
      else 
      {
        if(tempData[i] == 0)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        else
        {
          tempData[i] = 12;
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], SETTING_3_STATE_X, SETTING_3_UINT_Y_POS(i), 0);
        }
        fp = 0;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  
  return imgCnt;
}

uint16_t draw_sub_settings_32(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[8];
  uint8_t i;
  
  if(currentPageState == SETTINGS_32x_PAGE)
  {
    //YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE32x_GT[page_132_0]);
    for(i = 0; i < 24; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE32x_GT[page_132_0 + i]);
    }
    
    if(flagWarning.oilPressure)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 94, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 94, 0);
  
    ////---------------------- 2024.06.20--------------------// PCODE 오류가 나오면 온 오프
    if(flagInput.waterTemperature)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 94, 0); //tseveen removed ON 20250522
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 94, 0);
    
    if(flagWarning.airFilter)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 129, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 129, 0);
    
    if(flagWarning.waterSeparator)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 129, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 129, 0);
    
    //---------------------- 2024.06.20--------------------//
    if(flagInput.grain_4)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 182, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 182, 0);
    
    if(flagInput.grain_3)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 182, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 182, 0);
    
    if(flagInput.grain_2)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 214, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 214, 0);
    
    if(flagInput.grain_1)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 214, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 214, 0);
    
    if(flagInput.leftLamp)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 262, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 262, 0);
    
    if(flagInput.rightLamp)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 262, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 262, 0);
    
    if(flagInput.charge)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 294, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 294, 0);
    
    if(flagInput.tailLamp)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 665, 294, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 665, 294, 0);
    
    if(flagInput.buzzerStop)
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_on], 290, 326, 0);
    else
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_off], 290, 326, 0);
    
    memset(_string, 0, 6);
    ftoa(tEngineSpeed, _string, 0);
    _string[5] = 0;
    YVC1_SetChar(can_data_ck1_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE32x_GT[can_data_ck1]);
    
    memset(_string, 0, 6);
    ftoa(tPowerVoltage, _string, 2);
    _string[5] = 0;
    YVC1_SetChar(can_data_ck2_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE32x_GT[can_data_ck2]);
    
    memset(_string, 0, 6);
    ftoa(tFuelPercent, _string, 0);
    _string[3] = 0;
    YVC1_SetChar(can_data_ck3_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE32x_GT[can_data_ck3]);

    memset(_string, 0, 6);
    ftoa(tFuelVoltage, _string, 2);
    _string[5] = 0;
    // Data should converted into string
    YVC1_SetChar(can_data_ck4_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE32x_GT[can_data_ck4]);   // Fuel Voltage
    
    // ----------------------------- CPU_temp----------------------------- 
    memset(_string, 0, 8);
    
    ftoa(tTemperature, _string, 0);
    for(i = 0; i < 4; i++)
    {
      if(_string[i] == 0)
      {
        _string[i + 1] = 0;
        break;
      }
    }
    YVC1_SetChar(can_data_ck5_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE32x_GT[can_data_ck5]);
  }
  return imgCnt;
}

uint16_t draw_sub_settings_33(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  C08 _string[6];
  uint8_t i;
  
  if(currentPageState == SETTINGS_33x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_133_0]);
    for(i = 0; i < 8; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_133_1 + i]);
    }
    memset(_string, 0, 6);
    ftoa(numberOfEngineOilExchange, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(can_data_eg1_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE33x_GT[can_data_eg1]);
    
    memset(_string, 0, 6);
    ftoa(engineOilHour, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(can_data_eg2_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE33x_GT[can_data_eg2]);
    
    memset(_string, 0, 6);
    ftoa(numberOfMissionOilExchange, _string, 0);
    _string[4] = 0;
    YVC1_SetChar(can_data_eg3_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE33x_GT[can_data_eg3]);
    
    memset(_string, 0, 6);
    ftoa(missionOilHour, _string, 0);                                             
    _string[4] = 0;
    YVC1_SetChar(can_data_eg4_FC, _string);
    YVC1_SetFontAttr(imgCnt++, (const T_Y643_LYR_FONTATTR *)MODE33x_GT[can_data_eg4]);
  }
  else if(currentPageState == SETTINGS_331_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_133_0]);            // engine oil exchange
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_1332_1]);
  }
  else if(currentPageState == SETTINGS_332_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_133_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE33x_GT[page_1331_1]);           // mission oil exchange
  }
  return imgCnt;
}

uint16_t draw_sub_settings_34(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  uint8_t i;
  float tempData[12];
  uint8_t fp = 0;
  
  if(currentPageState == SETTINGS_34x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE34x_GT[page_134_0]);
    for(i = 0; i < 12; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE34x_GT[page_134_1 + i]);
      fp = 0;
      tempData[i] = can_341_data[i];
      if((i == 0) || (i == 1))
      {
        fp = 2;
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else if((i > 1) && (i < 9))
      {
        fp = 0;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_s], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else if(i == 11)
      {
        fp = 1;
        tempData[i] = tempData[i] * 0.1;
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  return imgCnt;
}

uint16_t draw_sub_settings_35(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  uint8_t i;
  float tempData[12];
  uint8_t fp;
  
  if(currentPageState == SETTINGS_35x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE35x_GT[page_135_0]);
    for(i = 0; i < 8; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE35x_GT[page_135_1 + i]);
      tempData[i] = can_351_data[i];
      if(i < 5)
      {
        fp = 2;
        tempData[i] = tempData[i] * VAL_10BIT_TO_VOLTAGE_5V;
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_v], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      }
      else if(i == 7)
      {
        fp = 1;
        tempData[i] = tempData[i] * 0.1;
      }
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  return imgCnt;
}

uint16_t draw_sub_settings_36(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  float tempData[2];
  uint8_t i;
  uint8_t fp = 0;
  
  if(currentPageState == SETTINGS_36x_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE36x_GT[page_1361_0]);
    for(i = 0; i < 2; i++)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE36x_GT[page_1361_1 + i]);
      tempData[i] = can_361_data[i];
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)UNITS_GT[units_val], SETTING_3_UNIT_X, SETTING_3_UINT_Y_POS(i), 0);
      imgCnt = draw_string(imgCnt, tempData[i], i, fp);
    }
  }
  return imgCnt;
}

/*------------------------------------- Setting 4 -------------------------------------*/
uint16_t draw_sub_settings_4(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  
  if(currentPageState == SETTINGS_40_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE4xx_GT[page_14_1]);
  }
  
  return imgCnt;
}

/*------------------------------------- Setting 5 -------------------------------------*/
uint16_t draw_sub_settings_5(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  int8_t i = 0;
  //C08 _string[11] = {'0','1','2','3','4','5','6','7','8','9',0};
  if((currentPageState == SETTINGS_50_PAGE) || (currentPageState == SETTINGS_511_PAGE))
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1]);
    
    for(i = 9; i >= 0; i--)
    {
      if((number_index == i) && (currentPageState == SETTINGS_511_PAGE))
      {
        if(number_index_blink >= 4)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[digits_0 + settings_pass[i]], 60*i, 0, 0);
          number_index_blink = 0;
        }
        else
        {
          number_index_blink++;
        }
      }
      else
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[digits_0 + settings_pass[i]], 60*i, 0, 0);
      }
    }
  }
  else if(currentPageState == SETTINGS_512_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_0]);
    
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1_2]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1_3]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1_4]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_1_5]);
  }
  else if((currentPageState == SETTINGS_513_PAGE) || (currentPageState == SETTINGS_514_PAGE))
  {
    if(currentPageState == SETTINGS_513_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_2_0]);           // title
    }
    else if(currentPageState == SETTINGS_514_PAGE) 
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_3_0]);
    }
    for(i = 4; i >= 0; i--)
    {
      if(number_index == i)
      {
        if(number_index_blink >= 4)
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[digits_0 + settings_time[i]], 120*i, 0, 0);
          number_index_blink = 0;
        }
        else
        {
          number_index_blink++;
        }
      }
      else
      {
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[digits_0 + settings_time[i]], 120*i, 0, 0);
      }
    }
  }
  else if(currentPageState == SETTINGS_5131_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_2_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_2_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_2_2]);
  }
  else if(currentPageState == SETTINGS_5141_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_3_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_3_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_3_2]);
  }
  else if(currentPageState == SETTINGS_515_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_4_0]);
  }
  else if(currentPageState == SETTINGS_516_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_5_0]);
  }
  else if(currentPageState == SETTINGS_517_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE5xx_GT[page_15_6_0]);
  }
  
  return imgCnt;
}

/*------------------------------------- Setting 6 -------------------------------------*/
uint16_t draw_sub_settings_6(uint16_t _imgCnt)
{
  //uint8_t i;
  uint16_t imgCnt = _imgCnt;
  C08 _string[7];
  //memset _string[6];
  if(currentPageState == SETTINGS_60_PAGE)
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_0]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_2]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_4]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_5]);
    
    if(row_selection == 1)
    {
      if(flag.isBrigthnessSetting == FALSE)
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 115, 181, 0);
      else
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 520, 181, 0);
    }
    else if(row_selection == 2)
    {
      if(flag.isBrigthnessSetting == FALSE)
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 115, 236, 0);
      else
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 520, 236, 0);
    }
    else
    {
      if(flag.isBrigthnessSetting == FALSE)
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 115, 291, 0);
      else
        YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[SelectRow], 520, 291, 0);
    }
    

    if(lcdBrigthnessDay >= 1000)
    {
      _string[0] = ((lcdBrigthnessDay % 10000) / 1000) + 0x30;
      _string[1] = ((lcdBrigthnessDay % 1000) / 100) + 0x30;
      _string[2] = ((lcdBrigthnessDay % 100) / 10) + 0x30;
      _string[3] = '.';
      _string[4] = ((lcdBrigthnessDay % 10) / 1) + 0x30;
      _string[5] = '%';
      _string[6] = 0;
    }
    else if(lcdBrigthnessDay >= 100)
    {
      _string[0] = ((lcdBrigthnessDay % 1000) / 100) + 0x30;
      _string[1] = ((lcdBrigthnessDay % 100) / 10) + 0x30;
      _string[2] = '.';
      _string[3] = ((lcdBrigthnessDay % 10) / 1) + 0x30;
      _string[4] = '%';
      _string[5] = 0;
    }
    else if(lcdBrigthnessDay >= 10)
    {
      _string[0] = ((lcdBrigthnessDay % 100) / 10) + 0x30;
      _string[1] = '.';
      _string[2] = ((lcdBrigthnessDay % 10) / 1) + 0x30;
      _string[3] = '%';
      _string[4] = 0;
    }
    else
    {
      _string[0] = '0';
      _string[1] = '.';
      _string[2] = ((lcdBrigthnessDay % 10) / 1) + 0x30;
      _string[3] = '%';
      _string[4] = 0;
    }
        
    YVC1_SetChar(lcd1_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++,(const T_Y643_LYR_FONTATTR *)MODE6xx_GT[lcd1_value]);
       
    if(lcdBrigthnessNight >= 1000)
    {
      _string[0] = ((lcdBrigthnessNight % 10000) / 1000) + 0x30;
      _string[1] = ((lcdBrigthnessNight % 1000) / 100) + 0x30;
      _string[2] = ((lcdBrigthnessNight % 100) / 10) + 0x30;
      _string[3] = '.';
      _string[4] = ((lcdBrigthnessNight % 10) / 1) + 0x30;
      _string[5] = '%';
      _string[6] = 0;
    }
    else if(lcdBrigthnessNight >= 100)
    {
      _string[0] = ((lcdBrigthnessNight % 1000) / 100) + 0x30;
      _string[1] = ((lcdBrigthnessNight % 100) / 10) + 0x30;
      _string[2] = '.';
      _string[3] = ((lcdBrigthnessNight % 10) / 1) + 0x30;
      _string[4] = '%';
      _string[5] = 0;
    }
    else if(lcdBrigthnessNight >= 10)
    {
      _string[0] = ((lcdBrigthnessNight % 100) / 10) + 0x30;
      _string[1] = '.';
      _string[2] = ((lcdBrigthnessNight % 10) / 1) + 0x30;
      _string[3] = '%';
      _string[4] = 0;
    }
    else
    {
      _string[0] = '0';
      _string[1] = '.';
      _string[2] = ((lcdBrigthnessNight % 10) / 1) + 0x30;
      _string[3] = '%';
      _string[4] = 0;
    }
    
    // 1 - OLD WITH AXEL
    // 2 - OLD
    // 3 - NEW
    
    if(modelSelection == MODEL_OLD)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_select_1]);                // OLD
    }
    else if(modelSelection == MODEL_NEW_WITH_AXEL)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_select_2]);                // NEW WITH AXEL
    }
    else
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE6xx_GT[page6_lcd_select_3]);                // NEXT
    }
    /*
    if(axelControlFunctionEnabledTemp == TRUE)
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_on], 90, 138 , 0);
    }
    else
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)WARNING_3_GT[unit_off], 80, 138 , 0);
    }
    */
    YVC1_SetChar(lcd2_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++,(const T_Y643_LYR_FONTATTR *)MODE6xx_GT[lcd2_value]);
    
    _string[0] = ' ';
    _string[1] = '0';
    _string[2] = '.';
    _string[3] = '5';
    _string[4] = 0;
    _string[5] = 0;
    
    YVC1_SetChar(lcd3_value_FC, _string);
    YVC1_SetFontAttr(imgCnt++,(const T_Y643_LYR_FONTATTR *)MODE6xx_GT[lcd3_value]);
  }
  return imgCnt; 
}

uint16_t display_function(uint16_t _imgCnt)
{
  uint16_t imgCnt = _imgCnt;
  
  if(currentPageState == MAIN_PAGE)
  {
    imgCnt = draw_main_page(imgCnt);
  }
  else
  {
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_BG_1]);
//    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_BG_2]);
    
    // TITLE BACKGROUND 
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_TITLE1]);
    YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)MODE000_GT[MODE000_TITLE2]);
    
    imgCnt = draw_settings_menu(imgCnt);
    imgCnt = draw_sub_settings_1(imgCnt);
    imgCnt = draw_sub_settings_2(imgCnt);
    
    imgCnt = draw_sub_settings_31(imgCnt);
    imgCnt = draw_sub_settings_32(imgCnt);
    imgCnt = draw_sub_settings_33(imgCnt);
    imgCnt = draw_sub_settings_34(imgCnt);
    imgCnt = draw_sub_settings_35(imgCnt);
    imgCnt = draw_sub_settings_36(imgCnt);
    
    imgCnt = draw_sub_settings_4(imgCnt);
    imgCnt = draw_sub_settings_5(imgCnt);
    imgCnt = draw_sub_settings_6(imgCnt);
  }
  
  return imgCnt;
}
      

uint16_t draw_button(uint16_t _imgCnt, uint8_t warningPage)
{
  uint16_t imgCnt = _imgCnt;
  uint8_t wIndex;

  //Button message checker
  if(btnState[0] == BTN_RELEASED)       YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]);
  else                                  YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_selected_1]);
  
  if(currentPageState != MAIN_PAGE)
  {
    if(btnState[1] == BTN_RELEASED)     YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
    else                                YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_selected_2]);
    if(btnState[2] == BTN_RELEASED)     YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_3]);
    else                                YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_selected_3]);
    if(btnState[3] == BTN_RELEASED)     YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_4]);
    else                                YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_selected_4]);
  }
  
  if(warningPage < TOTAL_NUMBER_OF_WARNINGS)
  {
    for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
    {
      if(warnings[wIndex].state == W_STATE_ACTIVE)
      {
        if((wIndex == W_ENGINE_OIL_EXCHANGE) || (wIndex == W_MISSION_OIL_EXCHANGE))
        {
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_ok]);                                 // replace button
          YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
        }
        else if((wIndex >= W_ENGINE_STOP_SWITCH) && (wIndex <= W_2_NASON_BLOCK_SWITCH))
        {  
          if(warnings[wIndex].page == W_PAGE_1)
          {
            YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_help]);                             //help btn daraad aldaagq bolood dahiad aldaa orj irvel
            YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_huagin]);
          }
          else if(warnings[wIndex].page == W_PAGE_2)
          {
            YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
          }
        }
        else
        {
          YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_huagin], 0, 0, 0);
        }
        break;
      }
    }
  }
  else
  {
//#if defined(USER_DEBUG)
//    printf("currentPageState = %d\r\n", currentPageState);
//#endif
    // Button Implementation
    if(currentPageState == MAIN_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_menu]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_3]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_4]);
    }
    else if((currentPageState == SETTINGS_PAGE) || (currentPageState == SETTINGS_10_PAGE) || (currentPageState == SETTINGS_20_PAGE) || 
            (currentPageState == SETTINGS_30_PAGE) || (currentPageState == SETTINGS_31x_PAGE) || (currentPageState == SETTINGS_311_PAGE) || 
            (currentPageState == SETTINGS_314_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_up]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_down]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if(currentPageState == SETTINGS_60_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_up]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_down]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      
      if(flag.isBrigthnessSetting == TRUE)
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_agree]);
      }
      else
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
      }
    }
    else if(currentPageState == SETTINGS_50_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if((currentPageState == SETTINGS_511_PAGE) || (currentPageState == SETTINGS_513_PAGE) || (currentPageState == SETTINGS_514_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_increase]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_decrease]);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select], -160, 0, 0);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_complete]);
    }
    else if(currentPageState == SETTINGS_512_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_up]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_down]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if((currentPageState == SETTINGS_5131_PAGE) || (currentPageState == SETTINGS_5141_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_cancel]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_agree]); 
    }
    else if((currentPageState == SETTINGS_515_PAGE) || (currentPageState == SETTINGS_516_PAGE) || (currentPageState == SETTINGS_517_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_cancel]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if(currentPageState == SETTINGS_40_PAGE)
    {
      // 4th page has no button so I just added one button information for back to main page
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_4]);
    }
    else if((currentPageState == SETTINGS_11_PAGE) || (currentPageState == SETTINGS_111_PAGE) || (currentPageState == SETTINGS_112_PAGE) ||
            (currentPageState == SETTINGS_113_PAGE) || (currentPageState == SETTINGS_12_PAGE) || (currentPageState == SETTINGS_121_PAGE) ||
            (currentPageState == SETTINGS_122_PAGE) || (currentPageState == SETTINGS_13_PAGE) || (currentPageState == SETTINGS_131_PAGE) ||
            (currentPageState == SETTINGS_132_PAGE) || (currentPageState == SETTINGS_133_PAGE) || (currentPageState == SETTINGS_14_PAGE) ||
            (currentPageState == SETTINGS_141_PAGE) || (currentPageState == SETTINGS_16_PAGE) || (currentPageState == SETTINGS_161_PAGE) || 
            (currentPageState == SETTINGS_162_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]); // Baljaa 20240319
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]); // Baljaa 20240319
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);// Baljaa 20240319 page1
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if((currentPageState == SETTINGS_15_PAGE) || (currentPageState == SETTINGS_151_PAGE) || (currentPageState == SETTINGS_152_PAGE) ||
            (currentPageState == SETTINGS_153_PAGE) || (currentPageState == SETTINGS_154_PAGE) || (currentPageState == SETTINGS_155_PAGE) ||
            (currentPageState == SETTINGS_156_PAGE) || (currentPageState == SETTINGS_157_PAGE) || (currentPageState == SETTINGS_158_PAGE) || 
            (currentPageState == SETTINGS_159_PAGE) || (currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE) ||
            (currentPageState == SETTINGS_1512_PAGE))
    {
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous], -305, 0, 0);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_han]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
      if((currentPageState == SETTINGS_15_PAGE) || (currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE))
      {
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_mode_han]);
      }
    }
    
    else if((currentPageState == SETTINGS_21x_PAGE) || (currentPageState == SETTINGS_22x_PAGE) || (currentPageState == SETTINGS_23x_PAGE) ||
            (currentPageState == SETTINGS_24x_PAGE) || (currentPageState == SETTINGS_26x_PAGE))
    {
      if(row_selection != 1)
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous_item]);
      if((row_selection != 8) && (currentPageState == SETTINGS_21x_PAGE))
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_item]);
      else if((row_selection != 7) && (currentPageState == SETTINGS_22x_PAGE))
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_item]);
      else if((row_selection != 8) && (currentPageState == SETTINGS_23x_PAGE))
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_item]);
      else if((row_selection != 1) && (currentPageState == SETTINGS_24x_PAGE))
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_item]);
      else if((row_selection != 2) && (currentPageState == SETTINGS_26x_PAGE))
        YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_item]);
      
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if((currentPageState >= SETTINGS_251_PAGE) && (currentPageState <= SETTINGS_2511_PAGE))
    {
      //YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select], -160, 0, 0);
      YVC1_SetSprtAttrWithParam(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous], -305, 0, 0);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_mode_han]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_next_han]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_select]);
    }
    else if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) ||
               (currentPageState == SETTINGS_217_PAGE) || 
               (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_223_PAGE) ||
               (currentPageState == SETTINGS_224_PAGE) || (currentPageState == SETTINGS_225_PAGE) || 
               (currentPageState == SETTINGS_231_PAGE) || (currentPageState == SETTINGS_232_PAGE) || (currentPageState == SETTINGS_233_PAGE) ||
               (currentPageState == SETTINGS_234_PAGE) ||
               (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
               (currentPageState == SETTINGS_238_PAGE) ||  
               (currentPageState == SETTINGS_241_PAGE) ||
               (currentPageState == SETTINGS_261_PAGE) || (currentPageState == SETTINGS_262_PAGE) )
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_increase]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_decrease]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_cancel]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_agree]);
    }
    else if((currentPageState == SETTINGS_214_PAGE) || (currentPageState == SETTINGS_215_PAGE) ||
               (currentPageState == SETTINGS_216_PAGE) || (currentPageState == SETTINGS_218_PAGE) ||
               (currentPageState == SETTINGS_226_PAGE)  || (currentPageState == SETTINGS_227_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_not_used]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_used]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_cancel]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_agree]);           
    }
    
    else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
            (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
            (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_315_PAGE) || (currentPageState == SETTINGS_3143_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_up]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_down]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_pgup]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_pgdn]);
    }
    else if((currentPageState == SETTINGS_32x_PAGE) || (currentPageState == SETTINGS_34x_PAGE) ||
            (currentPageState == SETTINGS_35x_PAGE) || (currentPageState == SETTINGS_36x_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_1]); // Baljaa 20240319
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_2]); // Baljaa 20240319
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_unselect_4]); // Baljaa 20240319
    }
    else if(currentPageState == SETTINGS_33x_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_engine]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_mission]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
    }
    else if((currentPageState == SETTINGS_331_PAGE) || (currentPageState == SETTINGS_332_PAGE))
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_exchange]);
    }
    else if(currentPageState == SETTINGS_60_PAGE)
    {
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_up]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_down]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_previous]);
      YVC1_SetSprtAttr(imgCnt++, (const T_Y643_LYR_SPRTATTR *)BTN_PAGE_GT[Btn_agree]);
    }
  }
  return imgCnt;
}

void check_display_buttons(void)
{
  static uint8_t prevBuzzerStop;
  static uint8_t btnLongPressed = 0;
  static uint16_t timerBtnLongPressed = 0;
  uint8_t wIndex;
  
  if(currentPageState == WARNING_PAGE)                                    // should check
  {
    if((flagInput.buzzerStop == ON) && (prevBuzzerStop == OFF))
    {
      for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
      {
        if(warnings[wIndex].state == W_STATE_ACTIVE)
        {
          set_button_sound();
          warnings[wIndex].state = W_STATE_PASSIVE;
          break;
        }
      }
    }
  }
  prevBuzzerStop = flagInput.buzzerStop;
  
  
  timerBtnLongPressed += 2;
  
  // 1st button
  if(btnState[0] == BTN_PRESSED)
  {    
    if(timerBtnLongPressed >= SETTINGS_PAGE_SHIFT_TIMER)
    {
      timerBtnLongPressed = SETTINGS_PAGE_SHIFT_TIMER;
      
      if(currentPageState == MAIN_PAGE)
      {
        currentPageState = SETTINGS_PAGE;
        previousPageState = MAIN_PAGE;
        row_selection = 1;
      }
      else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
            (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
            (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_3143_PAGE) || (currentPageState == SETTINGS_315_PAGE))
      {
        btnLongPressed = 1;
        update_pages_back();
      }
    }
  }
  else
  {
    timerBtnLongPressed = 0;
    
    if(prevBtnState[0] == BTN_PRESSED)
    {
      if(btnLongPressed == 1)
      {
        btnLongPressed = 0;
        return;
      }
      
      if(currentPageState == WARNING_PAGE)                                    // should check
      {
        for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
        {
          if((warnings[wIndex].state == W_STATE_ACTIVE) && (warnings[wIndex].page == W_PAGE_1))
          {
            if((wIndex == W_ENGINE_OIL_EXCHANGE) || (wIndex == W_MISSION_OIL_EXCHANGE))
            {
              warnings[wIndex].state = W_STATE_PASSIVE;
            }
            else if((wIndex >= W_ENGINE_STOP_SWITCH) && (wIndex <= W_2_NASON_BLOCK_SWITCH))
            {
              warnings[wIndex].page = W_PAGE_2;
            }
            break;
          }
        }
      }
      
      if(currentPageState == SETTINGS_33x_PAGE)
      {
        previousPageState = currentPageState;
        currentPageState = SETTINGS_331_PAGE;                                 // Engine oil
      }
      
      if(flag.isBrigthnessSetting == FALSE)
      {
        row_selection--;
        if(row_selection < 1)
        {
            row_selection = 1;
        }
      }

      // Added on 2021 March 9 - ESCAPE button
      if((currentPageState == SETTINGS_15_PAGE) || (currentPageState == SETTINGS_151_PAGE) || (currentPageState == SETTINGS_152_PAGE) ||
         (currentPageState == SETTINGS_153_PAGE) || (currentPageState == SETTINGS_154_PAGE) || (currentPageState == SETTINGS_155_PAGE) ||
         (currentPageState == SETTINGS_156_PAGE) || (currentPageState == SETTINGS_157_PAGE) || (currentPageState == SETTINGS_158_PAGE) ||
         (currentPageState == SETTINGS_159_PAGE) || (currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE) ||
         (currentPageState == SETTINGS_1512_PAGE) ) 
      {
        update_pages_back();
      }
      
      else if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) ||
               (currentPageState == SETTINGS_217_PAGE) || 
               (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_223_PAGE) ||
               (currentPageState == SETTINGS_224_PAGE) || (currentPageState == SETTINGS_225_PAGE) || 
               (currentPageState == SETTINGS_231_PAGE) || (currentPageState == SETTINGS_232_PAGE) || (currentPageState == SETTINGS_233_PAGE) ||
               (currentPageState == SETTINGS_234_PAGE) ||
               (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
               (currentPageState == SETTINGS_238_PAGE) ||
               (currentPageState == SETTINGS_241_PAGE) || 
               (currentPageState == SETTINGS_261_PAGE) || (currentPageState == SETTINGS_262_PAGE)
                 )
      {
        if(setup_mode == SETUP_MODE_NONE){
          if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) ||
             (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_233_PAGE) ||
             (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
             (currentPageState == SETTINGS_238_PAGE))
          {
            new_setup_data_received = new_setup_data_received + 5;
            if((currentPageState == SETTINGS_211_PAGE) && (new_setup_data_received >= 151)){
              new_setup_data_received = 151;
            }
            else if(((currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_233_PAGE)) && (new_setup_data_received >= 100)){
              new_setup_data_received = 100;
            }
            else if((currentPageState == SETTINGS_213_PAGE) && (new_setup_data_received >= 45)){
              new_setup_data_received = 45;
            }
            else if(((currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE)) && (new_setup_data_received >= 120)){
              new_setup_data_received = 120;
            }
            else if(((currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
             (currentPageState == SETTINGS_238_PAGE)) && (new_setup_data_received >= 300)){
              new_setup_data_received = 300;
            }
          }
          else if(currentPageState == SETTINGS_217_PAGE){
            new_setup_data_received = new_setup_data_received + 50;
            if(new_setup_data_received >= 2520){
              new_setup_data_received = 2520;
            }
          }
          else if(currentPageState == SETTINGS_225_PAGE){
            new_setup_data_received = new_setup_data_received + 10;
            if(new_setup_data_received >= 505){
              new_setup_data_received = 505;
            }
          }
          else if(currentPageState == SETTINGS_232_PAGE){
            new_setup_data_received = new_setup_data_received + 2;
            if(new_setup_data_received >= 66){
              new_setup_data_received = 66;
            }
          }
          else{
            new_setup_data_received = new_setup_data_received + 1;
            if(((currentPageState == SETTINGS_223_PAGE) || (currentPageState == SETTINGS_224_PAGE)) && (new_setup_data_received >= 60)){
              new_setup_data_received = 60;
            }
            else if((currentPageState == SETTINGS_231_PAGE) && (new_setup_data_received >= 40)){
              new_setup_data_received = 40;
            }
            else if(((currentPageState == SETTINGS_234_PAGE) || (currentPageState == SETTINGS_241_PAGE)) && (new_setup_data_received >= 25)){
              new_setup_data_received = 25;
            }
            else if((currentPageState == SETTINGS_261_PAGE) && (new_setup_data_received >= 1023)){
              new_setup_data_received = 1023;                                   // Check it again
            }
            else if((currentPageState == SETTINGS_262_PAGE) && (new_setup_data_received >= 1023)){
              new_setup_data_received = 1023;                                   // Check it again
            }
          }
        }
      }
      else if( (currentPageState == SETTINGS_214_PAGE) || (currentPageState == SETTINGS_215_PAGE) || (currentPageState == SETTINGS_216_PAGE) ||
               (currentPageState == SETTINGS_218_PAGE) || (currentPageState == SETTINGS_226_PAGE) || (currentPageState == SETTINGS_227_PAGE))
      {
        if(setup_mode == SETUP_MODE_NONE){
          if((new_setup_data_received == 0x5555) || ((uint16_t)new_setup_data_received == 0xAAAA)) new_setup_data_received = 0xAAAA;
          else if((new_setup_data_received == 0x0000) || (new_setup_data_received == 0x0001)) new_setup_data_received = 0x0000;
        }
      }
      else if((currentPageState >= SETTINGS_251_PAGE) && (currentPageState <= SETTINGS_2511_PAGE))
      {
        update_pages_back();
      }
      
      else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
            (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
            (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_3143_PAGE) || (currentPageState == SETTINGS_315_PAGE))
      {
        if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) ||
           (currentPageState == SETTINGS_3141_PAGE) || (currentPageState == SETTINGS_315_PAGE))
        {
          index -= 1;
           if(index < 0)
             index = 0;
        }
      }
      else if(currentPageState == SETTINGS_511_PAGE){
        settings_pass[number_index]++;
        if(settings_pass[number_index] > 9){
          settings_pass[number_index] = 9;
        }
      }
      else if((currentPageState == SETTINGS_513_PAGE) || (currentPageState == SETTINGS_514_PAGE)) {
        settings_time[number_index]++;
        if(settings_time[number_index] > 9){
          settings_time[number_index] = 9;
        }
      }
      else if(currentPageState == SETTINGS_60_PAGE)
      {
        if(flag.isBrigthnessSetting == TRUE)
        {
          // row
          if(row_selection == 1)
          {
            lcdBrigthnessDay += LCD_BRIGTHNESS_STEP;

            if(lcdBrigthnessDay > LCD_BRIGTHNESS_MAX)
            {
              lcdBrigthnessDay = LCD_BRIGTHNESS_MAX;
            }
            else if(lcdBrigthnessDay < LCD_BRIGTHNESS_MIN)
            {
              lcdBrigthnessDay = LCD_BRIGTHNESS_MIN;
            }
            
            if(flagInput.tailLamp == OFF)
            {
              update_timer(lcdBrigthnessDay);
            }
          }
          else if(row_selection == 2)
          {
            lcdBrigthnessNight += LCD_BRIGTHNESS_STEP;
          
            if(lcdBrigthnessNight > LCD_BRIGTHNESS_MAX)
            {
              lcdBrigthnessNight = LCD_BRIGTHNESS_MAX;
            }
            else if(lcdBrigthnessNight < LCD_BRIGTHNESS_MIN)
            {
              lcdBrigthnessNight = LCD_BRIGTHNESS_MIN;
            }
            
            if(flagInput.tailLamp == ON)
            {
              update_timer(lcdBrigthnessNight);
            }
          }
          else
          {
            if(modelSelection == MODEL_OLD)
            {
              modelSelection = MODEL_NEW_WITH_AXEL;
            }
            else if(modelSelection == MODEL_NEW_WITH_AXEL)
            {
              modelSelection = MODEL_NEXT;
            }
            else
            {
              modelSelection = MODEL_OLD;
            }
          }
        }
      }
    }
  }
  
  //2nd button
  if((btnState[1] == BTN_RELEASED) && (prevBtnState[1] == BTN_PRESSED))
  {
    if(flag.isBrigthnessSetting == FALSE)
    {
      row_selection++;
    }
    
    if((currentPageState == SETTINGS_512_PAGE) || (currentPageState == SETTINGS_31x_PAGE))
    {                   // Total row is 5 "SETTINGS_PAGE"
      if(row_selection > 5){ row_selection = 5; }                                                             // Changed 5 to 6 because of internal settings are added
    } 
    else if(currentPageState == SETTINGS_23x_PAGE)
    {                                                       // Total row is 4 "SETTINGS_23x_PAGE"
      if(row_selection > 8){ row_selection = 8; }                                                             // changed on 24 March 2021
    } 
    else if((currentPageState == SETTINGS_21x_PAGE))
    {                                                     // Total row is 8 "SETTINGS_21x_PAGE"
      if(row_selection > 8){ row_selection = 8; }
    } 
    else if(currentPageState == SETTINGS_22x_PAGE)
    {                                                       // Total row is 7 "SETTINGS_22x_PAGE"
      if(row_selection > 7){ row_selection = 7; }
    } 
    else if(currentPageState == SETTINGS_24x_PAGE)
    {                                                       // Total row is 1 "SETTINGS_24x_PAGE"
      if(row_selection > 1){ row_selection = 1; }
    } 
    else if((currentPageState == SETTINGS_26x_PAGE))
    {                                                       // Total row is 2 "SETTINGS_26x_PAGE"
      if(row_selection > 2){ row_selection = 2; }
    } 
    else if((currentPageState == SETTINGS_PAGE) || (currentPageState == SETTINGS_10_PAGE) || 
            (currentPageState == SETTINGS_20_PAGE) || (currentPageState == SETTINGS_30_PAGE))
    {           // Total row is 6 "SETTINGS_10_PAGE" & "SETTINGS_20_PAGE" & "SETTINGS_30_PAGE"
      if(row_selection > 6){ row_selection = 6; }
    } 
    else if((currentPageState == SETTINGS_311_PAGE) || (currentPageState == SETTINGS_314_PAGE) || (currentPageState == SETTINGS_60_PAGE))
    {
      if(row_selection > 3){ row_selection = 3; }
    } 
    
    
    else if(currentPageState == SETTINGS_511_PAGE)
    {
      settings_pass[number_index]--;
      if(settings_pass[number_index] < 0)
        settings_pass[number_index] = 0;
    }
    
    else if((currentPageState == SETTINGS_513_PAGE) || (currentPageState == SETTINGS_514_PAGE))
    {
      settings_time[number_index]--;
      if(settings_time[number_index] < 0)
        settings_time[number_index] = 0;
    }
    
    if(currentPageState == SETTINGS_33x_PAGE)
    {
      previousPageState = currentPageState;
      currentPageState = SETTINGS_332_PAGE;                                 // Mission oil
    }
    
    // Added on 2021 March 29 - mode change button
    if(currentPageState == SETTINGS_15_PAGE) 
    {
      if(new_setup_data_received == 0)
        new_setup_data_received = 1;
      else
        new_setup_data_received = 0;
    }
    else if((currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE))                // Added on 2021.06.29
    {
      new_setup_data_received_to_update++;
      if(new_setup_data_received_to_update > 5)
      {
        new_setup_data_received_to_update = 1;
      }
    }
    else if(currentPageState == SETTINGS_60_PAGE)
    {
      if(flag.isBrigthnessSetting == TRUE) 
      {
        if(row_selection == 1)
        {
          lcdBrigthnessDay -= LCD_BRIGTHNESS_STEP;
      
          if(lcdBrigthnessDay > LCD_BRIGTHNESS_MAX)
          {
            lcdBrigthnessDay = LCD_BRIGTHNESS_MAX;
          }
          else if(lcdBrigthnessDay < LCD_BRIGTHNESS_MIN)
          {
            lcdBrigthnessDay = LCD_BRIGTHNESS_MIN;
          }
          
          if(flagInput.tailLamp == OFF)
          {
            update_timer(lcdBrigthnessDay);
          }
        }
        else if(row_selection == 2)
        {
          lcdBrigthnessNight -= LCD_BRIGTHNESS_STEP;
      
          if(lcdBrigthnessNight > LCD_BRIGTHNESS_MAX)
          {
            lcdBrigthnessNight = LCD_BRIGTHNESS_MAX;
          }
          else if(lcdBrigthnessNight < LCD_BRIGTHNESS_MIN)
          {
            lcdBrigthnessNight = LCD_BRIGTHNESS_MIN;
          }
          
          if(flagInput.tailLamp == ON)
          {
            update_timer(lcdBrigthnessNight);
          }
        }
        else
        {
          if(modelSelection == MODEL_OLD)
          {
            modelSelection = MODEL_NEW_WITH_AXEL;
          }
          else if(modelSelection == MODEL_NEW_WITH_AXEL)
          {
            modelSelection = MODEL_NEXT;
          }
          else
          {
            modelSelection = MODEL_OLD;
          }
        }
      }
    }

    else if((currentPageState >= SETTINGS_253_PAGE) && (currentPageState <= SETTINGS_2510_PAGE))
    {         // Added on 2021.06.29
      if(currentPageState == SETTINGS_2510_PAGE)
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          new_setup_data_received_to_update += 50;
          if((new_setup_data_received_to_update > 2700) || (new_setup_data_received_to_update < 2400))
          {
            new_setup_data_received_to_update = 2400;
          }
        }
        else
        {
          // 
          axel_threshing_rpm_temp += 50;
          if((axel_threshing_rpm_temp > 2700) || (axel_threshing_rpm_temp < 2400))
          {
            axel_threshing_rpm_temp = 2400;
          }
        }
      }
      else
      {
        if(axelControlFunctionEnabled == FALSE)
        {
          new_setup_data_received_to_update++;
          if(new_setup_data_received_to_update > 5)
          {
            new_setup_data_received_to_update = 0;
          }
        }
        else
        {
          if(currentPageState == SETTINGS_253_PAGE)
          {
            axel_threshing_delay_temp++;
            if(axel_threshing_delay_temp > 5)
              axel_threshing_delay_temp = 0;
          }
          else if(currentPageState == SETTINGS_254_PAGE)
          {
            axel_auger_delay_temp++;
            if(axel_auger_delay_temp > 5)
              axel_auger_delay_temp = 0;
          }
          else if(currentPageState == SETTINGS_255_PAGE)
          {
            axel_auger_auto_delay_temp++;
            if(axel_auger_auto_delay_temp > 5)
              axel_auger_auto_delay_temp = 0;
          }
          else if(currentPageState == SETTINGS_256_PAGE)
          {
            axel_yeache_ku_time_temp++;
            if(axel_yeache_ku_time_temp > 5)
              axel_yeache_ku_time_temp = 0;
          }
          else if(currentPageState == SETTINGS_257_PAGE)
          {
            axel_tbs_ku_time_temp++;
            if(axel_tbs_ku_time_temp > 5)
              axel_tbs_ku_time_temp = 0;
          }
          else if(currentPageState == SETTINGS_258_PAGE)
          {
            axel_auger_ku_time_temp++;
            if(axel_auger_ku_time_temp > 5)
              axel_auger_ku_time_temp = 0;
          }
          else if(currentPageState == SETTINGS_259_PAGE)
          {
            axel_c_speed_ku_time_temp++;
            if(axel_c_speed_ku_time_temp > 5)
              axel_c_speed_ku_time_temp = 0;
          }
        }
      }
    }
            
    else if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) ||
             (currentPageState == SETTINGS_217_PAGE) || 
             (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_223_PAGE) ||
             (currentPageState == SETTINGS_224_PAGE) || (currentPageState == SETTINGS_225_PAGE) || 
             (currentPageState == SETTINGS_231_PAGE) || (currentPageState == SETTINGS_232_PAGE) || (currentPageState == SETTINGS_233_PAGE) ||
             (currentPageState == SETTINGS_234_PAGE) ||
             (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
             (currentPageState == SETTINGS_238_PAGE) ||
             (currentPageState == SETTINGS_241_PAGE) ||
             (currentPageState == SETTINGS_261_PAGE) || (currentPageState == SETTINGS_262_PAGE))
    {
      if(setup_mode == SETUP_MODE_NONE)
      {
        if((currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) ||
           (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_233_PAGE) ||
           (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
           (currentPageState == SETTINGS_238_PAGE))
        {
          new_setup_data_received = new_setup_data_received - 5;
          if((currentPageState == SETTINGS_211_PAGE) && (new_setup_data_received <= 6))
          {
            new_setup_data_received = 6;
          }
          else if(((currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) || (currentPageState == SETTINGS_222_PAGE)) && 
                  (new_setup_data_received <= 10))
          {
            new_setup_data_received = 10;
          }
          else if((currentPageState == SETTINGS_221_PAGE) && (new_setup_data_received <= 40))
          {
            new_setup_data_received = 40;
          }
          else if(((currentPageState == SETTINGS_233_PAGE) || (currentPageState == SETTINGS_235_PAGE) || 
                   (currentPageState == SETTINGS_236_PAGE) || (currentPageState == SETTINGS_237_PAGE) ||
                   (currentPageState == SETTINGS_238_PAGE)) && (new_setup_data_received <= 20))
          {
            new_setup_data_received = 20;
          }
        }
        else if(currentPageState == SETTINGS_217_PAGE)
        {
          new_setup_data_received = new_setup_data_received - 50;
          if(new_setup_data_received <= 20)
          {
            new_setup_data_received = 20;
          }
        }
        else if(currentPageState == SETTINGS_225_PAGE)
        {
          new_setup_data_received = new_setup_data_received - 10;
          if(new_setup_data_received <= 345)
          {
            new_setup_data_received = 345;
          }
        }
        else if(currentPageState == SETTINGS_232_PAGE)
        {
          new_setup_data_received = new_setup_data_received - 2;
          if(new_setup_data_received <= 20)
          {
            new_setup_data_received = 20;
          }
        }
        else
        {
          new_setup_data_received = new_setup_data_received - 1;
          if(((currentPageState == SETTINGS_223_PAGE) || (currentPageState == SETTINGS_224_PAGE)) && (new_setup_data_received <= 5))
          {
            new_setup_data_received = 5;
          }
          else if((currentPageState == SETTINGS_231_PAGE) && (new_setup_data_received <= 25))
          {
            new_setup_data_received = 25;
          }
          else if((currentPageState == SETTINGS_234_PAGE) && (new_setup_data_received <= 0))
          {
            new_setup_data_received = 0;
          }
          else if((currentPageState == SETTINGS_241_PAGE) && (new_setup_data_received <= 10))
          {
            new_setup_data_received = 10;
          }
          else if((currentPageState == SETTINGS_261_PAGE) && (new_setup_data_received <= 0))
          {
            new_setup_data_received = 0;
          }
          else if((currentPageState == SETTINGS_262_PAGE) && (new_setup_data_received <= 0))
          {
            new_setup_data_received = 0;
          }
        }
      }
    }
    else if( (currentPageState == SETTINGS_214_PAGE) || (currentPageState == SETTINGS_215_PAGE) || (currentPageState == SETTINGS_216_PAGE) ||
             (currentPageState == SETTINGS_218_PAGE) || (currentPageState == SETTINGS_226_PAGE) || (currentPageState == SETTINGS_227_PAGE))
    {
      if(setup_mode == SETUP_MODE_NONE)
      {
        if((new_setup_data_received == 0x0000) || (new_setup_data_received == 0x0001)) 
        {
          new_setup_data_received = 0x0001;
        }
        else if((new_setup_data_received == 0x5555) || ((uint16_t)new_setup_data_received == 0xAAAA)) 
        {
          new_setup_data_received = 0x5555;
        }
      }
    }
    else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
          (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
          (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_3143_PAGE) || (currentPageState == SETTINGS_315_PAGE))
    {
      if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) ||
         (currentPageState == SETTINGS_3141_PAGE) || (currentPageState == SETTINGS_315_PAGE))
      {
        index += 1;
             if((currentPageState == SETTINGS_3111_PAGE) && (index > 44)) { index = 44;  }                  // 56
        else if((currentPageState == SETTINGS_3112_PAGE) && (index > 7))  { index = 7;   }                  // 19
        else if((currentPageState == SETTINGS_3141_PAGE) && (index > 23)) { index = 23;  }                  // 35
        else if((currentPageState == SETTINGS_315_PAGE)  && (index > 129)){ index = 129; }                  // 141
      }
    }
  }
  
  // Back button - 3rd button
  if((btnState[2] == BTN_RELEASED) && (prevBtnState[2] == BTN_PRESSED))
  {
    // Added on 2021 March 9
    if(currentPageState == SETTINGS_15_PAGE)  { currentPageState = SETTINGS_151_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_151_PAGE) { currentPageState = SETTINGS_152_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_152_PAGE) { currentPageState = SETTINGS_153_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_153_PAGE) { currentPageState = SETTINGS_154_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_154_PAGE) { currentPageState = SETTINGS_155_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_155_PAGE) { currentPageState = SETTINGS_156_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_156_PAGE) { currentPageState = SETTINGS_157_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_157_PAGE) { currentPageState = SETTINGS_158_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_158_PAGE) { currentPageState = SETTINGS_159_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_159_PAGE) { currentPageState = SETTINGS_1510_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_1510_PAGE){ currentPageState = SETTINGS_1511_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_1511_PAGE){ currentPageState = SETTINGS_1512_PAGE; new_setup_data_received_flag = 0; setup_mode_rw = SETUP_READ; }
    else if(currentPageState == SETTINGS_1512_PAGE){  }
        
    //else if(currentPageState == SETTINGS_25x_PAGE){ currentPageState = SETTINGS_251_PAGE; new_setup_data_received_flag = 0; new_setup_data = -1; setup_mode = SETUP_READ; }
    else if(currentPageState == SETTINGS_251_PAGE)
    { 
      currentPageState = SETTINGS_252_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_252_PAGE)
    { 
      currentPageState = SETTINGS_253_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0;
      }
    }
    else if(currentPageState == SETTINGS_253_PAGE)
    { 
      currentPageState = SETTINGS_254_PAGE;       
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_254_PAGE)
    { 
      currentPageState = SETTINGS_255_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_255_PAGE)
    { 
      currentPageState = SETTINGS_256_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_256_PAGE)
    { 
      currentPageState = SETTINGS_257_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_257_PAGE)
    { 
      currentPageState = SETTINGS_258_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_258_PAGE)
    { 
      currentPageState = SETTINGS_259_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_259_PAGE)
    { 
      currentPageState = SETTINGS_2510_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_2510_PAGE)
    { 
      currentPageState = SETTINGS_2511_PAGE; 
      if(axelControlFunctionEnabled == FALSE)
      {
        new_setup_data_received_flag = 0; 
        setup_mode_rw = SETUP_READ; 
        new_setup_data = 0; 
      }
    }
    else if(currentPageState == SETTINGS_2511_PAGE){ /* nothing do in this page */ }

    else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
          (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
          (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_3143_PAGE) || (currentPageState == SETTINGS_315_PAGE))
    {
      if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) ||
          (currentPageState == SETTINGS_3141_PAGE) || (currentPageState == SETTINGS_315_PAGE))
      {
        index -= 12;
        if(index < 0){
          index = 0;
        }
      }
    }
    
    else if((currentPageState == SETTINGS_511_PAGE) || (currentPageState == SETTINGS_513_PAGE) || (currentPageState == SETTINGS_514_PAGE))
    {
      number_index++;                     
      if(currentPageState == SETTINGS_511_PAGE) 
      {
        if(number_index > 9) { number_index = 0; }
      }
      else 
      {
        if(number_index > 4) { number_index = 0; }
      }
    }
    else if(currentPageState == SETTINGS_512_PAGE)
    {
      currentPageState = SETTINGS_PAGE;  // SETTINGS_511_PAGE   2024.05.08 tym에서 바뀜.
      settings_pass[0] = 0;
      settings_pass[1] = 0;
      settings_pass[2] = 0;
      settings_pass[3] = 0;
      settings_pass[4] = 0;
      settings_pass[5] = 0;
      settings_pass[6] = 0;
      settings_pass[7] = 0;
      settings_pass[8] = 0;
      settings_pass[9] = 0;
    }
    else if((currentPageState == SETTINGS_5131_PAGE) || (currentPageState == SETTINGS_5141_PAGE) ||
            (currentPageState == SETTINGS_515_PAGE) || (currentPageState == SETTINGS_516_PAGE) || 
            (currentPageState == SETTINGS_517_PAGE))
    {
      currentPageState = SETTINGS_512_PAGE;
      //row_selection = 1;
      
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_5131_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_5141_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_515_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_516_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_517_PAGE; }
      currentPageState = SETTINGS_512_PAGE;
    }
    else
    {
        update_pages_back();
    }
  }
  
  // OK button - 4th button
  if((btnState[3] == BTN_RELEASED) && (prevBtnState[3] == BTN_PRESSED))
  {   
    if(currentPageState == WARNING_PAGE)
    {
      for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
      {
        if(warnings[wIndex].state == W_STATE_ACTIVE)
        {
          warnings[wIndex].state = W_STATE_PASSIVE;
          return;
        }
      }
    }
    
    if(currentPageState == SETTINGS_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_10_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_20_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_30_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_40_PAGE; } 
      else if(row_selection == 5){          currentPageState = SETTINGS_50_PAGE; }
      else if(row_selection == 6){          currentPageState = SETTINGS_60_PAGE; }
      row_selection = 1;
      flag.isBrigthnessSetting = FALSE;
    }
    else if(currentPageState == SETTINGS_10_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_11_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_12_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_13_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_14_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_15_PAGE; setup_mode_rw = SETUP_READ; new_setup_data_received_flag = 0; }                          // Added on 2021 March 9
      else if(row_selection == 6){          currentPageState = SETTINGS_16_PAGE; }
      row_selection = 1;
    }
    else if((currentPageState == SETTINGS_11_PAGE) || (currentPageState == SETTINGS_111_PAGE) || (currentPageState == SETTINGS_112_PAGE) || 
            (currentPageState == SETTINGS_113_PAGE) ||
            (currentPageState == SETTINGS_12_PAGE) || (currentPageState == SETTINGS_121_PAGE) || (currentPageState == SETTINGS_122_PAGE) ||
            (currentPageState == SETTINGS_13_PAGE) || (currentPageState == SETTINGS_131_PAGE) || (currentPageState == SETTINGS_132_PAGE) || 
            (currentPageState == SETTINGS_133_PAGE) ||
            (currentPageState == SETTINGS_14_PAGE) || (currentPageState == SETTINGS_141_PAGE) ||

            (currentPageState == SETTINGS_16_PAGE) || (currentPageState == SETTINGS_161_PAGE) || (currentPageState == SETTINGS_162_PAGE) ||
            (currentPageState == SETTINGS_163_PAGE) )
    {
      if(move_next_setup_page)
      {
        move_next_setup_page = 0;                                                                     // Added this row on 2021.05.13
        if(currentPageState == SETTINGS_11_PAGE)            currentPageState = SETTINGS_111_PAGE;
        else if(currentPageState == SETTINGS_111_PAGE)      currentPageState = SETTINGS_112_PAGE;
        else if(currentPageState == SETTINGS_112_PAGE)      currentPageState = SETTINGS_113_PAGE;
        else if(currentPageState == SETTINGS_12_PAGE)       currentPageState = SETTINGS_121_PAGE;
        else if(currentPageState == SETTINGS_121_PAGE)      currentPageState = SETTINGS_122_PAGE;
        else if(currentPageState == SETTINGS_13_PAGE)       currentPageState = SETTINGS_131_PAGE;
        else if(currentPageState == SETTINGS_131_PAGE)      currentPageState = SETTINGS_132_PAGE;
        else if(currentPageState == SETTINGS_132_PAGE)      currentPageState = SETTINGS_133_PAGE;
        else if(currentPageState == SETTINGS_14_PAGE)       currentPageState = SETTINGS_141_PAGE;
        else if(currentPageState == SETTINGS_16_PAGE)       currentPageState = SETTINGS_161_PAGE;
        else if(currentPageState == SETTINGS_161_PAGE)      currentPageState = SETTINGS_162_PAGE;
        else if(currentPageState == SETTINGS_162_PAGE)      currentPageState = SETTINGS_163_PAGE;
        
        else if((currentPageState == SETTINGS_113_PAGE) || (currentPageState == SETTINGS_122_PAGE) ||
           (currentPageState == SETTINGS_133_PAGE) || (currentPageState == SETTINGS_141_PAGE) ||
           (currentPageState == SETTINGS_1512_PAGE) || (currentPageState == SETTINGS_163_PAGE))                           // Added on 2021 March 9
        {
          currentPageState = SETTINGS_10_PAGE; 
          previousPageState = currentPageState;
          row_selection = 1;
          setup_mode = SETUP_MODE_NONE;
          new_setup_data_received_flag = 0;
          new_setup_data = -1;
        }
        else
        {
          move_next_setup_page = 1;
        }
      }
    }
    else if((currentPageState == SETTINGS_15_PAGE) || (currentPageState == SETTINGS_151_PAGE) || (currentPageState == SETTINGS_152_PAGE) ||
        (currentPageState == SETTINGS_153_PAGE) || (currentPageState == SETTINGS_154_PAGE) || (currentPageState == SETTINGS_155_PAGE) ||
        (currentPageState == SETTINGS_156_PAGE) || (currentPageState == SETTINGS_157_PAGE) || (currentPageState == SETTINGS_158_PAGE) ||
        (currentPageState == SETTINGS_159_PAGE) || (currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE))
    {
        
      setup_mode_rw = SETUP_WRITE;
      new_setup_data_received_flag = 0;
      if(currentPageState == SETTINGS_15_PAGE)
      {
        new_setup_data = new_setup_data_received;
      }
      else if((currentPageState == SETTINGS_151_PAGE) || (currentPageState == SETTINGS_152_PAGE) || (currentPageState == SETTINGS_153_PAGE))
      {
        new_setup_data = MC_steering_lever;
      }
      else if((currentPageState == SETTINGS_154_PAGE) || (currentPageState == SETTINGS_155_PAGE) || (currentPageState == SETTINGS_156_PAGE))
      {
        new_setup_data = MC_driveing_lever;
      }
      else if((currentPageState == SETTINGS_157_PAGE) || (currentPageState == SETTINGS_158_PAGE) || (currentPageState == SETTINGS_159_PAGE))
      {
        new_setup_data = MC_subshift_lever;
      }
      else if((currentPageState == SETTINGS_1510_PAGE) || (currentPageState == SETTINGS_1511_PAGE))       // Updated on 2021.06.29
      {
        new_setup_data = new_setup_data_received_to_update;
      }
    }
    else if(currentPageState == SETTINGS_20_PAGE)
    {
      previousPageState = currentPageState;
      new_setup_data = -1;                                                                   // Added on 22 March 2021
           if(row_selection == 1){          currentPageState = SETTINGS_21x_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_22x_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_23x_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_24x_PAGE; }
      //else if(row_selection == 5){          currentPageState = SETTINGS_25x_PAGE; }
      else if(row_selection == 5){          
        currentPageState = SETTINGS_251_PAGE;  
        if(axelControlFunctionEnabled == FALSE)
        {
          new_setup_data_received_flag = 0;
          new_setup_data = -1;
          setup_mode_rw = SETUP_READ; 
        }
        else
        {
          axel_auger_delay_temp = axel_auger_delay;
          axel_threshing_delay_temp = axel_threshing_delay;
          axel_yeache_ku_time_temp = axel_yeache_ku_time;
          axel_auger_auto_delay_temp = axel_auger_auto_delay;
          axel_auger_ku_time_temp = axel_auger_ku_time;
          axel_tbs_ku_time_temp = axel_tbs_ku_time;
          axel_c_speed_ku_time_temp = axel_c_speed_ku_time;
          axel_threshing_rpm_temp = axel_threshing_rpm;
        }
      }
      else if(row_selection == 6){          currentPageState = SETTINGS_26x_PAGE; }                 // Added on 2020 Sep 10
      row_selection = 1;
    }
    else if(currentPageState == SETTINGS_21x_PAGE)
    {
      previousPageState = currentPageState;
      new_setup_data = -1;                                                                    // Added on 22 March 2021
      setup_mode_rw = SETUP_READ;                                                             // Added on 30 March 2021
           if(row_selection == 1){          currentPageState = SETTINGS_211_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_212_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_213_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_214_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_215_PAGE; }
      else if(row_selection == 6){          currentPageState = SETTINGS_216_PAGE; }
      else if(row_selection == 7){          currentPageState = SETTINGS_217_PAGE; }
      else if(row_selection == 8){          currentPageState = SETTINGS_218_PAGE; }
      row_selection = 1;
    }
    
    else if(currentPageState == SETTINGS_22x_PAGE)
    {
      previousPageState = currentPageState;
      new_setup_data = -1;                                                                    // Added on 22 March 2021
      setup_mode_rw = SETUP_READ;                                                             // Added on 30 March 2021
           if(row_selection == 1){          currentPageState = SETTINGS_221_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_222_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_223_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_224_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_225_PAGE; }
      else if(row_selection == 6){          currentPageState = SETTINGS_226_PAGE; }
      else if(row_selection == 7){          currentPageState = SETTINGS_227_PAGE; }
      row_selection = 1;
    }
    
    else if(currentPageState == SETTINGS_23x_PAGE)
    {
      previousPageState = currentPageState;
      new_setup_data = -1;                                                                    // Added on 22 March 2021
      setup_mode_rw = SETUP_READ;                                                             // Added on 30 March 2021
           if(row_selection == 1){          currentPageState = SETTINGS_231_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_232_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_233_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_234_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_235_PAGE; }
      else if(row_selection == 6){          currentPageState = SETTINGS_236_PAGE; }
      else if(row_selection == 7){          currentPageState = SETTINGS_237_PAGE; }
      else if(row_selection == 8){          currentPageState = SETTINGS_238_PAGE; }
      row_selection = 1;
    }
    else if(currentPageState == SETTINGS_24x_PAGE) 
    {
      currentPageState = SETTINGS_241_PAGE; 
      setup_mode_rw = SETUP_READ; 
      new_setup_data_received_flag = 0;
    }
    else if((currentPageState >= SETTINGS_251_PAGE) && (currentPageState <= SETTINGS_2511_PAGE))
    {
      if(axelControlFunctionEnabled == FALSE)
      {
        setup_mode_rw = SETUP_WRITE;
        new_setup_data_received_flag = 0;
        if((currentPageState == SETTINGS_251_PAGE) || (currentPageState == SETTINGS_252_PAGE))
        {
          new_setup_data = axel_app_sensor1_position;
          new_setup_data_2 = axel_app_sensor2_position;
        }       
        else
        {
          new_setup_data_2 = 0;
          if((currentPageState >= SETTINGS_253_PAGE) && (currentPageState <= SETTINGS_2510_PAGE))
          {
            new_setup_data = new_setup_data_received_to_update;
          }
          else if(currentPageState == SETTINGS_2511_PAGE)
          {
            new_setup_data = 0;
          }
        }
      }
      else
      {
        if(currentPageState == SETTINGS_251_PAGE)
        {
          if(axel_app_sensor1_position > 512)
          {
            axel_app_sensor1_position_min = axel_app_sensor1_position - 5;
          }
          else
          {
            axel_app_sensor1_position_min = axel_app_sensor1_position + 5;
          }
          
          if(axel_app_sensor2_position > 512)
          {
            axel_app_sensor2_position_min = axel_app_sensor2_position - 5;
          }
          else
          {
            axel_app_sensor2_position_min = axel_app_sensor2_position + 5;
          }
          check_setting_data(TRUE, CONFIGURE_AXEL_APP_SENSOR1_POSITION_MIN, axel_app_sensor1_position_min);
          check_setting_data(TRUE, CONFIGURE_AXEL_APP_SENSOR2_POSITION_MIN, axel_app_sensor2_position_min);
        }
        else if(currentPageState == SETTINGS_252_PAGE)
        {
          if(axel_app_sensor1_position > 512)
          {
            axel_app_sensor1_position_max = axel_app_sensor1_position - 5;
          }
          else
          {
            axel_app_sensor1_position_max = axel_app_sensor1_position + 5;
          }
          
          if(axel_app_sensor2_position > 512)
          {
            axel_app_sensor2_position_max = axel_app_sensor2_position - 5;
          }
          else
          {
            axel_app_sensor2_position_max = axel_app_sensor2_position + 5;
          }
          check_setting_data(TRUE, CONFIGURE_AXEL_APP_SENSOR1_POSITION_MAX, axel_app_sensor1_position_max);
          check_setting_data(TRUE, CONFIGURE_AXEL_APP_SENSOR2_POSITION_MAX, axel_app_sensor2_position_max);
        }
        else if(currentPageState == SETTINGS_253_PAGE)
        {
          axel_threshing_delay = axel_threshing_delay_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_THRESHING_DELAY, axel_threshing_delay);
        }
        else if(currentPageState == SETTINGS_254_PAGE)
        {
          axel_auger_delay = axel_auger_delay_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_AUGER_DELAY, axel_auger_delay);
        }
        else if(currentPageState == SETTINGS_255_PAGE)
        {
          axel_auger_auto_delay = axel_auger_auto_delay_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_AUGER_AUTO_DELAY, axel_auger_auto_delay);
        }
        else if(currentPageState == SETTINGS_256_PAGE)
        {
          axel_yeache_ku_time = axel_yeache_ku_time_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_YEACHE_KU_TIME, axel_yeache_ku_time);
        }
        else if(currentPageState == SETTINGS_257_PAGE)
        {
          axel_tbs_ku_time = axel_tbs_ku_time_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_TBS_KU_TIME, axel_tbs_ku_time);
        }
        else if(currentPageState == SETTINGS_258_PAGE)
        {
          axel_auger_ku_time = axel_auger_ku_time_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_AUGER_KU_TIME, axel_auger_ku_time);
        }
        else if(currentPageState == SETTINGS_259_PAGE)
        {
          axel_c_speed_ku_time = axel_c_speed_ku_time_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_C_SPEED_KU_TIME, axel_c_speed_ku_time);
        }
        else if(currentPageState == SETTINGS_2510_PAGE)
        {
          axel_threshing_rpm = axel_threshing_rpm_temp;
          check_setting_data(TRUE, CONFIGURE_AXEL_THRESHING_RPM, axel_threshing_rpm);
        }
      }
    }
    else if(currentPageState == SETTINGS_26x_PAGE)
    {
      previousPageState = currentPageState;
      new_setup_data = -1;                                                                    // Added on 22 March 2021
      setup_mode_rw = SETUP_READ;                                                             // Added on 30 March 2021
      new_setup_data_received_flag = 0;                                                       // Added on 2021.06.29
           if(row_selection == 1){          currentPageState = SETTINGS_261_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_262_PAGE; }
      row_selection = 1;
    }
    
    else if(
               // Page 21x
               (currentPageState == SETTINGS_211_PAGE) || (currentPageState == SETTINGS_212_PAGE) || (currentPageState == SETTINGS_213_PAGE) || 
               (currentPageState == SETTINGS_214_PAGE) || (currentPageState == SETTINGS_215_PAGE) || (currentPageState == SETTINGS_216_PAGE) || 
               (currentPageState == SETTINGS_217_PAGE) || (currentPageState == SETTINGS_218_PAGE) || 
               // Page 22x
               (currentPageState == SETTINGS_221_PAGE) || (currentPageState == SETTINGS_222_PAGE) || (currentPageState == SETTINGS_223_PAGE) || 
               (currentPageState == SETTINGS_224_PAGE) || (currentPageState == SETTINGS_225_PAGE) || 
               (currentPageState == SETTINGS_226_PAGE) || (currentPageState == SETTINGS_227_PAGE) || 
               // Page 23x
               (currentPageState == SETTINGS_231_PAGE) || (currentPageState == SETTINGS_232_PAGE) || (currentPageState == SETTINGS_233_PAGE) || 
               (currentPageState == SETTINGS_234_PAGE) || (currentPageState == SETTINGS_235_PAGE) || (currentPageState == SETTINGS_236_PAGE) ||
               (currentPageState == SETTINGS_237_PAGE) || (currentPageState == SETTINGS_238_PAGE)  ||
               // Page 24x
               (currentPageState == SETTINGS_241_PAGE) || 
               // Page 26x
               (currentPageState == SETTINGS_261_PAGE) || (currentPageState == SETTINGS_262_PAGE)
              )
    {
      if(setup_mode == SETUP_MODE_NONE)                      // the configuration is done so it should be transmitted now
      {
        previousPageState = currentPageState;
        currentPageState = SETTINGS_2xxF_PAGE;
        row_selection = 1;
        new_setup_data = new_setup_data_received;             // Added on 24 March 2021        
      }
    }
    
    else if(currentPageState == SETTINGS_30_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_31x_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_32x_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_33x_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_34x_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_35x_PAGE; }
      else if(row_selection == 6){          currentPageState = SETTINGS_36x_PAGE; }
      row_selection = 1;
      index = 0;
    }
    else if(currentPageState == SETTINGS_31x_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_311_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_312_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_313_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_314_PAGE; }
      else if(row_selection == 5){          currentPageState = SETTINGS_315_PAGE; }
      row_selection = 1;
      index = 0;
    }
    else if(currentPageState == SETTINGS_311_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_3111_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_3112_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_3113_PAGE; }
      row_selection = 1;
      index = 0;
    }
    else if(currentPageState == SETTINGS_314_PAGE)
    {
      previousPageState = currentPageState;
           if(row_selection == 1){          currentPageState = SETTINGS_3141_PAGE; }
      else if(row_selection == 2){          currentPageState = SETTINGS_3142_PAGE; }
      else if(row_selection == 3){          currentPageState = SETTINGS_3143_PAGE; }
      row_selection = 1;
      index = 0;
    }
    else if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) || (currentPageState == SETTINGS_3113_PAGE) ||
          (currentPageState == SETTINGS_312_PAGE) || (currentPageState == SETTINGS_313_PAGE) || (currentPageState == SETTINGS_3141_PAGE) ||
          (currentPageState == SETTINGS_3142_PAGE) || (currentPageState == SETTINGS_3143_PAGE) || (currentPageState == SETTINGS_315_PAGE))
    {
      if((currentPageState == SETTINGS_3111_PAGE) || (currentPageState == SETTINGS_3112_PAGE) ||
          (currentPageState == SETTINGS_3141_PAGE) || (currentPageState == SETTINGS_315_PAGE))
      {
        index += 12;
             if((currentPageState == SETTINGS_3111_PAGE) && (index > 44)) { index = 44;  }                  // 56
        else if((currentPageState == SETTINGS_3112_PAGE) && (index > 7))  { index = 7;   }                  // 19
        else if((currentPageState == SETTINGS_3141_PAGE) && (index > 23)) { index = 23;  }                  // 35
        else if((currentPageState == SETTINGS_315_PAGE)  && (index > 129)){ index = 129; }                  // 141
      }
    }
    else if((currentPageState == SETTINGS_331_PAGE) || (currentPageState == SETTINGS_332_PAGE))
    {
      // this should be changed based on the algorithm
      if(currentPageState == SETTINGS_331_PAGE)
      {
        numberOfEngineOilExchange++;
        check_setting_data(TRUE, CONFIGURE_ENGINE_OIL_EXCHANGE_COUNTER, numberOfEngineOilExchange);
        engineOilHour = 0;
        check_setting_data(TRUE, CONFIGURE_ENGINE_OIL_HOUR_MSB, (uint16_t)((uint32_t)engineOilHour >> 16));               // clear engine oil time
        check_setting_data(TRUE, CONFIGURE_ENGINE_OIL_HOUR_LSB, (uint16_t)((uint32_t)engineOilHour));                     // clear engine oil time
      }
      else if(currentPageState == SETTINGS_332_PAGE)
      {
        numberOfMissionOilExchange++;
        check_setting_data(TRUE, CONFIGURE_MISSION_OIL_EXCHANGE_COUNTER, numberOfMissionOilExchange);
        missionOilHour = 0;
        check_setting_data(TRUE, CONFIGURE_MISSION_OIL_HOUR_MSB, (uint16_t)((uint32_t)missionOilHour >> 16));               // clear engine oil time
        check_setting_data(TRUE, CONFIGURE_MISSION_OIL_HOUR_LSB, (uint16_t)((uint32_t)missionOilHour));                     // clear engine oil time
      }
      currentPageState = SETTINGS_33x_PAGE;
      previousPageState = SETTINGS_30_PAGE;
      row_selection = 1;
    }
    else if(currentPageState == SETTINGS_50_PAGE)
    {
      currentPageState = SETTINGS_511_PAGE;
      settings_pass[0] = 0;   settings_pass[1] = 0;   settings_pass[2] = 0;   settings_pass[3] = 0;   settings_pass[4] = 0;   
      settings_pass[5] = 0;   settings_pass[6] = 0;   settings_pass[7] = 0;   settings_pass[8] = 0;   settings_pass[9] = 0;
      number_index = 0;
    }
    else if(currentPageState == SETTINGS_511_PAGE)
    {
      if((settings_pass[0] == PASSWORD[0]) && (settings_pass[1] == PASSWORD[1]) && (settings_pass[2] == PASSWORD[2]) &&
         (settings_pass[3] == PASSWORD[3]) && (settings_pass[4] == PASSWORD[4]) && (settings_pass[5] == PASSWORD[5]) &&
         (settings_pass[6] == PASSWORD[6]) && (settings_pass[7] == PASSWORD[7]) && (settings_pass[8] == PASSWORD[8]) && (settings_pass[9] == PASSWORD[9]))
      {
        currentPageState = SETTINGS_512_PAGE;
        row_selection = 1;
      }
    }
    else if(currentPageState == SETTINGS_512_PAGE)
    {
      if(row_selection == 1)
      {
        number_index = 0;
        currentPageState = SETTINGS_513_PAGE;

        settings_time[0] = (((uint32_t)engineHour % 100000) / 10000);
        settings_time[1] = (((uint32_t)engineHour % 10000) / 1000);
        settings_time[2] = (((uint32_t)engineHour % 1000) / 100);
        settings_time[3] = (((uint32_t)engineHour % 100) / 10);
        settings_time[4] = (((uint32_t)engineHour % 10) / 1);
      }
      else if(row_selection == 2)
      {
        number_index = 0;
        currentPageState = SETTINGS_514_PAGE;

        settings_time[0] = (((uint32_t)jobHour % 100000) / 10000);
        settings_time[1] = (((uint32_t)jobHour % 10000) / 1000);
        settings_time[2] = (((uint32_t)jobHour % 1000) / 100);
        settings_time[3] = (((uint32_t)jobHour % 100) / 10);
        settings_time[4] = (((uint32_t)jobHour % 10) / 1);
      }       
      else if(row_selection == 3)
      {
        currentPageState = SETTINGS_515_PAGE;
      }
      else if(row_selection == 4)
      {
        currentPageState = SETTINGS_516_PAGE;
      }
      else if(row_selection == 5)
      {
        currentPageState = SETTINGS_517_PAGE;
      }
    }
    else if(currentPageState == SETTINGS_513_PAGE)
    {
      currentPageState = SETTINGS_5131_PAGE;
    }
    else if(currentPageState == SETTINGS_514_PAGE)
    {
      currentPageState = SETTINGS_5141_PAGE;
    }
    else if((currentPageState == SETTINGS_5131_PAGE) || (currentPageState == SETTINGS_5141_PAGE)) 
    {
      if(currentPageState == SETTINGS_5131_PAGE) 
      {
        engineHour = (float)((settings_time[0] * 10000) + (settings_time[1] * 1000) + (settings_time[2] * 100) + (settings_time[3] * 10) + (settings_time[4]));
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_MSB, (uint16_t)((uint32_t)(engineHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_LSB, (uint16_t)((uint32_t)(engineHour * 100)));
      }
      else if(currentPageState == SETTINGS_5141_PAGE) 
      {
        jobHour = (float)((settings_time[0] * 10000) + (settings_time[1] * 1000) + (settings_time[2] * 100) + (settings_time[3] * 10) + (settings_time[4]));
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_MSB, (uint16_t)((uint32_t)(jobHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_LSB, (uint16_t)((uint32_t)(jobHour * 100)));
      }
       previousPageState = currentPageState;
           if(row_selection == 3){          currentPageState = SETTINGS_5131_PAGE; }
      else if(row_selection == 4){          currentPageState = SETTINGS_5141_PAGE; }
     // row_selection = 1;
      index = 0;
      row_selection = 2;
      currentPageState = SETTINGS_512_PAGE;
      
    }
    else if((currentPageState == SETTINGS_515_PAGE) || (currentPageState == SETTINGS_516_PAGE) || (currentPageState == SETTINGS_517_PAGE))
    {
      if(currentPageState == SETTINGS_515_PAGE)
      {
        engineHour = 0;
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_MSB, (uint16_t)((uint32_t)(engineHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_LSB, (uint16_t)((uint32_t)(engineHour * 100)));
      }
      else if(currentPageState == SETTINGS_516_PAGE)
      {
        jobHour = 0;
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_MSB, (uint16_t)((uint32_t)(jobHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_LSB, (uint16_t)((uint32_t)(jobHour * 100)));
      }
      else if(currentPageState == SETTINGS_517_PAGE)
      {
        engineHour = 0;
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_MSB, (uint16_t)((uint32_t)(engineHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_LSB, (uint16_t)((uint32_t)(engineHour * 100)));
        jobHour = 0;
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_MSB, (uint16_t)((uint32_t)(jobHour * 100) >> 16));
        check_setting_data(TRUE, CONFIGURE_JOB_HOUR_LSB, (uint16_t)((uint32_t)(jobHour * 100)));
      }
      previousPageState = currentPageState;
      //row_selection = 1;
      if(row_selection == 3)          { currentPageState = SETTINGS_515_PAGE; }
      else if(row_selection == 4)     { currentPageState = SETTINGS_516_PAGE; }
      else if(row_selection == 5)     { currentPageState = SETTINGS_517_PAGE; }
      currentPageState = SETTINGS_512_PAGE;
    }
    else if(currentPageState == SETTINGS_60_PAGE)
    {
      if(flag.isBrigthnessSetting == TRUE)
      {
        flag.isBrigthnessSetting = FALSE; 
        if(row_selection == 1)
        {
          check_setting_data(TRUE, CONFIGURE_LCD_BRIGHTNESS_DAY, lcdBrigthnessDay);
        }
        else if(row_selection == 2)
        {
          check_setting_data(TRUE, CONFIGURE_LCD_BRIGHTNESS_NIGHT, lcdBrigthnessNight);
        }
        else
        {
          check_setting_data(TRUE, CONFIGURE_MODEL_SELECTION, modelSelection);

          if((modelSelection == MODEL_NEW_WITH_AXEL) || (modelSelection == MODEL_NEXT))
          {
            axelControlFunctionEnabled = TRUE;
          }
          else
          {
            axelControlFunctionEnabled = FALSE;
          }
        }
      }
      else
      {
        flag.isBrigthnessSetting = TRUE;
      }
    }
  }
}

void lcd_process()
{
  uint16_t imgCnt = 0;
  
  if(flagTimer.hundredMs == TRUE)
  {
    YVC1_VinBcdDisp(0, 0);
    if(Vsync_Filp_Check() == TRUE)
    {
      if(currentPageState != LOGO_PAGE)
      {        
        if((flagWarning.detected == TRUE) && (flagWarning.index < TOTAL_NUMBER_OF_WARNINGS))
        {
#if defined(USER_DEBUG)
          printf("Warning detected = %d\r\n", flagWarning.index);
#endif
          imgCnt = DrawWarningCombineCheck(imgCnt);
          if(currentPageState != WARNING_PAGE)
          {
            previousPageState = currentPageState;
           currentPageState = WARNING_PAGE;    // tseveen 20250522
          }
        }
        
        if(imgCnt == 0)
        {
          if(currentPageState == WARNING_PAGE)
          {
            update_pages_back();
          }
          imgCnt = display_function(imgCnt);
        }
//        imgCnt = draw_button(imgCnt, flagWarning.index); tseveen 20250422
      }
      else
      {
        imgCnt = draw_logo_page(imgCnt);
      }
      YVC1_SetHostContLyrWithFlip(YVC_LYR_ADDR_CPU,imgCnt);
    }
  }
}

/*----------------------------------------------- Memory -----------------------------------------------*/
#define NUMBER_OF_BUFFER_FUEL           100
uint8_t tFuelPercentBuffer[NUMBER_OF_BUFFER_FUEL];
float tFuelPercentTotal;
uint8_t tFuelPercentBufferCounter = 0;

#define NUMBER_OF_BUFFER_ACCEL          100
float tAccelBuffer[NUMBER_OF_BUFFER_ACCEL];
uint8_t tAccelBufferCounter = 0;
float tAccelTotal;
/*----------------------------------------------- Algorithm -----------------------------------------------*/
void analog_sensors()
{
  static uint8_t firstTime = TRUE;
  static float ADC_FUEL;
  static float ADC_BATTERY;
  static float ADC_ACCELERATOR;
  //static float ADC_TEMP;
  static float ADC_CPU_TEMP;

  uint8_t i;
  
  updateLastAverageADC();
  
  ADC_FUEL              = getAverageADCValue(0);
  ADC_BATTERY           = getAverageADCValue(1);
  //ADC_TEMP      = getAverageADCValue(2);
  ADC_ACCELERATOR       = getAverageADCValue(3);
  ADC_CPU_TEMP          = getAverageADCValue(4);
    
  tAccelBufferCounter++;
  if(tAccelBufferCounter >= NUMBER_OF_BUFFER_ACCEL)
  {
    tAccelBufferCounter = 0;
  }
  tAccelBuffer[tAccelBufferCounter] = ADC_ACCELERATOR;
  
  tAccelTotal = 0;
  for(i = 0; i < NUMBER_OF_BUFFER_ACCEL; i++)
  {
    tAccelTotal += tAccelBuffer[i];
  }
  tAccelTotal /= NUMBER_OF_BUFFER_ACCEL;
    
  if(modelSelection == MODEL_NEXT)
  {
    if(flagTimer.hundredMs == TRUE)                                                  // every 100ms ADC data should be updated
    {
      if(ADC_FUEL < FUEL_ADC_PERCENTAGES_NEXT[NUMBER_OF_STEP_IN_FUEL - 1].raw_data)
      {
        ADC_FUEL = FUEL_ADC_PERCENTAGES_NEXT[NUMBER_OF_STEP_IN_FUEL - 1].raw_data;
      }
      else if(ADC_FUEL > FUEL_ADC_PERCENTAGES_NEXT[0].raw_data)
      {
        ADC_FUEL = FUEL_ADC_PERCENTAGES_NEXT[0].raw_data;
      }
        
      tFuelPercentBufferCounter++;
      if(tFuelPercentBufferCounter >= NUMBER_OF_BUFFER_FUEL)
        tFuelPercentBufferCounter = 0;
      
      for(i = 0; i < NUMBER_OF_STEP_IN_FUEL; i++)
      {
        if(ADC_FUEL >= FUEL_ADC_PERCENTAGES_NEXT[i].raw_data)
        {
          if(i == 0)
          {
            tFuelPercentBuffer[tFuelPercentBufferCounter] = 0;                       // 0 percentage
          }
          else 
          {
            tFuelPercentBuffer[tFuelPercentBufferCounter] = (uint8_t)(((float)FUEL_ADC_PERCENTAGES_NEXT[i - 1].raw_data - (float)ADC_FUEL) * 
                                     (FUEL_ADC_PERCENTAGES_NEXT[i].percentage - FUEL_ADC_PERCENTAGES_NEXT[i - 1].percentage) / 
                                     ((float)FUEL_ADC_PERCENTAGES_NEXT[i - 1].raw_data - (float)FUEL_ADC_PERCENTAGES_NEXT[i].raw_data));
            tFuelPercentBuffer[tFuelPercentBufferCounter] += FUEL_ADC_PERCENTAGES_NEXT[i - 1].percentage;
            break;
          }
        }
      }
      if(i == NUMBER_OF_STEP_IN_FUEL)
      {
        tFuelPercentBuffer[tFuelPercentBufferCounter] = 100;                         // 100 percentage
      }
      
      tFuelPercentTotal = 0;
      for(i = 0; i < NUMBER_OF_STEP_IN_FUEL; i++)
      {
        tFuelPercentTotal += tFuelPercentBuffer[i];
      }
        
      if(firstTime == TRUE)
      {
        tFuelPercent = tFuelPercentBuffer[tFuelPercentBufferCounter];
        firstTime = FALSE;
      }
      else
      {
        tFuelPercent = (uint8_t)(tFuelPercentTotal / (float)NUMBER_OF_STEP_IN_FUEL);  
      }
      
      if(tFuelPercent > 100)
        tFuelPercent = 100;
      
      tPowerVoltage = (ADC_BATTERY / 4095.0) * 3.3 * 5.72;
      
      tFuelVoltage = (ADC_FUEL / 4095.0) * 3.3;
      tTemperature = ((((ADC_CPU_TEMP / 4095.0) * 3.3) - 0.76) / 0.0025) + 19.0 + 0.5;
    }
  }
  else
  {
    if(flagTimer.hundredMs == TRUE)                                                  // every 100ms ADC data should be updated
    {
      if(ADC_FUEL < FUEL_ADC_PERCENTAGES[NUMBER_OF_STEP_IN_FUEL - 1].raw_data)
      {
        ADC_FUEL = FUEL_ADC_PERCENTAGES[NUMBER_OF_STEP_IN_FUEL - 1].raw_data;
      }
      else if(ADC_FUEL > FUEL_ADC_PERCENTAGES[0].raw_data)
      {
        ADC_FUEL = FUEL_ADC_PERCENTAGES[0].raw_data;
      }
        
      tFuelPercentBufferCounter++;
      if(tFuelPercentBufferCounter >= NUMBER_OF_BUFFER_FUEL)
        tFuelPercentBufferCounter = 0;
      
      for(i = 0; i < NUMBER_OF_STEP_IN_FUEL; i++)
      {
        if(ADC_FUEL >= FUEL_ADC_PERCENTAGES[i].raw_data)
        {
          if(i == 0)
          {
            tFuelPercentBuffer[tFuelPercentBufferCounter] = 0;                       // 0 percentage
          }
          else 
          {
            tFuelPercentBuffer[tFuelPercentBufferCounter] = (uint8_t)(((float)FUEL_ADC_PERCENTAGES[i - 1].raw_data - (float)ADC_FUEL) * 
                                     (FUEL_ADC_PERCENTAGES[i].percentage - FUEL_ADC_PERCENTAGES[i - 1].percentage) / 
                                     ((float)FUEL_ADC_PERCENTAGES[i - 1].raw_data - (float)FUEL_ADC_PERCENTAGES[i].raw_data));
            tFuelPercentBuffer[tFuelPercentBufferCounter] += FUEL_ADC_PERCENTAGES[i - 1].percentage;
            break;
          }
        }
      }
      if(i == NUMBER_OF_STEP_IN_FUEL)
      {
        tFuelPercentBuffer[tFuelPercentBufferCounter] = 100;                         // 100 percentage
      }
      
      tFuelPercentTotal = 0;
      for(i = 0; i < NUMBER_OF_STEP_IN_FUEL; i++)
      {
        tFuelPercentTotal += tFuelPercentBuffer[i];
      }
        
      if(firstTime == TRUE)
      {
        tFuelPercent = tFuelPercentBuffer[tFuelPercentBufferCounter];
        firstTime = FALSE;
      }
      else
      {
        tFuelPercent = (uint8_t)(tFuelPercentTotal / (float)NUMBER_OF_STEP_IN_FUEL);  
      }
      
      if(tFuelPercent > 100)
        tFuelPercent = 100;
      
      tPowerVoltage = (ADC_BATTERY / 4095.0) * 3.3 * 5.72;
      
      tFuelVoltage = (ADC_FUEL / 4095.0) * 3.3;
      tTemperature = ((((ADC_CPU_TEMP / 4095.0) * 3.3) - 0.76) / 0.0025) + 19.0 + 0.5;
    }
  }
  
  
  // Max 4095, Min 0 --> it should be changed to 0 ~ 100%
  // 4.7K and 1K, power 5V ==> 0.877V
  // 3.3V       - 4095
  // 0.877V     - 1088
  // 
  /* Enkhbat disabled
  acceleratorPedalPosition = (uint8_t) (((float)ADC_ACCELERATOR / (float)1088) * (float)250);
  if(acceleratorPedalPosition > 250)
  {
    acceleratorPedalPosition = 250;
  }
  */
  
}

void digital_sensors()
{
  read_inputs();
//  check_buttons(); tseveen 20250421
}

void sensor_process()
{
  static uint16_t timerEngineOn = 0;
  static uint16_t timerEngineOff= 0;
  static uint8_t prevTaillamp = 255;
  
  if(flagTimer.hundredMs == TRUE)
  {
    if((flagInput.tailLamp == TRUE) && (prevTaillamp == FALSE))
    {
      update_timer(lcdBrigthnessNight);
    }
    else if((flagInput.tailLamp == FALSE) && (prevTaillamp == TRUE))
    {
      update_timer(lcdBrigthnessDay);
    }
    prevTaillamp = flagInput.tailLamp;
  }

  tEngineSpeed = (float) engine_speed * 0.125f;
  if(tEngineSpeed >= 500.0)
  {
    timerEngineOn += 2;
    if(timerEngineOn >= 1000)
    {
      timerEngineOff = 0;
      timerEngineOn = 1000;
      flag.engineStarted = TRUE;
    }
  }
  else
  {
    timerEngineOff += 2;
    if(timerEngineOff >= 1000)
    {
      timerEngineOn = 0;
      timerEngineOff = 1000;
      flag.engineStarted = FALSE;
    }
  }

  flagOutput.lamp_1 = flagInput.grain_1;
  flagOutput.lamp_2 = flagInput.grain_2;
  flagOutput.lamp_3 = flagInput.grain_3;
  flagOutput.lamp_4 = flagInput.grain_4;
}

void update_hours()
{
  if(flag.engineStarted == TRUE)
  {
    localEngineHourTimer++;
    if(dataCAN310.IC_threshing_sw == 1)
    {
      localJobHourTimer++;
    }
  }
}

void time_process()
{  
  if(localEngineHourTimer >= 180000)                                     // 3 minute
  {
    engineHour = engineHour + 0.05;
    check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_MSB, (uint16_t)((uint32_t)(engineHour * 100) >> 16));
    check_setting_data(TRUE, CONFIGURE_ENGINE_HOUR_LSB, (uint16_t)((uint32_t)(engineHour * 100)));
          
    engineOilHour = engineOilHour + 0.05;
    check_setting_data(TRUE, CONFIGURE_ENGINE_OIL_HOUR_MSB, (uint16_t)((uint32_t)(engineOilHour * 100) >> 16));
    check_setting_data(TRUE, CONFIGURE_ENGINE_OIL_HOUR_LSB, (uint16_t)((uint32_t)(engineOilHour * 100)));
    
    missionOilHour = missionOilHour + 0.05;
    check_setting_data(TRUE, CONFIGURE_MISSION_OIL_HOUR_MSB, (uint16_t)((uint32_t)(missionOilHour * 100) >> 16));
    check_setting_data(TRUE, CONFIGURE_MISSION_OIL_HOUR_LSB, (uint16_t)((uint32_t)(missionOilHour * 100)));
    
    localEngineHourTimer = 0;
  }
  
  if(localJobHourTimer >= 180000)
  {
    jobHour = jobHour + 0.05;
    check_setting_data(TRUE, CONFIGURE_JOB_HOUR_MSB, (uint16_t)((uint32_t)(jobHour * 100) >> 16));
    check_setting_data(TRUE, CONFIGURE_JOB_HOUR_LSB, (uint16_t)((uint32_t)(jobHour * 100)));

    localJobHourTimer = 0;
  }
}

uint8_t warning_flag_checker()
{
  uint8_t wIndex;
  uint8_t checkRPM = FALSE;
  uint8_t checkTalkuk = FALSE;
    
  //////////////////////////////////////////////////////////////////////////////
  uint8_t warning_flags_array[TOTAL_NUMBER_OF_WARNINGS] = {
    flagWarning.fuel,                                                           //1  W_FUEL_EMPTY
    flagWarning.engineOilExchange,                                              //2  W_ENGINE_OIL_EXCHANGE
    flagWarning.missionOilExchange,                                             //3  W_MISSION_OIL_EXCHANGE
    flagWarning.MC_HST_chuhen_motor_error,                                      //4  W_CHUHEN_MOTOR
    dataCAN310.IC_engine_starter_sw,                                            //5  W_STARTER_SAFETY
    flagWarning.can_timeout_warning_31x,                                        //6  W_CAN_TIMEOUT_INTEGRATED_CONTROLLER
    flagWarning.can_timeout_warning_33x,                                        //7  W_CAN_TIMEOUT_CUTTING_CONTROLLER
    flagWarning.can_timeout_warning_340,                                        //8  W_CAN_TIMEOUT_AUGER_CONTROLLER
    flagWarning.can_timeout_warning_350,                                        //9  W_CAN_TIMEOUT_TALKUK_HEIGHT_CONTROLLER
    flagWarning.can_timeout_warning_035,                                        //10 W_CAN_TIMEOUT_GENERAL_SWITCH
    flagWarning.can_timeout_warning_025,                                        //11 W_CAN_TIMEOUT_AUGER_SWITCH
    flagWarning.can_timeout_warning_390,                                        //12 W_CAN_TIMEOUT_HST_CONTROLLER
    flagWarning.can_timeout_warning_360,                                        //13 W_CAN_TIMEOUT_AXEL_CONTROLLER
    flagWarning.can_timeout_warning_381,                                        //14 W_CAN_TIMEOUT_SONBYOL_CONTROLLER
    flagWarning.MC_HST_chuhyan_motor_error,                                     //15 W_CHUHYAN_MOTOR
    dataCAN340.AG_auger_limiting_current_error,                                 //16 W_AUGER_MOTOR
    dataCAN350.LSA_restraint_current_error,                                     //17 W_TALKUK_HEIGHT
    flagWarning.MC_HST_chuhen_lever_error,                                      //18 W_CHUHEN_LEVER
    flagWarning.MC_HST_chuhyan_lever_error,                                     //19 W_CHUHYAN_LEVER
    flagWarning.battery,                                                        //20 W_BATTERY
    dataCAN311.IC_engineStop_chuhenController,                                  //21 W_CHUHEN_CONTROLLER
      
    dataCAN340.AG_auger_rotation_sensor_error,                                  //22 W_AUGER_ROTATION_SENSOR
    dataCAN350.LSA_lsa_motor_position_sensor_error,                             //23 W_LSA_MOTOR_POSITION_SENSOR
    dataCAN311.IC_tbs_tilt_sensor_error,                                        //24 W_TBS_SLOPE_SENSOR
    dataCAN311.IC_tbs_grage_right_sensor_error,                                 //25 W_TBS_RIGHT_SENSOR
    dataCAN311.IC_tbs_grage_left_sensor_error,                                  //26 W_TBS_LEFT_SENSOR
    dataCAN311.IC_tbs_manual_sw_error,                                          //27 W_TBS_MANUAL_SWITCH
    dataCAN311.IC_auger_manual_sw_error,                                        //28 W_AUGER_MANAUL_SWITCH
    dataCAN311.IC_auger_setting_return_sw_error,                                //29 W_AUGER_SETTING_SWITCH
    dataCAN311.IC_lsa_m_h_sensor_error,                                         //30 W_LSA_MH_SENSOR
    dataCAN311.IC_lsa_manual_sw_error,                                          //31 W_LSA_MANUAL_SWITCH
    dataCAN330.CC_lift_sensor_error,                                            //32 W_YEACHE_LIFT_SENSOR
    dataCAN330.CC_notice_sensor_error,                                          //33 W_PREVIEW_SENSOR
    dataCAN330.CC_manual_sw_error,                                              //34 W_YEACHE_MANAUL_SWITCH
    dataCAN330.CC_threshing_clutch_motor_sw_error,                              //35 W_TALKUK_CLUTCH_CONNECTION_BLOCK
    dataCAN330.CC_cutting_clutch_motor_sw_error,                                //36 W_TALKUK_CLUTCH_CONNECTION_BLOCK_LSA_MOTOR
    dataCAN330.CC_one_touch_sw_error,                                           //37 W_ONETOUCH_UP_DOWN
    
    dataCAN311.IC_engineStop_emergencyStop,                                     //38 W_ENGINE_STOP_SWITCH
    flagWarning.charge,                                                         //39 W_CHARGE
    flagWarning.oilPressure,                                                    //40 W_ENGINE_OIL_PRESSURE
    flagInput.waterTemperature,                                                 //41 W_ENGINE_COOLING_TEMPERATURE   
    flagWarning.airFilter,                                                      //42 W_AIR_FILTER
    flagWarning.talkukBin,                                                      //43 W_TALKUK_BIN
    dataCAN311.IC_pick3_processing_sensor_alarm,                                //44 W_CHORI_BIN
    flagWarning.sensor2,                                                        //45 W_2_SENSOR
    flagWarning.chipbechulBlockage,                                             //46 W_CHIPBECHUL
    flagInput.grain_4,                                                          //47 W_GUGMUL_MANYANG
    flagWarning.waterSeparator,                                                 //48 W_MULBUNRIGI
    dataCAN311.IC_engineStop_gugmulManyang,                                     //49 W_GUGMUL_MANYANG_ENGINE_STOP
    dataCAN311.IC_engineStop_cutSafety,                                         //50 W_ENGINE_STOP_CUTTING_SAFETY
    dataCAN311.IC_engineStop_yeacheBlockage,                                    //51 W_ENGINE_STOP_YEACHE_SAFETY
    flagWarning.nason2,                                                         //52 W_2_NASON
    flagWarning.nasonYangug,                                                    //53 W_YANGUG_NASON
    flagWarning.nason2BlockSwitch                                               //54 W_2_NASON_BLOCK_SWITCH
  };

  for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
  {
    if(warning_flags_array[wIndex] != 0)
    {
      warnings[wIndex].timerWarning += 100;
      if(warnings[wIndex].timerWarning >= 1000)
      {
        warnings[wIndex].timerWarning = 1000;
        warnings[wIndex].flag = W_FLAG_ACTIVE;
      }
    }
    else
    {
      warnings[wIndex].timerWarning = 0;
      warnings[wIndex].flag = W_FLAG_PASSIVE;
    }
  }
  
  // for loop
  for(wIndex = 0; wIndex < TOTAL_NUMBER_OF_WARNINGS; wIndex++)
  {
    if(warnings[wIndex].checkRPM == TRUE)
    {
      if(flag.engineStarted == TRUE)
      {
        checkRPM = TRUE;
      }
      else
      {
        checkRPM = FALSE;
      }
    }
    else
    {
      checkRPM = TRUE;
    }
    
    if(warnings[wIndex].checkTalkuk == TRUE)
    {
      if(dataCAN310.IC_threshing_sw != 0)
      {
        checkTalkuk = TRUE;
      }
      else
      {
        checkTalkuk = FALSE;
      }
    }
    else 
    {
      checkTalkuk = TRUE;
    }
    
    if((warnings[wIndex].flag == FALSE) && (warnings[wIndex].level != W_LEVEL_ON_TOP))
    {
      warnings[wIndex].state = W_STATE_NO_ERROR;
      warnings[wIndex].page = W_NO_PAGE;
    }
       
    if(((warnings[wIndex].flag == W_FLAG_ACTIVE) && (checkRPM == TRUE) && (checkTalkuk == TRUE) &&
       ((warnings[wIndex].state == W_STATE_NO_ERROR) || (warnings[wIndex].state == W_STATE_ACTIVE)))    // Error is active and error is NOT passive
       ||
       ((warnings[wIndex].state == W_STATE_ACTIVE) && (warnings[wIndex].level == W_LEVEL_ON_TOP)))      // There is NO active error and it should be displayed always ON top
    {
      if(warnings[wIndex].state == W_STATE_NO_ERROR)
      {
        warnings[wIndex].page = W_PAGE_1;
        warnings[wIndex].state = W_STATE_ACTIVE;
      }
      
      set_warning_sound(warnings[wIndex].soundPeriodNumber, warnings[wIndex].soundPeriod, 
                        warnings[wIndex].soundOnDuration, warnings[wIndex].soundOffDuration);
             
      return wIndex;
    }
  }
  
  return TOTAL_NUMBER_OF_WARNINGS;
}

void warning_process()
{
  static uint16_t timerWarningCharge = 0;
  static uint16_t timerToCheckWarning = 0;
  static uint16_t timerBatteryOn = 0;
  static uint16_t timerBatteryOff = 0;
  static uint16_t timerFuelLow = 0; // tseveen 20250422
  
  if(flagTimer.hundredMs == FALSE)
  {
    return;
  }
  
  if(timerToCheckWarning < WARNING_START_TIME)
  {
    timerToCheckWarning += 100;
    flagWarning.detected = FALSE;
    flagWarning.index = TOTAL_NUMBER_OF_WARNINGS;
    return;
  }
  
  if((flagInput.charge == ON) && (flag.engineStarted == TRUE))
  {
    timerWarningCharge += 100;
    if(timerWarningCharge >= 1000)
    {
      timerWarningCharge = 1000;
      flagWarning.charge = FALSE; // tseveen 20250423 changed to TRUE
    }
  }
  else
  {
    flagWarning.charge = FALSE;
    timerWarningCharge = 0;
  }
  
  
  // Check warnings
  if(flagWarning.fuel == FALSE)
  {
    if(tFuelPercent < FUEL_WARNING_ON_LEVEL)
    { 
      flagWarning.fuel = FALSE;    // tseveen 20250425 removed TRUE 
      timerFuelLow += 100;        // tseveen 20250422
      if(timerFuelLow >= 1000)   // tseveen 20250422
      {
        flagWarning.fuel = FALSE;        // tseveen 20250422
//        currentPageState = MAIN_PAGE; // tseveen 20250422
        timerFuelLow = 1000;  // tseveen 20250422
      }
    }
  }
  else                                          // Middle of the warning
  {
    if(tFuelPercent > FUEL_WARNING_OFF_LEVEL)
    {
      timerFuelLow = 0;
      flagWarning.fuel = FALSE; 
    }
  }
  
  if(numberOfEngineOilExchange == 1)
  {
    if(engineOilHour >= ENGINE_OIL_HOUR_FIRST_TIME)
    { 
      flagWarning.engineOilExchange = TRUE;
    }
    else
    {
      flagWarning.engineOilExchange = FALSE;
    }
  }
  else
  {
    if(engineOilHour >= ENGINE_OIL_HOUR)
    {
      flagWarning.engineOilExchange = TRUE;
    }
    else
    {
      flagWarning.engineOilExchange = FALSE;
    }
  }
  
  if(numberOfMissionOilExchange == 1)
  {
    if(missionOilHour >= MISSION_OIL_HOUR_FIRST_TIME)
    {
      flagWarning.missionOilExchange = TRUE;
    }
    else
    {
      flagWarning.missionOilExchange = FALSE;
    }
  }
  else
  {
    if(missionOilHour >= MISSION_OIL_HOUR)
    {
      flagWarning.missionOilExchange = TRUE;
    }
    else
    {
      flagWarning.missionOilExchange = FALSE;
    }
  }
  
  if((tPowerVoltage < 10.3) && (flag.engineStarted == FALSE) && (flagWarning.battery == FALSE))                   // 10.5
  {
    timerBatteryOff = 0;
    timerBatteryOn += 100;
    if(timerBatteryOn >= 1000)
    {
      timerBatteryOn = 1000;
      flagWarning.battery = TRUE;      
    }
  }
  else
  {
    timerBatteryOn = 0;
    if(flagWarning.battery == TRUE)    
    {      
      if(tPowerVoltage > 11)    
      {
        timerBatteryOff += 100;
        if(timerBatteryOff >= 1000)
        {
          timerBatteryOff = 1000;
          flagWarning.battery = FALSE;
        }
      }
    }
    else
    {
      timerBatteryOff = 0;
    }
  }
  
  if((dataCAN390.MC_HST_error_code & 0x01) == 0x01)
  {
    flagWarning.MC_HST_chuhen_motor_error = TRUE;
  }
  else
  {
    flagWarning.MC_HST_chuhen_motor_error = FALSE;
  }
  
  if((dataCAN390.MC_HST_error_code & 0x02) == 0x02)
  {
    flagWarning.MC_HST_chuhyan_motor_error = TRUE;
  }
  else
  {
    flagWarning.MC_HST_chuhyan_motor_error = FALSE;
  }

  if((dataCAN390.MC_HST_error_code & 0x04) == 0x04)
  {
    flagWarning.MC_HST_chuhen_lever_error = TRUE;
  }
  else
  {
    flagWarning.MC_HST_chuhen_lever_error = FALSE;
  }

  if((dataCAN390.MC_HST_error_code & 0x08) == 0x08)
  {
    flagWarning.MC_HST_chuhyan_lever_error = TRUE;
  }
  else
  {
    flagWarning.MC_HST_chuhyan_lever_error = FALSE;
  }
  
  if(modelSelection == MODEL_NEXT)
  {
    flagWarning.chipbechulBlockage = FALSE;
    flagWarning.talkukBin = FALSE;
    flagWarning.sensor2 = FALSE;
    
    if(dataCAN311.IC_pick1_threshing_sensor_alarm == TRUE)
    {
      flagWarning.nason2 = TRUE; 
    }
    else
    {
      flagWarning.nason2 = FALSE;
    }
    
    if(dataCAN311.IC_pick2_sensor_alarm == TRUE)
    {
      flagWarning.nasonYangug = TRUE; 
    }
    else
    {
      flagWarning.nasonYangug = FALSE;
    }
    
    if(dataCAN311.IC_engineStop_chipbechulBlockage == TRUE)
    {
      flagWarning.nason2BlockSwitch = TRUE; 
    }
    else
    {
      flagWarning.nason2BlockSwitch = FALSE;
    }
  }
  else
  {
    flagWarning.nason2 = FALSE;
    flagWarning.nasonYangug = FALSE;
    flagWarning.nason2BlockSwitch = FALSE;
    
    if(dataCAN311.IC_pick2_sensor_alarm == TRUE)
    {
      flagWarning.sensor2 = TRUE;
    }
    else
    {
      flagWarning.sensor2 = FALSE;
    }
    
    if(dataCAN311.IC_pick1_threshing_sensor_alarm == TRUE)
    {
      flagWarning.talkukBin = TRUE;
    }
    else
    {
      flagWarning.talkukBin = FALSE;
    }
    
    if(dataCAN311.IC_engineStop_chipbechulBlockage == TRUE)
    {
      flagWarning.chipbechulBlockage = TRUE;
    }
    else
    {
      flagWarning.chipbechulBlockage = FALSE;
    }
  }
  
  flagWarning.airFilter = j1939AirFilter;
  flagWarning.waterSeparator = j1939WaterSeparator;
  flagWarning.oilPressure = j1939OilPressure;
  
  flagWarning.index = warning_flag_checker();
  if(flagWarning.index >= TOTAL_NUMBER_OF_WARNINGS)
  {
    sound_clear(FALSE);
    flagWarning.detected = FALSE;
  }
  else
  {
    flagWarning.detected = TRUE;
  }
}

/*----------------------------------------------- Axel control -----------------------------------------------*/

#define MIN_RPM                 1300
#define MAX_RPM                 2700
#define CONST_RPM_2100          2100
#define CONST_RPM_2600          2600
#define AXEL_OUTPUT_CHECK_TIME  100
#define AXEL_MAX_VALUE          250
uint16_t timerKuRPMDelay = 0;

void axel_process()
{
  static uint8_t axelTalkukEnable = FALSE;
  //static uint8_t axelSpeedEnable = FALSE;
  static uint8_t axelAugerMotorEnable = FALSE;
  static uint8_t axelKuEnable = FALSE;
  uint8_t sensorAutoPedal = 0;
  
  static uint16_t kuTime = 0;
      
  if(axelControlFunctionEnabled == FALSE)
  {
    return;
  }
  
  axel_app_sensor1_position = (uint16_t)tAccelTotal;
  axel_app_sensor2_position = (uint16_t)tAccelTotal;
  
  if(axel_app_sensor1_position_max > axel_app_sensor1_position_min)
  {
    // This is MAX is HIGH
    if(axel_app_sensor1_position >= axel_app_sensor1_position_max)
    {
      acceleratorPedalPosition = AXEL_MAX_VALUE;
    }
    else if(axel_app_sensor1_position <= axel_app_sensor1_position_min)
    {
      acceleratorPedalPosition = 0;
    }
    else
    {
      // Max value is high
      // 0 to 250
      acceleratorPedalPosition = (uint8_t)(((float)((float)axel_app_sensor1_position - (float)axel_app_sensor1_position_min) / (float)((float)axel_app_sensor1_position_max - (float)axel_app_sensor1_position_min)) * (float)AXEL_MAX_VALUE);
      if(acceleratorPedalPosition > AXEL_MAX_VALUE)
      {
        acceleratorPedalPosition = AXEL_MAX_VALUE;
      }
    }
  }
  else
  {
    // This is min is HIGH
    if(axel_app_sensor1_position >= axel_app_sensor1_position_min)
    {
      acceleratorPedalPosition = 0;
    }
    else if(axel_app_sensor1_position <= axel_app_sensor1_position_max)
    {
      acceleratorPedalPosition = AXEL_MAX_VALUE;
    }
    else
    {
      // Min value is high
      acceleratorPedalPosition = (uint8_t)(((float)((float)axel_app_sensor1_position - (float)axel_app_sensor1_position_min) / (float)((float)axel_app_sensor1_position_max - (float)axel_app_sensor1_position_min)) * (float)AXEL_MAX_VALUE);
      if(acceleratorPedalPosition > AXEL_MAX_VALUE)
      {
        acceleratorPedalPosition = AXEL_MAX_VALUE;
      }
    }
  }

  if(dataCAN035.general_auto_accel_led == ON)
  {
    // Threshing switch
    // Yeache up and down outputs
    // TBS up and down switches
    // Auger up and down outputs
    // Speed is over threshold
    // Auger "dataCAN310.IC_auger_disconnected_sw" and "dataCAN310.IC_auger_connected_sw"
    
    if((axel_threshing_delay != 0) || (axel_yeache_ku_time != 0) || 
       (axel_tbs_ku_time != 0) || (axel_auger_ku_time != 0) || 
       (axel_c_speed_ku_time != 0) || (axel_auger_delay != 0))
    {
           // 310 2.7
      if(((dataCAN310.IC_threshing_sw == ON) && ((axel_threshing_delay != 0)))                                                            ||
            // 330 4.2, 4.1
         (((dataCAN330.CC_cutting_up_output == ON) || (dataCAN330.CC_cutting_down_output == ON)) && (axel_yeache_ku_time != 0))           || 
            //310 7.2, 7.3
         (((dataCAN310.IC_tbs_left_up_output == ON) || (dataCAN310.IC_tbs_left_down_output == ON) ||
            // 310 7.5, 7.4
           (dataCAN310.IC_tbs_up_output == ON) || (dataCAN310.IC_tbs_down_output == ON) ||
            // 332 0.5, 0.3
           (dataCAN332.CC_pitching_manual_transfer == ON) || (dataCAN332.CC_pitching_manual_charge == ON)) && (axel_tbs_ku_time != 0))                                                    ||
            // 310 7.0, 7.1
         (((dataCAN310.IC_auger_up_output == ON) || (dataCAN310.IC_auger_down_output == ON)) && (axel_auger_ku_time != 0))                ||
           // 311 byte 0.0~1, byte1
         ((CC_vehicle_speed_val >= 20) && (axel_c_speed_ku_time != 0))                                                                    ||
         ((dataCAN310.IC_auger_connected_sw == ON) && (axel_auger_delay != 0)))
      {
        timerKuRPMDelay += 2;
        if(timerKuRPMDelay >= AXEL_OUTPUT_CHECK_TIME)
        {
          axelKuEnable = TRUE;
                  
          if(dataCAN310.IC_threshing_sw == ON)
          {
            kuTime = axel_threshing_delay * 1000;
            axelTalkukEnable = TRUE;
          }
          else if((dataCAN330.CC_cutting_up_output == ON) || (dataCAN330.CC_cutting_down_output == ON))
          {
            kuTime = axel_yeache_ku_time * 1000;
          }
          else if(dataCAN310.IC_auger_connected_sw == ON)
          {
            kuTime = axel_auger_delay * 1000;
            axelAugerMotorEnable = TRUE;
          }
          else if((dataCAN310.IC_tbs_left_up_output == ON) || (dataCAN310.IC_tbs_left_down_output == ON) ||
                  (dataCAN310.IC_tbs_up_output == ON) || (dataCAN310.IC_tbs_down_output == ON) ||
                  (dataCAN332.CC_pitching_manual_transfer == ON) || (dataCAN332.CC_pitching_manual_charge == ON))
          {
            kuTime = axel_tbs_ku_time * 1000;
          }
          else if((dataCAN310.IC_auger_up_output == ON) || (dataCAN310.IC_auger_down_output == ON))
          {
            kuTime = axel_auger_ku_time * 1000;
          }
          else if(CC_vehicle_speed_val >= 20)
          {
            kuTime = axel_c_speed_ku_time * 1000;
            //axelSpeedEnable = TRUE;
            timerKuRPMDelay = AXEL_OUTPUT_CHECK_TIME;                             // This time is increased when speed is down
          }
          else
          {
            // Wrong condition
            kuTime = 0;
          }
      
          if(timerKuRPMDelay >= (kuTime + AXEL_OUTPUT_CHECK_TIME))
          {
            timerKuRPMDelay = kuTime + AXEL_OUTPUT_CHECK_TIME;
          }
        }
      }
      else
      {      
        if((axelTalkukEnable == TRUE) || (axelAugerMotorEnable == TRUE))
        {
          axelAugerMotorEnable = FALSE;
          axelTalkukEnable = FALSE;
          axelKuEnable = FALSE;
        }
        else if(axelKuEnable == TRUE)
        {
          timerKuRPMDelay += 2;
          if(timerKuRPMDelay >= (kuTime + AXEL_OUTPUT_CHECK_TIME))
          {
            axelKuEnable = FALSE;
          }
        }
        
        if(axelKuEnable == FALSE)
        {
          timerKuRPMDelay = 0;
        }
      }
      
      if(axelKuEnable == TRUE)
      {
        if(axelTalkukEnable == TRUE)
        {
          // % = 250 * (2600 / 2700) = 240, (3000 - 500) / 2500
          sensorAutoPedal = (uint8_t)(((float)AXEL_MAX_VALUE * ((float)(axel_threshing_rpm - MIN_RPM) / (float)(MAX_RPM - MIN_RPM))) * 
            ((float)(timerKuRPMDelay - AXEL_OUTPUT_CHECK_TIME) / (float)(kuTime)));
        }
        else if(axelAugerMotorEnable == TRUE)
        {
          // % = 250 * (2600 / 2700) = 240, (3000 - 500) / 2500
          sensorAutoPedal = (uint8_t)(((float)AXEL_MAX_VALUE * ((float)(CONST_RPM_2600 - MIN_RPM) / (float)(MAX_RPM - MIN_RPM))) * 
            ((float)(timerKuRPMDelay - AXEL_OUTPUT_CHECK_TIME) / (float)(kuTime)));
        }
        else 
        {
          sensorAutoPedal = (uint8_t)((float)AXEL_MAX_VALUE * ((float)(CONST_RPM_2100 - MIN_RPM) / (float)(MAX_RPM - MIN_RPM)));
          
  //        sensorAutoPedal = 136;         // 2100 RPM
        }
        /*
        else
        {
          // 250 * (2100 / 2700)
          sensorAutoPedal = (uint8_t)((float)AXEL_MAX_VALUE * ((float)CONST_RPM / (float)3850));
        }
        */
        
        if(sensorAutoPedal > AXEL_MAX_VALUE)
        {
          sensorAutoPedal = AXEL_MAX_VALUE;
        }
      }
    }
    else
    {
      timerKuRPMDelay = 0;
      axelKuEnable = FALSE;
    }
    
    if(axelAugerMotorEnable == TRUE)
    {
      acceleratorPedalPosition = sensorAutoPedal;
    }
    else
    {  
      if(sensorAutoPedal > acceleratorPedalPosition)
      {
        acceleratorPedalPosition = sensorAutoPedal;
      }
    }
  }
}

/*----------------------------------------------- Axel control -----------------------------------------------*/

uint8_t control_init()
{
  localEngineHourTimer = 0;
  localJobHourTimer = 0;
  flagTimer.data = 0;
  flagWarning.data[0] = 0;
  flagWarning.data[1] = 0;
  
  sound_clear(TRUE);
  memory_update();
    
  ygb643_tw9990_initialize();
  init_error_msg_ecu();

  watchdog_enable();
  
  return TRUE;
}

void control_process()
{
  digital_sensors();
  
  analog_sensors();
  
//  can_receive_process();      tseveen 20250423
  
  sensor_process();
  warning_process();
  time_process();
  
  dtc_process();
//  check_display_buttons(); tseveen 20250421
      
  rpm_process();
  
  lcd_process();
  sound_process();
  
  axel_process();
  
  write_outputs();
}
/*----------------------------------------------- Algorithm -----------------------------------------------*/

void ygv643_init()
{
  GPIO_InitTypeDef GPIO_InitStructure = { 0 };
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(YGV643_CS_GPIO_Port, YGV643_CS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(YGV643_RESET_GPIO_Port, YGV643_RESET_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStructure.Pin = YGV643_CS_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YGV643_CS_GPIO_Port, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = YGV643_RESET_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YGV643_RESET_GPIO_Port, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = YGV643_INT_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(YGV643_INT_GPIO_Port, &GPIO_InitStructure);
  
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void lcd_init()
{
  GPIO_InitTypeDef GPIO_InitStructure = { 0 };
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(LCD_POWER_GPIO_Port, LCD_POWER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  
  GPIO_InitStructure.Pin = LCD_POWER_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_POWER_GPIO_Port, &GPIO_InitStructure);
    
  GPIO_InitStructure.Pin = LCD_RST_Pin;
  GPIO_InitStructure.Mode =  GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStructure);
    
  GPIO_InitStructure.Pin = LCD_ERR_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_ERR_GPIO_Port, &GPIO_InitStructure);
}

void eeprom_init(void)
{
  init_93c56();
}
