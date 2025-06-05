#ifndef __YVC1_HAL_H
#define __YVC1_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------------------------------------------------- Defenitions ----------------------------------------------------------*/
#define YGV643_INT_Pin                  GPIO_PIN_11
#define YGV643_INT_GPIO_Port            GPIOB
#define YGV643_INT_EXTI_IRQn            EXTI15_10_IRQn

#define YGV643_CS_Pin                   GPIO_PIN_12
#define YGV643_CS_GPIO_Port             GPIOB

#define YGV643_RESET_Pin                GPIO_PIN_8
#define YGV643_RESET_GPIO_Port          GPIOD

#define LCD_POWER_Pin                   GPIO_PIN_11
#define LCD_POWER_GPIO_Port             GPIOE

#define LCD_RST_Pin                     GPIO_PIN_7
#define LCD_RST_GPIO_Port               GPIOE

#define LCD_ERR_Pin                     GPIO_PIN_8
#define LCD_ERR_GPIO_Port               GPIOE

#define YGV643_ENABLE_HIGH              HAL_GPIO_WritePin(YGV643_CS_GPIO_Port, YGV643_CS_Pin, GPIO_PIN_SET)
#define YGV643_ENABLE_LOW               HAL_GPIO_WritePin(YGV643_CS_GPIO_Port, YGV643_CS_Pin, GPIO_PIN_RESET)

#define YGV643_RESET_LOW                HAL_GPIO_WritePin(YGV643_RESET_GPIO_Port, YGV643_RESET_Pin, GPIO_PIN_SET)
#define YGV643_RESET_HIGH               HAL_GPIO_WritePin(YGV643_RESET_GPIO_Port, YGV643_RESET_Pin, GPIO_PIN_RESET)

#define LCD_RST_HIGH                    HAL_GPIO_WritePin(LCD_RST_GPIO_Port , LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_RST_LOW                     HAL_GPIO_WritePin(LCD_RST_GPIO_Port , LCD_RST_Pin, GPIO_PIN_SET)

#define LCD_MD070NF04_60ID_18A_AM       1
#define LCD_BRIGTHNESS_MAX              1000
#define LCD_BRIGTHNESS_MIN              5
#define LCD_BRIGTHNESS_STEP             5

#define FUEL_WARNING_ON_LEVEL           2                                      // percentage
#define FUEL_WARNING_OFF_LEVEL          7
#define FUEL_RED_IMAGE_ON_LEVEL         5                                      // percentage
#define FUEL_RED_IMAGE_OFF_LEVEL        10


#define SETTINGS_PAGE_SHIFT_TIMER       1000                                    // 1 second
#define LOGO_TIME                       300                                     // 0.3 seconds
#define WARNING_START_TIME              5000                                    // 5 seconds
#define NUMBER_OF_LOGO_IMAGE            34                                      // Number of images used for boot logo

  
//#define UREA_UPDATE_TIME                300                                     // 30 seconds
//#define COOLEANT_UPDATE_TIME            300                                     // 30 seconds
#define PCODE_SWITCHING_TIME            500                                     // 500 ms
#define CLOCK_ACTION_NUMBER             8                                       // Main display clock action images
#define CLOCK_ACTION_TIME               200                                     // Main display clock action time

/* ---------------------------------------------------------- Page states & variables ----------------------------------------------------------*/
typedef enum
{
  LOGO_PAGE = 0,
  MAIN_PAGE,
  SETTINGS_PAGE,
  
  SETTINGS_10_PAGE,
  SETTINGS_20_PAGE,
  SETTINGS_30_PAGE,                                             //  Check mode menu is beginning
  SETTINGS_40_PAGE,
  SETTINGS_50_PAGE,
  SETTINGS_60_PAGE,
  
  SETTINGS_11_PAGE,
  SETTINGS_111_PAGE,
  SETTINGS_112_PAGE,  
  SETTINGS_113_PAGE,
  SETTINGS_12_PAGE,
  SETTINGS_121_PAGE,
  SETTINGS_122_PAGE,
  SETTINGS_13_PAGE,
  SETTINGS_131_PAGE,
  SETTINGS_132_PAGE,
  SETTINGS_133_PAGE,
  SETTINGS_14_PAGE,
  SETTINGS_141_PAGE,
  
  SETTINGS_15_PAGE,
  SETTINGS_151_PAGE,
  SETTINGS_152_PAGE,
  SETTINGS_153_PAGE,
  SETTINGS_154_PAGE,
  SETTINGS_155_PAGE,
  SETTINGS_156_PAGE,
  SETTINGS_157_PAGE,
  SETTINGS_158_PAGE,
  SETTINGS_159_PAGE,
  SETTINGS_1510_PAGE,
  SETTINGS_1511_PAGE,
  SETTINGS_1512_PAGE,
  
  SETTINGS_16_PAGE,
  SETTINGS_161_PAGE,
  SETTINGS_162_PAGE,
  SETTINGS_163_PAGE,
  
  SETTINGS_21x_PAGE,
  SETTINGS_22x_PAGE,
  SETTINGS_23x_PAGE,
  SETTINGS_24x_PAGE,
//  SETTINGS_25x_PAGE,
  SETTINGS_26x_PAGE,
  
  SETTINGS_211_PAGE,
  SETTINGS_212_PAGE,
  SETTINGS_213_PAGE,  
  SETTINGS_214_PAGE,
  SETTINGS_215_PAGE,
  SETTINGS_216_PAGE,
  SETTINGS_217_PAGE,
  SETTINGS_218_PAGE,
  
  SETTINGS_221_PAGE,
  SETTINGS_222_PAGE,
  SETTINGS_223_PAGE,
  SETTINGS_224_PAGE,
  SETTINGS_225_PAGE,
  SETTINGS_226_PAGE,
  SETTINGS_227_PAGE,
  
  SETTINGS_231_PAGE,
  SETTINGS_232_PAGE,
  SETTINGS_233_PAGE,
  SETTINGS_234_PAGE,
  SETTINGS_235_PAGE,
  SETTINGS_236_PAGE,
  SETTINGS_237_PAGE,
  SETTINGS_238_PAGE,
  
  SETTINGS_241_PAGE,
  
  SETTINGS_251_PAGE,
  SETTINGS_252_PAGE,
  SETTINGS_253_PAGE,
  SETTINGS_254_PAGE,
  SETTINGS_255_PAGE,
  SETTINGS_256_PAGE,
  SETTINGS_257_PAGE,
  SETTINGS_258_PAGE,
  SETTINGS_259_PAGE,
  SETTINGS_2510_PAGE,
  SETTINGS_2511_PAGE,
  
  SETTINGS_261_PAGE,
  SETTINGS_262_PAGE,
    
  SETTINGS_2xxF_PAGE,
  
  SETTINGS_31x_PAGE,                                            //  CAN service
  SETTINGS_311_PAGE,
  SETTINGS_3111_PAGE,
  SETTINGS_3112_PAGE,
  SETTINGS_3113_PAGE,
  SETTINGS_312_PAGE,
  SETTINGS_313_PAGE,
  SETTINGS_314_PAGE,
  SETTINGS_3141_PAGE,
  SETTINGS_3142_PAGE,
  SETTINGS_3143_PAGE,
  SETTINGS_315_PAGE,

  SETTINGS_32x_PAGE,                                            //  SWITCH CAN
  
  SETTINGS_33x_PAGE,                                            //  Oil Change CAN
  SETTINGS_331_PAGE,
  SETTINGS_332_PAGE,
  
  SETTINGS_34x_PAGE,                                            //  Axel CAN
  SETTINGS_35x_PAGE,                                            //  HST CAN
  
  SETTINGS_36x_PAGE,                                            //  HST CAN
  
  //SETTINGS_41x_PAGE,                                          // This page is not currently used so maybe later it will be
  SETTINGS_511_PAGE,
  SETTINGS_512_PAGE,
  SETTINGS_513_PAGE,
  SETTINGS_5131_PAGE,
  SETTINGS_514_PAGE,
  SETTINGS_5141_PAGE,
  SETTINGS_515_PAGE,
  SETTINGS_516_PAGE,
  SETTINGS_517_PAGE,
    
  WARNING_PAGE,
  
} pageState_t;

extern float jobHour;
  
extern uint8_t setup_mode;
extern uint16_t setup_mode_address;
extern uint8_t setup_mode_type;
extern uint8_t setup_mode_rw;

extern int16_t new_setup_data;                                                          // Added on 22 March 2021
extern int16_t new_setup_data_2;                                                        // Added on 24 March 2021

extern uint8_t  tFuelPercent;
extern float    tPowerVoltage;

extern uint8_t acceleratorPedalPosition;

/*----------------------------------------------- Warning -----------------------------------------------*/

typedef enum 
{
  W_FUEL_EMPTY = 0,                                                             // 0 --> 1 pages
  W_ENGINE_OIL_EXCHANGE,
  W_MISSION_OIL_EXCHANGE,
  W_CHUHEN_MOTOR,
  W_STARTER_SAFETY,
  W_CAN_TIMEOUT_INTEGRATED_CONTROLLER,
  W_CAN_TIMEOUT_CUTTING_CONTROLLER,
  W_CAN_TIMEOUT_AUGER_CONTROLLER,
  W_CAN_TIMEOUT_TALKUK_HEIGHT_CONTROLLER,
  W_CAN_TIMEOUT_GENERAL_SWITCH,
  W_CAN_TIMEOUT_AUGER_SWITCH,
  W_CAN_TIMEOUT_HST_CONTROLLER,
  W_CAN_TIMEOUT_AXEL_CONTROLLER,
  W_CAN_TIMEOUT_SONBYOL_CONTROLLER,
  W_CHUHYAN_MOTOR,
  W_AUGER_MOTOR,
  W_TALKUK_HEIGHT,
  W_CHUHEN_LEVER,
  W_CHUHYAN_LEVER,
  W_BATTERY,
  W_CHUHEN_CONTROLLER,                                                          // Until warning #1
  
  W_AUGER_ROTATION_SENSOR,
  W_LSA_MOTOR_POSITION_SENSOR,
  W_TBS_SLOPE_SENSOR,
  W_TBS_RIGHT_SENSOR,
  W_TBS_LEFT_SENSOR,
  W_TBS_MANUAL_SWITCH,
  W_AUGER_MANAUL_SWITCH,
  W_AUGER_SETTING_SWITCH,
  W_LSA_MH_SENSOR,
  W_LSA_MANUAL_SWITCH,
  W_YEACHE_LIFT_SENSOR,
  W_PREVIEW_SENSOR,
  W_YEACHE_MANAUL_SWITCH,
  W_TALKUK_CLUTCH_CONNECTION_BLOCK,
  W_TALKUK_CLUTCH_CONNECTION_BLOCK_LSA_MOTOR,
  W_ONETOUCH_UP_DOWN,
  
  W_ENGINE_STOP_SWITCH,                                                         // Two pages
  W_CHARGE,
  W_ENGINE_OIL_PRESSURE,
  W_ENGINE_COOLING_TEMPERATURE,
  W_AIR_FILTER,
  W_TALKUK_BIN,
  W_CHORI_BIN,
  W_2_SENSOR,
  W_CHIPBECHUL,
  W_GUGMUL_MANYANG,
  W_MULBUNRIGI,
  W_GUGMUL_MANYANG_ENGINE_STOP,
  W_ENGINE_STOP_CUTTING_SAFETY,
  W_ENGINE_STOP_YEACHE_SAFETY,
  W_2_NASON,
  W_YANGUG_NASON,
  W_2_NASON_BLOCK_SWITCH,
  
  TOTAL_NUMBER_OF_WARNINGS
} wIndex_t;

typedef enum
{
  W_FLAG_PASSIVE = 0,
  W_FLAG_ACTIVE
} wFlag_t;

typedef enum
{
  W_STATE_NO_ERROR = 0,
  W_STATE_ACTIVE,
  W_STATE_PASSIVE
} wState_t;

typedef enum
{
  W_LEVEL_PASSIVE = 0,                          // NO PAGE
  W_LEVEL_ACTIVE,                               // PAGE is shown when current error is active
  W_LEVEL_ON_TOP                                // PAGE is shown when error is occurred at least 1 time
} wLevel_t;

typedef enum
{
  W_NO_PAGE = 0,
  W_PAGE_1,
  W_PAGE_2
} wPage_t;

typedef union 
{
  uint32_t data[2];
  struct {
    uint32_t index                      : 8;
    
    uint32_t detected                   : 1;
    uint32_t engineOilExchange          : 1;
    uint32_t missionOilExchange         : 1;
    uint32_t fuel                       : 1;
    uint32_t airFilter                  : 1;                                    // PCODE = 1101
    uint32_t waterSeparator             : 1;                                    // PCODE = 1151
    uint32_t oilPressure                : 1;                                    // PCODE = 1192
    uint32_t can_timeout_warning_025    : 1;
    
    uint32_t can_timeout_warning_035    : 1;
    uint32_t can_timeout_warning_31x    : 1;
    uint32_t can_timeout_warning_33x    : 1;
    uint32_t can_timeout_warning_340    : 1;
    uint32_t can_timeout_warning_350    : 1;
    uint32_t can_timeout_warning_360    : 1;
    uint32_t can_timeout_warning_381    : 1;
    uint32_t can_timeout_warning_390    : 1;

    uint32_t MC_HST_chuhen_motor_error  : 1;
    uint32_t MC_HST_chuhyan_motor_error : 1;
    uint32_t MC_HST_chuhen_lever_error  : 1;
    uint32_t MC_HST_chuhyan_lever_error : 1;
  
    uint32_t charge                     : 1;
    uint32_t battery                    : 1;
    uint32_t nason2                     : 1;
    uint32_t nasonYangug                : 1;
    
    uint32_t nason2BlockSwitch          : 1;
    uint32_t talkukBin                  : 1;
    uint32_t sensor2                    : 1;
    uint32_t chipbechulBlockage         : 1;
    uint32_t res0                       : 4;
    
    uint32_t res1                       : 8;
    uint32_t res2                       : 8;
    uint32_t res3                       : 8;
  };
} flagWarning_t;
extern flagWarning_t flagWarning;
/*----------------------------------------------- Warning -----------------------------------------------*/

/*----------------------------------------------- Memory -----------------------------------------------*/
enum
{
  MEMORY_READ = 0,
  MEMORY_WRITE = 1
};
  
enum 
{
  CONFIGURE_JOB_HOUR_MSB = 0,
  CONFIGURE_JOB_HOUR_LSB,
  CONFIGURE_ENGINE_HOUR_MSB,
  CONFIGURE_ENGINE_HOUR_LSB,
  CONFIGURE_ENGINE_OIL_HOUR_MSB,
  CONFIGURE_ENGINE_OIL_HOUR_LSB,
  CONFIGURE_MISSION_OIL_HOUR_MSB,
  CONFIGURE_MISSION_OIL_HOUR_LSB,
  
  CONFIGURE_ENGINE_OIL_EXCHANGE_COUNTER,
  CONFIGURE_MISSION_OIL_EXCHANGE_COUNTER,
  CONFIGURE_LCD_BRIGHTNESS_DAY,
  CONFIGURE_LCD_BRIGHTNESS_NIGHT,
  
  CONFIGURE_MODEL_SELECTION,
  CONFIGURE_AXEL_THRESHING_DELAY,
  CONFIGURE_AXEL_AUGER_DELAY,
  CONFIGURE_AXEL_AUGER_AUTO_DELAY,
  CONFIGURE_AXEL_YEACHE_KU_TIME,
  CONFIGURE_AXEL_TBS_KU_TIME,
  CONFIGURE_AXEL_AUGER_KU_TIME,
  CONFIGURE_AXEL_C_SPEED_KU_TIME,
  CONFIGURE_AXEL_THRESHING_RPM,
  CONFIGURE_AXEL_KEEP_TIME_MODE,
  CONFIGURE_AXEL_APP_SENSOR_TOTAL_ERROR,
  CONFIGURE_AXEL_DELAY_MODE,
  CONFIGURE_AXEL_APP_SENSOR1_POSITION_MAX,
  CONFIGURE_AXEL_APP_SENSOR1_POSITION_MIN,
  CONFIGURE_AXEL_APP_SENSOR2_POSITION_MAX,
  CONFIGURE_AXEL_APP_SENSOR2_POSITION_MIN,
  
  NUMBER_OF_CONFIGURATION
};
 
extern uint8_t modelSelection;
#define MODEL_OLD               0
#define MODEL_NEW_WITH_AXEL     1
#define MODEL_NEXT              2
 
   
#define ENGINE_OIL_HOUR_FIRST_TIME      30
#define ENGINE_OIL_HOUR                 100
#define MISSION_OIL_HOUR_FIRST_TIME     50
#define MISSION_OIL_HOUR                250

float getEngineHour();
uint16_t get_memory(uint8_t* data);
uint16_t set_memory(uint8_t* data);
void memory_update();
void default_memory_update();
/*----------------------------------------------- Memory -----------------------------------------------*/



/*----------------------------------------------- LCD -----------------------------------------------*/
void     YVC_Reset(void);
void     WaitMSec(uint16_t cnt);
uint8_t  YVC_BurstWritePort(uint8_t pPort_num, const uint8_t *uWr_data, uint16_t num);
uint8_t  YVC_BurstReadPort(uint8_t pPort_num,  uint8_t *uRd_data, uint16_t num);
uint8_t  YVC_WritePort(uint8_t pPort_num, uint8_t uWr_data);
uint8_t  YVC_ReadPort(uint8_t pPort_num);

void ygv643_initialize();


uint16_t display_function(uint16_t _imgCnt );
uint16_t draw_main_page(uint16_t _imgCnt);
uint16_t draw_main_settings(uint16_t _imgCnt);
uint16_t draw_sub_settings_1(uint16_t _imgCnt);
uint16_t draw_sub_settings_2(uint16_t _imgCnt);
uint16_t draw_sub_settings_3(uint16_t _imgCnt);
uint16_t draw_sub_settings_4(uint16_t _imgCnt);
uint16_t draw_sub_settings_5(uint16_t _imgCnt);

uint16_t DrawWarningCombineCheckModeTwo(uint16_t _imgCnt);
uint16_t DrawWarningCombineCheckMessage(uint16_t _imgCnt);
/*----------------------------------------------- LCD -----------------------------------------------*/

/*----------------------------------------------- Algorithm -----------------------------------------------*/

#define SETUP_MODE_NONE                 0
#define SETUP_MODE_ONE                  1
#define SETUP_MODE_TWO                  2

#define SETUP_READ                      0x55
#define SETUP_WRITE                     0xAA
  
#define VAL_10BIT_TO_VOLTAGE_5V         0.004882

#define VAC_FUEL_GAP                    40

#define VAC_TEMP_H                      2234
#define VAC_TEMP_MIDDL_UP               2462
#define VAC_TEMP_MIDDL                  2610
#define VAC_TEMP_MIDDL_DN               2800
#define VAC_TEMP_L                      3000
#define VAC_TEMP_GAP                    20

#define VAC_TEMP_9                      VAC_TEMP_H              + ((VAC_TEMP_MIDDL_UP-VAC_TEMP_H)/3)
#define VAC_TEMP_8                      VAC_TEMP_MIDDL_UP       - ((VAC_TEMP_MIDDL_UP-VAC_TEMP_H)/3)

#define VAC_TEMP_7                      VAC_TEMP_MIDDL_UP       + ((VAC_TEMP_MIDDL-VAC_TEMP_MIDDL_UP)/3)
#define VAC_TEMP_6                      VAC_TEMP_MIDDL          - ((VAC_TEMP_MIDDL-VAC_TEMP_MIDDL_UP)/3)

#define VAC_TEMP_5                      VAC_TEMP_MIDDL          + ((VAC_TEMP_MIDDL_DN-VAC_TEMP_MIDDL)/3)
#define VAC_TEMP_4                      VAC_TEMP_MIDDL_DN       - ((VAC_TEMP_MIDDL_DN-VAC_TEMP_MIDDL)/3)

#define VAC_TEMP_3                      VAC_TEMP_MIDDL_DN       + ((VAC_TEMP_L-VAC_TEMP_MIDDL_DN)/3)
#define VAC_TEMP_2                      VAC_TEMP_L              - ((VAC_TEMP_L-VAC_TEMP_MIDDL_DN)/3)
  
  
//------------------ Global variables of corresponding ADC inputs -------------------------------------------
typedef union
{
  uint32_t data;
  struct {
    uint32_t sound                      : 1;
    uint32_t alarm                      : 1;
    uint32_t engineStarted              : 1;
    uint32_t isBrigthnessSetting        : 1;
    
    uint32_t res                        : 28;
  };
} flag_t;

typedef struct  {
  uint16_t minValue;
  uint16_t maxValue;
  uint16_t defaultValue;
  uint16_t *value;
} nvData_t;

uint8_t control_init();
void control_process();

extern uint16_t timerKuRPMDelay;
/*----------------------------------------------- Algorithm -----------------------------------------------*/

/*----------------------------------------------- Hardware -----------------------------------------------*/
void eeprom_init(void);
void ygv643_init();
void lcd_init();
void update_hours();
/*----------------------------------------------- Hardware -----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __YVC1_HAL_H */