#ifndef __J1939_H
#define __J1939_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pcode.h"

//-------------------------------------------   Defenitions   -------------------------------------------
#define J1939_STARTING_ADDRESS          0x80
#define J1939_ARBITRARY_ADDRESS         0x80                    // Arbitrary Address Claim is ON
#define J1939_INDUSTRY_GROUP            3                       // Industry group is Construction equipment
#define J1939_VEHICLE_INSTANCE          0
#define J1939_CA_NAME7                  (J1939_ARBITRARY_ADDRESS | (J1939_INDUSTRY_GROUP << 4) | J1939_VEHICLE_INSTANCE) 
#define J1939_VEHICLE_SYSTEM            0
#define J1939_CA_NAME6                  (J1939_VEHICLE_SYSTEM << 1) 
#define J1939_FUNCTION                  0x88                            // 0x88 = 136 is Slope Sensor
#define J1939_CA_NAME5                  J1939_FUNCTION 
#define J1939_FUNCTION_INSTANCE         0
#define J1939_ECU_INSTANCE              0
#define J1939_CA_NAME4                  ((J1939_FUNCTION_INSTANCE << 3) | J1939_ECU_INSTANCE) 
#define J1939_MANUFACTURER_CODE         0
#define J1939_IDENTITY_NUMBER           193399
  // 19 means last two number of a year ex: 2019
  // 33 means 33th week of this year which means second week of August, 2019
  // 01 means ID of the device
#define J1939_CA_NAME3                  (J1939_MANUFACTURER_CODE >> 3) 
#define J1939_CA_NAME2                  (((J1939_MANUFACTURER_CODE & 0x07) << 5) | (J1939_IDENTITY_NUMBER >> 16)) 
#define J1939_CA_NAME1                  ((J1939_IDENTITY_NUMBER >> 8) & 0xFF) 
#define J1939_CA_NAME0                  (J1939_IDENTITY_NUMBER & 0xFF) 
#define J1939_ACCEPT_CMDADD             J1939_FALSE
#define J1939_RX_QUEUE_SIZE             1
#define J1939_OVERWRITE_RX_QUEUE        J1939_TRUE
#define J1939_TX_QUEUE_SIZE             1
#define J1939_OVERWRITE_TX_QUEUE        J1939_TRUE
#define J1939_POLL_ECAN                 J1939_TRUE
#define J1939_PRIORITIZED_INT           J1939_TRUE

#define J1939_NULL_ADDR	                254U                            /* 0xFE */                        
#define J1939_GLOBAL_ADDR	        255U                            /* 0xFF */        
#define J1939_ANY_ADDR	                J1939_GLOBAL_ADDR               /* 0xFF */
#define J1939_PGN_REQUEST	        59904LU                         /* 0xEA00 */
#define J1939_PGN_ADDR_CLAIMED	        60928LU                         /* 0xEE00 */  
#define J1939_PGN_COMMANDED_ADDR	65240LU                         /* 0xFED8 */

#define J1939_ADDR_CLAIM_PRI            6

#define J1939_NAME_LENGTH	        8

#define J1939_ADDR_CLAIM_TIMEOUT        250
#define J1939_MIN_SC_ADDR	        J1939_STARTING_ADDRESS
#define J1939_MAX_SC_ADDR	        247U

//-------------------------------------------   Global new variable type   -------------------------------------------
typedef uint32_t pgn_t;

//-------------------------------------------   Global Enums   -------------------------------------------
enum J1939_STATE{
  J1939_IDLE = 0,
  J1939_WAIT_ADDR,
  J1939_WAIT_HAVE_ADDR,
  J1939_HAVE_ADDR,
  J1939_LOST_ADDR,
};

enum J1939_BOOL {
  J1939_FALSE = 0,
  J1939_TRUE
};

//-------------------------------------------   Global Variables  -------------------------------------------
struct j1939_mesg {
	pgn_t pgn;
	uint8_t dlen;
	uint8_t data[8] __attribute__((aligned(8)));
};

extern char error_message_ecu[8];                                                          // Added on 27 Nov 2020, Enkhbat
void init_error_msg_ecu();
void dtc_process();
void update_bam_timer();
extern uint8_t engine_gross_load_ratio_1;                                                      // Added on 3 Dec 2020, Enkhbat

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_TxHeaderTypeDef   CAN1_TxHeader;
extern CAN_RxHeaderTypeDef   CAN1_RxHeader;
extern uint8_t               CAN1_TxData[8];
extern uint8_t               CAN1_RxData[8];
extern uint32_t              CAN1_TxMailbox;

extern CAN_TxHeaderTypeDef   CAN2_TxHeader;
extern CAN_RxHeaderTypeDef   CAN2_RxHeader;
extern uint8_t               CAN2_TxData[8];
extern uint8_t               CAN2_RxData[8];
extern uint32_t              CAN2_TxMailbox;

extern uint8_t j1939EngineTempratureError;
extern uint8_t j1939WaterSeparator;
extern uint8_t j1939AirFilter;
extern uint8_t j1939OilPressure;

// Currently used variables of J1939 in App
extern uint16_t engine_speed;
extern uint32_t total_engine_hours;
extern uint8_t aftertreatment1_diesel_exhaust_fluid_tank_level;
extern uint8_t pending_scr_inducement_severity;
extern uint8_t inducement_severity_for_DEF_tank_level;
extern uint8_t operator_inducement_active_for_SCR_system_failure;
extern uint8_t digital_input4_status;
extern uint8_t engine_coolant_temperature;

extern uint8_t exhaust_system_high_temperature_lamp_command;
extern uint8_t diesel_particulate_filter_lamp_command;
extern uint8_t diesel_particulate_filter_active_regeneration_inhibited_inhibit_switch;

extern uint8_t engine_check_lamp_bam;
extern uint8_t warning_check_lamp_bam;

extern uint16_t ecu_bam_timer;
extern uint16_t dcu_bam_timer;

extern uint8_t  DCU_DTC_counter_received;
extern uint8_t  ECU_DTC_counter_received;

extern uint8_t  DCU_error_flag;
extern uint8_t  ECU_error_flag;

extern uint8_t digital_output2_status;
//-------------------------------------------   BAM Definition   -------------------------------------------
#define YMR_SCR_DATALOGING_1_PGN        0xFF03                          // Yanmar SCR status
#define YMR_SCR_DATALOGING_2_PGN        0xFF04                          // Yanmar SCR control 1
#define YMR_SCR_DATALOGING_3_PGN        0xFF05                          // Yanmar SCR control 2
#define YMR_SCR_DATALOGING_4_PGN        0xFF06                          // Yanmar SCR control model
#define YMR_SCR_DATALOGING_5_PGN        0xFF07                          // Yanmar SCR control 3
#define YMR_SCR_DATALOGING_6_PGN        0xFF08                          // Yanmar SCR Heter
#define YMR_SCR_DATALOGING_7_PGN        0xFF09                          // Yanmar SCR Shutoff
#define J1939_DM1_PGN                   0xFECA

#define BAM_OK                          0
#define BAM_FAILED                      1
#define BAM_TIMEOUT                     2
#define BAM_RECEIVER_TIME               750

#define MAXIMUM_DTC                     20
#define DTC_SWITCHING_TIME              1000
//-------------------------------------------   Global Functions   -------------------------------------------
int8_t J1939_Initialization( uint8_t InitNAMEandAddress);
void j1939_req_addr_claimed_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void j1939_addr_claimed_handler();
static inline uint8_t j1939_lose_addr();
uint8_t j1939_send_addr_claimed(uint8_t _address);
int8_t j1939_claim_addr();

// CAN Interrupt handler function
void J1939_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
// J1939 Protocol handler functions
void J1939_ECU_BAM_HEAD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);                     // Added BAM message on 13 April 2021
void J1939_ECU_BAM_DATA_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_DCU_BAM_HEAD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_DCU_BAM_DATA_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_DM1_ECU_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_DM1_DCU_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_ATS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_ATR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EEC2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EEC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_ET2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EEC3_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_HOURS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_VH_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_CI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_VI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_ET1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AMB_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_LFC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_LFE_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_VEP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_DPFC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AT1S_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EBC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_TFAC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_SEP1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EOI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AT1IMG_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AT1OG2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AT1IG2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_EFLP2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_SHUTDOWN_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_IC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_A1SCRDSI1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_A1SCREGT1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_A1DEFT1I_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_A1DEFI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_AT1T1I_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
/*************************************************************************************************************
*********************************************   Yanmar Original   ********************************************
**************************************************************************************************************
**************************************************************************************************************
**************************************************************************************************************
**************************************************************************************************************
**************************************************************************************************************
**************************************************************************************************************
**************************************************************************************************************
*************************************************************************************************************/
void J1939_YECR1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YECACK1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YIOS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YLF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YEC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSTP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YRSS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSRF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSRSI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YESI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YDPFIF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YEGRP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YETVP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YATF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YPMD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YDPFC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YPOI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YDPFC2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YEST_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YMPR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YPOFS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YEOM_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YINJQ_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRST_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRSTL1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRSTL2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRMOD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRCTL3_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRHTR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRSO_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);
void J1939_YSCRIS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data);

#ifdef __cplusplus
}
#endif

#endif /* __J1939_H */