
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
  
/* Defines -------------------------------------------------------------------*/
#define CAN_CHANNEL_1                   1
#define CAN_CHANNEL_2                   2

#define TYM_COMBINE                     0x01

#define CAN_CHECK_TIME                  2000
#define CAN_TRANSMISSION_TIME           200
#define CAN_AXEL_TIME                   125
#define CAN_SETTINGS_TIME               500
#define CAN_ENGINE_TSC1_TIME            10
#define CAN_ENGINE_ECR1_TIME            50

#define CAN_COMMAND_REQUEST_ID          0x19F00000

#define DIAGNOSTIC_REQUEST_CAN_ID       0x19FF20D0
#define AUGER_SWITCH_BOX_CAN_ID         0x19FFC025
#define GENERAL_SWITCH_BOX_CAN_ID       0x19FFC035
#define INTEGRATED_CONTROLLER_CAN_ID_1  0x19FFA310
#define INTEGRATED_CONTROLLER_CAN_ID_2  0x19FFA311
#define INTEGRATED_CONTROLLER_CAN_ID_3  0x19FFA312
#define INTEGRATED_CONTROLLER_CAN_ID_4  0x19FFA317
#define CUTTING_CONTROLLER_CAN_ID_1     0x19FFA330
#define CUTTING_CONTROLLER_CAN_ID_2     0x19FFA331
#define CUTTING_CONTROLLER_CAN_ID_3     0x19FFA332
#define AUGER_CONTROLLER_CAN_ID         0x19FFA340
#define LSA_CONTROLLER_CAN_ID           0x19FFA350                              // This message is absent for 
#define AXEL_CONTROLLER_CAN_ID          0x19FFA360
#define SENSOR_CAN_ID                   0x19FFA381                              // This message is absent for 
#define HST_CONTROLLER_CAN_ID           0x19FFA390
#define SETUP_MESSAGE_RECEIVE_CAN_ID    0x19FFA329
  
#define METAPANEL_DATA_CAN_ID           0x19FFA320
#define METAPANEL_SETUP_CAN_ID          0x19FFA000
#define DIAGNOSTIC_RESPONSE_CAN_ID      0x19FFD020
#define ENGINE_ECR1_CAN_ID              0x0CFF02FE                              // Engine control request 1        
#define ENGINE_TSC1_CAN_ID              0x0C0000FE                              // Engine control request 1        

typedef union {
  uint8_t data[8];
  struct {
    uint8_t IC_engine_stop_sw                           : 1;
    uint8_t IC_carter_safety_sw                         : 1;
    uint8_t IC_start_safety_sw                          : 1;
    uint8_t IC_driver_restriction_sw                    : 1;
    uint8_t IC_auger_disconnected_sw                    : 1;
    uint8_t IC_auger_connected_sw                       : 1;
    uint8_t IC_reverse_sw                               : 1;
    uint8_t IC_engine_starter_sw                        : 1;
    
    uint8_t IC_tbs_auto_start_sw                        : 1;
    uint8_t res0                                        : 1;
    uint8_t IC_auger_setting_sw                         : 1;
    uint8_t IC_auger_return_sw                          : 1;
    uint8_t IC_tbs_manual_down_sw                       : 1;
    uint8_t IC_tbs_manual_up_sw                         : 1;
    uint8_t IC_tbs_manual_right_down_sw                 : 1;
    uint8_t IC_tbs_manual_right_up_sw                   : 1;
    
    uint8_t IC_lsa_l_sensor_sw                          : 1;
    uint8_t res2                                        : 1;
    uint8_t IC_lsa_h_sensor_sw                          : 1;
    uint8_t IC_lsa_m_sensor_sw                          : 1;
    uint8_t IC_lsa_manual_descent_sw                    : 1;
    uint8_t IC_lsa_manual_rise_sw                       : 1;
    uint8_t IC_yew_sw                                   : 1;
    uint8_t IC_threshing_sw                             : 1;
    
    uint8_t IC_auger_manual_down_sw                     : 1;
    uint8_t IC_auger_manual_up_sw                       : 1;
    uint8_t IC_auger_manual_left_sw                     : 1;
    uint8_t IC_auger_manual_right_sw                    : 1;
    uint8_t IC_auger_emission_blockage_sensor_sw        : 1;
    uint8_t IC_auger_discharge_sw                       : 1;
    uint8_t IC_auger_grain_level_sensor_sw              : 1;    
    uint8_t IC_auger_external_operation_sw              : 1;
    
    uint8_t res3                                        : 1;
    uint8_t IC_emergency_switch_in_cabin                : 1;
    uint8_t res4                                        : 1;
    uint8_t IC_external_discharge_switch_blocking       : 1;
    uint8_t IC_auger_storage_sw                         : 1;
    uint8_t IC_external_discharge_switch_connection     : 1;
    uint8_t IC_auger_up_limit                           : 1;
    uint8_t res5                                        : 1;

    uint8_t res6                                        : 1;
    uint8_t IC_preheat_relay                            : 1;
    uint8_t res7                                        : 3;
    uint8_t IC_auger_emission_connection_relay          : 1;
    uint8_t IC_auger_discharge_blocking_relay           : 1;
    
    uint8_t IC_engine_start_relay                       : 1;
    uint8_t IC_auger_left_turn_output                   : 1;
    uint8_t IC_auger_priority_output                    : 1;
    uint8_t IC_lsa_rising_output                        : 1;
    uint8_t IC_lsa_falling_output                       : 1;
    uint8_t IC_engine_stop_relay                        : 1;
    uint8_t IC_auger_lift_limit_sw                      : 1;
    uint8_t IC_stop_light_sw                            : 1;
    uint8_t IC_auger_automatic_return_sw                : 1;
    
    uint8_t IC_auger_up_output                          : 1;
    uint8_t IC_auger_down_output                        : 1;
    uint8_t IC_tbs_left_up_output                       : 1;
    uint8_t IC_tbs_left_down_output                     : 1;
    uint8_t IC_tbs_up_output                            : 1;
    uint8_t IC_tbs_down_output                          : 1;
    uint8_t res8                                        : 1;
    uint8_t IC_auger_remote_control_opeation            : 1;
  };
} dataCAN310_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t IC_auger_setting_deal_msb                   : 2;
    uint8_t IC_pick1_thrising_bin_sensor_val_msb        : 2;
    uint8_t IC_pick2_treatment_sensor_val_msb           : 2;
    uint8_t IC_pick3_treatment_sensor_val_msb           : 2;

    uint8_t IC_auger_setting_deal_lsb                   : 8;
    uint8_t IC_pick1_thrising_bin_sensor_val_lsb        : 8;
    uint8_t IC_pick2_treatment_sensor_val_lsb           : 8;
    uint8_t IC_pick3_treatment_sensor_val_lsb           : 8;

    uint8_t IC_tbs_tilt_sensor_error                    : 1;
    uint8_t IC_tbs_grage_right_sensor_error             : 1;
    uint8_t IC_tbs_grage_left_sensor_error              : 1;
    uint8_t IC_tbs_manual_sw_error                      : 1;
    uint8_t IC_engine_emergency_stop_error              : 1;
    uint8_t IC_auger_manual_sw_error                    : 1;
    uint8_t IC_auger_setting_return_sw_error            : 1;
    uint8_t IC_engine_stop_cart_safety_sw               : 1;
    
    uint8_t IC_lsa_manual_sw_error                      : 1;
    uint8_t IC_lsa_m_h_sensor_error                     : 1;
    uint8_t IC_pick1_threshing_sensor_alarm             : 1;                    // 2번나선 경고
    uint8_t IC_pick2_sensor_alarm                       : 1;                    // 양곡나선경고
    uint8_t IC_pick3_processing_sensor_alarm            : 1;
    uint8_t IC_grain_discharge_alarm                    : 1;
    uint8_t IC_grain_clogging_alarm                     : 1;    
    uint8_t IC_auger_discharge_motor_alarm              : 1;
    
    uint8_t IC_engineStop_emergencyStop                 : 1;
    uint8_t IC_engineStop_chipbechulBlockage            : 1;
    uint8_t IC_engineStop_yeacheBlockage                : 1;
    uint8_t IC_engineStop_gugmulManyang                 : 1;
    uint8_t IC_engineStop_cutSafety                     : 1;
    uint8_t IC_engineStop_chuhenController              : 1;
    uint8_t IC_res0                                     : 2;    
    
//    uint8_t IC_setup_mode                               : 3;
//    uint8_t IC_setup_mode_error                         : 1;
//    uint8_t IC_setup_val                                : 4;
  };
} dataCAN311_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t IC_inclination_msb                          : 2;
    uint8_t IC_inclination_deal_msb                     : 2;
    uint8_t res0                                        : 2;
    uint8_t IC_garage_right_sensor_msb                  : 2;
    
    uint8_t IC_inclination_lsb                          : 8;
    uint8_t IC_inclination_deal_lsb                     : 8;
    uint8_t res1                                        : 8;
    uint8_t IC_garage_right_sensor_lsb                  : 8;

    uint8_t IC_garage_left_sensor_msb                   : 2;
    uint8_t IC_engine_temperature_msb                   : 2;
    uint8_t res2                                        : 4;
    
    uint8_t IC_garage_left_sensor_lsb                   : 8;
    uint8_t IC_engine_temperature_lsb                   : 8;    
  };
} dataCAN312_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t IC_tbs_right_sensor_minimum_setting_msb     : 8;
    uint8_t IC_tbs_right_sensor_minimum_setting_lsb     : 8;
    
    uint8_t IC_tbs_left_sensor_minimum_setting_msb      : 8;
    uint8_t IC_tbs_left_sensor_minimum_setting_lsb      : 8;
    
    uint8_t res0                                        : 8;
    uint8_t res1                                        : 8;
    uint8_t res2                                        : 8;
    uint8_t res3                                        : 8;
  };
} dataCAN317_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t res0                                        : 1;
    uint8_t CC_threshing_clutch_blocking_sw             : 1;
    uint8_t CC_threshing_clutch_connection_sw           : 1;
    uint8_t CC_yeache_clutch_blocking_sw                : 1;
    uint8_t CC_yeache_clutch_connection_sw              : 1;
    uint8_t res1                                        : 3;

    uint8_t res2                                        : 3;
    uint8_t CC_cutting_clutch_sw                        : 1;
    uint8_t CC_clutch_mode_manual                       : 1;
    uint8_t CC_clutch_mode_automatic                    : 1;
    uint8_t CC_one_touch_down                           : 1;
    uint8_t CC_one_touch_up                             : 1;
    
    uint8_t CC_quick_sw                                 : 1;
    uint8_t CC_quick_clutch_connection                  : 1;
    uint8_t CC_quick_clutch_cut_off                     : 1;
    uint8_t CC_harvesting_down_sw                       : 1;
    uint8_t CC_harvesting_up_sw                         : 1;
    uint8_t CC_unloader_SOL_operation                   : 1;
    uint8_t res3                                        : 2;
    
    uint8_t CC_cutting_clutch_blocking_relay            : 1;
    uint8_t CC_threshing_clutch_blocking_relay          : 1;
    uint8_t CC_cutting_clutch_connection_relay          : 1;
    uint8_t CC_threshing_clutch_connection_relay        : 1;
    uint8_t CC_cutting_clutch_lamp                      : 1;
    uint8_t CC_auto_lift_lamp                           : 1;
    uint8_t CC_automatic_cutting_lamp                   : 1;
    uint8_t CC_threshing_clutch_lamp                    : 1;

    uint8_t res4                                        : 1;
    uint8_t CC_cutting_up_output                        : 1;
    uint8_t CC_cutting_down_output                      : 1;
    uint8_t CC_load_breaking_relay                      : 1;
    uint8_t CC_clean_motor_relay                        : 1;
    uint8_t CC_steering_left_SOL                        : 1;
    uint8_t CC_steering_right_SOL                       : 1;
    uint8_t res5                                        : 1;
    
    uint8_t CC_lift_sensor_error                        : 1;
    uint8_t CC_notice_sensor_error                      : 1;
    uint8_t CC_manual_sw_error                          : 1;
    uint8_t CC_threshing_clutch_motor_sw_error          : 1;
    uint8_t CC_cutting_clutch_motor_sw_error            : 1;
    uint8_t CC_one_touch_sw_error                       : 1;
    uint8_t res6                                        : 2;
    
    uint8_t CC_cutting_clogging_sensor                  : 1;
    uint8_t CC_amount_of_grain                          : 1;
    uint8_t CC_conter_safety                            : 1;
    uint8_t res7                                        : 5;
    
    uint8_t CC_cutting_setup_mode                       : 3;
    uint8_t res8                                        : 1;
    uint8_t CC_cutting_setup                            : 4;
    
  };
} dataCAN330_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t CC_vehicle_speed_val_msb                    : 2;
    uint8_t CC_pitching_sensor_msb                      : 2;
    uint8_t CC_lift_sensor_val_msb                      : 2;
    uint8_t CC_power_clutch_sensor_msb                  : 2;
    
    uint8_t CC_vehicle_speed_val_lsb                    : 8;
    uint8_t CC_pitching_sensor_lsb                      : 8;
    uint8_t CC_lift_sensor_val_lsb                      : 8;
    uint8_t CC_power_clutch_sensor_lsb                  : 8;

    uint8_t CC_power_clutch_sensor_2                    : 1;
    uint8_t res0                                        : 7;
    
    uint8_t CC_program_version                          : 8;
    
    uint8_t CC_pitching_phase_inversion_condition_2     : 1;
    uint8_t CC_pitching_driving_restrictions            : 1;
    uint8_t CC_pitching_phase_inversion_condition       : 1;
    uint8_t CC_pitching_automatic_not_down              : 1;
    uint8_t CC_pitching_automatic_down                  : 1;
    uint8_t res1                                        : 3;
  };
} dataCAN331_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t res0                                        : 3;
    uint8_t CC_pitching_manual_charge                   : 1;
    uint8_t res1                                        : 1;
    uint8_t CC_pitching_manual_transfer                 : 1;
    uint8_t res2                                        : 2;
    
    uint8_t CC_pitching_cylinder_val_msb                : 2;
    uint8_t CC_cutting_manual_up_down_lever_sensor_msb  : 2;
    uint8_t res3                                        : 4;
    
    uint8_t CC_pitching_cylinder_val_lsb                : 8;
    
    uint8_t CC_cutting_manual_up_down_lever_sensor_lsb  : 8;
    
    uint8_t res4                                        : 8;
    uint8_t res5                                        : 8;
    uint8_t res6                                        : 8;
    uint8_t res7                                        : 8;
  };
} dataCAN332_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t AG_output_left_turn                         : 1;
    uint8_t AG_print_priority                           : 1;
    uint8_t AG_emergency_connection_manual_turn         : 1;
    uint8_t AG_emergency_connection_manual_bypass       : 1;
    uint8_t AG_auger_break_output                       : 1;
    uint8_t AG_auger_rotation_sensor_error              : 1;
    uint8_t AG_auger_limiting_current_error             : 1;
    uint8_t res0                                        : 1;
    
    uint8_t AG_auger_potentiometer_value_msb            : 2;
    uint8_t res1                                        : 6;
    
    uint8_t AG_auger_potentiometer_value_lsb            : 8;
    
    uint8_t AG_auger_motor_output_duty_ratio            : 8;
    
    uint8_t res2                                        : 8;
    
    uint8_t AG_program_version                          : 8;
    
    uint8_t res3                                        : 8;
    uint8_t res4                                        : 8;
  };
} dataCAN340_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t LSA_up_output                               : 1;
    uint8_t LSA_down_output                             : 1;
    uint8_t LSA_emergency_connection_manual_up_sw       : 1;
    uint8_t LSA_emergency_connection_manual_down_sw     : 1;
    uint8_t res0                                        : 1;
    uint8_t LSA_lsa_motor_position_sensor_error         : 1;
    uint8_t LSA_restraint_current_error                 : 1;
    uint8_t res1                                        : 1;
    
    uint8_t LSA_multiturn_position_msb                  : 2;
    uint8_t res2                                        : 6;
    
    uint8_t LSA_multiturn_position_lsb                  : 8;
    
    uint8_t LSA_lsa_motor_output_duty_ratio             : 8;
    uint8_t res3                                        : 8;
    uint8_t LSA_program_version                         : 8;
    
    uint8_t res4                                        : 8;
    uint8_t res5                                        : 8;
  };
} dataCAN350_t;


typedef union {
  uint8_t data[8];
  /*
  struct {
    uint8_t axel_threshing_delay                : 4;
    uint8_t cmd0                                : 4;
    
    
    uint8_t axel_auger_delay                    : 8;
    uint8_t axel_auger_auto_delay               : 8;
    uint8_t axel_yeache_ku_time                 : 8;
    uint8_t axel_tbs_ku_time                    : 8;
    uint8_t axel_auger_ku_time                  : 8;
    uint8_t axel_c_speed_ku_time                : 8;
    uint8_t axel_progVer                        : 8;
  } cmd_0;
  
  struct {
    uint8_t res1                                : 4;
    uint8_t cmdA                                : 4;
    
    uint8_t res2                                : 8;
    
    uint16_t axel_threshing_rpm                 : 16;
    
    uint8_t res3                                : 8;
    uint8_t res4                                : 8;
    uint8_t res5                                : 8;
    uint8_t res6                                : 8;
    
  } cmd_A;
  
  struct {
    uint8_t axel_app_sensor1_position_msb       : 4;
    uint8_t cmdF                                : 4;
    
    uint8_t axel_app_sensor1_position_lsb       : 8;
    
    uint8_t axel_app_sensor2_position_msb       : 4;
    uint8_t res7                                : 4;
    
    uint8_t axel_app_sensor2_position_lsb       : 8;
    
    uint8_t axel_keepTimeMode_msb               : 8;
    uint8_t axel_keepTimeMode_lsb               : 8;
    
    uint8_t axel_delay_mode                     : 4;
    uint8_t res8                                : 4;
    
    uint8_t axel_app_sensor_totalError          : 4;
    uint8_t res9                                : 4;
  } cmd_F;
  */
} dataCAN360_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t res0                                : 8;
    uint8_t res1                                : 8;
    uint8_t res2                                : 8;
    uint8_t res3                                : 8;

    uint8_t selection_mode_position_sensor_msb  : 8;
    uint8_t selection_mode_position_sensor_lsb  : 8;
    
    uint8_t airVolume_mode_position_sensor_msb  : 8;
    uint8_t airVolume_mode_position_sensor_lsb  : 8;
  };
} dataCAN381_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t MC_driving_motor_msb                : 2;
    uint8_t MC_subshift_lever_lsb               : 4;
    uint8_t MC_neutral_flag                     : 1;
    uint8_t MC_steering_motor_level             : 1;
    
    uint8_t MC_driving_motor_lsb                : 8;
    
    uint8_t MC_steering_motor_msb               : 2;
    uint8_t MC_subshift_lever_mid               : 4;
    uint8_t MC_subshift_lever_msb               : 2;
    
    uint8_t MC_steering_motor_lsb               : 8;
    
    uint8_t MC_driveing_lever_msb               : 2;
    uint8_t MC_subshift_mode_flag               : 2;
    uint8_t MC_threshing_load_flag              : 3;
    uint8_t MC_brake_signal_flag                : 1;
    
    uint8_t MC_driveing_lever_lsb               : 8;
    
    uint8_t MC_steering_lever_msb               : 2;
    uint8_t MC_HST_error_code                   : 4;
    //uint8_t MC_HST_chuhen_motor_error;
    //uint8_t MC_HST_chuhyan_motor_error;
    //uint8_t MC_HST_chuhen_lever_error;
    //uint8_t MC_HST_chuhyan_lever_error;
    uint8_t MC_vehicle_zero_flag                : 1;
    uint8_t MC_spinton_signal_flag              : 1;
    
    uint8_t MC_steering_lever_lsb               : 8;
  };
} dataCAN390_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t auger_auto_left_sw          : 1;
    uint8_t auger_auto_right_sw         : 1;
    uint8_t auger_auto_after_sw         : 1;
    uint8_t auger_storage_sw            : 1;
    uint8_t auger_discharge_on_sw       : 1;
    uint8_t auger_discharge_stop_sw     : 1;
    uint8_t auger_manual_up_sw          : 1;
    uint8_t auger_manual_down_sw        : 1;

    uint8_t auger_auto_swing_sw         : 1;
    uint8_t auger_swing_left_sw         : 1;
    uint8_t auger_swing_right_sw        : 1;
    uint8_t auger_manual_right_sw       : 1;
    uint8_t auger_manual_left_sw        : 1;
    uint8_t auger_rotation_stop_sw      : 1;
    uint8_t res0                        : 2;
    
    uint8_t res1                        : 8;

    uint8_t auger_program_version       : 8;
    
    uint8_t res2                        : 8;
    uint8_t res3                        : 8;
    uint8_t res4                        : 8;
    uint8_t res5                        : 8;
  };
} dataCAN025_t;

typedef union {
  uint8_t data[8];
  struct {
    uint8_t res0                                        : 2;
    uint8_t general_auto_left_right_sw                  : 1;
    uint8_t general_auto_forward_backward_sw            : 1;
    uint8_t general_auto_supply_depth_sw                : 1;
    uint8_t general_aircraft_height_fall_sw             : 1;
    uint8_t general_auto_accel_sw_1                     : 1;
    uint8_t res1                                        : 1;
    
    uint8_t general_up_sw                               : 1;
    uint8_t res2                                        : 2;
    uint8_t general_auto_sorting_sw                     : 1;
    uint8_t general_auto_accel_led                      : 1;
    uint8_t res3                                        : 2;
    uint8_t general_auto_sorting_led                    : 1;
    
    uint8_t general_spin_turn_sw                        : 1;
    uint8_t general_auto_accel_sw_2                     : 1;
    uint8_t res4                                        : 1;
    uint8_t general_auto_cutting_height_led             : 1;
    uint8_t general_auto_left_right_led                 : 1;
    uint8_t general_auto_forward_backward_led           : 1;
    uint8_t general_auto_supply_depth_led               : 1;
    uint8_t general_spin_turn_led                       : 1;
    
    uint8_t general_air_volume_control                  : 8;
    uint8_t general_sheave_setting                      : 8;
    uint8_t general_mowing_height_setting               : 8;
    uint8_t general_inclination_angle_adjustment        : 8;
    uint8_t general_program_version                     : 8;
  };
} dataCAN035_t;
    
extern dataCAN310_t dataCAN310;
extern dataCAN311_t dataCAN311;
extern dataCAN312_t dataCAN312;
extern dataCAN317_t dataCAN317;
extern dataCAN330_t dataCAN330;
extern dataCAN331_t dataCAN331;
extern dataCAN332_t dataCAN332;
extern dataCAN340_t dataCAN340;
extern dataCAN350_t dataCAN350;
extern dataCAN360_t dataCAN360;
extern dataCAN381_t dataCAN381;
extern dataCAN390_t dataCAN390;
extern dataCAN025_t dataCAN025;
extern dataCAN035_t dataCAN035;
/*
// CAN RX Buffer for corresponding message
extern uint8_t data_310[8];
extern uint8_t data_311[8];
extern uint8_t data_312[8];
extern uint8_t data_317[8];
extern uint8_t data_330[8];
extern uint8_t data_331[8];
extern uint8_t data_332[8];
extern uint8_t data_340[8];
extern uint8_t data_350[8];
extern uint8_t data_360[8];
extern uint8_t data_381[8]; 
extern uint8_t data_390[8];
*/
// New data variables for CAN Check menu                --> Added on 03 Sep 2020
extern uint16_t can_3111_data[56];
extern uint16_t can_3112_data[19];
extern uint16_t can_3113_data[6];
extern uint16_t can_312_data[9];
extern uint16_t can_313_data[8];
extern uint16_t can_3141_data[35];
extern uint16_t can_3142_data[4];
extern uint16_t can_3143_data[4];

extern uint16_t can_315_data[141];

extern uint16_t can_341_data[12];
extern uint16_t can_351_data[8];
extern uint8_t can_36x_data[1];
extern uint16_t can_361_data[2];
  

extern int16_t new_setup_data_received;
extern int16_t new_setup_data_2_received;
extern uint8_t new_setup_data_received_flag;

// 0x19FFA312 -> Repetation time = 499ms      Integrated Controller
extern uint16_t IC_garage_right_sensor;
extern uint16_t IC_inclination_deal;
extern uint16_t IC_inclination;

extern uint16_t IC_engine_temperature;
extern uint16_t IC_garage_left_sensor;

// 0x19FFA317 -> Repetation time = 310ms      Integrated Controller
extern uint16_t IC_tbs_right_sensor_minimum_setting;
extern uint16_t IC_tbs_left_sensor_minimum_setting;

// 0x19FFA331 -> Repetation time = 179ms      Cutting controller
extern uint16_t CC_power_clutch_sensor;
extern uint16_t CC_lift_sensor_val;
extern uint16_t CC_pitching_sensor;
extern uint16_t CC_vehicle_speed_val;

// 0x19FFA332 -> Repetation time = 234ms      Cutting controller
extern uint16_t CC_pitching_cylinder_val;
extern uint16_t CC_cutting_manual_up_down_lever_sensor;

// 0x19FFA340 -> Repetation time = 7ms        Auger driver related
extern uint16_t AG_auger_potentiometer_value;

// 0x19FFA350 -> Repetation time = 11ms       LSA driver related
extern uint16_t LSA_multiturn_position;

// 0x19FFA360 -> Repetation time = 125ms      ( Axel )
extern uint8_t axel_auger_delay;
extern uint8_t axel_threshing_delay;
extern uint8_t axel_yeache_ku_time;
extern uint8_t axel_auger_auto_delay;
extern uint8_t axel_auger_ku_time;
extern uint8_t axel_tbs_ku_time;
extern uint8_t axel_progVer;
extern uint8_t axel_c_speed_ku_time;
extern uint16_t axel_threshing_rpm;
extern uint16_t axel_app_sensor1_position;
extern uint16_t axel_app_sensor2_position;
extern uint16_t axel_keepTimeMode;
extern uint8_t axel_app_sensor_totalError;
extern uint8_t axel_delay_mode;

extern uint8_t axelControlFunctionEnabled;                                      //

// 0x19FFA381
extern uint16_t selection_mode_position_sensor;
extern uint16_t airVolume_mode_position_sensor;

// 0x19FFA390 -> Repetation time = 300ms      ( Mission Controller )
extern uint16_t MC_subshift_lever;
extern uint16_t MC_driving_motor;
extern uint16_t MC_steering_motor;
extern uint16_t MC_driveing_lever;
extern uint16_t MC_steering_lever;

/*
// 0x19FFC025 -> Repetation time = 20ms         Auger control switch box
extern uint8_t auger_auto_left_sw;
extern uint8_t auger_auto_right_sw;
extern uint8_t auger_auto_after_sw;
extern uint8_t auger_storage_sw;
extern uint8_t auger_discharge_on_sw;
extern uint8_t auger_discharge_stop_sw;
extern uint8_t auger_manual_up_sw;
extern uint8_t auger_manual_down_sw;

extern uint8_t auger_auto_swing_sw;
extern uint8_t auger_swing_left_sw;
extern uint8_t auger_swing_right_sw;
extern uint8_t auger_manual_right_sw;
extern uint8_t auger_manual_left_sw;
extern uint8_t auger_rotation_stop_sw;

extern uint8_t auger_program_version;

// 0x19FFC035 -> Repetation time = 20ms         Operation switch box
extern uint8_t general_auto_accel_sw_1;
extern uint8_t general_aircraft_height_fall_sw;
extern uint8_t general_auto_supply_depth_sw;
extern uint8_t general_auto_forward_backward_sw;
extern uint8_t general_auto_left_right_sw;

extern uint8_t general_auto_sorting_led;
extern uint8_t general_auto_accel_led;
extern uint8_t general_auto_sorting_sw;
extern uint8_t general_up_sw;

extern uint8_t general_spin_turn_led;
extern uint8_t general_auto_supply_depth_led;
extern uint8_t general_auto_forward_backward_led;
extern uint8_t general_auto_left_right_led;
extern uint8_t general_auto_cutting_height_led;
extern uint8_t general_auto_accel_sw_2;
extern uint8_t general_spin_turn_sw;

extern uint8_t general_air_volume_control;
extern uint8_t general_sheave_setting;
extern uint8_t general_mowing_height_setting;
extern uint8_t general_inclination_angle_adjustment;
extern uint8_t general_program_version;

// 0x19FFA310 -> Repetation time = 13ms       Integrated Controller
extern uint8_t IC_engine_starter_sw;
extern uint8_t IC_reverse_sw;
extern uint8_t IC_auger_connected_sw;
extern uint8_t IC_auger_disconnected_sw;
extern uint8_t IC_driver_restriction_sw;
extern uint8_t IC_start_safety_sw;
extern uint8_t IC_carter_safety_sw;
extern uint8_t IC_engine_stop_sw;

extern uint8_t IC_tbs_manual_rising_sw;
extern uint8_t IC_tbs_manual_rainfall_sw;
extern uint8_t IC_auger_return_sw;
extern uint8_t IC_tbs_auto_start_sw;

extern uint8_t IC_threshing_sw;
extern uint8_t IC_yew_sw;
extern uint8_t IC_lsa_manual_rise_sw;
extern uint8_t IC_lsa_manual_descent_sw;
extern uint8_t IC_lsa_m_sensor_sw;
extern uint8_t IC_lsa_h_sensor_sw;
extern uint8_t IC_lsa_l_sensor_sw;

extern uint8_t IC_auger_external_operation_sw;
extern uint8_t IC_auger_grain_level_sensor_sw;
extern uint8_t IC_auger_discharge_sw;
extern uint8_t IC_auger_emission_blockage_sensor_sw;
extern uint8_t IC_auger_manual_right_sw;
extern uint8_t IC_auger_manual_left_sw;
extern uint8_t IC_auger_manual_up_sw;
extern uint8_t IC_auger_manual_down_sw;
        
extern uint8_t IC_emergency_switch_in_cabin;
extern uint8_t IC_auger_lift_limit_sw;
extern uint8_t IC_external_discharge_switch_connection;
extern uint8_t IC_auger_remote_control_opeation;
extern uint8_t IC_auger_storage_sw;
extern uint8_t IC_external_discharge_switch_blocking;
        
extern uint8_t IC_auger_discharge_blocking_relay;

extern uint8_t IC_auger_emission_connection_relay;
extern uint8_t IC_preheat_relay;

extern uint8_t IC_auger_automatic_return_sw;
extern uint8_t IC_stop_light_sw;
extern uint8_t IC_engine_stop_relay;
extern uint8_t IC_lsa_falling_output;
extern uint8_t IC_lsa_rising_output;
extern uint8_t IC_auger_priority_output;
extern uint8_t IC_auger_left_turn_output;
extern uint8_t IC_engine_start_relay;
    
extern uint8_t IC_tbs_down_output;
extern uint8_t IC_tbs_up_output;
extern uint8_t IC_tbs_left_down_output;
extern uint8_t IC_tbs_left_up_output;
extern uint8_t IC_auger_down_output;
extern uint8_t IC_auger_up_output;
    
// 0x19FFA311 -> Repetation time = 139ms      Integrated Controller
extern uint16_t IC_auger_setting_deal;
extern uint16_t IC_pick1_thrising_bin_sensor_val;
extern uint16_t IC_pick2_treatment_sensor_val;
extern uint16_t IC_pick3_treatment_sensor_val;

extern uint8_t IC_engine_stop_safety_sw;
extern uint8_t IC_auger_setting_return_sw_error;
extern uint8_t IC_auger_manual_sw_error;
extern uint8_t IC_engine_emergency_stop_error;
extern uint8_t IC_tbs_manual_sw_error;
extern uint8_t IC_tbs_grage_left_sensor_error;
extern uint8_t IC_tbs_grage_right_sensor_error;
extern uint8_t IC_tbs_tilt_sensor_error;
    
extern uint8_t IC_auger_discharge_motor_alarm;
extern uint8_t IC_grain_clogging_alarm;
extern uint8_t IC_grain_discharge_alarm;
extern uint8_t IC_pick3_processing_sensor_alarm;
extern uint8_t IC_pick2_sensor_alarm;
extern uint8_t IC_pick1_threshing_sensor_alarm;
extern uint8_t IC_lsa_m_h_sensor_error;
extern uint8_t IC_lsa_manual_sw_error;

extern uint8_t IC_setup_val;
extern uint8_t IC_setup_mode_error;
extern uint8_t IC_setup_mode;
    
// 0x19FFA312 -> Repetation time = 499ms      Integrated Controller
extern uint16_t IC_garage_right_sensor;
extern uint16_t IC_inclination_deal;
extern uint16_t IC_inclination;

extern uint16_t IC_engine_temperature;
extern uint16_t IC_garage_left_sensor;

// 0x19FFA317 -> Repetation time = 310ms      Integrated Controller
extern uint16_t IC_tbs_right_sensor_minimum_setting;
extern uint16_t IC_tbs_left_sensor_minimum_setting;

// 0x19FFA330 -> Repetation time = 19ms       Cutting controller connection
extern uint8_t CC_threshing_clutch_connection_sw;
extern uint8_t CC_threshing_clutch_blocking_sw;

extern uint8_t CC_one_touch_up;
extern uint8_t CC_one_touch_down;
extern uint8_t CC_clutch_mode_automatic;
extern uint8_t CC_clutch_mode_manual;
extern uint8_t CC_cutting_clutch_sw;

extern uint8_t CC_unloader_SOL_operation;
extern uint8_t CC_harvesting_up_sw;
extern uint8_t CC_harvesting_down_sw;
extern uint8_t CC_quick_clutch_cut_off;
extern uint8_t CC_quick_clutch_connection;
extern uint8_t CC_quick_sw;

extern uint8_t CC_threshing_clutch_lamp;
extern uint8_t CC_automatic_cutting_lamp;
extern uint8_t CC_auto_lift_lamp;
extern uint8_t CC_cutting_clutch_lamp;
extern uint8_t CC_threshing_clutch_connection_relay;
extern uint8_t CC_cutting_clutch_connection_relay;
extern uint8_t CC_threshing_clutch_blocking_relay;
extern uint8_t CC_cutting_clutch_blocking_relay;

extern uint8_t CC_steering_right_SOL;
extern uint8_t CC_steering_left_SOL;
extern uint8_t CC_clean_motor_relay;
extern uint8_t CC_load_breaking_relay;
extern uint8_t CC_cutting_down_output;
extern uint8_t CC_cutting_up_output;

extern uint8_t CC_one_touch_sw_error;
extern uint8_t CC_cutting_clutch_motor_sw_error;
extern uint8_t CC_threshing_clutch_motor_sw_error;
extern uint8_t CC_manual_sw_error;
extern uint8_t CC_notice_sensor_error;
extern uint8_t CC_lift_sensor_error;

extern uint8_t CC_conter_safety;
extern uint8_t CC_amount_of_grain;
extern uint8_t CC_cutting_clogging_sensor;

extern uint8_t CC_cutting_setup;
extern uint8_t CC_cutting_setup_mode;

// 0x19FFA331 -> Repetation time = 179ms      Cutting controller
extern uint16_t CC_power_clutch_sensor;
extern uint16_t CC_lift_sensor_val;
extern uint16_t CC_pitching_sensor;
extern uint16_t CC_vehicle_speed_val;

extern uint8_t CC_power_clutch_sensor_2;
extern uint8_t CC_program_version;
extern uint8_t CC_pitching_automatic_down;
extern uint8_t CC_pitching_automatic_not_down;
extern uint8_t CC_pitching_phase_inversion_condition;
extern uint8_t CC_pitching_driving_restrictions;
extern uint8_t CC_pitching_phase_inversion_condition_2;

// 0x19FFA332 -> Repetation time = 234ms      Cutting controller
extern uint16_t CC_pitching_manual_charge;
extern uint16_t CC_pitching_manual_transfer;
extern uint16_t CC_pitching_cylinder_val;
extern uint16_t CC_cutting_manual_up_down_lever_sensor;

// 0x19FFA340 -> Repetation time = 7ms        Auger driver related
extern uint8_t AG_auger_limiting_current_error;
extern uint8_t AG_auger_rotation_sensor_error;
extern uint8_t AG_auger_break_output;
extern uint8_t AG_emergency_connection_manual_bypass;
extern uint8_t AG_emergency_connection_manual_turn;
extern uint8_t AG_print_priority;
extern uint8_t AG_output_left_turn;

extern uint16_t AG_auger_potentiometer_value;
extern uint8_t AG_auger_motor_output_duty_ratio;
extern uint8_t AG_program_version;

// 0x19FFA350 -> Repetation time = 11ms       LSA driver related
extern uint8_t LSA_restraint_current_error;
extern uint8_t LSA_lsa_motor_position_sensor_error;
extern uint8_t LSA_emergency_connection_manual_down_sw;
extern uint8_t LSA_emergency_connection_manual_up_sw;
extern uint8_t LSA_down_output;
extern uint8_t LSA_up_output;

extern uint16_t LSA_multiturn_position;
extern uint8_t LSA_lsa_motor_output_duty_ratio;
extern uint8_t LSA_program_version;

// 0x19FFA360 -> Repetation time = 125ms      ( Axel )
extern uint8_t axel_auger_delay;
extern uint8_t axel_threshing_delay;
extern uint8_t axel_yeache_ku_time;
extern uint8_t axel_auger_auto_delay;
extern uint8_t axel_auger_ku_time;
extern uint8_t axel_tbs_ku_time;
extern uint8_t axel_progVer;
extern uint8_t axel_c_speed_ku_time;
extern uint16_t axel_threshing_rpm;
extern uint16_t axel_app_sensor1_position;
extern uint16_t axel_app_sensor2_position;
extern uint16_t axel_keepTimeMode;
extern uint8_t axel_app_sensor_totalError;
extern uint8_t axel_delay_mode;

// 0x19FFA381
extern uint16_t selection_mode_position_sensor;
extern uint16_t airVolume_mode_position_sensor;

// 0x19FFA390 -> Repetation time = 300ms      ( Mission Controller )
extern uint8_t MC_steering_motor_level;
extern uint8_t MC_neutral_flag;
extern uint16_t MC_subshift_lever;
extern uint16_t MC_driving_motor;
extern uint16_t MC_steering_motor;
extern uint8_t MC_brake_signal_flag;
extern uint8_t MC_threshing_load_flag;
extern uint8_t MC_subshift_mode_flag;
extern uint16_t MC_driveing_lever;

extern uint8_t MC_spinton_signal_flag;
extern uint8_t MC_vehicle_zero_flag;
extern uint8_t MC_HST_error_code;
extern uint8_t MC_HST_chuhen_motor_error;
extern uint8_t MC_HST_chuhyan_motor_error;
extern uint8_t MC_HST_chuhen_lever_error;
extern uint8_t MC_HST_chuhyan_lever_error;
extern uint16_t MC_steering_lever;
*/
typedef union
{
  uint32_t data;
  struct {
    uint32_t diagnosticRequest          : 1;
    uint32_t diagnosticResponse         : 1;
    uint32_t res                        : 30;
  };
} flagCan_t;

void can_init();
void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

void can_receive_process();
void can_transmit_process();
void can_transmit_process_engine_line();

uint8_t can_transmit(uint8_t channel, uint32_t id, uint8_t _data[]);
void can_start();
#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
