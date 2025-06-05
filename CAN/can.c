#include "can.h"
#include "input.h"
#include "J1939.h"
#include "Yvc1_HAL.h"

/* --------------------------------------------------------- STM32 CAN variables ---------------------------------------------------*/
CAN_HandleTypeDef       hcan1;
CAN_HandleTypeDef       hcan2;

/* --------------------------------------------------------- CAN variables ---------------------------------------------------------*/
CAN_TxHeaderTypeDef     canOneTxHeader;
CAN_RxHeaderTypeDef     canOneRxHeader;
uint8_t                 canOneTxData[8];
uint8_t                 canOneRxData[8];
uint32_t                canOneTxMailbox;

CAN_TxHeaderTypeDef     canTwoTxHeader;
CAN_RxHeaderTypeDef     canTwoRxHeader;
uint8_t                 canTwoTxData[8];
uint8_t                 canTwoRxData[8];
uint32_t                canTwoTxMailbox;
int16_t                 canErrorCounter[2];

flagCan_t               flagCan;

dataCAN310_t dataCAN310;
dataCAN311_t dataCAN311;
dataCAN312_t dataCAN312;
dataCAN317_t dataCAN317;
dataCAN330_t dataCAN330;
dataCAN331_t dataCAN331;
dataCAN332_t dataCAN332;
dataCAN340_t dataCAN340;
dataCAN350_t dataCAN350;
dataCAN360_t dataCAN360;
dataCAN381_t dataCAN381;
dataCAN390_t dataCAN390;
dataCAN025_t dataCAN025;
dataCAN035_t dataCAN035;

// 0x19FFA311 -> Repetation time = 139ms      Integrated Controller
uint16_t IC_auger_setting_deal;
uint16_t IC_pick1_thrising_bin_sensor_val;
uint16_t IC_pick2_treatment_sensor_val;
uint16_t IC_pick3_treatment_sensor_val;

// 0x19FFA312 -> Repetation time = 499ms      Integrated Controller
uint16_t IC_garage_right_sensor;
uint16_t IC_inclination_deal;
uint16_t IC_inclination;

uint16_t IC_engine_temperature;
uint16_t IC_garage_left_sensor;

// 0x19FFA317 -> Repetation time = 310ms      Integrated Controller
uint16_t IC_tbs_right_sensor_minimum_setting;
uint16_t IC_tbs_left_sensor_minimum_setting;

// 0x19FFA332 -> Repetation time = 234ms      Cutting controller

uint16_t CC_pitching_cylinder_val;
uint16_t CC_cutting_manual_up_down_lever_sensor;

// 0x19FFA331 -> Repetation time = 179ms      Cutting controller
uint16_t CC_power_clutch_sensor;
uint16_t CC_lift_sensor_val;
uint16_t CC_pitching_sensor;
uint16_t CC_vehicle_speed_val;

// 0x19FFA340 -> Repetation time = 7ms        Auger driver related
uint16_t AG_auger_potentiometer_value;

// 0x19FFA350 -> Repetation time = 11ms       LSA driver related
uint16_t LSA_multiturn_position;

// 0x19FFA360 -> Repetation time = 125ms      Axel
uint8_t axel_auger_delay;
uint8_t axel_threshing_delay;
uint8_t axel_yeache_ku_time;
uint8_t axel_auger_auto_delay;
uint8_t axel_auger_ku_time;
uint8_t axel_tbs_ku_time;
uint8_t axel_progVer;
uint8_t axel_c_speed_ku_time;
uint16_t axel_threshing_rpm;
uint16_t axel_app_sensor1_position;
uint16_t axel_app_sensor2_position;
uint16_t axel_keepTimeMode;
uint8_t axel_app_sensor_totalError;
uint8_t axel_delay_mode;

uint8_t axelControlFunctionEnabled;                                             // Added on 2024.09.26

// 0x19FFA381
uint16_t selection_mode_position_sensor;
uint16_t airVolume_mode_position_sensor;

// 0x19FFA390 -> Repetation time = 300ms        Mission Controller
uint16_t MC_subshift_lever;
uint16_t MC_driving_motor;
uint16_t MC_steering_motor;
uint16_t MC_driveing_lever;
uint16_t MC_steering_lever;

uint8_t data_diagnostic[8];
uint8_t data_diagnostic_t[8];

int16_t new_setup_data_received;
int16_t new_setup_data_2_received;
uint8_t new_setup_data_received_flag;
/*
// 0x19FFC025 -> Repetation time = 20ms         Auger control switch box
uint8_t auger_auto_left_sw;
uint8_t auger_auto_right_sw;
uint8_t auger_auto_after_sw;
uint8_t auger_storage_sw;
uint8_t auger_discharge_on_sw;
uint8_t auger_discharge_stop_sw;
uint8_t auger_manual_up_sw;
uint8_t auger_manual_down_sw;

uint8_t auger_auto_swing_sw;
uint8_t auger_swing_left_sw;
uint8_t auger_swing_right_sw;
uint8_t auger_manual_right_sw;
uint8_t auger_manual_left_sw;
uint8_t auger_rotation_stop_sw;

uint8_t auger_program_version;

// 0x19FFC035 -> Repetation time = 20ms         Operation switch box
uint8_t general_auto_accel_sw_1;
uint8_t general_aircraft_height_fall_sw;
uint8_t general_auto_supply_depth_sw;
uint8_t general_auto_forward_backward_sw;
uint8_t general_auto_left_right_sw;

uint8_t general_auto_sorting_led;
uint8_t general_auto_accel_led;
uint8_t general_auto_sorting_sw;
uint8_t general_up_sw;

uint8_t general_spin_turn_led;
uint8_t general_auto_supply_depth_led;
uint8_t general_auto_forward_backward_led;
uint8_t general_auto_left_right_led;
uint8_t general_auto_cutting_height_led;
uint8_t general_auto_accel_sw_2;
uint8_t general_spin_turn_sw;

uint8_t general_air_volume_control;
uint8_t general_sheave_setting;
uint8_t general_mowing_height_setting;
uint8_t general_inclination_angle_adjustment;
uint8_t general_program_version;
    
// 0x19FFA310 -> Repetation time = 13ms       Integrated Controller
uint8_t IC_engine_starter_sw;
uint8_t IC_reverse_sw;
uint8_t IC_auger_connected_sw;
uint8_t IC_auger_disconnected_sw;
uint8_t IC_driver_restriction_sw;
uint8_t IC_start_safety_sw;
uint8_t IC_carter_safety_sw;
uint8_t IC_engine_stop_sw;

uint8_t IC_tbs_manual_rising_sw;
uint8_t IC_tbs_manual_rainfall_sw;
uint8_t IC_auger_return_sw;
uint8_t IC_tbs_auto_start_sw;

uint8_t IC_threshing_sw;
uint8_t IC_yew_sw;
uint8_t IC_lsa_manual_rise_sw;
uint8_t IC_lsa_manual_descent_sw;
uint8_t IC_lsa_m_sensor_sw;
uint8_t IC_lsa_h_sensor_sw;
uint8_t IC_lsa_l_sensor_sw;

uint8_t IC_auger_external_operation_sw;
uint8_t IC_auger_grain_level_sensor_sw;
uint8_t IC_auger_discharge_sw;
uint8_t IC_auger_emission_blockage_sensor_sw;
uint8_t IC_auger_manual_right_sw;
uint8_t IC_auger_manual_left_sw;
uint8_t IC_auger_manual_up_sw;
uint8_t IC_auger_manual_down_sw;
        
uint8_t IC_emergency_switch_in_cabin;
uint8_t IC_auger_lift_limit_sw;
uint8_t IC_external_discharge_switch_connection;
uint8_t IC_auger_remote_control_opeation;
uint8_t IC_auger_storage_sw;
uint8_t IC_external_discharge_switch_blocking;
        
uint8_t IC_auger_discharge_blocking_relay;

uint8_t IC_auger_emission_connection_relay;
uint8_t IC_preheat_relay;

uint8_t IC_auger_automatic_return_sw;
uint8_t IC_stop_light_sw;
uint8_t IC_engine_stop_relay;
uint8_t IC_lsa_falling_output;
uint8_t IC_lsa_rising_output;
uint8_t IC_auger_priority_output;
uint8_t IC_auger_left_turn_output;
uint8_t IC_engine_start_relay;
    
uint8_t IC_tbs_down_output;
uint8_t IC_tbs_up_output;
uint8_t IC_tbs_left_down_output;
uint8_t IC_tbs_left_up_output;
uint8_t IC_auger_down_output;
uint8_t IC_auger_up_output;
    
// 0x19FFA311 -> Repetation time = 139ms      Integrated Controller
uint16_t IC_auger_setting_deal;
uint16_t IC_pick1_thrising_bin_sensor_val;
uint16_t IC_pick2_treatment_sensor_val;
uint16_t IC_pick3_treatment_sensor_val;

uint8_t IC_engine_stop_safety_sw;
uint8_t IC_auger_setting_return_sw_error;
uint8_t IC_auger_manual_sw_error;
uint8_t IC_engine_emergency_stop_error;
uint8_t IC_tbs_manual_sw_error;
uint8_t IC_tbs_grage_left_sensor_error;
uint8_t IC_tbs_grage_right_sensor_error;
uint8_t IC_tbs_tilt_sensor_error;
    
uint8_t IC_auger_discharge_motor_alarm;
uint8_t IC_grain_clogging_alarm;
uint8_t IC_grain_discharge_alarm;
uint8_t IC_pick3_processing_sensor_alarm;
uint8_t IC_pick2_sensor_alarm;
uint8_t IC_pick1_threshing_sensor_alarm;
uint8_t IC_lsa_m_h_sensor_error;
uint8_t IC_lsa_manual_sw_error;

uint8_t IC_setup_val;
uint8_t IC_setup_mode_error;
uint8_t IC_setup_mode;
    
// 0x19FFA312 -> Repetation time = 499ms      Integrated Controller
uint16_t IC_garage_right_sensor;
uint16_t IC_inclination_deal;
uint16_t IC_inclination;

uint16_t IC_engine_temperature;
uint16_t IC_garage_left_sensor;

// 0x19FFA317 -> Repetation time = 310ms      Integrated Controller
uint16_t IC_tbs_right_sensor_minimum_setting;
uint16_t IC_tbs_left_sensor_minimum_setting;

// 0x19FFA330 -> Repetation time = 19ms       Cutting controller connection
uint8_t CC_threshing_clutch_connection_sw;
uint8_t CC_threshing_clutch_blocking_sw;

uint8_t CC_one_touch_up;
uint8_t CC_one_touch_down;
uint8_t CC_clutch_mode_automatic;
uint8_t CC_clutch_mode_manual;
uint8_t CC_cutting_clutch_sw;

uint8_t CC_unloader_SOL_operation;
uint8_t CC_harvesting_up_sw;
uint8_t CC_harvesting_down_sw;
uint8_t CC_quick_clutch_cut_off;
uint8_t CC_quick_clutch_connection;
uint8_t CC_quick_sw;

uint8_t CC_threshing_clutch_lamp;
uint8_t CC_automatic_cutting_lamp;
uint8_t CC_auto_lift_lamp;
uint8_t CC_cutting_clutch_lamp;
uint8_t CC_threshing_clutch_connection_relay;
uint8_t CC_cutting_clutch_connection_relay;
uint8_t CC_threshing_clutch_blocking_relay;
uint8_t CC_cutting_clutch_blocking_relay;

uint8_t CC_steering_right_SOL;
uint8_t CC_steering_left_SOL;
uint8_t CC_clean_motor_relay;
uint8_t CC_load_breaking_relay;
uint8_t CC_cutting_down_output;
uint8_t CC_cutting_up_output;

uint8_t CC_one_touch_sw_error;
uint8_t CC_cutting_clutch_motor_sw_error;
uint8_t CC_threshing_clutch_motor_sw_error;
uint8_t CC_manual_sw_error;
uint8_t CC_notice_sensor_error;
uint8_t CC_lift_sensor_error;

uint8_t CC_conter_safety;
uint8_t CC_amount_of_grain;
uint8_t CC_cutting_clogging_sensor;

uint8_t CC_cutting_setup;
uint8_t CC_cutting_setup_mode;

// 0x19FFA331 -> Repetation time = 179ms      Cutting controller
uint16_t CC_power_clutch_sensor;
uint16_t CC_lift_sensor_val;
uint16_t CC_pitching_sensor;
uint16_t CC_vehicle_speed_val;

uint8_t CC_power_clutch_sensor_2;
uint8_t CC_program_version;
uint8_t CC_pitching_automatic_down;
uint8_t CC_pitching_automatic_not_down;
uint8_t CC_pitching_phase_inversion_condition;
uint8_t CC_pitching_driving_restrictions;
uint8_t CC_pitching_phase_inversion_condition_2;

// 0x19FFA332 -> Repetation time = 234ms      Cutting controller
uint16_t CC_pitching_manual_charge;
uint16_t CC_pitching_manual_transfer;
uint16_t CC_pitching_cylinder_val;
uint16_t CC_cutting_manual_up_down_lever_sensor;

// 0x19FFA340 -> Repetation time = 7ms        Auger driver related
uint8_t AG_auger_limiting_current_error;
uint8_t AG_auger_rotation_sensor_error;
uint8_t AG_auger_break_output;
uint8_t AG_emergency_connection_manual_bypass;
uint8_t AG_emergency_connection_manual_turn;
uint8_t AG_print_priority;
uint8_t AG_output_left_turn;

uint16_t AG_auger_potentiometer_value;
uint8_t AG_auger_motor_output_duty_ratio;
uint8_t AG_program_version;


// 0x19FFA350 -> Repetation time = 11ms       LSA driver related
uint8_t LSA_restraint_current_error;
uint8_t LSA_lsa_motor_position_sensor_error;
uint8_t LSA_emergency_connection_manual_down_sw;
uint8_t LSA_emergency_connection_manual_up_sw;
uint8_t LSA_down_output;
uint8_t LSA_up_output;

uint16_t LSA_multiturn_position;
uint8_t LSA_lsa_motor_output_duty_ratio;
uint8_t LSA_program_version;

// 0x19FFA360 -> Repetation time = 125ms      Axel
uint8_t axel_auger_delay;
uint8_t axel_threshing_delay;
uint8_t axel_yeache_ku_time;
uint8_t axel_auger_auto_delay;
uint8_t axel_auger_ku_time;
uint8_t axel_tbs_ku_time;
uint8_t axel_progVer;
uint8_t axel_c_speed_ku_time;
uint16_t axel_threshing_rpm;
uint16_t axel_app_sensor1_position;
uint16_t axel_app_sensor2_position;
uint16_t axel_keepTimeMode;
uint8_t axel_app_sensor_totalError;
uint8_t axel_delay_mode;

// 0x19FFA381
uint16_t selection_mode_position_sensor;
uint16_t airVolume_mode_position_sensor;

// 0x19FFA390 -> Repetation time = 300ms        Mission Controller
uint8_t MC_steering_motor_level;
uint8_t MC_neutral_flag;
uint16_t MC_subshift_lever;
uint16_t MC_driving_motor;
uint16_t MC_steering_motor;
uint8_t MC_brake_signal_flag;
uint8_t MC_threshing_load_flag;
uint8_t MC_subshift_mode_flag;
uint16_t MC_driveing_lever;

uint8_t MC_spinton_signal_flag;
uint8_t MC_vehicle_zero_flag;
uint8_t MC_HST_error_code;
uint8_t MC_HST_chuhen_motor_error;
uint8_t MC_HST_chuhyan_motor_error;
uint8_t MC_HST_chuhen_lever_error;
uint8_t MC_HST_chuhyan_lever_error;
uint16_t MC_steering_lever;
*/
/*
// Last received CAN messages
uint8_t data_025[8];                            // Added on 31 March 2021
uint8_t data_035[8];                            // Added on 31 March 2021
uint8_t data_310[8];
uint8_t data_311[8];
uint8_t data_312[8];
uint8_t data_317[8];                            // Added on 31 March 2021
uint8_t data_330[8];
uint8_t data_331[8];
uint8_t data_332[8];
uint8_t data_340[8];
uint8_t data_350[8];
uint8_t data_360[8];
uint8_t data_381[8];
uint8_t data_390[8];
*/

uint16_t timer_data_025;
uint16_t timer_data_035;
uint16_t timer_data_310;
uint16_t timer_data_311;
uint16_t timer_data_312;
uint16_t timer_data_317;
uint16_t timer_data_340;
uint16_t timer_data_350;
uint16_t timer_data_330;
uint16_t timer_data_331;
uint16_t timer_data_332;
uint16_t timer_data_360;
uint16_t timer_data_390;
uint16_t timer_data_381;

// New data variables for CAN Check menu                --> Added on 03 Sep 2020
uint16_t can_3111_data[56];
uint16_t can_3112_data[19];
uint16_t can_3113_data[6];
uint16_t can_312_data[9];
uint16_t can_313_data[8];
uint16_t can_3141_data[35];
uint16_t can_3142_data[4];
uint16_t can_3143_data[4];
uint16_t can_315_data[141];

uint16_t can_341_data[12];                                      // Changed type on 31 March 2021
uint16_t can_351_data[8];
uint8_t can_36x_data[1];
uint16_t can_361_data[2];

/*--------------------------------------------------------- Bootloader handler function on user side start -----------------------------------------------------*/

void run_bootloader(void)
{
  watchdog_disable();
  while(1);
}
/*--------------------------------------------------------- Bootloader handler function on user side end -----------------------------------------------------*/

/* --------------------------------------------------------- Local Functions ----------------------------------------------------*/
void MX_CAN_FILTER_Init(uint8_t filter, uint32_t id, uint32_t mask);
void can_interrupt_enable_1();
void can_interrupt_enable_2();

void can_data_handler(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rx_data)
{
  uint8_t i;
  
  
  if(rxHeader->ExtId == CAN_COMMAND_REQUEST_ID)
  {
    if((rx_data[0] == TYM_COMBINE) && (rx_data[1] == 0x80))
    {
      run_bootloader();
    }
  }
  else if(rxHeader->ExtId == DIAGNOSTIC_REQUEST_CAN_ID)
  {
#if defined(USER_DEBUG)
  printf("CAN Data received from Diagnostic program.\r\n");
#endif
    flagCan.diagnosticRequest = TRUE;
    for(i = 0; i < 8; i++)
    {
      data_diagnostic[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == AUGER_SWITCH_BOX_CAN_ID)
  {
    timer_data_025 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN025.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == GENERAL_SWITCH_BOX_CAN_ID)
  {
    timer_data_035 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN035.data[i] = rx_data[i];
    }
  }  
  else if(rxHeader->ExtId == INTEGRATED_CONTROLLER_CAN_ID_1)
  {
    timer_data_310 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN310.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == INTEGRATED_CONTROLLER_CAN_ID_2)
  {
    timer_data_311 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN311.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == INTEGRATED_CONTROLLER_CAN_ID_3)
  {
    timer_data_312 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN312.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == INTEGRATED_CONTROLLER_CAN_ID_4)
  {
    timer_data_317 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN317.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == CUTTING_CONTROLLER_CAN_ID_1)
  {
    timer_data_330 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN330.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == CUTTING_CONTROLLER_CAN_ID_2)
  {
    timer_data_331 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN331.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == CUTTING_CONTROLLER_CAN_ID_3)
  {
    timer_data_332 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN332.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == AUGER_CONTROLLER_CAN_ID)
  {
    timer_data_340 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN340.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == LSA_CONTROLLER_CAN_ID)
  {
    timer_data_350 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN350.data[i] = rx_data[i];
    }
  }
  
  else if(rxHeader->ExtId == AXEL_CONTROLLER_CAN_ID)
  {
    timer_data_360 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN360.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == SENSOR_CAN_ID)
  {
    timer_data_381 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN381.data[i] = rx_data[i];
    }
  }
  else if(rxHeader->ExtId == HST_CONTROLLER_CAN_ID)
  {
#if defined(USER_DEBUG)
  printf("CAN Data received from HST controller.\r\n");
#endif
    timer_data_390 = 0;
    for(i = 0; i < 8; i++)
    {
      dataCAN390.data[i] = rx_data[i];
    }
  }  
  // Added following condition on 22 March 2021
  else if(rxHeader->ExtId == SETUP_MESSAGE_RECEIVE_CAN_ID)
  {
    if((rx_data[3] == (uint8_t)setup_mode_address) && (rx_data[4] == setup_mode_type))
    {
      new_setup_data_received = (rx_data[5] << 8) | rx_data[6];
      if(rx_data[3] == 0x69)
      {
        new_setup_data_2_received = (rx_data[0] << 8) | rx_data[1];
      }
      new_setup_data_received_flag = 1;
    }
  }
}

void can_receive_process()
{
  uint32_t temp;
  uint8_t i;
  
  if(timer_data_025 > CAN_CHECK_TIME)
  {
    flagWarning.can_timeout_warning_025 = TRUE;
  }
  else
  {
    timer_data_025++;
    flagWarning.can_timeout_warning_025 = FALSE;
  }
  
  if(timer_data_035 > CAN_CHECK_TIME)
  {
    flagWarning.can_timeout_warning_035 = TRUE;
  }
  else
  {
    timer_data_035++;
    flagWarning.can_timeout_warning_035 = FALSE;
  }
  
  if((timer_data_310 > CAN_CHECK_TIME) || (timer_data_311 > CAN_CHECK_TIME) || (timer_data_312 > CAN_CHECK_TIME) || (timer_data_317 > CAN_CHECK_TIME))
  {
    flagWarning.can_timeout_warning_31x = TRUE;
  }
  else
  {
    timer_data_310++;
    timer_data_311++;
    timer_data_312++;
    timer_data_317++;
    flagWarning.can_timeout_warning_31x = FALSE;
  }
  
  if((timer_data_330 > CAN_CHECK_TIME) || (timer_data_331 > CAN_CHECK_TIME) || (timer_data_332 > CAN_CHECK_TIME))
  {
    flagWarning.can_timeout_warning_33x = TRUE;
  }
  else
  {
    timer_data_330++;
    timer_data_331++;
    timer_data_332++;
    flagWarning.can_timeout_warning_33x = FALSE;
  }
  
  if(timer_data_340 > CAN_CHECK_TIME)
  {
    flagWarning.can_timeout_warning_340 = TRUE;
  }
  else
  {
    timer_data_340++;
    flagWarning.can_timeout_warning_340 = FALSE;
  }
  
  if(timer_data_350 > CAN_CHECK_TIME)
  {
    if(modelSelection == MODEL_NEXT)
    {
      flagWarning.can_timeout_warning_381 = FALSE;
    }
    else
    {
      flagWarning.can_timeout_warning_350 = TRUE;
    }
  }
  else
  {
    timer_data_350++;
    flagWarning.can_timeout_warning_350 = FALSE;
  }

  if(axelControlFunctionEnabled == FALSE)
  {
    if(timer_data_360 > CAN_CHECK_TIME)
    {
      flagWarning.can_timeout_warning_360 = TRUE;
    }
    else
    {
      timer_data_360++;
      flagWarning.can_timeout_warning_360 = FALSE;
    }  
  }
  
  if(timer_data_381 > CAN_CHECK_TIME)
  {
    if(modelSelection == MODEL_NEXT)
    {
      flagWarning.can_timeout_warning_381 = FALSE;
    }
    else
    {
      flagWarning.can_timeout_warning_381 = TRUE;
    }
  }
  else
  {
    timer_data_381++;
    flagWarning.can_timeout_warning_381 = FALSE;
  }

  if(timer_data_390 > CAN_CHECK_TIME)
  {
    flagWarning.can_timeout_warning_390 = TRUE;
  }
  else
  {
    timer_data_390++;
    flagWarning.can_timeout_warning_390 = FALSE;
  }

  // Diagnostic message check
  if(flagCan.diagnosticRequest == TRUE)
  {
    flagCan.diagnosticRequest = FALSE;
    flagCan.diagnosticResponse = TRUE;
    
    data_diagnostic_t[0] = data_diagnostic[0];
    data_diagnostic_t[1] = data_diagnostic[1];
    data_diagnostic_t[2] = data_diagnostic[2];
    data_diagnostic_t[3] = data_diagnostic[3];

    if(data_diagnostic[0] == 0xF2) // Read
    {
      temp = get_memory(data_diagnostic);
      
      data_diagnostic_t[6] = (uint8_t)temp;
      data_diagnostic_t[7] = (uint8_t)(temp >> 8);
    }
    else if(data_diagnostic[0] == 0xF3) // Write
    {
      temp = set_memory(data_diagnostic);
      
      data_diagnostic_t[6] = (uint8_t)temp;
      data_diagnostic_t[7] = (uint8_t)(temp >> 8);
    }
    else if(data_diagnostic[0] == 0xF4) // Default setting
    {
      if((data_diagnostic[1] == 0x0F) && (data_diagnostic[2] == 0x27))          // 0x270F = 9999
      {
        default_memory_update();
      }
    }
  }
  
  IC_auger_setting_deal = (dataCAN311.IC_auger_setting_deal_msb << 8) + dataCAN311.IC_auger_setting_deal_lsb;
  IC_pick1_thrising_bin_sensor_val = ((dataCAN311.IC_pick1_thrising_bin_sensor_val_msb << 8) + dataCAN311.IC_pick1_thrising_bin_sensor_val_lsb) * 2;
  IC_pick2_treatment_sensor_val = ((dataCAN311.IC_pick2_treatment_sensor_val_msb << 8) + dataCAN311.IC_pick2_treatment_sensor_val_lsb) * 15;
  IC_pick3_treatment_sensor_val = ((dataCAN311.IC_pick3_treatment_sensor_val_msb << 8) + dataCAN311.IC_pick3_treatment_sensor_val_lsb) * 30;

  
  IC_inclination = (dataCAN312.IC_inclination_msb << 8) + dataCAN312.IC_inclination_lsb;
  IC_inclination_deal = (dataCAN312.IC_inclination_deal_msb << 8) + dataCAN312.IC_inclination_deal_lsb;
  IC_garage_right_sensor = (dataCAN312.IC_garage_right_sensor_msb << 8) + dataCAN312.IC_garage_right_sensor_lsb;
  IC_garage_left_sensor = (dataCAN312.IC_garage_left_sensor_msb << 8) + dataCAN312.IC_garage_left_sensor_lsb;
  IC_engine_temperature = (dataCAN312.IC_engine_temperature_msb << 8) + dataCAN312.IC_engine_temperature_lsb;
  IC_tbs_right_sensor_minimum_setting = (dataCAN317.IC_tbs_left_sensor_minimum_setting_msb << 8) + dataCAN317.IC_tbs_left_sensor_minimum_setting_lsb;
  IC_tbs_left_sensor_minimum_setting = (dataCAN317.IC_tbs_left_sensor_minimum_setting_msb << 8) + dataCAN317.IC_tbs_left_sensor_minimum_setting_lsb;
  
  CC_pitching_cylinder_val = (dataCAN332.CC_pitching_cylinder_val_msb << 8) + dataCAN332.CC_pitching_cylinder_val_lsb;
  CC_cutting_manual_up_down_lever_sensor = (dataCAN332.CC_cutting_manual_up_down_lever_sensor_msb << 8) + dataCAN332.CC_cutting_manual_up_down_lever_sensor_lsb;

  AG_auger_potentiometer_value = (dataCAN340.AG_auger_potentiometer_value_msb << 8) + dataCAN340.AG_auger_potentiometer_value_lsb;
  LSA_multiturn_position = (dataCAN350.LSA_multiturn_position_msb << 8) + dataCAN350.LSA_multiturn_position_lsb;
  
  
  CC_vehicle_speed_val = (dataCAN331.CC_vehicle_speed_val_msb << 8) + dataCAN331.CC_vehicle_speed_val_lsb;
  CC_pitching_sensor = (dataCAN331.CC_pitching_sensor_msb << 8) + dataCAN331.CC_pitching_sensor_lsb;
  CC_lift_sensor_val = (dataCAN331.CC_lift_sensor_val_msb << 8) + dataCAN331.CC_lift_sensor_val_lsb;
  CC_power_clutch_sensor = (dataCAN331.CC_power_clutch_sensor_msb << 8) + dataCAN331.CC_power_clutch_sensor_lsb;

  // HST controller
  MC_steering_lever = (dataCAN390.MC_steering_lever_msb << 8) + dataCAN390.MC_steering_lever_lsb;
  MC_driveing_lever = (dataCAN390.MC_driveing_lever_msb << 8) + dataCAN390.MC_driveing_lever_lsb;
  MC_steering_motor = (dataCAN390.MC_steering_motor_msb << 8) + dataCAN390.MC_steering_motor_lsb;
  MC_driving_motor = (dataCAN390.MC_driving_motor_msb << 8) + dataCAN390.MC_driving_motor_lsb;
  MC_subshift_lever = (dataCAN390.MC_subshift_lever_msb << 8) + (dataCAN390.MC_subshift_lever_mid << 4) + (dataCAN390.MC_subshift_lever_lsb);
  
  selection_mode_position_sensor = (dataCAN381.selection_mode_position_sensor_msb << 8) + dataCAN381.selection_mode_position_sensor_lsb;
  airVolume_mode_position_sensor = (dataCAN381.airVolume_mode_position_sensor_msb << 8) + dataCAN381.airVolume_mode_position_sensor_lsb;
  
  // Integrated controller
  can_3111_data[ 0] = dataCAN310.IC_engine_stop_sw;
  can_3111_data[ 1] = dataCAN310.IC_carter_safety_sw;
  can_3111_data[ 2] = dataCAN310.IC_start_safety_sw;
  can_3111_data[ 3] = dataCAN310.IC_driver_restriction_sw;
  can_3111_data[ 4] = dataCAN310.IC_auger_connected_sw;
  can_3111_data[ 5] = dataCAN310.IC_auger_disconnected_sw;
  can_3111_data[ 6] = dataCAN310.IC_reverse_sw;
  can_3111_data[ 7] = dataCAN310.IC_engine_starter_sw;
  can_3111_data[ 8] = dataCAN310.IC_tbs_auto_start_sw;
  can_3111_data[ 9] = 0;
  can_3111_data[10] = 0;
  can_3111_data[11] = dataCAN310.IC_auger_return_sw;
  can_3111_data[12] = 0;
  can_3111_data[13] = 0;
  can_3111_data[14] = dataCAN310.IC_tbs_manual_right_down_sw;
  can_3111_data[15] = dataCAN310.IC_tbs_manual_right_up_sw;
  can_3111_data[16] = dataCAN310.IC_lsa_l_sensor_sw;
  can_3111_data[17] = 0;
  can_3111_data[18] = dataCAN310.IC_lsa_h_sensor_sw;
  can_3111_data[19] = dataCAN310.IC_lsa_m_sensor_sw;
  can_3111_data[20] = dataCAN310.IC_lsa_manual_descent_sw;
  can_3111_data[21] = dataCAN310.IC_lsa_manual_rise_sw;
  can_3111_data[22] = dataCAN310.IC_yew_sw;
  can_3111_data[23] = dataCAN310.IC_threshing_sw;
  can_3111_data[24] = dataCAN310.IC_auger_manual_down_sw;
  can_3111_data[25] = dataCAN310.IC_auger_manual_up_sw;
  can_3111_data[26] = dataCAN310.IC_auger_manual_left_sw;
  can_3111_data[27] = dataCAN310.IC_auger_manual_right_sw;
  can_3111_data[28] = dataCAN310.IC_auger_emission_blockage_sensor_sw;
  can_3111_data[29] = dataCAN310.IC_auger_discharge_sw;
  can_3111_data[30] = dataCAN310.IC_auger_grain_level_sensor_sw;
  can_3111_data[31] = dataCAN310.IC_auger_external_operation_sw;
  can_3111_data[32] = 0;
  can_3111_data[33] = dataCAN310.IC_preheat_relay;
  can_3111_data[34] = 0;
  can_3111_data[35] = 0;
  can_3111_data[36] = 0;
  can_3111_data[37] = dataCAN310.IC_auger_emission_connection_relay;
  can_3111_data[38] = 0;
  can_3111_data[39] = dataCAN310.IC_auger_discharge_blocking_relay;
  can_3111_data[40] = dataCAN310.IC_engine_start_relay;
  can_3111_data[41] = dataCAN310.IC_auger_left_turn_output;
  can_3111_data[42] = dataCAN310.IC_auger_priority_output;
  can_3111_data[43] = dataCAN310.IC_lsa_rising_output;
  can_3111_data[44] = dataCAN310.IC_lsa_falling_output;
  can_3111_data[45] = dataCAN310.IC_engine_stop_relay;
  can_3111_data[46] = dataCAN310.IC_stop_light_sw;
  can_3111_data[47] = dataCAN310.IC_auger_automatic_return_sw;
  can_3111_data[48] = dataCAN310.IC_auger_up_output;
  can_3111_data[49] = dataCAN310.IC_auger_down_output;
  can_3111_data[50] = dataCAN310.IC_tbs_left_up_output;
  can_3111_data[51] = dataCAN310.IC_tbs_left_down_output;
  can_3111_data[52] = dataCAN310.IC_tbs_up_output;
  can_3111_data[53] = dataCAN310.IC_tbs_down_output;
  can_3111_data[54] = dataCAN310.IC_auger_storage_sw;
  can_3111_data[55] = dataCAN310.IC_auger_lift_limit_sw;
  
  // Integrated controller
  can_3112_data[ 0] = IC_auger_setting_deal;
  can_3112_data[ 1] = IC_pick1_thrising_bin_sensor_val;
  can_3112_data[ 2] = IC_pick2_treatment_sensor_val;
  can_3112_data[ 3] = IC_pick3_treatment_sensor_val;
  can_3112_data[ 4] = dataCAN311.IC_tbs_tilt_sensor_error;
  can_3112_data[ 5] = dataCAN311.IC_tbs_grage_right_sensor_error;
  can_3112_data[ 6] = dataCAN311.IC_tbs_grage_left_sensor_error;
  can_3112_data[ 7] = dataCAN311.IC_tbs_manual_sw_error;
  can_3112_data[ 8] = dataCAN311.IC_engine_emergency_stop_error;
  can_3112_data[ 9] = dataCAN311.IC_auger_manual_sw_error;
  can_3112_data[10] = dataCAN311.IC_auger_setting_return_sw_error;
  can_3112_data[11] = dataCAN311.IC_engine_stop_cart_safety_sw;
  can_3112_data[12] = dataCAN311.IC_lsa_manual_sw_error;
  can_3112_data[13] = dataCAN311.IC_lsa_m_h_sensor_error;
  can_3112_data[14] = dataCAN311.IC_pick1_threshing_sensor_alarm;
  can_3112_data[15] = dataCAN311.IC_pick2_sensor_alarm;
  can_3112_data[16] = dataCAN311.IC_pick3_processing_sensor_alarm;
  can_3112_data[17] = dataCAN311.IC_grain_discharge_alarm;
  can_3112_data[18] = dataCAN311.IC_grain_clogging_alarm;
  
  can_3113_data[0] = IC_inclination;
  can_3113_data[1] = IC_inclination_deal;
  can_3113_data[2] = 0;
  can_3113_data[3] = IC_garage_right_sensor;
  can_3113_data[4] = IC_garage_left_sensor;
  can_3113_data[5] = IC_engine_temperature;

  // Cutting controller
  can_3143_data[0]  = dataCAN332.CC_pitching_manual_charge;
  can_3143_data[1]  = dataCAN332.CC_pitching_manual_transfer;
  can_3143_data[2]  = CC_pitching_cylinder_val;
  can_3143_data[3]  = CC_cutting_manual_up_down_lever_sensor;
    
  // Auger drive
  can_312_data[0] = dataCAN340.AG_output_left_turn;
  can_312_data[1] = dataCAN340.AG_print_priority;
  can_312_data[2] = dataCAN340.AG_emergency_connection_manual_turn;
  can_312_data[3] = dataCAN340.AG_emergency_connection_manual_bypass;
  can_312_data[4] = dataCAN340.AG_auger_break_output;
  can_312_data[5] = dataCAN340.AG_auger_rotation_sensor_error;
  can_312_data[6] = dataCAN340.AG_auger_limiting_current_error;
  can_312_data[7] = AG_auger_potentiometer_value;
  can_312_data[8] = dataCAN340.AG_auger_motor_output_duty_ratio;

  
  // LSA driver related
  can_313_data[0] = dataCAN350.LSA_up_output;
  can_313_data[1] = dataCAN350.LSA_down_output;
  can_313_data[2] = dataCAN350.LSA_emergency_connection_manual_up_sw;
  can_313_data[3] = dataCAN350.LSA_emergency_connection_manual_down_sw;
  can_313_data[4] = dataCAN350.LSA_lsa_motor_position_sensor_error;
  can_313_data[5] = dataCAN350.LSA_restraint_current_error;
  can_313_data[6] = LSA_multiturn_position;
  can_313_data[7] = dataCAN350.LSA_lsa_motor_output_duty_ratio;
  
 // Cutting controller connnection
  can_3141_data[ 0] = dataCAN330.CC_threshing_clutch_blocking_sw;
  can_3141_data[ 1] = dataCAN330.CC_threshing_clutch_connection_sw;
  can_3141_data[ 2] = 0;
  can_3141_data[ 3] = 0;
  can_3141_data[ 4] = 0;
  can_3141_data[ 5] = 0;
  can_3141_data[ 6] = 0;
  can_3141_data[ 7] = dataCAN330.CC_cutting_clutch_sw;
  can_3141_data[ 8] = dataCAN330.CC_clutch_mode_manual;
  can_3141_data[ 9] = dataCAN330.CC_clutch_mode_automatic;
  can_3141_data[10] = dataCAN330.CC_one_touch_up;
  can_3141_data[11] = dataCAN330.CC_one_touch_down;  
  can_3141_data[12] = dataCAN330.CC_quick_sw;
  can_3141_data[13] = dataCAN330.CC_quick_clutch_connection;
  can_3141_data[14] = dataCAN330.CC_quick_clutch_cut_off;
  can_3141_data[15] = dataCAN330.CC_harvesting_up_sw;
  can_3141_data[16] = dataCAN330.CC_harvesting_down_sw;
  can_3141_data[17] = dataCAN330.CC_cutting_clutch_blocking_relay;
  can_3141_data[18] = dataCAN330.CC_threshing_clutch_blocking_relay;
  can_3141_data[19] = dataCAN330.CC_cutting_clutch_connection_relay;
  can_3141_data[20] = dataCAN330.CC_threshing_clutch_connection_relay;
  can_3141_data[21] = dataCAN330.CC_cutting_clutch_lamp;
  can_3141_data[22] = dataCAN330.CC_auto_lift_lamp;
  can_3141_data[23] = dataCAN330.CC_automatic_cutting_lamp;
  can_3141_data[24] = dataCAN330.CC_threshing_clutch_lamp;
  can_3141_data[25] = dataCAN330.CC_cutting_up_output;
  can_3141_data[26] = dataCAN330.CC_cutting_down_output;
  can_3141_data[27] = dataCAN330.CC_load_breaking_relay;
  can_3141_data[28] = dataCAN330.CC_clean_motor_relay;
  can_3141_data[29] = dataCAN330.CC_lift_sensor_error;
  can_3141_data[30] = dataCAN330.CC_notice_sensor_error;
  can_3141_data[31] = dataCAN330.CC_manual_sw_error;
  can_3141_data[32] = dataCAN330.CC_threshing_clutch_motor_sw_error;
  can_3141_data[33] = dataCAN330.CC_cutting_clutch_motor_sw_error;
  can_3141_data[34] = dataCAN330.CC_one_touch_sw_error;
  
  // Cutting controller
  can_3142_data[0]  = CC_vehicle_speed_val;
  can_3142_data[1]  = CC_pitching_sensor;
  can_3142_data[2]  = CC_lift_sensor_val;
  can_3142_data[3]  = CC_power_clutch_sensor;

  // HST controller
  can_351_data[0] = MC_steering_lever;
  can_351_data[1] = MC_driveing_lever;
  can_351_data[2] = MC_steering_motor;
  can_351_data[3] = MC_driving_motor;
  can_351_data[4] = MC_subshift_lever;
  can_351_data[5] = dataCAN390.MC_subshift_mode_flag;
  can_351_data[6] = dataCAN390.MC_HST_error_code;
  can_351_data[7] = dataCAN390.data[6];
  
  if(axelControlFunctionEnabled == FALSE)
  {
    // Axel
    if(((dataCAN360.data[0] >> 4) & 0x0F) == 0x00)                     // CMD = 0
    { 
      can_341_data[2]           = axel_threshing_delay          = dataCAN360.data[0] & 0x0F;
      can_341_data[3]           = axel_auger_delay              = dataCAN360.data[1];
      can_341_data[4]           = axel_auger_auto_delay         = dataCAN360.data[2];
      can_341_data[5]           = axel_yeache_ku_time           = dataCAN360.data[3];
      can_341_data[6]           = axel_tbs_ku_time              = dataCAN360.data[4];
      can_341_data[7]           = axel_auger_ku_time            = dataCAN360.data[5];
      can_341_data[8]           = axel_c_speed_ku_time          = dataCAN360.data[6];
      
      can_341_data[11]          = axel_progVer                  = dataCAN360.data[7];
      
    }
    else if(((dataCAN360.data[0] >> 4) & 0x0F) == 0x0A)                // CMD = A 
    {
      can_341_data[10]          = axel_threshing_rpm            = (dataCAN360.data[2] << 8) + dataCAN360.data[3];
    }
    else if(((dataCAN360.data[0] >> 4) & 0x0F) == 0x0F)                // CMD = F
    {
      can_341_data[0]           = axel_app_sensor1_position     = ((dataCAN360.data[0]&0x0F) << 8) + dataCAN360.data[1];
      can_341_data[1]           = axel_app_sensor2_position     = ((dataCAN360.data[2]&0x0F) << 8) + dataCAN360.data[3];
                                  axel_keepTimeMode             = (dataCAN360.data[4] << 8) + dataCAN360.data[5];
      can_341_data[9]           = axel_app_sensor_totalError    = dataCAN360.data[7] & 0x0F;
                                  axel_delay_mode               = dataCAN360.data[6] & 0x0F;
    }
  }
  else
  {
    can_341_data[0]           = axel_app_sensor1_position;
    can_341_data[1]           = axel_app_sensor2_position;
    can_341_data[2]           = axel_threshing_delay;
    can_341_data[3]           = axel_auger_delay;
    can_341_data[4]           = axel_auger_auto_delay;
    can_341_data[5]           = axel_yeache_ku_time;
    can_341_data[6]           = axel_tbs_ku_time;
    can_341_data[7]           = axel_auger_ku_time;
    can_341_data[8]           = axel_c_speed_ku_time;
    can_341_data[9]           = axel_app_sensor_totalError;
    can_341_data[10]          = axel_threshing_rpm;
    can_341_data[11]          = axel_progVer;
  }
  
  for( i = 0; i < 56; i++){
    can_315_data[i] = can_3111_data[i];
  }
  for( i = 0; i < 19; i++){
    can_315_data[i + 56] = can_3112_data[i];
  }
  for( i = 0; i < 6; i++){
    can_315_data[i + 56 + 19] = can_3113_data[i];
  }
  for( i = 0; i < 9; i++){
    can_315_data[i + 56 + 19 + 6] = can_312_data[i];
  }
  for( i = 0; i < 8; i++){
    can_315_data[i + 56 + 19 + 6 + 9] = can_313_data[i];
  }
  for( i = 0; i < 35; i++){
    can_315_data[i + 56 + 19 + 6 + 9 + 8] = can_3141_data[i];
  }
  for( i = 0; i < 4; i++){
    can_315_data[i + 56 + 19 + 6 + 9 + 8 + 35] = can_3142_data[i];
  }
  for( i = 0; i < 4; i++){
    can_315_data[i + 56 + 19 + 6 + 9 + 8 + 35 + 4] = can_3143_data[i];
  }
  
  /*
    // Auger operation switch box
    auger_auto_left_sw                                          = data_025[0] & 0x01;
    auger_auto_right_sw                                         = data_025[0] & 0x02;
    auger_auto_after_sw                                         = data_025[0] & 0x04;
    auger_storage_sw                                            = data_025[0] & 0x08;
    auger_discharge_on_sw                                       = data_025[0] & 0x10;
    auger_discharge_stop_sw                                     = data_025[0] & 0x20;
    auger_manual_up_sw                                          = data_025[0] & 0x40;
    auger_manual_down_sw                                        = data_025[0] & 0x80;
    auger_auto_swing_sw                                         = data_025[1] & 0x01;
    auger_swing_left_sw                                         = data_025[1] & 0x02;
    auger_swing_right_sw                                        = data_025[1] & 0x04;
    auger_manual_right_sw                                       = data_025[1] & 0x08;
    auger_manual_left_sw                                        = data_025[1] & 0x10;
    auger_rotation_stop_sw                                      = data_025[1] & 0x20;
    auger_program_version                                       = data_025[3];
    
    // General operation switch box
    general_auto_accel_sw_1                                     = data_035[0] & 0x40;
    general_aircraft_height_fall_sw                             = data_035[0] & 0x20;
    general_auto_supply_depth_sw                                = data_035[0] & 0x10;
    general_auto_forward_backward_sw                            = data_035[0] & 0x08;
    general_auto_left_right_sw                                  = data_035[0] & 0x04;

    general_auto_sorting_led                                    = data_035[1] & 0x80;
    general_auto_accel_led                                      = data_035[1] & 0x10;
    general_auto_sorting_sw                                     = data_035[1] & 0x08;
    general_up_sw                                               = data_035[1] & 0x01;
    
    general_spin_turn_led                                       = data_035[2] & 0x80;
    general_auto_supply_depth_led                               = data_035[2] & 0x40;
    general_auto_forward_backward_led                           = data_035[2] & 0x20;
    general_auto_left_right_led                                 = data_035[2] & 0x10;
    general_auto_cutting_height_led                             = data_035[2] & 0x08;
    general_auto_accel_sw_2                                     = data_035[2] & 0x02;
    general_spin_turn_sw                                        = data_035[2] & 0x01;
    
    general_air_volume_control                                  = data_035[3];
    general_sheave_setting                                      = data_035[4];
    general_mowing_height_setting                               = data_035[5];
    general_inclination_angle_adjustment                        = data_035[6];
    general_program_version                                     = data_035[7];
  
    
    // Integrated controller
    can_3111_data[ 0] = IC_engine_stop_sw                       = (data_310[0] & 0x01) >> 0;
    can_3111_data[ 1] = IC_carter_safety_sw                     = (data_310[0] & 0x02) >> 1;
    can_3111_data[ 2] = IC_start_safety_sw                      = (data_310[0] & 0x04) >> 2;
    can_3111_data[ 3] = IC_driver_restriction_sw                = (data_310[0] & 0x08) >> 3;
    can_3111_data[ 4] = IC_auger_connected_sw                   = (data_310[0] & 0x20) >> 5;
    can_3111_data[ 5] = IC_auger_disconnected_sw                = (data_310[0] & 0x10) >> 4;
    can_3111_data[ 6] = IC_reverse_sw                           = (data_310[0] & 0x40) >> 6;
    can_3111_data[ 7] = IC_engine_starter_sw                    = (data_310[0] & 0x80) >> 7;
    can_3111_data[ 8] = IC_tbs_auto_start_sw                    = (data_310[1] & 0x01) >> 0;
    can_3111_data[ 9] = 0;
    can_3111_data[10] = 0;
    can_3111_data[11] = IC_auger_return_sw                      = (data_310[1] & 0x08) >> 3;
    can_3111_data[12] = 0;
    can_3111_data[13] = 0;
    can_3111_data[14] = IC_tbs_manual_rainfall_sw               = (data_310[1] & 0x40) >> 6;
    can_3111_data[15] = IC_tbs_manual_rising_sw                 = (data_310[1] & 0x80) >> 7;
    can_3111_data[16] = IC_lsa_l_sensor_sw                      = (data_310[2] & 0x01) >> 0;
    can_3111_data[17] = 0;
    can_3111_data[18] = IC_lsa_h_sensor_sw                      = (data_310[2] & 0x04) >> 2;
    can_3111_data[19] = IC_lsa_m_sensor_sw                      = (data_310[2] & 0x08) >> 3;
    can_3111_data[20] = IC_lsa_manual_descent_sw                = (data_310[2] & 0x10) >> 4;
    can_3111_data[21] = IC_lsa_manual_rise_sw                   = (data_310[2] & 0x20) >> 5;
    can_3111_data[22] = IC_yew_sw                               = (data_310[2] & 0x40) >> 6;
    can_3111_data[23] = IC_threshing_sw                         = (data_310[2] & 0x80) >> 7;
    can_3111_data[24] = IC_auger_manual_down_sw                 = (data_310[3] & 0x01) >> 0;
    can_3111_data[25] = IC_auger_manual_up_sw                   = (data_310[3] & 0x02) >> 1;
    can_3111_data[26] = IC_auger_manual_left_sw                 = (data_310[3] & 0x04) >> 2;
    can_3111_data[27] = IC_auger_manual_right_sw                = (data_310[3] & 0x08) >> 3;
    can_3111_data[28] = IC_auger_emission_blockage_sensor_sw    = (data_310[3] & 0x10) >> 4;
    can_3111_data[29] = IC_auger_discharge_sw                   = (data_310[3] & 0x20) >> 5;
    can_3111_data[30] = IC_auger_grain_level_sensor_sw          = (data_310[3] & 0x40) >> 6;
    can_3111_data[31] = IC_auger_external_operation_sw          = (data_310[3] & 0x80) >> 7;
    can_3111_data[32] = 0;
    can_3111_data[33] = IC_preheat_relay                        = (data_310[5] & 0x02) >> 1;
    can_3111_data[34] = 0;
    can_3111_data[35] = 0;
    can_3111_data[36] = 0;
    can_3111_data[37] = IC_auger_emission_connection_relay      = (data_310[5] & 0x20) >> 5;
    can_3111_data[38] = 0;
    can_3111_data[39] = IC_auger_discharge_blocking_relay       = (data_310[5] & 0x80) >> 7;
    can_3111_data[40] = IC_engine_start_relay                   = data_310[6] & 0x01;
    can_3111_data[41] = IC_auger_left_turn_output               = (data_310[6] & 0x02) >> 1;
    can_3111_data[42] = IC_auger_priority_output                = (data_310[6] & 0x04) >> 2;
    can_3111_data[43] = IC_lsa_rising_output                    = (data_310[6] & 0x08) >> 3;
    can_3111_data[44] = IC_lsa_falling_output                   = (data_310[6] & 0x10) >> 4;
    can_3111_data[45] = IC_engine_stop_relay                    = (data_310[6] & 0x20) >> 5;
    can_3111_data[46] = IC_stop_light_sw                        = (data_310[6] & 0x40) >> 6;
    can_3111_data[47] = IC_auger_automatic_return_sw            = (data_310[6] & 0x80) >> 7;
    can_3111_data[48] = IC_auger_up_output                      = data_310[7] & 0x01;
    can_3111_data[49] = IC_auger_down_output                    = (data_310[7] & 0x02) >> 1;
    can_3111_data[50] = IC_tbs_left_up_output                   = (data_310[7] & 0x04) >> 2;
    can_3111_data[51] = IC_tbs_left_down_output                 = (data_310[7] & 0x08) >> 3;
    can_3111_data[52] = IC_tbs_up_output                        = (data_310[7] & 0x10) >> 4;
    can_3111_data[53] = IC_tbs_down_output                      = (data_310[7] & 0x20) >> 5;
    can_3111_data[54] = IC_auger_storage_sw                     = (data_310[4] & 0x10) >> 4;
    can_3111_data[55] = IC_auger_lift_limit_sw                  = (data_310[4] & 0x40) >> 6;
                        IC_emergency_switch_in_cabin            = (data_310[4] & 0x02) >> 1;
    
                        IC_external_discharge_switch_blocking   = (data_310[4] & 0x08) >> 3;
                        IC_external_discharge_switch_connection = (data_310[4] & 0x20) >> 1;
                        IC_auger_remote_control_opeation        = (data_310[7] & 0x80) >> 7;
                        

    // Integrated controller
    can_3112_data[ 0] = IC_auger_setting_deal                   = ((data_311[0] & 0x03) << 8) + data_311[1];
    can_3112_data[ 1] = IC_pick1_thrising_bin_sensor_val        = (((data_311[0] & 0x0C) << 6) + data_311[2]) * 2;
    can_3112_data[ 2] = IC_pick2_treatment_sensor_val           = (((data_311[0] & 0x30) << 4) + data_311[3]) * 15;
    can_3112_data[ 3] = IC_pick3_treatment_sensor_val           = (((data_311[0] & 0xC0) << 2) + data_311[4]) * 30;
    can_3112_data[ 4] = IC_tbs_tilt_sensor_error                = data_311[5] & 0x01;
    can_3112_data[ 5] = IC_tbs_grage_right_sensor_error         = (data_311[5] & 0x02) >> 1;
    can_3112_data[ 6] = IC_tbs_grage_left_sensor_error          = (data_311[5] & 0x04) >> 2;
    can_3112_data[ 7] = IC_tbs_manual_sw_error                  = (data_311[5] & 0x08) >> 3;
    can_3112_data[ 8] = IC_engine_emergency_stop_error          = (data_311[5] & 0x10) >> 4;
    can_3112_data[ 9] = IC_auger_manual_sw_error                = (data_311[5] & 0x20) >> 5;
    can_3112_data[10] = IC_auger_setting_return_sw_error        = (data_311[5] & 0x40) >> 6;
    can_3112_data[11] = IC_engine_stop_safety_sw                = (data_311[5] & 0x80) >> 7;
    can_3112_data[12] = IC_lsa_manual_sw_error                  = data_311[6] & 0x01;
    can_3112_data[13] = IC_lsa_m_h_sensor_error                 = (data_311[6] & 0x02) >> 1;
    can_3112_data[14] = IC_pick1_threshing_sensor_alarm         = (data_311[6] & 0x04) >> 2;
    can_3112_data[15] = IC_pick2_sensor_alarm                   = (data_311[6] & 0x08) >> 3;
    can_3112_data[16] = IC_pick3_processing_sensor_alarm        = (data_311[6] & 0x10) >> 4;
    can_3112_data[17] = IC_grain_discharge_alarm                = (data_311[6] & 0x20) >> 5;
    can_3112_data[18] = IC_grain_clogging_alarm                 = (data_311[6] & 0x40) >> 6;
    
                        IC_auger_discharge_motor_alarm          = (data_311[6] & 0x80) >> 7;
                        IC_setup_val                            = (data_311[7] & 0xF0);
                        IC_setup_mode_error                     = (data_311[7] & 0x08);
                        IC_setup_mode                           = (data_311[7] & 0x07);

    // Integrated controller
    can_3113_data[0] = IC_inclination                           = ((data_312[0] & 0x03) << 8) + data_312[1];
    can_3113_data[1] = IC_inclination_deal                      = ((data_312[0] & 0x0C) << 6) + data_312[2];
    can_3113_data[2] = 0;
    can_3113_data[3] = IC_garage_right_sensor                   = ((data_312[0] & 0xC0) << 2) + data_312[4];
    can_3113_data[4] = IC_garage_left_sensor                    = ((data_312[5] & 0x03) << 8) + data_312[6];
    can_3113_data[5] = IC_engine_temperature                    = ((data_312[5] & 0x0C) << 6) + data_312[7];
    
                       IC_tbs_right_sensor_minimum_setting      = (data_317[0] << 8) + data_317[1];
                       IC_tbs_left_sensor_minimum_setting       = (data_317[2] << 8) + data_317[3];

                       
    // Cutting controller connnection
    can_3141_data[ 0] = CC_threshing_clutch_blocking_sw         = (data_330[0] & 0x02);
    can_3141_data[ 1] = CC_threshing_clutch_connection_sw       = (data_330[0] & 0x04);
    can_3141_data[ 2] = 0;
    can_3141_data[ 3] = 0;
    can_3141_data[ 4] = 0;
    can_3141_data[ 5] = 0;
    can_3141_data[ 6] = 0;
    can_3141_data[ 7] = CC_cutting_clutch_sw                    = (data_330[1] & 0x08);
    can_3141_data[ 8] = CC_clutch_mode_manual                   = (data_330[1] & 0x10);
    can_3141_data[ 9] = CC_clutch_mode_automatic                = (data_330[1] & 0x20);
    can_3141_data[10] = CC_one_touch_up                         = (data_330[1] & 0x40);
    can_3141_data[11] = CC_one_touch_down                       = (data_330[1] & 0x80);
    
    can_3141_data[12] = CC_quick_sw                             = (data_330[2] & 0x01);
    can_3141_data[13] = CC_quick_clutch_connection              = (data_330[2] & 0x02);
    can_3141_data[14] = CC_quick_clutch_cut_off                 = (data_330[2] & 0x04);
    
    can_3141_data[15] = CC_harvesting_up_sw                     = (data_330[2] & 0x10);
    can_3141_data[16] = CC_harvesting_down_sw                   = (data_330[2] & 0x08);
                        CC_unloader_SOL_operation               = (data_330[2] & 0x20);
    
    can_3141_data[17] = CC_cutting_clutch_blocking_relay        = (data_330[3] & 0x01);
    can_3141_data[18] = CC_threshing_clutch_blocking_relay      = (data_330[3] & 0x02);
    can_3141_data[19] = CC_cutting_clutch_connection_relay      = (data_330[3] & 0x04);
    can_3141_data[20] = CC_threshing_clutch_connection_relay    = (data_330[3] & 0x08);
    can_3141_data[21] = CC_cutting_clutch_lamp                  = (data_330[3] & 0x10);
    can_3141_data[22] = CC_auto_lift_lamp                       = (data_330[3] & 0x20);
    can_3141_data[23] = CC_automatic_cutting_lamp               = (data_330[3] & 0x40);
    can_3141_data[24] = CC_threshing_clutch_lamp                = (data_330[3] & 0x80);
    
    can_3141_data[25] = CC_cutting_up_output                    = (data_330[4] & 0x02);
    can_3141_data[26] = CC_cutting_down_output                  = (data_330[4] & 0x04);
    can_3141_data[27] = CC_load_breaking_relay                  = (data_330[4] & 0x08);
    can_3141_data[28] = CC_clean_motor_relay                    = (data_330[4] & 0x10);
                        CC_steering_right_SOL                   = (data_330[4] & 0x40);
                        CC_steering_left_SOL                    = (data_330[4] & 0x20);
    can_3141_data[29] = CC_lift_sensor_error                    = (data_330[5] & 0x01);
    can_3141_data[30] = CC_notice_sensor_error                  = (data_330[5] & 0x02);
    can_3141_data[31] = CC_manual_sw_error                      = (data_330[5] & 0x04);
    can_3141_data[32] = CC_threshing_clutch_motor_sw_error      = (data_330[5] & 0x08);
    can_3141_data[33] = CC_cutting_clutch_motor_sw_error        = (data_330[5] & 0x10);
    can_3141_data[34] = CC_one_touch_sw_error                   = (data_330[5] & 0x20);
                        CC_conter_safety                        = (data_330[6] & 0x04);
                        CC_amount_of_grain                      = (data_330[6] & 0x02);
                        CC_cutting_clogging_sensor              = (data_330[6] & 0x01);
   
                        CC_cutting_setup                        = (data_330[7] & 0xF0) >> 4;
                        CC_cutting_setup_mode                   = (data_330[7] & 0x07);

    // Cutting controller
    can_3142_data[0]  = CC_vehicle_speed_val                    = ((data_331[0] & 0x03) << 8) + data_331[1];
    can_3142_data[1]  = CC_pitching_sensor                      = ((data_331[0] & 0x0C) << 6) + data_331[2];
    can_3142_data[2]  = CC_lift_sensor_val                      = ((data_331[0] & 0x30) << 4) + data_331[3];
    can_3142_data[3]  = CC_power_clutch_sensor                  = ((data_331[0] & 0xC0) << 2) + data_331[4];
                        CC_power_clutch_sensor_2                = (data_331[5] & 0x01);
                        CC_program_version                      = data_331[6];
                        CC_pitching_automatic_down              = (data_331[7] & 0x10);
                        CC_pitching_automatic_not_down          = (data_331[7] & 0x08);
                        CC_pitching_phase_inversion_condition   = (data_331[7] & 0x04);
                        CC_pitching_driving_restrictions        = (data_331[7] & 0x02);
                        CC_pitching_phase_inversion_condition_2 = (data_331[7] & 0x01);

    
    // Cutting controller
    can_3143_data[0]  = CC_pitching_manual_charge               = (data_332[0] & 0x08);
    can_3143_data[1]  = CC_pitching_manual_transfer             = (data_332[0] & 0x20);
    can_3143_data[2]  = CC_pitching_cylinder_val                = ((data_332[1] & 0x03) << 8) + data_332[2];
    can_3143_data[3]  = CC_cutting_manual_up_down_lever_sensor  = ((data_332[1] & 0x0C) << 6) + data_332[3];
    
    // Auger drive
    can_312_data[0] = AG_output_left_turn                       = data_340[0] & 0x01;
    can_312_data[1] = AG_print_priority                         = data_340[0] & 0x02;
    can_312_data[2] = AG_emergency_connection_manual_turn       = data_340[0] & 0x04;
    can_312_data[3] = AG_emergency_connection_manual_bypass     = data_340[0] & 0x08;
    can_312_data[4] = AG_auger_break_output                     = data_340[0] & 0x10;
    can_312_data[5] = AG_auger_rotation_sensor_error            = data_340[0] & 0x20;
    can_312_data[6] = AG_auger_limiting_current_error           = data_340[0] & 0x40;
    can_312_data[7] = AG_auger_potentiometer_value              = ((data_340[1] & 0x03) << 8) + data_340[2];
    can_312_data[8] = AG_auger_motor_output_duty_ratio          = data_340[3];
                      AG_program_version                        = data_340[5];

    // LSA driver related
    can_313_data[0] = LSA_up_output                             = (data_350[0] & 0x01);
    can_313_data[1] = LSA_down_output                           = (data_350[0] & 0x02);
    can_313_data[2] = LSA_emergency_connection_manual_up_sw     = (data_350[0] & 0x04);
    can_313_data[3] = LSA_emergency_connection_manual_down_sw   = (data_350[0] & 0x08);
    can_313_data[4] = LSA_lsa_motor_position_sensor_error       = (data_350[0] & 0x20);
    can_313_data[5] = LSA_restraint_current_error               = (data_350[0] & 0x40);
    can_313_data[6] = LSA_multiturn_position                    = ((data_350[1] & 0x03) << 8) + data_350[2];
    can_313_data[7] = LSA_lsa_motor_output_duty_ratio           = data_350[3];
                      LSA_program_version                       = data_350[5];
                      
    // Axel
    if(((data_360[0] >> 4) & 0x0F) == 0x00)                     // CMD = 0
    { 
      can_341_data[2]           = axel_threshing_delay          = data_360[0] & 0x0F;
      can_341_data[3]           = axel_auger_delay              = data_360[1];
      can_341_data[4]           = axel_auger_auto_delay         = data_360[2];
      can_341_data[5]           = axel_yeache_ku_time           = data_360[3];
      can_341_data[6]           = axel_tbs_ku_time              = data_360[4];
      can_341_data[7]           = axel_auger_ku_time            = data_360[5];
      can_341_data[8]           = axel_c_speed_ku_time          = data_360[6];
      
      can_341_data[11]          = axel_progVer                  = data_360[7];
      
    }
    else if(((data_360[0] >> 4) & 0x0F) == 0x0A)                // CMD = A 
    {
      can_341_data[10]          = axel_threshing_rpm            = (data_360[2] << 8) + data_360[3];
    }
    else if(((data_360[0] >> 4) & 0x0F) == 0x0F)                // CMD = F
    {
      can_341_data[0]           = axel_app_sensor1_position     = ((data_360[0]&0x0F) << 8) + data_360[1];
      can_341_data[1]           = axel_app_sensor2_position     = ((data_360[2]&0x0F) << 8) + data_360[3];
                                  axel_keepTimeMode             = (data_360[4] << 8) + data_360[5];
      can_341_data[9]           = axel_app_sensor_totalError    = data_360[7] & 0x0F;
                                  axel_delay_mode               = data_360[6] & 0x0F;
    }
 
    // HST controller
    can_351_data[0]             = MC_steering_lever             = ((data_390[6] & 0x03) << 8) + data_390[7];
    can_351_data[1]             = MC_driveing_lever             = ((data_390[4] & 0x03) << 8) + data_390[5];
    can_351_data[2]             = MC_steering_motor             = ((data_390[2] & 0x03) << 8) + data_390[3];
    can_351_data[3]             = MC_driving_motor              = ((data_390[0] & 0x03) << 8) + data_390[1];                                           
    can_351_data[4]             = MC_subshift_lever             = ((data_390[2] & 0xC0) << 2) + (((data_390[2] & 0x3C) << 2) | ((data_390[0] & 0x3C) >> 2));
    can_351_data[5]             = MC_subshift_mode_flag         = (data_390[4] & 0x0C) >> 2;
    can_351_data[6]             = MC_HST_error_code             = (data_390[6] & 0x3C) >> 2;
    can_351_data[7]                                             = (data_390[6] & 0xC0) >> 6;
                                  MC_steering_motor_level       = (data_390[0] & 0x80);
                                  MC_neutral_flag               = (data_390[0] & 0x40);
                                  MC_brake_signal_flag          = (data_390[4] & 0x80);
                                  MC_threshing_load_flag        = (data_390[4] & 0x70) >> 4;
                                  MC_spinton_signal_flag        = (data_390[6] & 0x80);
                                  MC_vehicle_zero_flag          = (data_390[6] & 0x40);
    
                                  MC_HST_chuhen_motor_error     = MC_HST_error_code & 0x01;
                                  MC_HST_chuhyan_motor_error    = MC_HST_error_code & 0x02;
                                  MC_HST_chuhen_lever_error     = MC_HST_error_code & 0x04;
                                  MC_HST_chuhyan_lever_error    = MC_HST_error_code & 0x08;    
    
                                  
                                  //                                  
                                  selection_mode_position_sensor = (data_381[4] << 8) + data_381[5];
                                  airVolume_mode_position_sensor = (data_381[6] << 8) + data_381[7];
    
    for( i = 0; i < 56; i++){
      can_315_data[i] = can_3111_data[i];
    }
    for( i = 0; i < 19; i++){
      can_315_data[i + 56] = can_3112_data[i];
    }
    for( i = 0; i < 6; i++){
      can_315_data[i + 56 + 19] = can_3113_data[i];
    }
    for( i = 0; i < 9; i++){
      can_315_data[i + 56 + 19 + 6] = can_312_data[i];
    }
    for( i = 0; i < 8; i++){
      can_315_data[i + 56 + 19 + 6 + 9] = can_313_data[i];
    }
    for( i = 0; i < 35; i++){
      can_315_data[i + 56 + 19 + 6 + 9 + 8] = can_3141_data[i];
    }
    for( i = 0; i < 4; i++){
      can_315_data[i + 56 + 19 + 6 + 9 + 8 + 35] = can_3142_data[i];
    }
    for( i = 0; i < 4; i++){
      can_315_data[i + 56 + 19 + 6 + 9 + 8 + 35 + 4] = can_3143_data[i];
    }
    */
}

void can_transmit_tsc1_packet()
{
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  
  data[0] = 0x0C;
  
  can_transmit(CAN_CHANNEL_1, ENGINE_TSC1_CAN_ID, data);
}

void can_transmit_ecr1_packet()
{
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  
  data[3] = acceleratorPedalPosition;
    
  can_transmit(CAN_CHANNEL_1, ENGINE_ECR1_CAN_ID, data);
}

void can_transmit_packet()
{
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };  
  
  uint16_t engineSpeedLocal = (uint16_t) (engine_speed * 0.125 / 10);
  uint16_t jobHourLocal = (uint16_t) (jobHour / 1000);
  
  if(flagInput.rightLamp == ON)         data[0] |= 0x01;
  if(flagInput.leftLamp == ON)          data[0] |= 0x02;
  if(flagInput.tailLamp == ON)          data[0] |= 0x04;
  if(flagInput.charge == ON)            data[0] |= 0x08;
  if(flagWarning.oilPressure == ON)     data[0] |= 0x10;
  //if(flagInput.buzzerStop == ON)        data[0] |= 0x20;
  if(flagInput.grain_1 == ON)           data[0] |= 0x40;
  if(flagInput.grain_2 == ON)           data[0] |= 0x80;

  if(flagInput.grain_3 == ON)           data[1] |= 0x01;
  if(flagInput.grain_4 == ON)           data[1] |= 0x02;
  
  data[1] = 0x01 & (engineSpeedLocal >> 8) << 6;
  
  data[2] = 0x03 & (jobHourLocal >> 8);
  
  data[3] = (uint8_t)(tPowerVoltage * 10);
  data[4] = tFuelPercent;
  data[5] = (uint8_t) engineSpeedLocal;                                  // Engine RPM LSB
  data[6] = (uint8_t) jobHourLocal;
  data[7] = 0;
  if(setup_mode == SETUP_MODE_ONE)
  {
    data[7] = setup_mode_type;
  }
  /*
  //if(flagInput.buzzerStop == ON)        data[0] |= 0x20;
  
  engineSpeed9bit = 
  
  data[2] = (uint8_t)(tPowerVoltage * 10);

  data[3] = (uint8_t)(engine_speed);
  data[4] = (uint8_t)(engine_speed >> 8);
  
  data[5] = tFuelPercent;
  data[6] = VAC_PROGRAM_VERSION;
  data[7] = 0;
  
  if(setup_mode == SETUP_MODE_ONE)
  {
    data[7] = setup_mode_type;
  }
  */
  can_transmit(CAN_CHANNEL_2, METAPANEL_DATA_CAN_ID, data);
}

void can_transmit_setting()
{
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  // the first four byte is my setting address
  data[0] = 0x19;                                          // 1st byte of the receiver address
  data[1] = 0xFF;                                          // 2nd byte of the receiver address
  if(setup_mode_address == 0x369){
    data[0] = new_setup_data_2 >> 8;
    data[1] = new_setup_data_2;
  }
  data[2] = 0xA3;                                          // 3rd byte of the receiver address
  data[3] = 0x29;                                          // 4th byte of the receiver address
  data[4] = setup_mode_type;
  
  if(setup_mode_rw == SETUP_WRITE){
    data[5] = new_setup_data >> 8;
    data[6] = new_setup_data;
  }else{
    data[5] = 0;
    data[6] = 0;
  }
  data[7] = setup_mode_rw;
  
  can_transmit(CAN_CHANNEL_2, METAPANEL_SETUP_CAN_ID | setup_mode_address, data);
}

void can_transmit_axel()
{
  static uint8_t cmd = 0;
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
 
  if(cmd == 0)
  {
    // CMD  = 0
    data[0] = ((cmd & 0x0F) << 4) | (axel_threshing_delay & 0x0F);
    data[1] = axel_auger_delay;
    data[2] = axel_auger_auto_delay;
    data[3] = axel_yeache_ku_time;
    data[4] = axel_tbs_ku_time;
    data[5] = axel_auger_ku_time;
    data[6] = axel_c_speed_ku_time;
    data[7] = axel_progVer;
    cmd = 0x0A;
  }
  else if(cmd == 0x0A)
  {
    // CMD = A
    data[0] = ((cmd & 0x0F) << 4);
    data[1] = 0;
    data[2] = (uint8_t) axel_threshing_rpm;
    data[3] = (uint8_t) (axel_threshing_rpm << 8);
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    cmd = 0x0F;
  }
  else
  {
    // CMD = F
    data[0] = ((cmd & 0x0F) << 4) | ((axel_app_sensor1_position >> 8) & 0x0F);
    data[1] = (uint8_t) axel_app_sensor1_position;
    
    data[2] = ((axel_app_sensor2_position >> 8) & 0x0F);
    data[3] = (uint8_t) axel_app_sensor2_position;
    
    data[4] = (uint8_t) (axel_keepTimeMode >> 8);
    data[5] = (uint8_t) (axel_keepTimeMode);
    data[6] = axel_delay_mode & 0x0F;
    data[7] = axel_app_sensor_totalError & 0x0F;
    cmd = 0x00;
  }
    
  can_transmit(CAN_CHANNEL_2, AXEL_CONTROLLER_CAN_ID, data);
}

void can_transmit_diagnostic()
{
  uint8_t data[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  
  data[0] = data_diagnostic_t[0];
  data[1] = data_diagnostic_t[1];
  data[2] = data_diagnostic_t[2];
  data[3] = data_diagnostic_t[3];
  data[4] = data_diagnostic_t[4];
  data[5] = data_diagnostic_t[5];
  data[6] = data_diagnostic_t[6];
  data[7] = data_diagnostic_t[7];
    
  can_transmit(CAN_CHANNEL_2, DIAGNOSTIC_RESPONSE_CAN_ID, data);
}

void can_transmit_process_engine_line()
{
  static uint16_t timerTSC1 = 0;
  static uint16_t timerECR1 = 0;
 
  if(axelControlFunctionEnabled == FALSE)
  {
    return;
  }
  
  timerTSC1++;
  timerECR1++;
  
  if(timerTSC1 >= CAN_ENGINE_TSC1_TIME)
  {
    timerTSC1 = 0;
    can_transmit_tsc1_packet();
  }
  else if(timerECR1 >= CAN_ENGINE_ECR1_TIME)
  {
    timerECR1 = 0;
    can_transmit_ecr1_packet();
  }
}

void can_transmit_process()
{
  static uint16_t timerPacket;
  static uint16_t timerSetting;
  
  static uint16_t timerAxel;
  
  timerPacket++;
  timerSetting++; 
  
  timerAxel++;
  
  if(timerPacket >= CAN_TRANSMISSION_TIME)
  {
    timerPacket = 0;
    can_transmit_packet();
  }
  else if(timerAxel >= CAN_AXEL_TIME)
  {
    timerAxel = 0;
    if(axelControlFunctionEnabled == TRUE)
    {
      can_transmit_axel();
    }
  }
  else if(timerSetting >= CAN_SETTINGS_TIME)
  {
    timerSetting = 0;
    if(setup_mode == SETUP_MODE_TWO)
    {
      can_transmit_setting();
    }
  }
  else if(flagCan.diagnosticResponse == TRUE)
  {
    flagCan.diagnosticResponse = FALSE;
    can_transmit_diagnostic();
  }
}

uint8_t can_transmit(uint8_t channel, uint32_t id, uint8_t _data[])
{
  if(channel == CAN_CHANNEL_1)
  {
    canOneTxHeader.IDE = CAN_ID_EXT;
    canOneTxHeader.RTR = CAN_RTR_DATA;
    canOneTxHeader.ExtId = id;
    canOneTxHeader.DLC = 8;
    
    canOneTxData[0] = _data[0];
    canOneTxData[1] = _data[1];
    canOneTxData[2] = _data[2];
    canOneTxData[3] = _data[3];
    canOneTxData[4] = _data[4];
    canOneTxData[5] = _data[5];
    canOneTxData[6] = _data[6];
    canOneTxData[7] = _data[7];
    
    canOneTxHeader.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &canOneTxHeader, canOneTxData, &canOneTxMailbox) != HAL_OK)
    {
      canErrorCounter[0]++;
      if(canErrorCounter[0] > 20)
      {
        HAL_CAN_DeInit(&hcan1);
        HAL_CAN_DeInit(&hcan2);
        MX_CAN1_Init();
        MX_CAN2_Init();
        canErrorCounter[0] = 0;
        canErrorCounter[1] = 0;
        
        timer_data_025 = 0;
        timer_data_035 = 0;
        timer_data_310 = 0;
        timer_data_311 = 0;
        timer_data_312 = 0;
        timer_data_317 = 0;
        timer_data_340 = 0;
        timer_data_350 = 0;
        timer_data_330 = 0;
        timer_data_331 = 0;
        timer_data_332 = 0;
        timer_data_360 = 0;
        timer_data_390 = 0;
        timer_data_381 = 0;
        
        can_interrupt_enable_1();
        can_interrupt_enable_2();
      }
      return FALSE;
    }
    else {
      if(canErrorCounter[0] > 0) 
      {
        canErrorCounter[0]--;  
      }
    }
  }
  else if(channel == CAN_CHANNEL_2)
  {
    canTwoTxHeader.IDE = CAN_ID_EXT;
    canTwoTxHeader.RTR = CAN_RTR_DATA;
    canTwoTxHeader.ExtId = id;
    canTwoTxHeader.DLC = 8;
    
    canTwoTxData[0] = _data[0];
    canTwoTxData[1] = _data[1];
    canTwoTxData[2] = _data[2];
    canTwoTxData[3] = _data[3];
    canTwoTxData[4] = _data[4];
    canTwoTxData[5] = _data[5];
    canTwoTxData[6] = _data[6];
    canTwoTxData[7] = _data[7];
    
    canTwoTxHeader.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan2, &canTwoTxHeader, canTwoTxData, &canTwoTxMailbox) != HAL_OK)
    {
      canErrorCounter[1]++;
      if(canErrorCounter[1] > 20) 
      {
        HAL_CAN_DeInit(&hcan2);
        MX_CAN2_Init();
        canErrorCounter[1] = 0;
                
        timer_data_025 = 0;
        timer_data_035 = 0;
        timer_data_310 = 0;
        timer_data_311 = 0;
        timer_data_312 = 0;
        timer_data_317 = 0;
        timer_data_340 = 0;
        timer_data_350 = 0;
        timer_data_330 = 0;
        timer_data_331 = 0;
        timer_data_332 = 0;
        timer_data_360 = 0;
        timer_data_390 = 0;
        timer_data_381 = 0;
        
        can_interrupt_enable_2();
      }
      return FALSE;
    }
    else {
      if(canErrorCounter[1] > 0) 
      {
        canErrorCounter[1]--;  
      }
    }
  }
  return TRUE;
}

/* CAN Interrupt function */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canOneRxHeader, canOneRxData) != HAL_OK)
    {
      /* Reception Error */
    }
    J1939_message_handler(&canOneRxHeader, canOneRxData);
    canOneRxHeader.DLC = 0;
  }
  else if(hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &canTwoRxHeader, canTwoRxData) != HAL_OK)
    {
      /* Reception Error */
    }
    can_data_handler(&canTwoRxHeader, canTwoRxData);
    canTwoRxHeader.DLC = 0;
  }
}

void can_init()
{
  MX_CAN1_Init();
//  MX_CAN2_Init();     //tseveen 20250604
}

void can_start()
{
  can_interrupt_enable_1();
//  can_interrupt_enable_2();   //tseveen 20250604
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  // Filter CAN ID is updated on 2021.12.28 for mass product
  MX_CAN_FILTER_Init(0, 0, 0);

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  //can_interrupt_enable_1();
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 21;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  // Filter CAN ID is updated on 2021.12.28 for mass product
  MX_CAN_FILTER_Init(14, 0, 0);

  if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  //can_interrupt_enable_2();
  /* USER CODE END CAN2_Init 2 */

}

/* CAN FILTER init function*/
void MX_CAN_FILTER_Init(uint8_t filter, uint32_t id, uint32_t mask)
{
  CAN_FilterTypeDef  sFilterConfig;

  if(filter > 27){
    filter = 27;
  }
  
  sFilterConfig.FilterBank = filter;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                             // Changed on 2024.07.18
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
    
  id = (id << 3) | (1 << 2);                                    // IDE should be 1        
  mask = (mask << 3) | (1 << 2);                                // IDE should be 1
  
  sFilterConfig.FilterIdLow = id;
  sFilterConfig.FilterIdHigh = id >> 16;
  sFilterConfig.FilterMaskIdLow = id & mask;
  sFilterConfig.FilterMaskIdHigh = (id & mask) >> 16;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    // Filter configuration Error 
    Error_Handler();
  }
}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;
/**
* @brief CAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB13     ------> CAN2_TX
    PB5     ------> CAN2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void can_interrupt_enable_1()
{
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

void can_interrupt_enable_2()
{
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
}

/**
* @brief CAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcan: CAN handle pointer
* @retval None
*/
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB13     ------> CAN2_TX
    PB5     ------> CAN2_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_5);

    /* CAN2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }

}
/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupts.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}