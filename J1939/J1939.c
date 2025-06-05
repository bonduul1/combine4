
#include "J1939.h"


extern CAN_HandleTypeDef hcan2;

//-------------------------------------------   Address Claim Variables   -------------------------------------------
uint8_t   CA_Name[J1939_NAME_LENGTH];
uint8_t   J1939_Address;
uint8_t   J1939_Address_pref;
uint8_t   J1939_Address_available;

enum J1939_STATE J1939_state;
uint8_t sc_addrs[J1939_MAX_SC_ADDR - J1939_MIN_SC_ADDR + 1];

/* Netowrk management messages */
const struct j1939_mesg req_addr_claimed_mesg = {
	J1939_PGN_REQUEST,
	3,
	{(J1939_PGN_ADDR_CLAIMED >> 16) & 0xFF,
		(J1939_PGN_ADDR_CLAIMED >> 8) & 0xFF,
		J1939_PGN_ADDR_CLAIMED & 0xFF},
};
const struct j1939_mesg addr_claimed_mesg = {
	J1939_PGN_ADDR_CLAIMED,
	8,
	{0, 0, 0, 0, 0, 0, 0, 0},
};

/* Macros for going between CAN IDs and PDU/PGN fields */
#define J1939_PRI_POS	        26
#define J1939_PRI_MASK	        0x07U
#define J1939_PGN_POS	        8
#define J1939_PGN_MASK	        0x03FFFFLU
#define J1939_PGN1_MASK	        0x03FF00LU
#define J1939_PS_POS	        8
#define J1939_PS_MASK	        0xFFU
#define J1939_PF_POS	        16
#define J1939_PF_MASK	        0xFFU
#define J1939_SA_POS	        0
#define J1939_SA_MASK	        0xFFU
#define J1939_DP_POS	        24
#define J1939_DP_MASK	        0x01U
#define J1939_EDP_POS	        25
#define J1939_EDP_MASK	        0x01U

#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */

#define CANID(pri, pgn, da, sa)	( \
		CAN_EFF_FLAG | \
		((pri & J1939_PRI_MASK) << J1939_PRI_POS) | \
		((pgn & J1939_PGN_MASK) << J1939_PGN_POS) | \
		((da & J1939_PS_MASK) << J1939_PS_POS) | \
		((sa & J1939_SA_MASK) << J1939_SA_POS) )

#define ID_FIELD(id, field)	((id >> J1939_ ## field ## _POS) & J1939_ ## field ## _MASK)
#define PGN_FIELD(pgn, field)	ID_FIELD(pgn << J1939_PGN_POS, field)
#define J1939_MIN_PDU2	        240
#define ID_PDU_FMT(id)          (ID_FIELD(id, PF) < J1939_MIN_PDU2 ? 1 : 2)
#define PGN_PDU_FMT(pgn)	ID_PDU_FMT(pgn << J1939_PGN_POS)


uint8_t can_send()
{  
    CAN1_TxHeader.IDE = CAN_ID_EXT;
    CAN1_TxHeader.RTR = CAN_RTR_DATA;
    CAN1_TxHeader.TransmitGlobalTime = DISABLE;
    
    return HAL_CAN_AddTxMessage(&hcan1, &CAN1_TxHeader, CAN1_TxData, &CAN1_TxMailbox);
}
/*********************************************************************
CompareName

This routine compares the passed in array data NAME with the CA's
current NAME as stored in CA_Name.

Parameters:	unsigned char *		Array of NAME bytes
Return:		-1 - CA_Name is less than the data
		 	0  - CA_Name is equal to the data
			1  - CA_Name is greater than the data
*********************************************************************/
signed char CompareName( unsigned char *OtherName )
{
    unsigned char	i;

    for (i = 0; (i<J1939_NAME_LENGTH) && (OtherName[i] == CA_Name[i]); i++);

    if (i == J1939_NAME_LENGTH)
            return 0;
    else if (CA_Name[i] < OtherName[i] )
            return -1;
    else
            return 1;
}

/*********************************************************************
CopyName

This routine copies the CA's NAME into the message buffer's data array.
We can afford to make this a function, since another function is always
called after this is done, and we won't be using any additional stack
space.

Parameters:	None
Return:		None
*********************************************************************/
void CopyName()
{
    unsigned char i;

    for (i=0; i<J1939_NAME_LENGTH; i++)
      CAN1_TxData[i] = CA_Name[i];
}
/* Send request for address claimed message */
uint8_t j1939_send_req_addr_claim(uint8_t _address)
{
    CAN1_TxHeader.ExtId = CANID(J1939_ADDR_CLAIM_PRI, req_addr_claimed_mesg.pgn, J1939_GLOBAL_ADDR, _address);
    CAN1_TxHeader.DLC = req_addr_claimed_mesg.dlen;
    CAN1_TxData[0] = req_addr_claimed_mesg.data[0];
    CAN1_TxData[1] = req_addr_claimed_mesg.data[1];
    CAN1_TxData[2] = req_addr_claimed_mesg.data[2];
    return can_send();
}
/* Send request for address claimed message */
uint8_t j1939_send_addr_claimed(uint8_t _address)
{
    CAN1_TxHeader.ExtId = CANID(J1939_ADDR_CLAIM_PRI, addr_claimed_mesg.pgn, J1939_GLOBAL_ADDR, _address);
    CAN1_TxHeader.DLC = J1939_NAME_LENGTH;
    CopyName();
    return can_send();
}
/*
 * This 1-bit field indicates whether a CA can use an arbitrary source address to resolve an address claim
 * conflict. if this bit is set to "1", the CA will resolve an address conflict with a CA whose NAME has a
 * higher priority (lower numeric value) by selecting an arbitrary source address from the range 128 to 247
 * inclusive and claiming that source address. 
 * A self-configurable CA that computes its address but can claim only from a more restricted set of addresses
 * is not considered arbitrary address capable ( e.g. On-Highway Trailers. )
 */
uint8_t IsArbitraryAddressCapable(){
  return CA_Name[7] & 0x80;
}

uint8_t avail_sc_addr()
{
    int i;
    for(i = 0; i < J1939_MAX_SC_ADDR - J1939_MIN_SC_ADDR + 1; i++) {
      if(sc_addrs[i]) {
        return i + J1939_MIN_SC_ADDR;
      }
    }
    return J1939_NULL_ADDR;
}

/* This function is claiming the address */
int8_t j1939_claim_addr()
{
        uint32_t wait;
	
        J1939_Address_available = J1939_TRUE;
	/* Send request for address claimed message */
        J1939_state = J1939_WAIT_ADDR;
        
	j1939_send_req_addr_claim(J1939_NULL_ADDR);
        
	/* Wait until we have tried to claim an address */
	wait = J1939_ADDR_CLAIM_TIMEOUT;
        while((wait > 1) && (J1939_state == J1939_WAIT_ADDR)){
          user_delay(1);
          wait--;
        }
        
	if(J1939_state == J1939_LOST_ADDR)
            return -1;

	/* See if there was an address available */
	if(J1939_Address_pref != J1939_ANY_ADDR && J1939_Address_available)
		J1939_Address = J1939_Address_pref;
	else if(IsArbitraryAddressCapable() != 0)
		J1939_Address = avail_sc_addr();

	if(J1939_Address == J1939_NULL_ADDR) {
            /* Send request for address claimed message */
            J1939_state = J1939_LOST_ADDR;
            j1939_send_addr_claimed(J1939_NULL_ADDR);
            return -1;
	}

	/* Send address claimed message */
        J1939_state = J1939_WAIT_HAVE_ADDR;
	j1939_send_addr_claimed(J1939_Address );

	/* Set timer to give ECUs time to respond with address contentions */
	wait = J1939_ADDR_CLAIM_TIMEOUT;
	while((wait > 1) && (J1939_state == J1939_WAIT_HAVE_ADDR)){
          user_delay(1);
          wait--;
        }
        
	/* Check if we still have an address */
	if(J1939_state == J1939_LOST_ADDR) {
          return -1;
	}

	J1939_state = J1939_HAVE_ADDR;

	return 0;
}


/* Function for network management to process address claimed messages */
void j1939_addr_claimed_handler()
{
  uint8_t sa;
  /* Get source address of message */
  sa = ID_FIELD(CAN1_TxHeader.ExtId, SA);

  /* No action for cannot claim address messages */
  if(sa == J1939_NULL_ADDR)
      return;

  if(J1939_state == J1939_WAIT_ADDR) {
    /* Record occupied addresses in the self-configurable range */
    if(sa <= J1939_MAX_SC_ADDR && sa >= J1939_MIN_SC_ADDR) {
        sc_addrs[sa - J1939_MIN_SC_ADDR] = J1939_FALSE;
    }
    /* Determine whether or not preferred address is available */
    if(sa == J1939_Address_pref) {
      if(CompareName(CAN1_TxData) == 1){
        J1939_state = J1939_WAIT_HAVE_ADDR;

      } else {
        J1939_Address_available = J1939_FALSE;
        if(IsArbitraryAddressCapable() == 0)
          /* Send request for address claimed message */
          J1939_state = J1939_LOST_ADDR;
          j1939_send_addr_claimed(J1939_NULL_ADDR);
      }
    }
  } else {
    /* Determine if address must be given up */
    if(sa == J1939_Address) {
      if(CompareName(CAN1_TxData) != 1){
        /* Send request for address claimed message */
        //J1939_state = J1939_HAVE_ADDR;
        j1939_send_addr_claimed(J1939_Address);
      }else{
        /* Send request for address claimed message */
        J1939_state = J1939_LOST_ADDR;
        j1939_send_addr_claimed(J1939_NULL_ADDR);
      }
    }
  }
}

/* 
 * Function for network management to process request for address claimed messages
 * @return      : NULL
 * @hcan        : pointer (CanRxMsgTypeDef) of received CAN packet
 */
void j1939_req_addr_claimed_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
    /* check the received tx sock reference */
    if (J1939_state == J1939_WAIT_ADDR) {
      return;
    }
        
    /* Discard request for things besides address claimed */
    if((rx->DLC == 3) && (rx_data[0] == req_addr_claimed_mesg.data[0]) &&
       (rx_data[1] == req_addr_claimed_mesg.data[1]) && (rx_data[2] == req_addr_claimed_mesg.data[2])){
      /* Check if claimed address is mine */
      /* TODO: Should this check be done with filters? */
      if(ID_FIELD(rx->ExtId, PS) == J1939_Address || ID_FIELD(rx->ExtId, PS) == J1939_GLOBAL_ADDR) {
        //J1939_state = J1939_HAVE_ADDR;
        j1939_send_addr_claimed(J1939_Address);
      }
    }
}
/*********************************************************************
J1939_Initialization

This routine is called on system initialization.  It initializes
global variables, microcontroller peripherals, and interrupts.
It then starts the process of claiming the CA's address.
If the CA has a fixed NAME and Address, then it should call this
routine with a TRUE value passed in.  If it is Self-configurable or
Arbitrary Address Capable, then the CA should initialize NAME and
Address before calling this routine and call it with FALSE passed in.

NOTE: CA NAME is initialized by setting the CA_Name byte array.  The
Address is initialized by setting the value of J1939_Address.

NOTE: This routine will NOT enable global interrupts.  The CA needs
to do that when it's ready.

Parameters:		BOOL	Whether or not to initialize NAME and Address
						values.
Return:			None
*********************************************************************/
int8_t J1939_Initialization( uint8_t InitNAMEandAddress)
{
    // Initialize global variables;
    //J1939_Flags.FlagVal = 1;	// Cannot Claim Address, all other flags cleared.
  
    // Added following condition for condering saved address, 20190809, Enkhbat
  
    if((J1939_Address >= J1939_STARTING_ADDRESS) && (J1939_Address <= J1939_MAX_SC_ADDR)  && (InitNAMEandAddress == J1939_TRUE))
    {
      J1939_state = J1939_HAVE_ADDR;
    }else{      
      J1939_state = J1939_IDLE;
      J1939_Address = J1939_NULL_ADDR;
    }
    J1939_Address_pref = J1939_ANY_ADDR;
    
    if (InitNAMEandAddress)
    {
      J1939_Address_pref = J1939_STARTING_ADDRESS;
      CA_Name[7] = J1939_CA_NAME7;
      CA_Name[6] = J1939_CA_NAME6;
      CA_Name[5] = J1939_CA_NAME5;
      CA_Name[4] = J1939_CA_NAME4;
      CA_Name[3] = J1939_CA_NAME3;
      CA_Name[2] = J1939_CA_NAME2;
      CA_Name[1] = J1939_CA_NAME1;
      CA_Name[0] = J1939_CA_NAME0;
    }
    // all source addresses are available
    memset(sc_addrs, 1, sizeof(sc_addrs));
    
    // for testing
    // j1939_lose_addr(hcan);
    
    if((J1939_Address >= J1939_STARTING_ADDRESS) && (J1939_Address <= J1939_MAX_SC_ADDR) && (InitNAMEandAddress == J1939_TRUE) ){
      // we already have an address which was saved in non-volatile memory
      j1939_send_addr_claimed(J1939_Address);
      return 0;
    }else{
      return j1939_claim_addr();
    }
    
    // Start the process of claiming our address
    //j1939_send_addr_claimed();
}

//-------------------------------------------   General CAN Message handler function   -------------------------------------------

void J1939_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  switch(rx->ExtId){
    /*********************************************   J1939 Broadcast Announce message  ***********************************/
    case 0x1CECFF00:{ J1939_ECU_BAM_HEAD_message_handler(rx, rx_data); break; }
    case 0x1CEBFF00:{ J1939_ECU_BAM_DATA_message_handler(rx, rx_data); break; }
    case 0x18ECFF3D:{ J1939_DCU_BAM_HEAD_message_handler(rx, rx_data); break; }
    case 0x18EBFF3D:{ J1939_DCU_BAM_DATA_message_handler(rx, rx_data); break; }
    /*********************************************   J1939 Diagnostic Layer   ********************************************/
    case 0x18FECA00:{ J1939_DM1_ECU_message_handler(rx, rx_data); break; }
    case 0x18FECA3D:{ J1939_DM1_DCU_message_handler(rx, rx_data); break; }
    /*********************************************   J1939 Application Layer  ********************************************/
    case 0x14DC1D00:{ J1939_ATS_message_handler(rx, rx_data); break; }
    case 0x14DD001D:{ J1939_ATR_message_handler(rx, rx_data); break; }
    case 0x0CF00300:{ J1939_EEC2_message_handler(rx, rx_data); break; }
    case 0x0CF00400:{ J1939_EEC1_message_handler(rx, rx_data); break; }
//    case 0x18FD9200:{ J1939_EOI_message_handler(rx, rx_data); break; }               // not supported originally
    case 0x0CFEA400:{ J1939_ET2_message_handler(rx, rx_data); break; }
    case 0x0CFEDF00:{ J1939_EEC3_message_handler(rx, rx_data); break; }
//    case 0x18FEE400:{ J1939_SHUTDOWN_message_handler(rx, rx_data); break; }          // not supported originally
    case 0x18FEE500:{ J1939_HOURS_message_handler(rx, rx_data); break; }
//    case 0x18FEE7**:{ J1939_VH_message_handler(rx, rx_data); break; }                // not supported originally
    case 0x18FEEB00:{ J1939_CI_message_handler(rx, rx_data); break; }
    case 0x18FEEC00:{ J1939_VI_message_handler(rx, rx_data); break; }
//    case 0x18FEEC**:{ J1939_VI_message_handler(rx, rx_data); break; }                // not supported originally
    case 0x18FEEE00:{ J1939_ET1_message_handler(rx, rx_data); break; }
    case 0x18FEF500:{ J1939_AMB_message_handler(rx, rx_data); break; }
    case 0x18FEE900:{ J1939_LFC_message_handler(rx, rx_data); break; }
    case 0x18FEF200:{ J1939_LFE_message_handler(rx, rx_data); break; }
    case 0x18FEF700:{ J1939_VEP_message_handler(rx, rx_data); break; }
    case 0x18FD7C00:{ J1939_DPFC1_message_handler(rx, rx_data); break; }
    case 0x18FD7B00:{ J1939_AT1S_message_handler(rx, rx_data); break; }
//    case 0x18F001**:{ J1939_EBC1_message_handler(rx, rx_data); break; }              // see the default case
    case 0x10F01A00:{ J1939_TFAC_message_handler(rx, rx_data); break; }
    case 0x18FD9D00:{ J1939_SEP1_message_handler(rx, rx_data); break; }
    
    case 0x18FDB200:{ J1939_AT1IMG_message_handler(rx, rx_data); break; }
//    case 0x18FDB300:{ J1939_AT1OG2_message_handler(rx, rx_data); break; }            // not supported originally
    case 0x18FDB400:{ J1939_AT1IG2_message_handler(rx, rx_data); break; }
    case 0x18FEDB00:{ J1939_EFLP2_message_handler(rx, rx_data); break; }
    case 0x18FEF600:{ J1939_IC1_message_handler(rx, rx_data); break; }
    case 0x0CF0233D:{ J1939_A1SCRDSI1_message_handler(rx, rx_data); break; }
    case 0x14FD3E3D:{ J1939_A1SCREGT1_message_handler(rx, rx_data); break; }
    case 0x18FE563D:{ J1939_A1DEFT1I_message_handler(rx, rx_data); break; }
    case 0x18FD9BA3:{ J1939_A1DEFI_message_handler(rx, rx_data); break; }
    case 0x18FE56A3:{ J1939_AT1T1I_message_handler(rx, rx_data); break; }
    
    /*********************************************   Yanmar Original   ********************************************/
//    case 0x0CFF02**:{ J1939_YECR1_message_handler(rx, rx_data); break; }              // see the default case
    case 0x0CFF0C00:{ J1939_YECACK1_message_handler(rx, rx_data); break; }
    case 0x18FF1100:{ J1939_YIOS_message_handler(rx, rx_data); break; }
    case 0x0CFF1700:{ J1939_YLF_message_handler(rx, rx_data); break; }
//    case 0x18FF1C**:{ J1939_YEC_message_handler(rx, rx_data); break; }              // see the default case
//    case 0x0CFF1D**:{ J1939_YSTP_message_handler(rx, rx_data); break; }              // see the default case
//    case 0x18FF1E**:{ J1939_YRSS_message_handler(rx, rx_data); break; }            // not supported originally
    case 0x18FF1F00:{ J1939_YSRF_message_handler(rx, rx_data); break; }
    case 0x18FF2600:{ J1939_YSRSI_message_handler(rx, rx_data); break; }
    case 0x18FF2700:{ J1939_YESI_message_handler(rx, rx_data); break; }
//    case 0x18FF18**:{ J1939_YDPFIF_message_handler(rx, rx_data); break; }              // see the default case
    case 0x18FF2800:{ J1939_YEGRP_message_handler(rx, rx_data); break; }
    case 0x18FF2900:{ J1939_YETVP_message_handler(rx, rx_data); break; }
    case 0x18FF2A00:{ J1939_YATF_message_handler(rx, rx_data); break; }
    case 0x18FF2F00:{ J1939_YPMD_message_handler(rx, rx_data); break; }
    case 0x18FF2200:{ J1939_YDPFC1_message_handler(rx, rx_data); break; }
    case 0x18FF0F00:{ J1939_YPOI_message_handler(rx, rx_data); break; }
    case 0x18FF1000:{ J1939_YDPFC2_message_handler(rx, rx_data); break; }
    case 0x18FF1900:{ J1939_YEST_message_handler(rx, rx_data); break; }
    case 0x18FF0000:{ J1939_YMPR_message_handler(rx, rx_data); break; }
    case 0x18FF0100:{ J1939_YPOFS_message_handler(rx, rx_data); break; }
//    case 0x18FF0B00:{ J1939_YEOM_message_handler(rx, rx_data); break; }            // not supported originally
    case 0x18FF0E00:{ J1939_YINJQ_message_handler(rx, rx_data); break; }
    case 0x18FF033D:{ J1939_YSCRST_message_handler(rx, rx_data); break; }
    case 0x18FF043D:{ J1939_YSCRSTL1_message_handler(rx, rx_data); break; }
    case 0x18FF053D:{ J1939_YSCRSTL2_message_handler(rx, rx_data); break; }
    case 0x18FF063D:{ J1939_YSCRMOD_message_handler(rx, rx_data); break; }
    case 0x18FF073D:{ J1939_YSCRCTL3_message_handler(rx, rx_data); break; }
    case 0x18FF083D:{ J1939_YSCRHTR_message_handler(rx, rx_data); break; }
    case 0x18FF093D:{ J1939_YSCRSO_message_handler(rx, rx_data); break; }
    case 0x18FFF83D:{ J1939_YSCRIS_message_handler(rx, rx_data); break; }
    default:{ 
      uint32_t temp_id = (rx->ExtId) >> 8;
      switch(temp_id){
        case 0x0018F001:{ J1939_EBC1_message_handler(rx, rx_data); break; }
        case 0x000CFF02:{ J1939_YECR1_message_handler(rx, rx_data); break; }
        case 0x0018FF1C:{ J1939_YEC_message_handler(rx, rx_data); break; }
        case 0x000CFF1D:{ J1939_YSTP_message_handler(rx, rx_data); break; }
        case 0x0018FF18:{ J1939_YDPFIF_message_handler(rx, rx_data); break; }
        default:{ break; }
      }
      break; 
    }
  }
}
//-------------------------------------------   BAM Variables   -------------------------------------------
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

uint32_t ecu_bam_pgn;
uint16_t ecu_bam_timer;
uint16_t ecu_bam_messsage_size;
uint8_t  ecu_bam_flag;
uint8_t  ecu_bam_packet_size;
uint8_t  ecu_bam_packet_counter;

uint32_t ECU_SPN[MAXIMUM_DTC];
uint8_t  ECU_FMI[MAXIMUM_DTC];
uint8_t  ECU_Occurrence_count[MAXIMUM_DTC];
uint8_t  ECU_SPN_Conversion_method[MAXIMUM_DTC];
uint8_t  ECU_DTC_counter;
uint8_t  ECU_DTC_counter_received;
uint8_t  ECU_DTC_current;
uint8_t  ECU_error_flag;

uint32_t dcu_bam_pgn;
uint16_t dcu_bam_timer;
int16_t  dcu_bam_messsage_size;
uint8_t  dcu_bam_flag;
uint8_t  dcu_bam_packet_size;
uint8_t  dcu_bam_packet_counter;

uint32_t DCU_SPN[MAXIMUM_DTC];
uint8_t  DCU_FMI[MAXIMUM_DTC];
uint8_t  DCU_Occurrence_count[MAXIMUM_DTC];
uint8_t  DCU_SPN_Conversion_method[MAXIMUM_DTC];
uint8_t  DCU_DTC_counter;
uint8_t  DCU_DTC_counter_received;
uint8_t  DCU_DTC_current;
uint8_t  DCU_error_flag;

// uint8_t ecu_bam_packet_counter_local;

//-------------------------------------------   DM1 Variables   -------------------------------------------
/*
  DM1 = Diagnostic message 1  
  PGN = 65226
  Repetition time = 1000 ms
*/
uint8_t protect_lamp_status;
uint8_t amber_warning_lamp_status;
uint8_t red_stop_lamp_status;
uint8_t malfunction_indicator_lamp_status;

uint8_t flash_protect_lamp;
uint8_t flash_amber_warning_lamp;
uint8_t flash_red_stop_lamp;
uint8_t flash_malfunction_indicator_lamp;

uint8_t engine_check_lamp_bam;
uint8_t warning_check_lamp_bam;

//-------------------------------------------   Timer functions -------------------------------------------
uint8_t ecu_bam_timer_counter;
uint8_t dcu_bam_timer_counter;

void update_bam_timer()
{
  ecu_bam_timer++;
  if(ecu_bam_timer >= BAM_RECEIVER_TIME)
  {
    ecu_bam_timer = BAM_RECEIVER_TIME;
  }
  else
  {
    ecu_bam_timer_counter = 0;
  }

  dcu_bam_timer++;
  if(dcu_bam_timer >= BAM_RECEIVER_TIME) 
  { 
    dcu_bam_timer = BAM_RECEIVER_TIME;   
  }
  else
  {
    dcu_bam_timer_counter = 0; 
  }
}

//-------------------------------------------   BAM Functions   -------------------------------------------
void J1939_ECU_BAM_HEAD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  uint32_t bam_pgn;
  if(rx_data[0] == 0x20)                                                        // Control byte should be 32 == 0x20 --> BAM
  {
    bam_pgn = (rx_data[7] << 16) + (rx_data[6] << 8) + rx_data[5];
    ecu_bam_messsage_size = (rx_data[2] << 8) + rx_data[1];
    ecu_bam_packet_size = rx_data[3];

    
    if(ecu_bam_pgn == bam_pgn){
      return;                                                                   // Nothing do because the previous packet is not finished yet
    }
    // rx_data[4] is reserved on the J1939 standard
    // ecu_bam_packet_counter_local = 0;
    
    switch(bam_pgn)
    {
      case YMR_SCR_DATALOGING_5_PGN:
        // status message
        ecu_bam_pgn = bam_pgn;
        break;
        
      case J1939_DM1_PGN: 
        ECU_DTC_counter = 0;
        ecu_bam_timer = 0;
        ecu_bam_pgn = bam_pgn;
        break;
        
      default:
        break;
    }
    ecu_bam_flag = BAM_OK;
  }
}

void J1939_ECU_BAM_DATA_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  if(ecu_bam_timer >= BAM_RECEIVER_TIME)
  {
    ecu_bam_pgn = 0;
    return;                             // Timed out BAM message
  }
  
  if(ecu_bam_pgn == YMR_SCR_DATALOGING_5_PGN)
  {
    //if(rx_data[0] != (ecu_bam_packet_counter_local + 1)){
    //  return;                                                   // The packet number is different
    //}
    //ecu_bam_packet_counter_local = rx_data[0];
    if(rx_data[0] == 1)
    {
      // Engine check
      ecu_bam_timer = 0;
      if((rx_data[6] & 0x40) == 0x40)
      {
        if((rx_data[5] & 0x40) == 0)          engine_check_lamp_bam = 2;
        else                                  engine_check_lamp_bam = 0;
      }
      else
      {
        if((rx_data[5] & 0x40) == 0x40)       engine_check_lamp_bam = 1;
        else                                  engine_check_lamp_bam = 0;
      }
      // Check warning 
      warning_check_lamp_bam = (rx_data[6] & 0x0C) >> 2;
    }
    else if(rx_data[0] == 2)
    {
      ecu_bam_timer = 0;
      ecu_bam_pgn = 0;
    }
  }
  
  else if(ecu_bam_pgn == J1939_DM1_PGN)
  {
    //if(rx_data[0] != (ecu_bam_packet_counter_local + 1)){
    //  return;                                                   // The packet number is different
    //}
    //ecu_bam_packet_counter_local = rx_data[0];
    ecu_bam_packet_counter = rx_data[0];
    if(ecu_bam_packet_counter > ecu_bam_packet_size)
    {
      ecu_bam_pgn = 0;
      return;                                                                           // too much packet received
    }
    ecu_bam_timer = 0;
    if(ecu_bam_packet_counter == 1)
    {
      protect_lamp_status = (rx_data[1] & 0xC0) >> 6;
      amber_warning_lamp_status = (rx_data[1] & 0x30) >> 4;
      red_stop_lamp_status = (rx_data[1] & 0x0C) >> 2;
      malfunction_indicator_lamp_status = (rx_data[1] & 0x03);

      flash_protect_lamp = (rx_data[2] & 0xC0) >> 6;
      flash_amber_warning_lamp = (rx_data[2] & 0x30) >> 4;
      flash_red_stop_lamp = (rx_data[2] & 0x0C) >> 2;
      flash_malfunction_indicator_lamp = (rx_data[2] & 0x03);
      
      ecu_bam_messsage_size = ecu_bam_messsage_size - 2;
    }
    
    if((ecu_bam_packet_counter%4) == 1)
    {
      if(ecu_bam_packet_counter != 1)
      {                                                  // If this is first packet it includes Lamp status 1 & 2, otherwise there has another DTC
        ECU_SPN[ECU_DTC_counter] = ((rx_data[1] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
        ECU_FMI[ECU_DTC_counter] = (rx_data[1] & 0x1F);
        ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[2] & 0x7F);
        ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[2] & 0x80) >> 7;
        ECU_DTC_counter++;
        ecu_bam_messsage_size = ecu_bam_messsage_size - 2;
        if(ecu_bam_messsage_size <= 0)
        {                              // This is final packet
          ECU_DTC_counter_received = ECU_DTC_counter;
          ecu_bam_pgn = 0;
          return;
        }
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[3];
      ECU_SPN[ECU_DTC_counter] = (rx_data[4] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[5] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[5] & 0x1F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[6] & 0x80) >> 7;
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[6] & 0x7F);
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 4;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[7];
      ecu_bam_messsage_size = ecu_bam_messsage_size - 1;
    }
    
    else if((ecu_bam_packet_counter%4) == 2)
    {
      ECU_SPN[ECU_DTC_counter] = (rx_data[1] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[2] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[2] & 0x1F);
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[3] & 0x7F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[3] & 0x80) >> 7;
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 3;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[4];
      ECU_SPN[ECU_DTC_counter] = (rx_data[5] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[6] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[6] & 0x1F);
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[7] & 0x7F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[7] & 0x80) >> 7;
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 4;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
    }
    
    else if((ecu_bam_packet_counter%4) == 3)
    {
      ECU_SPN[ECU_DTC_counter] = rx_data[1];
      ECU_SPN[ECU_DTC_counter] = (rx_data[2] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[3] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[3] & 0x1F);
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[4] & 0x7F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[4] & 0x80) >> 7;
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 4;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[5];
      ECU_SPN[ECU_DTC_counter] = (rx_data[6] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[7] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[7] & 0x1F);
      ecu_bam_messsage_size = ecu_bam_messsage_size - 3;
    }
    
    else if((ecu_bam_packet_counter%4) == 0)
    {
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[1] & 0x7F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[1] & 0x80) >> 7;
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 1;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[2];
      ECU_SPN[ECU_DTC_counter] = (rx_data[3] << 8) | ECU_SPN[ECU_DTC_counter];
      ECU_SPN[ECU_DTC_counter] = ((rx_data[4] & 0xE0) << 11) | ECU_SPN[ECU_DTC_counter];
      ECU_FMI[ECU_DTC_counter] = (rx_data[4] & 0x1F);
      ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[5] & 0x7F);
      ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[5] & 0x80) >> 7;
      ECU_DTC_counter++;
      ecu_bam_messsage_size = ecu_bam_messsage_size - 4;
      if(ecu_bam_messsage_size <= 0)
      {                                // This is final packet
        ECU_DTC_counter_received = ECU_DTC_counter;
        ecu_bam_pgn = 0;
        return;
      }
      ECU_SPN[ECU_DTC_counter] = rx_data[6];
      ECU_SPN[ECU_DTC_counter] = (rx_data[7] << 8) | ECU_SPN[ECU_DTC_counter];
      ecu_bam_messsage_size = ecu_bam_messsage_size - 2;
    }
  }
}

void J1939_DCU_BAM_HEAD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  uint32_t bam_pgn;
  if(rx_data[0] == 0x20)                                                        // Control byte should be 32 == 0x20 --> BAM
  {
    bam_pgn = (rx_data[7] << 16) + (rx_data[6] << 8) + rx_data[5];
    dcu_bam_messsage_size = (rx_data[2] << 8) + rx_data[1];
    dcu_bam_packet_size = rx_data[3];

    
    if(dcu_bam_pgn == bam_pgn){
      return;                                                                   // Nothing do because the previous packet is not finished yet
    }
    // rx_data[4] is reserved on the J1939 standard
    // dcu_bam_packet_counter_local = 0;
    
    switch(bam_pgn)
    {
      case J1939_DM1_PGN: 
        dcu_bam_timer = 0;
        DCU_DTC_counter = 0;
        dcu_bam_pgn = bam_pgn;
        break;
      default:
        break;
    }
    dcu_bam_flag = BAM_OK;
  }
}

void J1939_DCU_BAM_DATA_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  if(dcu_bam_timer >= BAM_RECEIVER_TIME){
    dcu_bam_pgn = 0;
    return;                             // Timed out BAM message
  }
  
  if(dcu_bam_pgn == J1939_DM1_PGN){
    //if(rx_data[0] != (dcu_bam_packet_counter_local + 1)){
    //  return;                                                   // The packet number is different
    //}
    //dcu_bam_packet_counter_local = rx_data[0];
    dcu_bam_packet_counter = rx_data[0];
    if(dcu_bam_packet_counter > dcu_bam_packet_size){
      dcu_bam_pgn = 0;
      return;                                                                           // too much packet received
    }
    dcu_bam_timer = 0;
    if(dcu_bam_packet_counter == 1){
      protect_lamp_status = (rx_data[1] & 0xC0) >> 6;
      amber_warning_lamp_status = (rx_data[1] & 0x30) >> 4;
      red_stop_lamp_status = (rx_data[1] & 0x0C) >> 2;
      malfunction_indicator_lamp_status = (rx_data[1] & 0x03);

      flash_protect_lamp = (rx_data[2] & 0xC0) >> 6;
      flash_amber_warning_lamp = (rx_data[2] & 0x30) >> 4;
      flash_red_stop_lamp = (rx_data[2] & 0x0C) >> 2;
      flash_malfunction_indicator_lamp = (rx_data[2] & 0x03);
      
      dcu_bam_messsage_size = dcu_bam_messsage_size - 2;
    }
    
    if((dcu_bam_packet_counter%4) == 1)
    {
      if(dcu_bam_packet_counter != 1){                                                  // If this is first packet it includes Lamp status 1 & 2, otherwise there has another DTC
        DCU_SPN[DCU_DTC_counter] = ((rx_data[1] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
        DCU_FMI[DCU_DTC_counter] = (rx_data[1] & 0x1F);
        DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[2] & 0x7F);
        DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[2] & 0x80) >> 7;
        DCU_DTC_counter++;
        dcu_bam_messsage_size = dcu_bam_messsage_size - 2;
        if(dcu_bam_messsage_size <= 0){                              // This is final packet
          DCU_DTC_counter_received = DCU_DTC_counter;
          dcu_bam_pgn = 0;
          return;
        }
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[3];
      DCU_SPN[DCU_DTC_counter] = (rx_data[4] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[5] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[5] & 0x1F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[6] & 0x80) >> 7;
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[6] & 0x7F);
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 4;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[7];
      dcu_bam_messsage_size = dcu_bam_messsage_size - 1;
    }
    else if((dcu_bam_packet_counter%4) == 2)
    {
      DCU_SPN[DCU_DTC_counter] = (rx_data[1] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[2] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[2] & 0x1F);
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[3] & 0x7F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[3] & 0x80) >> 7;
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 3;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[4];
      DCU_SPN[DCU_DTC_counter] = (rx_data[5] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[6] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[6] & 0x1F);
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[7] & 0x7F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[7] & 0x80) >> 7;
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 4;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
    }
    else if((dcu_bam_packet_counter%4) == 3)
    {
      DCU_SPN[DCU_DTC_counter] = rx_data[1];
      DCU_SPN[DCU_DTC_counter] = (rx_data[2] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[3] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[3] & 0x1F);
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[4] & 0x7F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[4] & 0x80) >> 7;
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 4;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[5];
      DCU_SPN[DCU_DTC_counter] = (rx_data[6] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[7] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[7] & 0x1F);
      dcu_bam_messsage_size = dcu_bam_messsage_size - 3;
    }
    else if((dcu_bam_packet_counter%4) == 0)
    {
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[1] & 0x7F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[1] & 0x80) >> 7;
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 1;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[2];
      DCU_SPN[DCU_DTC_counter] = (rx_data[3] << 8) | DCU_SPN[DCU_DTC_counter];
      DCU_SPN[DCU_DTC_counter] = ((rx_data[4] & 0xE0) << 11) | DCU_SPN[DCU_DTC_counter];
      DCU_FMI[DCU_DTC_counter] = (rx_data[4] & 0x1F);
      DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[5] & 0x7F);
      DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[5] & 0x80) >> 7;
      DCU_DTC_counter++;
      dcu_bam_messsage_size = dcu_bam_messsage_size - 4;
      if(dcu_bam_messsage_size <= 0){                                // This is final packet
        DCU_DTC_counter_received = DCU_DTC_counter;
        dcu_bam_pgn = 0;
        return;
      }
      DCU_SPN[DCU_DTC_counter] = rx_data[6];
      DCU_SPN[DCU_DTC_counter] = (rx_data[7] << 8) | DCU_SPN[DCU_DTC_counter];
      dcu_bam_messsage_size = dcu_bam_messsage_size - 2;
    }
  }
}

//-------------------------------------------   DM1 Functions   -------------------------------------------
char error_message_ecu[8];
uint8_t ecu_or_dcu_dtc;
  
uint8_t j1939EngineTempratureError;
uint8_t j1939WaterSeparator;
uint8_t j1939AirFilter;
uint8_t j1939OilPressure;

void init_error_msg_ecu()
{
  uint8_t i;
  for(i = 0; i < 8; i++){
    error_message_ecu[i] = ' ';
  }
  error_message_ecu[7] = 0;
  
  j1939EngineTempratureError = 0;
  j1939WaterSeparator = 0;
  j1939AirFilter = 0;
  j1939OilPressure = 0;
  
  DCU_DTC_current = 0;
  ECU_DTC_current = 0;
  
  ECU_error_flag = 0;
  DCU_error_flag = 0;
  
  ecu_or_dcu_dtc = 0;
}

void dtc_process()
{
  uint16_t i = 0;
      
  if(flagTimer.oneSecond == FALSE)
  {
    return;
  }
  
  if(ecu_bam_timer_counter++ >= 3)
  {
    ecu_bam_timer_counter = 3; 
    if(ECU_error_flag == 0) 
    {
      ECU_DTC_counter_received = 0;
    }
  }
  
  if(dcu_bam_timer_counter++ >= 3)
  {
    dcu_bam_timer_counter = 3; 
    if(DCU_error_flag == 0)
    {
      DCU_DTC_counter_received = 0; 
    }
  }
  
  for(i = 0; i < 8; i++)
  {
    error_message_ecu[i] = ' ';
  }
  error_message_ecu[7] = 0;
  
  if((DCU_DTC_counter_received == 0) && (ECU_DTC_counter_received == 0))
  {
    j1939EngineTempratureError = 0;
    j1939WaterSeparator = 0;
    j1939AirFilter = 0;
    j1939OilPressure = 0;
    
    DCU_DTC_current = 0;
    ECU_DTC_current = 0;
    return;
  }
  else if(DCU_DTC_counter_received == 0) {
    ecu_or_dcu_dtc = 1;                         // Select ECU
  }
  else if(ECU_DTC_counter_received == 0) {
    ecu_or_dcu_dtc = 0;                         // Select DCU
  }
  else {
    // DCU and ECU both send errors
  }

  if((DCU_DTC_counter_received > 0) && (ecu_or_dcu_dtc == 0))                   // DCU selected and Received error
  {
    DCU_DTC_current++;
    if(DCU_DTC_current >= DCU_DTC_counter_received)
    {
      ecu_or_dcu_dtc = 1;                                                       // ECU selected
      DCU_DTC_current = 0;
    }
    for(i = 0; i < NUMBER_OF_PCODE; i++)
    {
      //if(pcode_pgn[i] != 65535)                                                   // Added on 9 April 2021, this condition is needed because of there is lot of unused PCODE
      {
        if(DCU_SPN[DCU_DTC_current] == pcode_spn[i]) 
        {
          if(DCU_FMI[DCU_DTC_current] == pcode_fmi[i])
          {
            // Updated on 2021.06.29
            error_message_ecu[0] = 'E';;
            error_message_ecu[1] = '-';
            error_message_ecu[2] = pcode[i][0];
            error_message_ecu[3] = pcode[i][1];
            error_message_ecu[4] = pcode[i][2];
            error_message_ecu[5] = pcode[i][3];
            error_message_ecu[6] = pcode[i][4];
            error_message_ecu[7] = 0;
            
            if((pcode[i][0] == 'P') && (pcode[i][1] == '0') && (pcode[i][2] == '2') && (pcode[i][3] == '1') && (pcode[i][4] == '7')) 
              j1939EngineTempratureError = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '5') && (pcode[i][4] == '1'))
              j1939WaterSeparator = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '9') && (pcode[i][4] == '2'))
              j1939OilPressure = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '0') && (pcode[i][4] == '1'))
              j1939AirFilter = 1;
            break;
          }
        }
      }
    }
  }  
  else if((ECU_DTC_counter_received > 0))// && (ecu_or_dcu_dtc == 1))              // ECU Selected
  {
    ECU_DTC_current++;
    if(ECU_DTC_current >= ECU_DTC_counter_received){
      ecu_or_dcu_dtc = 0;                                                       // DCU Selected
      ECU_DTC_current = 0;
    }
    for(i = 0; i < NUMBER_OF_PCODE; i++)
    {
      //if(pcode_pgn[i] != 65535)                                                   // Added on 9 April 2021, this condition is needed because of there is lot of unused PCODE
      {
        if(ECU_SPN[ECU_DTC_current] == pcode_spn[i]) 
        {
          if(ECU_FMI[ECU_DTC_current] == pcode_fmi[i])
          {
            // Updated on 2021.06.29
            error_message_ecu[0] = 'E';
            error_message_ecu[1] = '-';
            error_message_ecu[2] = pcode[i][0];
            error_message_ecu[3] = pcode[i][1];
            error_message_ecu[4] = pcode[i][2];
            error_message_ecu[5] = pcode[i][3];
            error_message_ecu[6] = pcode[i][4];
            error_message_ecu[7] = 0;
            
            if((pcode[i][0] == 'P') && (pcode[i][1] == '0') && (pcode[i][2] == '2') && (pcode[i][3] == '1') && (pcode[i][4] == '7')) 
              j1939EngineTempratureError = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '5') && (pcode[i][4] == '1'))
              j1939WaterSeparator = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '9') && (pcode[i][4] == '2'))
              j1939OilPressure = 1;
            else if((pcode[i][0] == 'P') && (pcode[i][1] == '1') && (pcode[i][2] == '1') && (pcode[i][3] == '0') && (pcode[i][4] == '1'))
              j1939AirFilter = 1;
            break;
          }
        }
      }
    }
  }
  
  error_message_ecu[7] = 0;
}

void J1939_DM1_ECU_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  uint32_t local_spn;
  
  local_spn = rx_data[4] & 0xE0;
  local_spn = (local_spn << 3) | rx_data[3];
  local_spn = (local_spn << 8) | rx_data[2];

  protect_lamp_status = (rx_data[0] & 0xC0) >> 6;
  amber_warning_lamp_status = (rx_data[0] & 0x30) >> 4;
  red_stop_lamp_status = (rx_data[0] & 0x0C) >> 2;
  malfunction_indicator_lamp_status = (rx_data[0] & 0x03);

  flash_protect_lamp = (rx_data[1] & 0xC0) >> 6;
  flash_amber_warning_lamp = (rx_data[1] & 0x30) >> 4;
  flash_red_stop_lamp = (rx_data[1] & 0x0C) >> 2;
  flash_malfunction_indicator_lamp = (rx_data[1] & 0x03);
  
  if(local_spn == 0)
  {
    ECU_error_flag = 0;
    return;             // there is no error
  }
  
  if(ECU_error_flag == 1)                                                       // This condition is added on 2024.10.21
    return;
  
  ECU_error_flag = 1;
  ECU_DTC_counter = 0;
  
  ECU_SPN[ECU_DTC_counter] = local_spn;
  ECU_FMI[ECU_DTC_counter] = (rx_data[4] & 0x1F);
  ECU_Occurrence_count[ECU_DTC_counter] = (rx_data[5] & 0x7F);
  ECU_SPN_Conversion_method[ECU_DTC_counter] = (rx_data[5] & 0x80) >> 7;
  ECU_DTC_counter++;
  ECU_DTC_counter_received = ECU_DTC_counter;
}

void J1939_DM1_DCU_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  uint32_t local_spn;
  
  local_spn = rx_data[4] & 0xE0;
  local_spn = (local_spn << 3) | rx_data[3];
  local_spn = (local_spn << 8) | rx_data[2];
  
  if(local_spn == 0){
    DCU_error_flag = 0;
    return;             // there is no error
  }
  
  if(DCU_error_flag == 1)                                                       // This condition is added on 2024.10.21
    return;
  
  // Only one error is occured so
  DCU_error_flag = 1;
  DCU_DTC_counter = 0;
  
  DCU_SPN[DCU_DTC_counter] = local_spn;
  DCU_FMI[DCU_DTC_counter] = (rx_data[4] & 0x1F);
  DCU_Occurrence_count[DCU_DTC_counter] = (rx_data[5] & 0x7F);
  DCU_SPN_Conversion_method[DCU_DTC_counter] = (rx_data[5] & 0x80) >> 7;
  DCU_DTC_counter++;
  DCU_DTC_counter_received = DCU_DTC_counter;
}
//-------------------------------------------   ATS Variables   -------------------------------------------
/*
  ATS = Anti-theft status
  PGN = 56320
  Repetition time = As needed
*/
uint8_t ats_encryption_seed_present_indicator;
uint8_t ats_password_valid_indicator;
uint8_t ats_component_status_states;
uint8_t ats_modify_password_state;
uint8_t ats_random_number_ascii[7];
uint32_t ats_random_number;

//-------------------------------------------   ATS Functions   -------------------------------------------
void J1939_ATS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  uint8_t i;
  ats_encryption_seed_present_indicator = (rx_data[0] & 0x03);
  ats_password_valid_indicator = (rx_data[0] & 0x0C) >> 2;
  ats_component_status_states = (rx_data[0] & 0x30) >> 4;
  ats_modify_password_state = (rx_data[0] & 0xC0) >> 6;
  ats_random_number = 0;
  for(i = 0; i < 7; i++){
    ats_random_number_ascii[i] = rx_data[i + 1];
    ats_random_number = ats_random_number * 10 + (ats_random_number_ascii[i] - 0x30);
  }
}

//-------------------------------------------   ATR Variables   -------------------------------------------
/*
  ATR = Anti-theft request
  PGN = 56576
  Repetition time = As needed
*/
uint8_t atr_encryption_indicator_state;
uint8_t atr_desired_exit_mode_state;
uint8_t atr_command_state;
uint8_t atr_password_representation_ascii[7];
uint32_t atr_password_representation;
//-------------------------------------------   ATR Functions   -------------------------------------------
void J1939_ATR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  uint8_t i;
  atr_encryption_indicator_state = (rx_data[0] & 0x06) >> 1;
  atr_desired_exit_mode_state = (rx_data[0] & 0x18) >> 3;
  atr_command_state = (rx_data[0] & 0xE0) >> 5;

  atr_password_representation = 0;
  for(i = 0; i < 7; i++){
    atr_password_representation_ascii[i] = rx_data[i + 1];
    atr_password_representation = atr_password_representation * 10 + (atr_password_representation_ascii[i] - 0x30);
  }
}
//-------------------------------------------   EEC2 Variables   -------------------------------------------
/*
  EEC2 = Electronic Engine Controller 2
  PGN = 61443
  Repetition time = 50 ms
*/
uint8_t accelerator_pedal_low_idle_switch;
uint8_t accelerator_pedal_kickdown_switch;
uint8_t road_speed_limit_status;
uint8_t accelerator_pedal2_low_idle_switch;
uint8_t accelerator_pedal_position;
uint8_t percent_load_at_current_speed;
uint8_t remote_accelerator;
uint8_t accelerator_pedal2_position;
uint8_t vehicle_acceleration_rate_limit_status;
uint8_t actual_maximum_available_engine_percent_torque;

//-------------------------------------------   EEC2 Function   -------------------------------------------
void J1939_EEC2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  accelerator_pedal_low_idle_switch = (rx_data[0] & 0x03);                                                   // Not Available
  accelerator_pedal_kickdown_switch = (rx_data[0] & 0x0C) >> 2;                                              // Not Available
  road_speed_limit_status = (rx_data[0] & 0x30) >> 4;                                                        // Not Available
  accelerator_pedal2_low_idle_switch = (rx_data[0] & 0xC0) >> 6;                                             // Not Available
  accelerator_pedal_position = rx_data[1];
  percent_load_at_current_speed = rx_data[2];
  remote_accelerator = rx_data[3];                                                                           // Not Available
  accelerator_pedal2_position = rx_data[4];                                                                  // Not Available
  vehicle_acceleration_rate_limit_status = rx_data[5] & 0x03;                                                // Not Available
  actual_maximum_available_engine_percent_torque = rx_data[6];                                               // Not Available
  //rx_data[7] is NOT defined.
}

//-------------------------------------------   EEC1 Variables   -------------------------------------------
/*
  EEC1 = Electronic Engine Controller 1
  PGN = 61444
  Repetition time = 20 ms
*/
uint8_t engine_torque_mode;
uint8_t actual_engine_percent_torque_high_resolution;
uint8_t driver_demand_engine_percent_torque;
uint8_t actual_engine_percent_torque;
uint16_t engine_speed;
uint8_t source_address_of_controlling_device_for_engine_control;
uint8_t engine_starter_mode;
uint8_t engine_demand_percent_torque;

//-------------------------------------------   EEC1 Function   -------------------------------------------
void J1939_EEC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data)
{
  engine_torque_mode = (rx_data[0] & 0x0F);                                                                  // Not Available
  actual_engine_percent_torque_high_resolution = (rx_data[0] & 0xF0) >> 4;                                   // Not Available
  driver_demand_engine_percent_torque = rx_data[1];                                                          // Not Available
  actual_engine_percent_torque = rx_data[2];                                                                 // Not Available
  //engine_speed = (rx_data[4] << 8) + rx_data[3]; // tseveen 20250422
  source_address_of_controlling_device_for_engine_control = rx_data[5];                                      // Not Available
  engine_starter_mode = rx_data[6] & 0x0F;
  engine_demand_percent_torque = rx_data[7];                                                                 // Not Available
}

//-------------------------------------------   ET2 Variables   -------------------------------------------
/*
  ET2 = Engine temperature 2
  PGN = 65188
  Repetition time = 1000 ms
*/
uint16_t engine_oil_temperature_2;
uint16_t engine_ecu_temperature;
uint16_t engine_egr_differential_pressure;
uint16_t engine_egr_temperature;
//-------------------------------------------   ET2 Function   -------------------------------------------
void J1939_ET2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_oil_temperature_2 = (rx_data[1] << 8) + rx_data[0];                                             // Not Available
  engine_ecu_temperature = (rx_data[3] << 8) + rx_data[2];                                                // Not Available
  engine_egr_differential_pressure = (rx_data[5] << 8) + rx_data[4];
  engine_egr_temperature = (rx_data[7] << 8) + rx_data[6];
}
//-------------------------------------------   EEC3 Variables   -------------------------------------------
/*
  ET2 = Electronic Engine Controller 3
  PGN = 65247
  Repetition time = 250 ms
*/
uint8_t nominal_friction_percent_torque;
uint16_t engine_desired_operating_speed;
uint8_t engine_desired_operating_speed_asymmetry_adjustment;
uint8_t estimated_engine_parasitic_losses_percent_torque;
uint16_t aftertreatment_1_exhaust_gas_mass_flow;
uint8_t aftertreatment_1_intake_dew_point;
uint8_t aftertreatment_1_exhaust_dew_point;
uint8_t aftertreatment_2_intake_dew_point;
uint8_t aftertreatment_2_exhaust_dew_point;

//-------------------------------------------   EEC3 Function   -------------------------------------------
void J1939_EEC3_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  nominal_friction_percent_torque = rx_data[0];                                                                      // Not Available
  engine_desired_operating_speed = (rx_data[2] << 8) + rx_data[1];
  engine_desired_operating_speed = (uint16_t)((float)engine_desired_operating_speed * 0.125);
    
  engine_desired_operating_speed_asymmetry_adjustment = rx_data[3];                                                  // Not Available
  estimated_engine_parasitic_losses_percent_torque = rx_data[4];                                                     // Not Available
  aftertreatment_1_exhaust_gas_mass_flow = (rx_data[6] << 8) + rx_data[5];                                        // Not Available
  aftertreatment_1_intake_dew_point = (rx_data[7] & 0xC0) >> 6;                                                      // Not Available
  aftertreatment_1_exhaust_dew_point = (rx_data[7] & 0x30) >> 4;                                                     // Not Available
  aftertreatment_2_intake_dew_point = (rx_data[7] & 0x0C) >> 2;                                                      // Not Available
  aftertreatment_2_exhaust_dew_point = (rx_data[7] & 0x03);                                                          // Not Available
}

//-------------------------------------------   HOURS Variables   -------------------------------------------
/*
  HOURS = Engine hours, revolutions
  PGN = 65253
  Repetition time = using PGN 59904
*/
uint32_t total_engine_hours;
uint32_t total_engine_revolution;
//-------------------------------------------   HOURS Function   -------------------------------------------
void J1939_HOURS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  total_engine_hours = (rx_data[3] << 24) + (rx_data[2]<<16) + (rx_data[1] << 8) + rx_data[0];
  total_engine_revolution = (rx_data[7] << 24) + (rx_data[6]<<16) + (rx_data[5] << 8) + rx_data[4];         // Not Available
}

//-------------------------------------------   VH Variables   -------------------------------------------
/*
  VH = Vehicle hours
  PGN = 65255
  Repetition time = 1000 ms
*/
uint32_t total_vehicle_hours;
uint32_t total_power_takeoff_hours;
//-------------------------------------------   VH Function   -------------------------------------------
void J1939_VH_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  total_vehicle_hours = (rx_data[3] << 24) + (rx_data[2]<<16) + (rx_data[1] << 8) + rx_data[0];             // Not Available
  total_power_takeoff_hours = (rx_data[7] << 24) + (rx_data[6]<<16) + (rx_data[5] << 8) + rx_data[4];       // Not Available
}

//-------------------------------------------   CI Variables   -------------------------------------------
/*
  CI = Component Identification
  PGN = 65259
  Repetition time = using PGN 59904
*/
uint8_t ci_make[5];
uint8_t ci_delimiter_1;
uint8_t ci_engine_model_number[20];
uint8_t ci_delimiter_2;
uint8_t ci_engine_serial_number[20];
uint8_t ci_delimiter_3;
uint8_t ci_ecu_number[14];
uint8_t ci_delimiter_4;

//-------------------------------------------   CI Function   -------------------------------------------
void J1939_CI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){

}

//-------------------------------------------   VI Variables   -------------------------------------------
/*
  VI = Vehicle Identification
  PGN = 65260
  Repetition time = using PGN 59904
*/
uint8_t vi_vehicle_identification_number[32];
uint8_t vi_delimiter_1;

//-------------------------------------------   CI Function   -------------------------------------------
void J1939_VI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){

}

//-------------------------------------------   ET1 Variables   -------------------------------------------
/*
  ET1 = Engine temperature 1
  PGN = 65262
  Repetition time = 1000 ms
*/
uint8_t engine_coolant_temperature;
uint8_t fuel_temperature;
uint16_t engine_oil_temperature;
uint16_t turbo_oil_temperature;
uint8_t engine_intercooler_temperature;
uint8_t engine_intercooler_thermostat_opening;
uint8_t ET1_message_flag;
//-------------------------------------------   ET1 Function   -------------------------------------------
void J1939_ET1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_coolant_temperature = rx_data[0];
  fuel_temperature = rx_data[1];
  engine_oil_temperature = (rx_data[3] << 8) + rx_data[2];                                                // Not Available
  turbo_oil_temperature = (rx_data[5] << 8) + rx_data[4];                                                 // Not Available
  engine_intercooler_temperature = rx_data[6];                                                               // Not Available
  engine_intercooler_thermostat_opening = rx_data[7];                                                        // Not Available
  ET1_message_flag = 1;
}

//-------------------------------------------   AMB Variables   -------------------------------------------
/*
  AMB = Ambient conditions
  PGN = 65269
  Repetition time = 1000 ms
*/
uint8_t barometric_pressure;
uint16_t cab_interior_temperature;
uint16_t ambient_air_temperature;
uint8_t engine_air_inlet_temperature;
uint16_t road_surface_temperature;

//-------------------------------------------   AMB Function   -------------------------------------------
void J1939_AMB_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  barometric_pressure = rx_data[0];
  cab_interior_temperature = (rx_data[2] << 8) + rx_data[1];                                              // Not Available
  ambient_air_temperature = (rx_data[4] << 8) + rx_data[3];
  engine_air_inlet_temperature = rx_data[5];                                                                 // Not Available
  road_surface_temperature = (rx_data[7] << 8) + rx_data[6];                                              // Not Available
}

//-------------------------------------------   LFC Variables   -------------------------------------------
/*
  LFC = Liquid Fuel Consumption
  PGN = 65257
  Repetition time = 1000 ms
*/
uint32_t trip_fuel;
uint32_t total_trip_fuel;

//-------------------------------------------   LFC Function   -------------------------------------------
void J1939_LFC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  trip_fuel = (rx_data[3] << 24) + (rx_data[2]<<16) + (rx_data[1] << 8) + rx_data[0];                       // Not available
  total_trip_fuel = (rx_data[7] << 24) + (rx_data[6] << 16) + (rx_data[5] << 8) + rx_data[4];
}

//-------------------------------------------   LFE Variables   -------------------------------------------
/*
  LFE = Liquid Fuel Economy
  PGN = 65266
  Repetition time = 100 ms
*/
uint16_t fuel_rate;
uint16_t instantaneous_fuel_economy;
uint16_t average_fuel_economy;
uint8_t throttle_position;

//-------------------------------------------   LFE Function   -------------------------------------------
void J1939_LFE_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  fuel_rate = (rx_data[1] << 8) + rx_data[0];
  instantaneous_fuel_economy = (rx_data[3] << 8) + rx_data[2];                                                    // Not available
  average_fuel_economy = (rx_data[5] << 8) + rx_data[4];                                                          // Not available
  throttle_position = rx_data[6];
}

//-------------------------------------------   VEP Variables   -------------------------------------------
/*
  VEP = Vehicle Electrical Power
  PGN = 65271
  Repetition time = 1000 ms
*/
uint8_t net_battery_current;
uint16_t alternator_potential_voltage;
uint16_t electrical_potential_voltage;
uint16_t battery_potential_voltage_switched;
uint8_t VEP_message_flag;
//-------------------------------------------   VEP Function   -------------------------------------------
void J1939_VEP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  net_battery_current = rx_data[0];                                                                          // Not available
  //rx_data[1] --> NOT defined
  alternator_potential_voltage = (rx_data[3] << 8) + rx_data[2];                                          // Not available
  electrical_potential_voltage = (rx_data[5] << 8) + rx_data[4];                                          // Not available
  battery_potential_voltage_switched = (rx_data[7] << 8) + rx_data[6];
  VEP_message_flag = 1;
}

//-------------------------------------------   DPFC1 Variables   -------------------------------------------
/*
  DPFC1 = Diesel Particulate Filter Control 1
  PGN = 64892
  Repetition time = 1000 ms
*/
uint8_t diesel_particulate_filter_lamp_command;
uint8_t diesel_particulate_filter_passive_regeneration_status;
uint8_t diesel_particulate_filter_active_regeneration_status;
uint8_t diesel_particulate_filter_status;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_status;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_inhibit_switch;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_clutch_disengaged;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_service_brake_active;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_PTO_active;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_accelerator_pedal_off_idle;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_out_of_neutral;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_vehicle_speed_above_allowed_speed;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_parking_brake_not_set;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_low_exhaust_gas_temperature;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_system_fault_active;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_system_timeout;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_temporary_system_lockout;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_permanent_system_lockout;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_engine_not_warmed_up;
uint8_t diesel_particulate_filter_active_regeneration_inhibited_vehicle_speed_below_allowed_speed;
uint8_t diesel_particulate_filter_automatic_active_regeneration_initiation_configuration;
uint8_t exhaust_system_high_temperature_lamp_command;
uint8_t diesel_particulate_filter_active_regeneration_forced_status;

//-------------------------------------------   DPFC1 Function   -------------------------------------------
void J1939_DPFC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  diesel_particulate_filter_lamp_command = rx_data[0] & 0x07;
  diesel_particulate_filter_passive_regeneration_status = (rx_data[1] & 0x03);                                                       // Not available
  diesel_particulate_filter_active_regeneration_status = (rx_data[1] & 0x0C) >> 2;
  diesel_particulate_filter_status = (rx_data[1] & 0x70) >> 4;
  diesel_particulate_filter_active_regeneration_inhibited_status = (rx_data[2] & 0x03);
  diesel_particulate_filter_active_regeneration_inhibited_inhibit_switch = (rx_data[2] & 0x0C) >> 2;
  diesel_particulate_filter_active_regeneration_inhibited_clutch_disengaged = (rx_data[2] & 0x30) >> 4;                              // Not available
  diesel_particulate_filter_active_regeneration_inhibited_service_brake_active = (rx_data[2] & 0xC0) >> 6;                           // Not available
  diesel_particulate_filter_active_regeneration_inhibited_PTO_active = (rx_data[3] & 0x03);                                          // Not available
  diesel_particulate_filter_active_regeneration_inhibited_accelerator_pedal_off_idle = (rx_data[3] & 0x0C) >> 2;
  diesel_particulate_filter_active_regeneration_inhibited_out_of_neutral = (rx_data[3] & 0x30) >> 4;                                 // Not available
  diesel_particulate_filter_active_regeneration_inhibited_vehicle_speed_above_allowed_speed = (rx_data[3] & 0xC0) >> 6;              // Not available
  
  diesel_particulate_filter_active_regeneration_inhibited_parking_brake_not_set = (rx_data[4] & 0x03);                               // Not available
  diesel_particulate_filter_active_regeneration_inhibited_low_exhaust_gas_temperature = (rx_data[4] & 0x0C) >> 2;                    // Not available
  diesel_particulate_filter_active_regeneration_inhibited_system_fault_active = (rx_data[4] & 0x30) >> 4;                            // Not available
  diesel_particulate_filter_active_regeneration_inhibited_system_timeout = (rx_data[4] & 0xC0) >> 6;                                 // Not available
  diesel_particulate_filter_active_regeneration_inhibited_temporary_system_lockout = (rx_data[5] & 0x03);                            // Not available
  diesel_particulate_filter_active_regeneration_inhibited_permanent_system_lockout = (rx_data[5] & 0x0C) >> 2;                       // Not available
  diesel_particulate_filter_active_regeneration_inhibited_engine_not_warmed_up = (rx_data[5] & 0x30) >> 4;
  diesel_particulate_filter_active_regeneration_inhibited_vehicle_speed_below_allowed_speed = (rx_data[5] & 0xC0) >> 6;              // Not available
  diesel_particulate_filter_automatic_active_regeneration_initiation_configuration = (rx_data[6] & 0x03);                            // Not available
  exhaust_system_high_temperature_lamp_command = (rx_data[6] & 0x1C) >> 2;
  diesel_particulate_filter_active_regeneration_forced_status = (rx_data[6] & 0xE0) >> 5;
  //rx_data[7] NOT USED
}

//-------------------------------------------   AT1S Variables   -------------------------------------------
/*
  AT1S = Aftertreatment 1 Service
  PGN = 64891
  Repetition time = using PGN 59904
*/
uint8_t diesel_particulate_filter1_soot_load_percent;
uint8_t diesel_particulate_filter1_ash_load_percent;
uint32_t diesel_particulate_filter1_time_since_last_active_regeneration;

//-------------------------------------------   AT1S Function   -------------------------------------------
void J1939_AT1S_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  diesel_particulate_filter1_soot_load_percent = (rx_data[0]);
  diesel_particulate_filter1_ash_load_percent = (rx_data[1]);
  diesel_particulate_filter1_time_since_last_active_regeneration = (rx_data[5] << 24) + (rx_data[4]<<16) + (rx_data[3] << 8) + rx_data[2];
  // rx_data[6] & rx_data[7] are NOT defined
}

//-------------------------------------------   EBC1 Variables   -------------------------------------------
/*
  EBC1 = Electronic Break Controller 1
  PGN = 61441
  Repetition time = 100 ms
*/
uint8_t accelerator_interlock_switch;
uint8_t auxiliary_engine_shutdown_switch;

//-------------------------------------------   EBC1 Function   -------------------------------------------
void J1939_EBC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  accelerator_interlock_switch = (rx_data[3] & 0x03);                                                        // Not Available
  auxiliary_engine_shutdown_switch = (rx_data[3] & 0x30) >> 4;
  // Other bits and bytes are NOT used
}

//-------------------------------------------   TFAC Variables   -------------------------------------------
/*
  TFAC = Engine Throttle/Fuel Actuator Control Command
  PGN = 61466
  Repetition time = 50 ms
*/
uint16_t engine_throttle_actuator1_control_command;
uint16_t engine_throttle_actuator2_control_command;
uint16_t engine_fuel_actuator1_control_command;
uint16_t engine_fuel_actuator2_control_command;

//-------------------------------------------   TFAC Function   -------------------------------------------
void J1939_TFAC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_throttle_actuator1_control_command = (rx_data[1] << 8) + rx_data[0];
  engine_throttle_actuator2_control_command = (rx_data[3] << 8) + rx_data[2];                             // Not Available
  engine_fuel_actuator1_control_command = (rx_data[5] << 8) + rx_data[4];                                 // Not Available
  engine_fuel_actuator2_control_command = (rx_data[7] << 8) + rx_data[6];                                 // Not Available
}

//-------------------------------------------   SEP1 Variables   -------------------------------------------
/*
  SEP1 = Sensor Electric Power 1
  PGN = 64925
  Repetition time = 1000 ms
*/
uint16_t sensor_ecu_supply_voltage_1;
uint16_t sensor_ecu_supply_voltage_2;
uint16_t sensor_ecu_supply_voltage_3;
uint16_t sensor_ecu_supply_voltage_4;

//-------------------------------------------   SEP1 Function   -------------------------------------------
void J1939_SEP1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  sensor_ecu_supply_voltage_1 = (rx_data[1] << 8) + rx_data[0];
  sensor_ecu_supply_voltage_2 = (rx_data[3] << 8) + rx_data[2];
  sensor_ecu_supply_voltage_3 = (rx_data[5] << 8) + rx_data[4];                                           // Not Available
  sensor_ecu_supply_voltage_4 = (rx_data[7] << 8) + rx_data[6];                                           // Not Available
}

//-------------------------------------------   EOI Variables   -------------------------------------------
/*
  EOI = Engine operation information
  PGN = 64914
  Repetition time = 200 ms
*/
uint8_t engine_operating_status;
uint8_t fuel_pump_primer_control;
uint16_t time_remaining_in_engine_operating_state;
uint8_t engine_fuel_shutoff_vent_control;
uint8_t engine_fuel_shutoff1_control;
uint8_t engine_fuel_shutoff2_control;
uint8_t engine_fuel_shutoff_valve_leak_test_control;
uint8_t engine_oil_priming_pump_control;
uint8_t engine_oil_preheater_control;
uint8_t engine_electrical_system_power_conversation_1;
uint8_t engine_block_coolant_preheater_control;
uint8_t engine_coolant_circulating_pump_control;
uint8_t engine_controlled_shutdown_request;
uint8_t engine_electrical_system_power_conversation_2;
uint8_t engine_derate_request;
//-------------------------------------------   EOI Function   -------------------------------------------
void J1939_EOI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_operating_status = (rx_data[0] & 0x0F);                                                             // Not Available
  fuel_pump_primer_control = (rx_data[0] & 0x30) >> 4;                                                       // Not Available
  time_remaining_in_engine_operating_state = (rx_data[2] << 8) + rx_data[1];                              // Not Available
  engine_fuel_shutoff_vent_control = (rx_data[3] & 0x03);                                                    // Not Available
  engine_fuel_shutoff1_control = (rx_data[3] & 0x0C) >> 2;                                                   // Not Available
  engine_fuel_shutoff2_control = (rx_data[3] & 0x30) >> 4;                                                   // Not Available
  engine_fuel_shutoff_valve_leak_test_control = (rx_data[3] & 0xC0) >> 6;                                    // Not Available
  engine_oil_priming_pump_control = (rx_data[4] & 0x03);                                                     // Not Available
  engine_oil_preheater_control = (rx_data[4] & 0x0C) >> 2;                                                   // Not Available
  engine_electrical_system_power_conversation_1 = (rx_data[4] & 0x30) >> 4;                                  // Not Available
  engine_block_coolant_preheater_control = (rx_data[4] & 0xC0) >> 6;                                         // Not Available
  engine_coolant_circulating_pump_control = (rx_data[5] & 0x03);                                             // Not Available
  engine_controlled_shutdown_request = (rx_data[5] & 0x0C) >> 2;                                             // Not Available
  engine_electrical_system_power_conversation_2 = (rx_data[5] & 0x30) >> 4;                                  // Not Available
  engine_derate_request = (rx_data[7] & 0x80) >> 7;                                                          // Not Available
}

//-------------------------------------------   AT1IMG Variables   -------------------------------------------
/*
  AT1IMG = Aftertreatment 1 Intermediate Gas
  PGN = 64946
  Repetition time = 500 ms
*/
uint16_t aftertreatment1_exhaust_gas_temperature_2;
uint16_t aftertreatment1_diesel_particulate_filter_intermediate_gas_temperature;
uint16_t aftertreatment1_diesel_particulate_filter_differential_pressure;
uint8_t aftertreatment1_exhaust_gas_temperature_2_preliminary_FMI;
uint8_t aftertreatment1_diesel_particulate_filter_differential_pressure_preliminary_FMI;
uint8_t aftertreatment1_diesel_particulate_filter_intermediate_gas_temperature_preliminary_FMI;

//-------------------------------------------   AT1IMG Function   -------------------------------------------
void J1939_AT1IMG_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_exhaust_gas_temperature_2 = (rx_data[1] << 8) + rx_data[0];                                                                     // Not Available
  aftertreatment1_diesel_particulate_filter_intermediate_gas_temperature = (rx_data[3] << 8) + rx_data[2];
  aftertreatment1_diesel_particulate_filter_differential_pressure = (rx_data[5] << 8) + rx_data[4];
  aftertreatment1_exhaust_gas_temperature_2_preliminary_FMI = (rx_data[6] & 0x1F);                                                                   // Not Available
  aftertreatment1_diesel_particulate_filter_differential_pressure_preliminary_FMI = ((rx_data[7] & 0x03) << 3) + ((rx_data[6] & 0xE0) >> 5);      // Not Available
  aftertreatment1_diesel_particulate_filter_intermediate_gas_temperature_preliminary_FMI = ((rx_data[7] & 0x7C) >> 2);                               // Not Available
}

//-------------------------------------------   AT1OG2 Variables   -------------------------------------------
/*
  AT1OG2 = Aftertreatment 1 Outlet Gas 2
  PGN = 64947
  Repetition time = 500 ms
*/
uint16_t aftertreatment1_exhaust_gas_temperature_3;
uint16_t aftertreatment1_diesel_particulate_filter_outlet_gas_temperature;
uint8_t aftertreatment1_exhaust_gas_temperature_3_preliminary_FMI;
uint8_t aftertreatment1_diesel_particulate_filter_outlet_gas_temperature_preliminary_FMI;

//-------------------------------------------   AT1OG2 Function   -------------------------------------------
void J1939_AT1OG2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_exhaust_gas_temperature_3 = (rx_data[1] << 8) + rx_data[0];                             // Not Available
  aftertreatment1_diesel_particulate_filter_outlet_gas_temperature = (rx_data[3] << 8) + rx_data[2];      // Not Available
  aftertreatment1_exhaust_gas_temperature_3_preliminary_FMI = (rx_data[4] & 0x1F);                           // Not Available
  aftertreatment1_diesel_particulate_filter_outlet_gas_temperature_preliminary_FMI = (rx_data[5] & 0x1F);    // Not Available
}

//-------------------------------------------   AT1IG2 Variables   -------------------------------------------
/*
  AT1IG2 = Aftertreatment 1 Intake Gas 2
  PGN = 64948
  Repetition time = 500 ms
*/
uint16_t aftertreatment1_exhaust_gas_temperature_1;
uint16_t aftertreatment1_diesel_particulate_filter_intake_gas_temperature;
uint8_t aftertreatment1_exhaust_gas_temperature_1_preliminary_FMI;
uint8_t aftertreatment1_diesel_particulate_filter_intake_gas_temperature_preliminary_FMI;

//-------------------------------------------   AT1IG2 Function   -------------------------------------------
void J1939_AT1IG2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_exhaust_gas_temperature_1 = (rx_data[1] << 8) + rx_data[0];
  aftertreatment1_diesel_particulate_filter_intake_gas_temperature = (rx_data[3] << 8) + rx_data[2];
  aftertreatment1_exhaust_gas_temperature_1_preliminary_FMI = (rx_data[4] & 0x1F);                                   // Not Available
  aftertreatment1_diesel_particulate_filter_intake_gas_temperature_preliminary_FMI = (rx_data[5] & 0x1F);            // Not Available
}

//-------------------------------------------   EFL/P2 Variables   -------------------------------------------
/*
  EFL/P2 = Engine Fliud Level/Pressure 2
  PGN = 65243
  Repetition time = 500 ms
*/
uint16_t engine_iniector_control_pressure;
uint16_t engine_iniector_metering_rail1_pressure;
uint16_t engine_iniector_timing_rail1_pressure;
uint16_t engine_iniector_metering_rail2_pressure;

//-------------------------------------------   EFL/P2 Function   -------------------------------------------
void J1939_EFLP2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_iniector_control_pressure = (rx_data[1] << 8) + rx_data[0];                                              // Not Available
  engine_iniector_metering_rail1_pressure = (rx_data[3] << 8) + rx_data[2];
  engine_iniector_timing_rail1_pressure = (rx_data[5] << 8) + rx_data[4];                                         // Not Available
  engine_iniector_metering_rail2_pressure = (rx_data[7] << 8) + rx_data[6];                                       // Not Available
}

//-------------------------------------------   SHUTDOWN Variables   -------------------------------------------
/*
  SHUTDOWN = Shutdown
  PGN = 65252
  Repetition time = 1000 ms
*/
uint8_t engine_idle_shutdown_has_shutdown_engine;
uint8_t engine_idle_shutdown_driver_alert_mode;
uint8_t engine_idle_shutdown_timer1_override;
uint8_t engine_idle_shutdown_timer_state;
uint8_t engine_idle_shutdown_timer_function;
uint8_t ac_high_pressure_fan_switch;
uint8_t refrigerant_low_pressure_switch;
uint8_t refrigerant_high_pressure_switch;
uint8_t ready_to_crank;
uint8_t engine_protection_system_has_shutdown_engine;
uint8_t engine_protection_system_approaching_shutdown;
uint8_t engine_protection_system_timer_override;
uint8_t engine_protection_system_timer_state;
uint8_t engine_protection_system_configuration;
uint8_t engine_alarm_acknowledge;
uint8_t engine_alarm_output_command_status;
uint8_t engine_air_shutoff_command_status;
uint8_t engine_overspeed_test;
uint8_t engine_air_shutoff_status;

//-------------------------------------------   SHUTDOWN Function   -------------------------------------------
void J1939_SHUTDOWN_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_idle_shutdown_has_shutdown_engine = (rx_data[0] & 0x03);                                                    // Not Available
  engine_idle_shutdown_driver_alert_mode = (rx_data[0] & 0x0C) >> 2;                                                 // Not Available
  engine_idle_shutdown_timer1_override = (rx_data[0] & 0x30) >> 4;                                                   // Not Available
  engine_idle_shutdown_timer_state = (rx_data[0] & 0xC0) >> 6;                                                       // Not Available
  engine_idle_shutdown_timer_function = (rx_data[1] & 0xC0) >> 6;                                                    // Not Available
  ac_high_pressure_fan_switch = (rx_data[2] & 0x03);                                                                 // Not Available
  refrigerant_low_pressure_switch = (rx_data[2] & 0x0C) >> 2;                                                        // Not Available
  refrigerant_high_pressure_switch = (rx_data[2] & 0x30) >> 4;                                                       // Not Available
  ready_to_crank = (rx_data[3] & 0x03);                                                                              // Not Available
  engine_protection_system_has_shutdown_engine = (rx_data[4] & 0x03);                                                // Not Available
  engine_protection_system_approaching_shutdown = (rx_data[4] & 0x0C) >> 2;                                          // Not Available
  engine_protection_system_timer_override = (rx_data[4] & 0x30) >> 4;                                                // Not Available
  engine_protection_system_timer_state = (rx_data[4] & 0xC0) >> 6;                                                   // Not Available
  engine_protection_system_configuration = (rx_data[5] & 0xC0) >> 6;                                                 // Not Available
  engine_alarm_acknowledge = (rx_data[6] & 0x03);                                                                    // Not Available
  engine_alarm_output_command_status = (rx_data[6] & 0x0C) >> 2;                                                     // Not Available
  engine_air_shutoff_command_status = (rx_data[6] & 0x30) >> 4;                                                      // Not Available
  engine_overspeed_test = (rx_data[6] & 0xC0) >> 6;                                                                  // Not Available
  engine_air_shutoff_status = (rx_data[7] & 0x03);                                                                   // Not Available
}

//-------------------------------------------   IC1 Variables   -------------------------------------------
/*
  IC1 = Intake/Exhaust Condition 1
  PGN = 65270
  Repetition time = 500 ms
*/
uint8_t engine_diesel_particulate_filter_inlet_pressure;
uint8_t engine_intake_manifold1_pressure;
uint8_t engine_intake_manifold1_temperature;
uint8_t engine_air_inlet_pressure;
uint8_t engine_air_filter1_differential_pressure;
uint16_t engine_exhaust_gas_temperature;
uint8_t engine_coolant_filter_differential_pressure;

//-------------------------------------------   IC1 Function   -------------------------------------------
void J1939_IC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_diesel_particulate_filter_inlet_pressure = rx_data[0];
  engine_intake_manifold1_pressure = rx_data[1];                                                                     // Not Available
  engine_intake_manifold1_temperature = rx_data[2];
  engine_air_inlet_pressure = rx_data[3];
  engine_air_filter1_differential_pressure = rx_data[4];                                                             // Not Available
  engine_exhaust_gas_temperature = (rx_data[6] << 8) + rx_data[5];                                                // Not Available
  engine_coolant_filter_differential_pressure = rx_data[7];                                                          // Not Available
}

//-------------------------------------------   A1SCRDSI1 Variables   -------------------------------------------
/*
  A1SCRDSI1 = Aftertreatment 1 SCR Dosing System Information 1
  PGN = 61475
  Repetition time = 50 ms
*/
uint16_t aftertreatment1_diesel_exhaust_fluid_actual_dosing_quantity;
uint8_t aftertreatment1_SCR_system_state;
uint16_t aftertreatment1_diesel_exhaust_fluid_actual_dosing_quantity_of_integrator;
uint8_t aftertreatment1_diesel_exhaust_fluid_doser_absolute_pressure;

//-------------------------------------------   A1SCRDSI1 Function   -------------------------------------------
void J1939_A1SCRDSI1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_diesel_exhaust_fluid_actual_dosing_quantity = (rx_data[1] << 8) + rx_data[0];
  aftertreatment1_SCR_system_state = (rx_data[2] & 0x0F);
  aftertreatment1_diesel_exhaust_fluid_actual_dosing_quantity_of_integrator = (rx_data[4] << 8) + rx_data[3];
  aftertreatment1_diesel_exhaust_fluid_doser_absolute_pressure = rx_data[5];
  // rx_data[6] & rx_data[7] are NOT used
}

//-------------------------------------------   A1SCREGT1 Variables   -------------------------------------------
/*
  A1SCREGT1 = Aftertreatment 1 SCR Dosing System Information 1
  PGN = 64830
  Repetition time = 500 ms
*/
uint16_t aftertreatment1_SCR_catalyst_intake_gas_temperature;
uint16_t aftertreatment1_SCR_catalyst_outlet_gas_temperature;

//-------------------------------------------   A1SCREGT1 Function   -------------------------------------------
void J1939_A1SCREGT1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_SCR_catalyst_intake_gas_temperature = (rx_data[1] << 8) + rx_data[0];
  aftertreatment1_SCR_catalyst_outlet_gas_temperature = (rx_data[4] << 8) + rx_data[3];
}

//-------------------------------------------   A1DEFT1I Variables   -------------------------------------------
/*
  A1DEFT1I = Aftertreatment 1 Diesel Exhaust Fluid Tank 1 Information
  PGN = 65110
  Repetition time = 1000 ms
*/
uint8_t aftertreatment1_diesel_exhaust_fluid_tank_level;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank_temperature;
uint16_t aftertreatment1_diesel_exhaust_fluid_tank_level_2;
uint8_t aftertreatment_selective_catalytic_reduction_operator_inducement_active;
uint8_t aftertreatment_SCR_operator_inducement_severity;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank_heater;

//-------------------------------------------   A1DEFT1I Function   -------------------------------------------
void J1939_A1DEFT1I_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_diesel_exhaust_fluid_tank_level = rx_data[0];
  aftertreatment1_diesel_exhaust_fluid_tank_temperature = rx_data[1];
  aftertreatment1_diesel_exhaust_fluid_tank_level_2 = (rx_data[3] << 8) + rx_data[2];                             // Not Available
  aftertreatment_selective_catalytic_reduction_operator_inducement_active = (rx_data[4] & 0xE0) >> 5;
  aftertreatment_SCR_operator_inducement_severity = (rx_data[5] & 0xE0) >> 5;
  aftertreatment1_diesel_exhaust_fluid_tank_heater = rx_data[6];                                                     // Not Available
  //rx_data[7] is NOT used
}

//-------------------------------------------   A1DEFI Variables   -------------------------------------------
/*
  A1DEFI = Aftertreatment 1 Diesel Exhaust Fluid Information
  PGN = 64923
  Repetition time = 1000 ms
*/
uint8_t aftertreatment1_diesel_exhaust_fluid_temperature_2;
uint8_t aftertreatment1_diesel_exhaust_fluid_concentration;
uint8_t aftertreatment1_diesel_exhaust_fluid_conductivity;
uint8_t aftertreatment1_diesel_exhaust_fluid_temperature_2_preliminary_FMI;
uint8_t aftertreatment1_diesel_exhaust_fluid_properties_preliminary_FMI;
uint8_t aftertreatment1_diesel_exhaust_fluid_property;

//-------------------------------------------   A1DEFI Function   -------------------------------------------
void J1939_A1DEFI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_diesel_exhaust_fluid_temperature_2 = rx_data[0];
  aftertreatment1_diesel_exhaust_fluid_concentration = rx_data[1];
  aftertreatment1_diesel_exhaust_fluid_conductivity = rx_data[2];                                                    // Not Available
  aftertreatment1_diesel_exhaust_fluid_temperature_2_preliminary_FMI = rx_data[3] & 0x1F;
  aftertreatment1_diesel_exhaust_fluid_properties_preliminary_FMI = rx_data[4] & 0x1F;
  aftertreatment1_diesel_exhaust_fluid_property = rx_data[5] & 0x0F;
}

//-------------------------------------------   AT1T1I Variables   -------------------------------------------
/*
  AT1T1I = Aftertreatment 1 SCR Reagent Tank 1 Information
  PGN = 65110
  Repetition time = 1000 ms
*/
uint8_t aftertreatment1_diesel_exhaust_tank1_level;
uint8_t aftertreatment1_diesel_exhaust_tank1_temperature;
uint16_t aftertreatment1_diesel_exhaust_tank_level_2;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank_level2_preliminary_FMI;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank1_low_level_indicator;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank1_temperature_preliminary_FMI;
uint8_t aftertreatment_SCR_operator_inducement_severity;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank_heater;
uint8_t aftertreatment1_diesel_exhaust_fluid_tank1_heater_preliminary_FMI;

//-------------------------------------------   AT1T1I Function   -------------------------------------------
void J1939_AT1T1I_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  aftertreatment1_diesel_exhaust_tank1_level = rx_data[0];
  aftertreatment1_diesel_exhaust_tank1_temperature = rx_data[1];
  aftertreatment1_diesel_exhaust_tank_level_2 = (rx_data[3] << 8) + rx_data[2];
  aftertreatment1_diesel_exhaust_fluid_tank_level2_preliminary_FMI = rx_data[4] & 0x1F;
  aftertreatment1_diesel_exhaust_fluid_tank1_low_level_indicator = (rx_data[4] & 0xE0) >> 5;                                 // Not Available
  aftertreatment1_diesel_exhaust_fluid_tank1_temperature_preliminary_FMI = rx_data[5] & 0x1F;
  aftertreatment_SCR_operator_inducement_severity = (rx_data[5] & 0xE0) >> 5;                                                // Not Available
  aftertreatment1_diesel_exhaust_fluid_tank_heater = rx_data[6];                                                             // Not Available
  aftertreatment1_diesel_exhaust_fluid_tank1_heater_preliminary_FMI = rx_data[7] & 0x1F;                                     // Not Available
}

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

//-------------------------------------------   YECR1 Variables   -------------------------------------------
/*
  YECR1 = Engine Control Request 1
  PGN = 65282
  Repetition time = 50 ms
*/
uint8_t shutdown_requests;
uint8_t power_supply_key_position;
uint8_t accelerator_pedal_position;
//-------------------------------------------   YECR1 Function   -------------------------------------------
void J1939_YECR1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  shutdown_requests = (rx_data[1] & 0x0C) >> 2;
  power_supply_key_position = (rx_data[1] & 0xC0) >> 6;
  accelerator_pedal_position = rx_data[3];
}

//-------------------------------------------   YECACK1 Variables   -------------------------------------------
/*
  YECACK1 = Engine Control Acknowledge 1
  PGN = 65292
  Repetition time = 100 ms
*/
uint8_t preheat_function_acknowledge;
uint8_t power_down_enable;
uint8_t remaining_preheat_time;
//-------------------------------------------   YECACK1 Function   -------------------------------------------
void J1939_YECACK1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  preheat_function_acknowledge = (rx_data[1] & 0x18) >> 3;
  power_down_enable = (rx_data[1] & 0x80) >> 7;
  remaining_preheat_time = rx_data[2];
}

//-------------------------------------------   YIOS Variables   -------------------------------------------
/*
  YIOS = State of digital In/Out
  PGN = 65297
  Repetition time = 100 ms
*/
uint8_t digital_input1_status;
uint8_t digital_input2_status;
uint8_t digital_output1_status;
uint8_t digital_input3_status;
uint8_t digital_input4_status;
uint8_t digital_output2_status;
uint8_t digital_output3_status;
uint8_t digital_input5_status;
//-------------------------------------------   YIOS Function   -------------------------------------------
void J1939_YIOS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  digital_input1_status = (rx_data[0] & 0x07);
  digital_input2_status = rx_data[1];
  digital_output1_status = rx_data[2];
  digital_input3_status = rx_data[3];
  digital_input4_status = rx_data[4] & 0x07;
  digital_output2_status = rx_data[5];
  digital_output3_status = rx_data[6];
  digital_input5_status = (rx_data[1] & 0x01);
}

//-------------------------------------------   YLF Variables   -------------------------------------------
/*
  YLF = Engine Load
  PGN = 65303
  Repetition time = 20 ms
*/
uint8_t engine_gross_load_ratio_1;
uint8_t engine_gross_load_ratio_2;
uint8_t engine_gross_load_ratio_UFO;
uint16_t engine_desired_operating_speed;
uint8_t acceleration_detection_status_1;
uint8_t acceleration_detection_status_2;
uint8_t engine_gross_load_ratio_smoke_limit;
//-------------------------------------------   YLF Function   -------------------------------------------
void J1939_YLF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_gross_load_ratio_1 = rx_data[0];
  engine_gross_load_ratio_2 = rx_data[1];
  engine_gross_load_ratio_UFO = rx_data[2];
  engine_desired_operating_speed = (rx_data[4] << 8) + rx_data[3];
  acceleration_detection_status_1 = rx_data[5] & 0x01;
  acceleration_detection_status_2 = rx_data[5] & 0x02;
  engine_gross_load_ratio_smoke_limit = rx_data[6];
}

//-------------------------------------------   YEC Variables   -------------------------------------------
/*
  YEC = Governor Functions
  PGN = 65308
  Repetition time = 100 ms
*/
uint8_t governor_mode;
uint8_t droop_mode;
uint8_t starter_protection;
uint8_t hi_idle_control;
uint8_t input_of_operation_information_with_condition;
//-------------------------------------------   YEC Function   -------------------------------------------
void J1939_YEC_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  governor_mode = (rx_data[0] & 0x04) >> 2;
  droop_mode = (rx_data[0] & 0x08) >> 3;
  starter_protection = (rx_data[0] & 0x10) >> 4;
  hi_idle_control = (rx_data[0] & 0x60) >> 5;
  input_of_operation_information_with_condition = (rx_data[7] & 0x01);
}

//-------------------------------------------   YSTP Variables   -------------------------------------------
/*
  YSTP = Engine Stop Command
  PGN = 65309
  Repetition time = As Needed
*/
uint8_t engine_stop;
//-------------------------------------------   YSTP Function   -------------------------------------------
void J1939_YSTP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_stop = (rx_data[0] & 0x03);
}

//-------------------------------------------   YRSS Variables   -------------------------------------------
/*
  YRSS = Speed Selection Function
  PGN = 65310
  Repetition time = 50 ms
*/
uint8_t speed_selection_function;
uint8_t yanmar_special_function;
//-------------------------------------------   YRSS Function   -------------------------------------------
void J1939_YRSS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  speed_selection_function = (rx_data[0] & 0x1F);
  yanmar_special_function = rx_data[1];
}

//-------------------------------------------   YSRF Variables   -------------------------------------------
/*
  YSRF = Engine Control Factor
  PGN = 65311
  Repetition time = 20 ms
*/
uint16_t starter_protection_factor;
uint8_t safety_relay_operation;
uint8_t under_ECU_initial_operation;
uint8_t external_switch;
uint8_t immobilizer;
uint8_t starter_over_time;
uint8_t yec_status;
uint8_t engine_stop_operation;
uint8_t key_switch_off;
uint8_t protection_permission;

uint16_t engine_stop_factor;
uint8_t engine_stall;
uint8_t key_switch_off;
uint8_t engine_stop1_sw;
uint8_t engine_stop2_sw;
uint8_t other_engine_stop_operation;
uint8_t ecu_error_or_normal;
uint8_t immobilizer_status;
uint8_t digital_status;
//-------------------------------------------   YSRF Function   -------------------------------------------
void J1939_YSRF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  starter_protection_factor = (rx_data[1] << 8) + rx_data[0];
  safety_relay_operation = rx_data[0] & 0x01;
  under_ECU_initial_operation = (rx_data[0] & 0x02) >> 1;
  external_switch = (rx_data[0] & 0x04) >> 2;
  immobilizer = (rx_data[0] & 0x08) >> 3;
  starter_over_time = (rx_data[0] & 0x10) >> 4;
  yec_status = (rx_data[0] & 0x20) >> 5;
  engine_stop_operation = (rx_data[0] & 0x40) >> 6;
  key_switch_off = (rx_data[0] & 0x80) >> 7;
  protection_permission = (rx_data[1] & 0x08) >> 3;

  engine_stop_factor = (rx_data[3] << 8) + rx_data[2];
  engine_stall = rx_data[2] & 0x01;
  key_switch_off = (rx_data[2] & 0x02) >> 1;
  engine_stop1_sw = (rx_data[2] & 0x04) >> 2;
  engine_stop2_sw = (rx_data[2] & 0x08) >> 3;
  other_engine_stop_operation = (rx_data[3] & 0x02) >> 1;
  ecu_error_or_normal = (rx_data[3] & 0x04) >> 2;
  immobilizer_status = rx_data[4];
  
  digital_status = rx_data[7] & 0x01;
}

//-------------------------------------------   YSRSI Variables   -------------------------------------------
/*
  YSRSI = Engine Speed Specifications
  PGN = 65318
  Repetition time = 250 ms
*/
uint16_t low_idle_speed;
uint16_t high_idle_speed_under_droop_mode;
uint16_t high_idle_speed_under_isochronous_mode;
uint16_t available_maximum_speed;
//-------------------------------------------   YSRSI Function   -------------------------------------------
void J1939_YSRSI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  low_idle_speed = (rx_data[1] << 8) + rx_data[0];
  high_idle_speed_under_droop_mode = (rx_data[3] << 8) + rx_data[2];
  high_idle_speed_under_isochronous_mode = (rx_data[5] << 8) + rx_data[4];
  available_maximum_speed = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YESI Variables   -------------------------------------------
/*
  YESI = Engine Control Status
  PGN = 65319
  Repetition time = 100 ms
*/
uint8_t engine_control_state;
uint8_t derate_mode;
uint8_t operation_control;
uint8_t governor_mode;
//-------------------------------------------   YESI Function   -------------------------------------------
void J1939_YESI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_control_state = rx_data[0];
  derate_mode = rx_data[1] & 0x03;
  operation_control = rx_data[2] & 0x0F;
  governor_mode = rx_data[3] & 0x07;
}

//-------------------------------------------   YDPFIF Variables   -------------------------------------------
/*
  YDPFIF = Engine DPF Regeneration Control Request
  PGN = 65304
  Repetition time = 1000 ms
*/
uint8_t accelerator_interlock_switch;
uint8_t can_dsl_part_fltr_ref_inhb_swtch;
uint8_t can_dsl_part_fltr_ref_frce_swtch;
//-------------------------------------------   YDPFIF Function   -------------------------------------------
void J1939_YDPFIF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  accelerator_interlock_switch = rx_data[0] & 0x03;
  can_dsl_part_fltr_ref_inhb_swtch = (rx_data[0] & 0x0C) >> 2;
  can_dsl_part_fltr_ref_frce_swtch = (rx_data[0] & 0x30) >> 4;
}

//-------------------------------------------   YEGRP Variables   -------------------------------------------
/*
  YEGRP = YMR Dataloging 1 = EGR Valve Position
  PGN = 65320
  Repetition time = 100 ms
*/
uint8_t egr_valve_target_position;
uint8_t egr_valve_target_position_calculated;
uint8_t exhaust_gas_lambda;
uint8_t egr_valve_actual_position;
uint16_t target_nox_final_valve;
uint16_t dpf_outlet_no2_nox_ratio;
//-------------------------------------------   YEGRP Function   -------------------------------------------
void J1939_YEGRP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  egr_valve_target_position = rx_data[0];
  egr_valve_target_position_calculated = rx_data[1];
  exhaust_gas_lambda = rx_data[2];
  egr_valve_actual_position = rx_data[3];
  target_nox_final_valve = (rx_data[5] << 8) + rx_data[4];
  dpf_outlet_no2_nox_ratio = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YETVP Variables   -------------------------------------------
/*
  YETVP = YMR Dataloging 2 = Exhaust Throttle Valve Position
  PGN = 65321
  Repetition time = 100 ms
*/
uint16_t exhaust_throttle_target_position;
uint16_t exhaust_throttle_actual_position;
uint16_t exhaust_gas_mass_flow;
uint16_t exhaust_gas_volume_flow;
//-------------------------------------------   YETVP Function   -------------------------------------------
void J1939_YETVP_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  exhaust_throttle_target_position = (rx_data[1] << 8) + rx_data[0];
  exhaust_throttle_actual_position = (rx_data[3] << 8) + rx_data[2];
  exhaust_gas_mass_flow = (rx_data[5] << 8) + rx_data[4];
  exhaust_gas_volume_flow = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YATF Variables   -------------------------------------------
/*
  YATF = YMR Dataloging 3 = Regeneration mode status
  PGN = 65322
  Repetition time = 100 ms
*/
uint8_t dpf_regeneration_mode_status;
uint8_t dpf_regeneration_control_status;
uint8_t emergency_mode_status;
uint8_t stationary_regeneration_request_flag;
uint8_t ash_cleaning_request_flag;
uint8_t dpf_regeneration_control_start_flag;
uint8_t required_dpf_regeneration_status;
uint8_t dpf_regeneration_permission_status;
uint8_t dpf_regeneration_status;
uint8_t scr_control_status;
uint8_t dpf_regenration_progress_percentage;
//-------------------------------------------   YATF Function   -------------------------------------------
void J1939_YATF_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  dpf_regeneration_mode_status = rx_data[0] & 0x0F;
  dpf_regeneration_control_status = (rx_data[0] & 0xF0) >> 4;
  emergency_mode_status = rx_data[1] & 0x0F;
  stationary_regeneration_request_flag = (rx_data[1] & 0xF0) >> 4;
  ash_cleaning_request_flag = rx_data[2] & 0x0F;
  dpf_regeneration_control_start_flag = (rx_data[2] & 0x0F) >> 4;
  required_dpf_regeneration_status = rx_data[3] & 0x0F;
  dpf_regeneration_permission_status = (rx_data[3] & 0x30) >> 4;
  dpf_regeneration_status = (rx_data[3] & 0xC0) >> 6;
  scr_control_status = (rx_data[5] & 0x0F);
  dpf_regenration_progress_percentage = (rx_data[7] & 0x80) >> 7;
}

//-------------------------------------------   YPMD Variables   -------------------------------------------
/*
  YPMD = YMR Dataloging 5 = Amount of PM deposition
  PGN = 65327
  Repetition time = 1000 ms
*/
uint16_t amount_of_particulate_matter_c_method;
uint16_t amount_of_particulate_matter_p_method;
uint16_t amount_of_particulate_matter_t_method;
uint16_t amount_of_particulate_matter_f_method;
//-------------------------------------------   YPMD Function   -------------------------------------------
void J1939_YPMD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  amount_of_particulate_matter_c_method = (rx_data[1] << 8) + rx_data[0];
  amount_of_particulate_matter_p_method = (rx_data[3] << 8) + rx_data[2];
  amount_of_particulate_matter_t_method = (rx_data[5] << 8) + rx_data[4];
  amount_of_particulate_matter_f_method = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YDPFC1 Variables   -------------------------------------------
/*
  YDPFC1 = YMR Dataloging 6 = DOC/DPF surface temperatures
  PGN = 65314
  Repetition time = 1000 ms
*/
uint16_t doc_surface_temperature;
uint16_t dpf_surface_temperature;
uint16_t amount_of_dpf_ash_accumulation;
uint16_t scr_catalyst_surface_temperature;
//-------------------------------------------   YDPFC1 Function   -------------------------------------------
void J1939_YDPFC1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  doc_surface_temperature = (rx_data[1] << 8) + rx_data[0];
  dpf_surface_temperature = (rx_data[3] << 8) + rx_data[2];
  amount_of_dpf_ash_accumulation = (rx_data[5] << 8) + rx_data[4];
  scr_catalyst_surface_temperature = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YPOI Variables   -------------------------------------------
/*
  YPOI = YMR Dataloging 7 = Actual Post Injection Amount
  PGN = 65295
  Repetition time = 100 ms
*/
uint16_t actual_post_injection_amount;
uint16_t target_after_injection_amount;
uint16_t target_post_injection_correction_amount;
uint16_t total_injection_amount;
//-------------------------------------------   YPOI Function   -------------------------------------------
void J1939_YPOI_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  actual_post_injection_amount = (rx_data[1] << 8) + rx_data[0];
  target_after_injection_amount = (rx_data[3] << 8) + rx_data[2];
  target_post_injection_correction_amount = (rx_data[5] << 8) + rx_data[4];
  total_injection_amount = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YDPFC2 Variables   -------------------------------------------
/*
  YDPFC2 = YMR Dataloging 8 = Target DPF temperature
  PGN = 65296
  Repetition time = 1000 ms
*/
uint16_t target_dpf_temperature_outrlop;
uint16_t dpf_temperature_deviation_amount;
uint16_t target_dpf_temperature;
uint16_t dpf_temperature_deviation_amount_intrlop;
//-------------------------------------------   YDPFC2 Function   -------------------------------------------
void J1939_YDPFC2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  target_dpf_temperature_outrlop = (rx_data[1] << 8) + rx_data[0];
  dpf_temperature_deviation_amount = (rx_data[3] << 8) + rx_data[2];
  target_dpf_temperature = (rx_data[5] << 8) + rx_data[4];
  dpf_temperature_deviation_amount_intrlop = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YEST Variables   -------------------------------------------
/*
  YEST = YMR Dataloging 9 = Target Engine Speed
  PGN = 65305
  Repetition time = 100 ms
*/
uint16_t target_engine_speed;
uint16_t target_injection_amount;
uint16_t basic_injection_timing;
uint8_t acceleration_detection_status_1;
uint8_t acceleration_detection_status_2;
uint8_t load_detection_status_1;
uint8_t load_detection_status_2;

//-------------------------------------------   YEST Function   -------------------------------------------
void J1939_YEST_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  target_engine_speed = (rx_data[1] << 8) + rx_data[0];
  target_injection_amount = (rx_data[3] << 8) + rx_data[2];
  basic_injection_timing = (rx_data[5] << 8) + rx_data[4];
  acceleration_detection_status_1 = rx_data[6] & 0x01;
  acceleration_detection_status_2 = (rx_data[6] & 0x02) >> 1;
  load_detection_status_1 = (rx_data[6] & 0x04) >> 2;
  load_detection_status_2 = (rx_data[6] & 0x08) >> 3;
}

//-------------------------------------------   YMPR Variables   -------------------------------------------
/*
  YMPR = YMR Dataloging 10 = Actual Exhaust/Intake Manifold Pressure
  PGN = 65280
  Repetition time = 100 ms
*/
uint16_t actual_exhaust_manifold_pressure;
uint16_t actual_intake_manifold_pressure;
uint8_t target_intake_pressure;
uint8_t target_intake_throttle_rate;
uint16_t scr_catalyst_average_temperature;
//-------------------------------------------   YMPR Function   -------------------------------------------
void J1939_YMPR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  actual_exhaust_manifold_pressure = (rx_data[1] << 8) + rx_data[0];
  actual_intake_manifold_pressure = (rx_data[3] << 8) + rx_data[2];
  target_intake_pressure = rx_data[4];
  target_intake_throttle_rate = rx_data[5];
  scr_catalyst_average_temperature = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YPOFS Variables   -------------------------------------------
/*
  YPOFS = YMR Dataloging 11 = Exhaust/Intake Manifold Pressure
  PGN = 65281
  Repetition time = 100 ms
*/
uint16_t exhaust_manifold_pressure_offset_valve;
uint16_t intake_manifold_pressure_offset_valve;
uint16_t smoke_limit_injection_volume;
uint8_t smoke_limit_injection_volume_lambda_minimum_value;
//-------------------------------------------   YPOFS Function   -------------------------------------------
void J1939_YPOFS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  exhaust_manifold_pressure_offset_valve = (rx_data[1] << 8) + rx_data[0];
  intake_manifold_pressure_offset_valve = (rx_data[3] << 8) + rx_data[2];
  smoke_limit_injection_volume = (rx_data[5] << 8) + rx_data[4];
  smoke_limit_injection_volume_lambda_minimum_value = rx_data[6];
}

//-------------------------------------------   YEOM Variables   -------------------------------------------
/*
  YEOM = YMR Dataloging 12
  PGN = 65291
  Repetition time = 100 ms
*/
uint32_t engine_operation_mode;
//-------------------------------------------   YEOM Function   -------------------------------------------
void J1939_YEOM_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_operation_mode = (rx_data[3] << 24) + (rx_data[2] << 16) + (rx_data[1] << 8) + rx_data[0];
}

//-------------------------------------------   YINJQ Variables   -------------------------------------------
/*
  YINJQ = YMR Dataloging 13 = Injection amount
  PGN = 65294
  Repetition time = 100 ms
*/
uint16_t pre_injection_amount;
uint16_t main_injection_amount;
uint16_t after_injection_amount;
uint16_t post_injection_amount;
//-------------------------------------------   YINJQ Function   -------------------------------------------
void J1939_YINJQ_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  pre_injection_amount = (rx_data[1] << 8) + rx_data[0];
  main_injection_amount = (rx_data[3] << 8) + rx_data[2];
  after_injection_amount = (rx_data[5] << 8) + rx_data[4];
  post_injection_amount = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRST Variables   -------------------------------------------
/*
  YSCRST = YMR SCR Dataloging 1 = SCR Status
  PGN = 65283
  Repetition time = 100 ms
*/
uint8_t dosing_system_state;
uint8_t dosing_system_substate;
uint16_t dosing_module_valve_target_duty;
uint16_t reduction_agent_pump_motor_target_duty;
uint8_t reduction_agent_reverting_valve_state;
uint16_t actual_nh3_dosing_quantity;
//-------------------------------------------   YSCRST Function   -------------------------------------------
void J1939_YSCRST_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  dosing_system_state = rx_data[1] & 0x07;
  dosing_system_substate = (rx_data[1] & 0xF8) >> 3;
  dosing_module_valve_target_duty = (rx_data[2] << 8) + rx_data[1];
  reduction_agent_pump_motor_target_duty = (rx_data[4] << 8) + rx_data[3];
  reduction_agent_reverting_valve_state = rx_data[5];
  actual_nh3_dosing_quantity = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRSTL1 Variables   -------------------------------------------
/*
  YSCRSTL1 = YMR SCR Dataloging 2 = SCR Control 1
  PGN = 65284
  Repetition time = 100 ms
*/
uint16_t scr_catalyst_inlet_nox_concentration;
uint16_t nh3_dosing_quantity_for_purifying;
uint16_t nh3_dosing_quantity_for_loading;
uint16_t scr_feed_back_control_compensation_factor;
//-------------------------------------------   YSCRSTL1 Function   -------------------------------------------
void J1939_YSCRSTL1_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  scr_catalyst_inlet_nox_concentration = (rx_data[1] << 8) + rx_data[0];
  nh3_dosing_quantity_for_purifying = (rx_data[3] << 8) + rx_data[2];
  nh3_dosing_quantity_for_loading = (rx_data[5] << 8) + rx_data[4];
  scr_feed_back_control_compensation_factor = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRSTL2 Variables   -------------------------------------------
/*
  YSCRSTL1 = YMR SCR Dataloging 3 = SCR Control 2
  PGN = 65285
  Repetition time = 100 ms
*/
uint8_t detection_mode_state;
uint8_t scr_feedback_control_active_status;
uint8_t current_state_of_the_power_stages_for_heating_for_urea_tank;
uint8_t current_state_of_the_power_stages_for_heating_for_supply_module;
uint8_t current_state_of_the_power_stages_for_heating_for_pressure_line;
uint8_t current_state_of_the_power_stages_for_heating_for_back_flow_line;
uint8_t current_state_of_the_power_stages_for_heating_for_suction_line;
uint16_t factor_to_convert_from_nox_to_nh3;
uint16_t scr_catalyst_outlet_nox_concentration_after_compensation;
//-------------------------------------------   YSCRSTL2 Function   -------------------------------------------
void J1939_YSCRSTL2_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  detection_mode_state = rx_data[0] & 0x1F;
  scr_feedback_control_active_status = (rx_data[0] & 0x20) >> 5;
  current_state_of_the_power_stages_for_heating_for_urea_tank = (rx_data[0] & 0xC0) >> 6;
  current_state_of_the_power_stages_for_heating_for_supply_module = (rx_data[1] & 0x03);
  current_state_of_the_power_stages_for_heating_for_pressure_line = (rx_data[1] & 0x0C) >> 2;
  current_state_of_the_power_stages_for_heating_for_back_flow_line = (rx_data[1] & 0x30) >> 4;
  current_state_of_the_power_stages_for_heating_for_suction_line = (rx_data[1] & 0xC0) >> 6;
  factor_to_convert_from_nox_to_nh3 = (rx_data[3] << 8) + rx_data[2];
  scr_catalyst_outlet_nox_concentration_after_compensation = (rx_data[5] << 8) + rx_data[4];
}

//-------------------------------------------   YSCRMOD Variables   -------------------------------------------
/*
  YSCRMOD = YMR SCR Dataloging 4 = SCR Control Mode
  PGN = 65286
  Repetition time = 100 ms
*/
uint16_t no2_nox_ratio_before_scr;
uint16_t estimated_nh3_loading_quantity;
uint16_t scr_catalyst_efficiency;
uint16_t scr_catalyst_outlet_nox_concentration_model_value;
//-------------------------------------------   YSCRMOD Function   -------------------------------------------
void J1939_YSCRMOD_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  no2_nox_ratio_before_scr = (rx_data[1] << 8) + rx_data[0];
  estimated_nh3_loading_quantity = (rx_data[3] << 8) + rx_data[2];
  scr_catalyst_efficiency = (rx_data[5] << 8) + rx_data[4];
  scr_catalyst_outlet_nox_concentration_model_value = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRCTL3 Variables   -------------------------------------------
/*
  YSCRCTL3 = YMR SCR Dataloging 5 = SCR Control 3
  PGN = 65287
  Repetition time = 100 ms
*/
uint16_t compensation_map_for_nox_efficiency_by_exhaust_gas_mass_flow;
uint16_t compensation_map_for_nox_efficiency_by_no2_nox_ratio;
uint16_t deviation_of_nh3_loading_quantity_target_value_estimated_value;
uint16_t nh3_dosing_quantity_loading_purifying;
//-------------------------------------------   YSCRCTL3 Function   -------------------------------------------
void J1939_YSCRCTL3_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  compensation_map_for_nox_efficiency_by_exhaust_gas_mass_flow = (rx_data[1] << 8) + rx_data[0];
  compensation_map_for_nox_efficiency_by_no2_nox_ratio = (rx_data[3] << 8) + rx_data[2];
  deviation_of_nh3_loading_quantity_target_value_estimated_value = (rx_data[5] << 8) + rx_data[4];
  nh3_dosing_quantity_loading_purifying = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRHTR Variables   -------------------------------------------
/*
  YSCRHTR = YMR SCR Dataloging 6 = SCR Heter
  PGN = 65288
  Repetition time = 1000 ms
*/
uint8_t urea_heater_control_heater_status;
uint8_t urea_tank_heating_valve_status;
uint8_t supply_module_heater_status;
uint8_t pressure_line_heater_status;
uint8_t back_flow_line_heater_status;
uint16_t supply_module_temperature;
uint16_t scr_catalyst_average_temperature;
uint8_t suction_line_heater_status;
//-------------------------------------------   YSCRHTR Function   -------------------------------------------
void J1939_YSCRHTR_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  urea_heater_control_heater_status = rx_data[0] & 0x03;
  urea_tank_heating_valve_status = (rx_data[0] & 0x1C) >> 2;
  supply_module_heater_status = (rx_data[0] & 0xE0) >> 5;
  pressure_line_heater_status = rx_data[1] & 0x07;
  back_flow_line_heater_status = (rx_data[1] & 0x38) >> 3;
  supply_module_temperature = (rx_data[3] << 8) + rx_data[2];
  scr_catalyst_average_temperature = (rx_data[5] << 8) + rx_data[4];
  suction_line_heater_status = rx_data[6] & 0x07;
}

//-------------------------------------------   YSCRSO Variables   -------------------------------------------
/*
  YSCRSO = YMR SCR Dataloging 7 = SCR Shutoff
  PGN = 65289
  Repetition time = 1000 ms
*/
uint32_t engine_shutoff_time;
uint16_t scr_catalyst_temperature_initial_valve;
uint16_t scr_catalyst_temperature_at_engine_shutoff;
//-------------------------------------------   YSCRSO Function   -------------------------------------------
void J1939_YSCRSO_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  engine_shutoff_time = (rx_data[3] << 24) + (rx_data[2] << 16) + (rx_data[1] << 8) + rx_data[0];
  scr_catalyst_temperature_initial_valve = (rx_data[5] << 8) + rx_data[4];
  scr_catalyst_temperature_at_engine_shutoff = (rx_data[7] << 8) + rx_data[6];
}

//-------------------------------------------   YSCRIS Variables   -------------------------------------------
/*
  YSCRIS = Aftertreatment SCR Inducement State
  PGN = 65528
  Repetition time = 1000 ms
*/
uint8_t command_byte;
uint16_t time_remaining_at_present_inducement;
uint8_t pending_scr_inducement_severity;
uint8_t inducement_severity_for_DEF_tank_level;
uint8_t operator_inducement_active_for_SCR_system_failure;
uint8_t remaining_restored_operation;
uint16_t restored_operation_time_remaining;
//-------------------------------------------   YSCRIS Function   -------------------------------------------
void J1939_YSCRIS_message_handler(CAN_RxHeaderTypeDef* rx, uint8_t* rx_data){
  command_byte = rx_data[0];
  time_remaining_at_present_inducement = (rx_data[3] << 8) + rx_data[2];
  pending_scr_inducement_severity = rx_data[4] & 0x07;
  inducement_severity_for_DEF_tank_level = (rx_data[4] & 0x38) >> 3;
  operator_inducement_active_for_SCR_system_failure = (rx_data[4] & 0xC0) >> 6;
  remaining_restored_operation = rx_data[5];
  restored_operation_time_remaining = (rx_data[7] << 8) + rx_data[6];
}