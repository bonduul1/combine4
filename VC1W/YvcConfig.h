/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright (C) 2011-2015 Yamaha Corporation. All rights reserved.            */
/*  Module          : $Workfile: YvcConfig.h $                                  */
/*  Description     : XFL Driver Configuration parameter include header         */
/*  Version         : $Rev: 46 $                                                 */
/*  Last UpDate Time: $Date:: 2015-01-23 14:31:18#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/********************************************************************************/
#ifndef _def_YVC_CONFIG_H_
#define _def_YVC_CONFIG_H_

/********************************************************************************************
 *                          Hardware-dependent setting
 ********************************************************************************************/
/*------------------------------------------------------------------------------------------*/
/* As for the setting using Yamaha's demonstration or evaluation board,                     */
/* [YD]:demonstration board or [YE]:evaluation board is shown in its comment field.         */
/*------------------------------------------------------------------------------------------*/
/*--- Port connection selection ---                   (Set only one to (1).)                */
#define YVC_BUS_PARALLEL                    (0)     /* 0:Not used   1:Used             [YE] */
#define YVC_BUS_SERIAL                      (1)     /* 0:Not used   1:Used             [YD] */

/*--- Board selection ---                             (Set only one to (1).)                */
#define YVC_DEMOBOARD                       (1)     /* 0:Not used   1:Used             [YD] */
#define YVC_EVABOARD                        (0)     /* 0:Not used   1:Used             [YE] */
#define YVC_USERBOARD                       (0)     /* 0:Not used   1:Used                  */

/********************************************************************************************
 *                          Development environment setting
 ********************************************************************************************/
 
/*--- Bit filed allocation ---*/
#define YVC_BIT_FIELD                       (1)     /* 0: MSB priority allocation           */
                                                    /* 1: LSB priority allocation           */

/********************************************************************************************
 *                          expanded function setting
 ********************************************************************************************/
/*--- YVC1_Init function ---*/
#define YVC_GENERAL_TABLE_INIT              (1)     /* 0: General table is not initialized  */
                                                    /* 1: General table is initialized      */
#define YVC_GENERAL_TABLE_INIT_ADDR    (0x0000U)    /* Start address of general tables to   */
                                                    /*            initialize (set=12n)      */
#define YVC_GENERAL_TABLE_INIT_NUM     (341)        /* Number of layer attribute data to    */
                                                    /*            initialize (Num * 12Byte) */

/*--- Serial Burst Access ---*/
//#define YVC_BUS_SERIAL_BURST                (0)     /* 0:Not used   1:Used                  */
  #define YVC_BUS_SERIAL_BURST                (1)     /* 0:Not used   1:Used  2018.08.29. 느려서 변경함                */ 

/********************************************************************************************
 *                          General table area setting
 ********************************************************************************************/
// Following four definitions, are originally defined in layoutData.h file, are added by Enkhbat,
#ifndef FontTypeAttr_ADDR
  #define FontTypeAttr_ADDR		(0x0FD8)	/* Start address of Font Type Attr	*/
#endif
#ifndef FontTypeAttr_SIZE
  #define FontTypeAttr_SIZE		(0x0028)	/* Data size of Font Type Attr		*/
#endif
#ifndef FCodeData_ADDR
  #define FCodeData_ADDR		(0x0C70)	/* Start address of Font Code Data	*/
#endif
#ifndef FCodeData_SIZE
  #define FCodeData_SIZE		(0x0368)	/* Data size of Font Code Data		*/
#endif

/*--- Host Controlled Layer area setting ---*/
#define YVC_LYR_ADDR_CPU               (0x0204U)    /* Host Controlled Layer start address  */
//#define YVC_LYR_ADDR_CPU               (0x04B0U)    /* Host Controlled Layer start address  */                        // ENkhbat
#define YVC_LYR_NUM_CPU                (150)         /* Number of Host Controlled Layers     */                          //Enkhbat changed this number 30 to 100

/*--- Character Code area setting ---*/
//#define YVC_TBL_PTEXT_ADDR             (0x0F7C)    /* Character Code area start address    */
#define YVC_TBL_PTEXT_ADDR             (FCodeData_ADDR)    /* Character Code area start address    */                        // ENkhbat
#define YVC_TBL_PTEXT_NUM              (FCodeData_SIZE/4)        /* Number of Character Code             */

/*--- Font Type Attribute Data area setting ---*/
#define YVC_TBL_FONT_ADDR              FontTypeAttr_ADDR    /* Font Type Attribute Data area start address */                    // 0xFFF - 0x40 = 0x0FBF
//#define YVC_TBL_FONT_ADDR              (0x0FC0)    /* Font Type Attribute Data area start address */                        // ENkhbat 0xFFF - 0x40 = 0x0FBF
#define YVC_TBL_FONT_NUM               (8)          /* Number of Font Type Attribute Data   */                          // which means 8 font can be supportted

/********************************************************************************************
 *                          Register settings
 ********************************************************************************************/
/*--- Enables or disables the extended layer function ---*/
#define YVC1_EXLY                      (1)          /* 0:Not used	1:Used                  */

/*--- Specifies the size of alpha values in pattern data alpha value tables ---*/
#define YVC1_ALPBSEL                   (1)          /* 0:4bit	    1:8bit                  */

#endif /* _def_YVC_CONFIG_H_	*/

/*------------------------------ E O F -----------------------------------------*/
