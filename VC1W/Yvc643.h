/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2015 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc643.h $                                     */
/*  Description     : Port Access Include header                                */
/*  Version         : $Rev: 55 $                                                */
/*  Last UpDate Time: $Date:: 2015-02-27 13:15:26#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/********************************************************************************/
#ifndef _def_YVC643_H_
#define _def_YVC643_H_

/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "YvcConfig.h"

/*------------------------------------------------------------------------------*/
/*                  B A S I C - C O D E                                         */
/*------------------------------------------------------------------------------*/
typedef char			C08;			/* 8 bit character					*/
typedef signed char		S08;			/* signed  8 bit integer			*/
typedef signed short	S16;			/* signed 16 bit integer			*/
typedef signed long		S32;			/* signed 32 bit integer			*/
typedef unsigned char	U08;			/* unsigned  8 bit integer			*/
typedef unsigned short	U16;			/* unsigned 16 bit integer			*/
typedef unsigned long	U32;			/* unsigned 32 bit integer			*/
typedef float			F32;			/* 32 bit floating-point number		*/
typedef void*			VP;				/* pointer to variable data type	*/
typedef int				BOOL;			/* boolean							*/


/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------*/
/* 1msec timer wait function                                          */
/* e.g.) Set 1-msec wait function according to use environment.       */
/*--------------------------------------------------------------------*/
/*--- Timer wait function ---*/
/* extern void WaitMSec(const unsigned long Count); */
#define TIMER_WAIT_MSEC(a)		 WaitMSec(a) 

/*--------------------------------------------------------------------*/
/* Exclusive control definition                                       */
/* e.g.) Set exclusive control to use environment.                    */
/*--------------------------------------------------------------------*/
/*--- Access Register ---*/
#define EXCTRL_PORT_LOCK_REG					/* e.g.) (__DI;)	*/
#define EXCTRL_PORT_UNLOCK_REG					/* e.g.) (__EI;)	*/

/*--------------------------------------------------------------------*/
/* Port connection selection : PARALLEL                               */
/*--------------------------------------------------------------------*/
#if YVC_BUS_PARALLEL
	/* e.g. Set up codes in accordance with user's environment.								*/
	/* YVC_WRITE_PORT(p, d)	(*(volatile U08 *)(Y643_TOP_PORT_ADDR + (p)) = (d))				*/
	/* YVC_READ_PORT(p)		(*(volatile U08 *)(Y643_TOP_PORT_ADDR + (p)))					*/
	#define Y643_TOP_PORT_ADDR		(0x00400000)	/* Y643 port allocation start address	*/
	#define Y643_BOTTOM_PORT_ADDR	(0x00400007)	/* Y643 port allocation end address		*/

	#define YVC_WRITE_PORT(p, d)	(*(volatile U08 *)(Y643_TOP_PORT_ADDR + (p)) = (d))
	#define YVC_READ_PORT(p)		(*(volatile U08 *)(Y643_TOP_PORT_ADDR + (p)))
#endif /* YVC_BUS_PARALLEL	*/

/*--------------------------------------------------------------------*/
/* Port connection selection : SERIAL                                 */
/*--------------------------------------------------------------------*/

#if YVC_BUS_SERIAL
	/* e.g. Set up codes in accordance with user's environment.		*/
	#define YVC_WRITE_PORT(p, d)    YVC_WritePort(p, d)
	#define YVC_READ_PORT(p)        YVC_ReadPort(p)

	#if YVC_BUS_SERIAL_BURST
			#define YVC_BURST_WRITE_PORT(p, d, n)       YVC_BurstWritePort(p, d, n)
			#define YVC_BURST_READ_PORT(p, d, n)        YVC_BurstReadPort(p, d, n)
	#endif /* YVC_BUS_SERIAL_BURST */
	
#endif /* YVC_BUS_SERIAL	*/


/*------------------------------------------------*/
/* Basic Definition                               */
/*------------------------------------------------*/
#ifndef NULL
	#define NULL	((void *)0)			/* NULL setting				*/
#endif
#ifndef TRUE
	#define TRUE	0x01				/* TRUE						*/
#endif
#ifndef FALSE
	#define FALSE	0x00				/* FALSE					*/
#endif


/*--- Shift operation definition ---*/
#define SHIFT_01BIT				(1)
#define SHIFT_02BIT				(2)
#define SHIFT_03BIT				(3)
#define SHIFT_04BIT				(4)
#define SHIFT_05BIT				(5)
#define SHIFT_06BIT				(6)
#define SHIFT_07BIT				(7)
#define SHIFT_08BIT				(8)
#define SHIFT_09BIT				(9)
#define SHIFT_10BIT				(10)
#define SHIFT_11BIT				(11)
#define SHIFT_12BIT				(12)
#define SHIFT_13BIT				(13)
#define SHIFT_14BIT				(14)
#define SHIFT_15BIT				(15)
#define SHIFT_16BIT				(16)
#define SHIFT_17BIT				(17)
#define SHIFT_18BIT				(18)
#define SHIFT_19BIT				(19)
#define SHIFT_20BIT				(20)
#define SHIFT_21BIT				(21)
#define SHIFT_22BIT				(22)
#define SHIFT_23BIT				(23)
#define SHIFT_24BIT				(24)
#define SHIFT_25BIT				(25)
#define SHIFT_26BIT				(26)
#define SHIFT_27BIT				(27)
#define SHIFT_28BIT				(28)
#define SHIFT_29BIT				(29)
#define SHIFT_30BIT				(30)
#define SHIFT_31BIT				(31)

/*--- Mask operation definition ---*/
#define MASK_01H				(0x01U)
#define MASK_02H				(0x02U)
#define MASK_04H				(0x04U)
#define MASK_08H				(0x08U)
#define MASK_10H				(0x10U)
#define MASK_20H				(0x20U)
#define MASK_40H				(0x40U)
#define MASK_80H				(0x80U)
#define MASK_03H				(0x03U)
#define MASK_07H				(0x07U)
#define MASK_0FH				(0x0FU)
#define MASK_1FH				(0x1FU)
#define MASK_3FH				(0x3FU)
#define MASK_7FH				(0x7FU)
#define MASK_BFH				(0xBFU)
#define MASK_E0H				(0xE0U)
#define MASK_EFH				(0xEFU)
#define MASK_F0H				(0xF0U)
#define MASK_F8H				(0xF8U)
#define MASK_FCH				(0xFCU)
#define MASK_FFH				(0xFFU)
#define MASK_00FFH				(0x00FFU)
#define MASK_FF00H				(0xFF00U)



/*------------------------------------------------*/
/* YGV643 Basic Definition                        */
/*------------------------------------------------*/
#define Y643_POLLING_MAX        (300000UL)      /* Driver polling maximum                   */
#define Y643_DOUBLE_BUFF_ADDR   (0x1000U)       /* Double buffer start address              */
#define Y643_EXGTBL_ADDR        (0x2000U)       /* Extended general table start address     */
#define Y643_REG_SIZE           (256)           /* Number of registers                      */
#define Y643_GTBL_SIZE          (8192)          /* General table size                       */
#define Y643_EXGTBL_SIZE        (512)           /* Extended general table size              */
#define Y643_LYR_SIZE           (12)            /* General table layer size                 */
#define Y643_EXLYR_SIZE         (1)             /* Extended general table layer size        */
#define Y643_LINEATTR_SIZE      (8)             /* General table line attribute data size   */
#define Y643_FONTTYPATTR_SIZE   (8)             /* Font type attribute data size            */
#define Y643_FTEXT_SIZE         (2)             /* Fixed-pitch font data size               */
#define Y643_PTEXT_SIZE         (4)             /* Proportional font data size              */
#define Y643_COLORPLT_SIZE      (4096)          /* Color palette size                       */
#define Y643_GAMTBL_SIZE        (768)           /* Gamma Correction Table                   */
#define Y643_WRPTBL_SIZE        (8192)          /* Warping Parameter Table size             */
#define Y643_PLTI_SIZE          (1024)          /* PLTI size                                */
#define Y643_PMEM_SIZE          (0x04000000UL)  /* Pattern memory size                      */
#define Y643_PMEM_TOP_ADDR      (0x00000000UL)  /* Pattern memory start address             */
#define Y643_PMEM_END_ADDR      (0x03FFFFFFUL)  /* Pattern memory end address               */
#define Y643_EXLYR_ADDR_CALCPRM (8)             /* Parameter for the address computation of an extended layer */

/*--- Layer type ---*/
#define LYSEL_FONT				(6)
#define LYSEL_LINE				(7)
#define LYSEL_CLP				(7)

/*--- Register ---*/
#define Y643REG_AIRG			(U08)(0x80)

/*--- General table ---*/
#define SET_LYR_ERASE_ATTR_DEF      {0x10U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U,0x00U}
#define SET_EXLYR_ERASE_ATTR_DEF    {0x00U}

/*------------------------------------------------*/ 
/* Port Access Number                             */
/*------------------------------------------------*/
typedef enum {
	Y643_PORT_GENTBL_DATA = 0,
	Y643_PORT_PLT_GAM_WRP_DATA,
	Y643_PORT_PTNMEM_READACS,
	Y643_PORT_PTNMEM_DATA,
	Y643_PORT_REG_SEL,
	Y643_PORT_REG_DATA,
	Y643_PORT_FLG_STATUS1,
	Y643_PORT_FLG_STATUS2
} T_Y643_PORT_NUMBER;

/*------------------------------------------------*/ 
/* Table Mapping                                  */
/*------------------------------------------------*/
#define Y643_INTTBL_PLTTBL_SADDR		(0x0000)
#define Y643_INTTBL_PLTTBL_EADDR		(0x0FFF)
#define Y643_INTTBL_GAMTBL_SADDR		(0x2000)
#define Y643_INTTBL_GAMTBL_EADDR		(0x22FF)
#define Y643_INTTBL_WRPTBL_SADDR		(0x4000)
#define Y643_INTTBL_WRPTBL_EADDR		(0x5FFF)

/*------------------------------------------------*/
/* Register  number                               */
/*------------------------------------------------*/
typedef enum {
	REG_R00H = 0,	REG_R01H,	REG_R02H,	REG_R03H,	REG_R04H,	REG_R05H,	REG_R06H,	REG_R07H,
	REG_R08H,		REG_R09H,	REG_R0AH,	REG_R0BH,	REG_R0CH,	REG_R0DH,	REG_R0EH,	REG_R0FH,
	REG_R10H,		REG_R11H,	REG_R12H,	REG_R13H,	REG_R14H,	REG_R15H,	REG_R16H,	REG_R17H,
	REG_R18H,		REG_R19H,	REG_R1AH,	REG_R1BH,	REG_R1CH,	REG_R1DH,	REG_R1EH,	REG_R1FH,
	REG_R20H,		REG_R21H,	REG_R22H,	REG_R23H,	REG_R24H,	REG_R25H,	REG_R26H,	REG_R27H,
	REG_R28H,		REG_R29H,	REG_R2AH,	REG_R2BH,	REG_R2CH,	REG_R2DH,	REG_R2EH,	REG_R2FH,
	REG_R30H,		REG_R31H,	REG_R32H,	REG_R33H,	REG_R34H,	REG_R35H,	REG_R36H,	REG_R37H,
	REG_R38H,		REG_R39H,	REG_R3AH,	REG_R3BH,	REG_R3CH,	REG_R3DH,	REG_R3EH,	REG_R3FH,
	REG_R40H,		REG_R41H,	REG_R42H,	REG_R43H,	REG_R44H,	REG_R45H,	REG_R46H,	REG_R47H,
	REG_R48H,		REG_R49H,	REG_R4AH,	REG_R4BH,	REG_R4CH,	REG_R4DH,	REG_R4EH,	REG_R4FH,
	REG_R50H,		REG_R51H,	REG_R52H,	REG_R53H,	REG_R54H,	REG_R55H,	REG_R56H,	REG_R57H,
	REG_R58H,		REG_R59H,	REG_R5AH,	REG_R5BH,	REG_R5CH,	REG_R5DH,	REG_R5EH,	REG_R5FH,
	REG_R60H,		REG_R61H,	REG_R62H,	REG_R63H
} T_Y643_REG_NUMBER_0;

typedef enum {
	REG_R64H = 100,	REG_R65H,	REG_R66H,	REG_R67H,	REG_R68H,	REG_R69H,	REG_R6AH,	REG_R6BH,
	REG_R6CH,		REG_R6DH,	REG_R6EH,	REG_R6FH,	REG_R70H,	REG_R71H,	REG_R72H,	REG_R73H,
	REG_R74H,		REG_R75H,	REG_R76H,	REG_R77H,	REG_R78H,	REG_R79H,	REG_R7AH,	REG_R7BH,
	REG_R7CH,		REG_R7DH,	REG_R7EH,	REG_R7FH,	REG_R80H,	REG_R81H,	REG_R82H,	REG_R83H,
	REG_R84H,		REG_R85H,	REG_R86H,	REG_R87H,	REG_R88H,	REG_R89H,	REG_R8AH,	REG_R8BH,
	REG_R8CH,		REG_R8DH,	REG_R8EH,	REG_R8FH,	REG_R90H,	REG_R91H,	REG_R92H,	REG_R93H,
	REG_R94H,		REG_R95H,	REG_R96H,	REG_R97H,	REG_R98H,	REG_R99H,	REG_R9AH,	REG_R9BH,
	REG_R9CH,		REG_R9DH,	REG_R9EH,	REG_R9FH,	REG_RA0H,	REG_RA1H,	REG_RA2H,	REG_RA3H,
	REG_RA4H,		REG_RA5H,	REG_RA6H,	REG_RA7H,	REG_RA8H,	REG_RA9H,	REG_RAAH,	REG_RABH,
	REG_RACH,		REG_RADH,	REG_RAEH,	REG_RAFH,	REG_RB0H,	REG_RB1H,	REG_RB2H,	REG_RB3H,
	REG_RB4H,		REG_RB5H,	REG_RB6H,	REG_RB7H,	REG_RB8H,	REG_RB9H,	REG_RBAH,	REG_RBBH,
	REG_RBCH,		REG_RBDH,	REG_RBEH,	REG_RBFH,	REG_RC0H,	REG_RC1H,	REG_RC2H,	REG_RC3H,
	REG_RC4H,		REG_RC5H,	REG_RC6H,	REG_RC7H
} T_Y643_REG_NUMBER_1;

typedef enum {
	REG_RC8H = 200,	REG_RC9H,	REG_RCAH,	REG_RCBH,	REG_RCCH,	REG_RCDH,	REG_RCEH,	REG_RCFH,
	REG_RD0H,		REG_RD1H,	REG_RD2H,	REG_RD3H,	REG_RD4H,	REG_RD5H,	REG_RD6H,	REG_RD7H,
	REG_RD8H,		REG_RD9H,	REG_RDAH,	REG_RDBH,	REG_RDCH,	REG_RDDH,	REG_RDEH,	REG_RDFH,
	REG_RE0H,		REG_RE1H,	REG_RE2H,	REG_RE3H,	REG_RE4H,	REG_RE5H,	REG_RE6H,	REG_RE7H,
	REG_RE8H,		REG_RE9H,	REG_REAH,	REG_REBH,	REG_RECH,	REG_REDH,	REG_REEH,	REG_REFH,
	REG_RF0H,		REG_RF1H,	REG_RF2H,	REG_RF3H,	REG_RF4H,	REG_RF5H,	REG_RF6H,	REG_RF7H,
	REG_RF8H,		REG_RF9H,	REG_RFAH,	REG_RFBH,	REG_RFCH,	REG_RFDH,	REG_RFEH,	REG_RFFH
} T_Y643_REG_NUMBER_2;


/*--- Y***_ -> YVC_ Convert Setting ---*/
/*--- Port ---*/
#define YVC1_PORT_GENTBL_DATA		Y643_PORT_GENTBL_DATA
#define YVC1_PORT_PLT_GAM_WRP_DATA	Y643_PORT_PLT_GAM_WRP_DATA
#define YVC1_PORT_PTNMEM_READACS	Y643_PORT_PTNMEM_READACS
#define YVC1_PORT_PTNMEM_DATA		Y643_PORT_PTNMEM_DATA
#define YVC1_PORT_REG_SEL			Y643_PORT_REG_SEL
#define YVC1_PORT_REG_DATA			Y643_PORT_REG_DATA
#define YVC1_PORT_FLG_STATUS1		Y643_PORT_FLG_STATUS1
#define YVC1_PORT_FLG_STATUS2		Y643_PORT_FLG_STATUS2

/*--- Register ---*/
#define YVC1REG_AIRG				Y643REG_AIRG

/*--- Device ---*/
#define YVC1_POLLING_MAX            Y643_POLLING_MAX
#define YVC1_DOUBLE_BUFF_ADDR       Y643_DOUBLE_BUFF_ADDR
#define YVC1_EXGTBL_ADDR            Y643_EXGTBL_ADDR
#define YVC1_REG_SIZE               Y643_REG_SIZE
#define YVC1_GTBL_SIZE              Y643_GTBL_SIZE
#define YVC1_EXGTBL_SIZE            Y643_EXGTBL_SIZE
#define YVC1_LYR_SIZE               Y643_LYR_SIZE
#define YVC1_EXLYR_SIZE             Y643_EXLYR_SIZE
#define YVC1_LINEATTR_SIZE          Y643_LINEATTR_SIZE
#define YVC1_FONTTYPATTR_SIZE       Y643_FONTTYPATTR_SIZE
#define YVC1_FTEXT_SIZE             Y643_FTEXT_SIZE
#define YVC1_PTEXT_SIZE             Y643_PTEXT_SIZE
#define YVC1_COLORPLT_SIZE          Y643_COLORPLT_SIZE
#define YVC1_PLTI_SIZE              Y643_PLTI_SIZE
#define YVC1_GAMTBL_SIZE            Y643_GAMTBL_SIZE
#define YVC1_WRPTBL_SIZE            Y643_WRPTBL_SIZE
#define YVC1_PMEM_SIZE              Y643_PMEM_SIZE
#define YVC1_PMEM_TOP_ADDR          Y643_PMEM_TOP_ADDR
#define YVC1_PMEM_END_ADDR          Y643_PMEM_END_ADDR

#define YVC1_EXGTBL1_START_ADDR     YVC1_EXGTBL_ADDR
#define YVC1_EXGTBL2_START_ADDR     YVC1_DOUBLE_BUFF_ADDR + YVC1_EXGTBL_ADDR
#define YVC1_EXGTBL_ADDR_CALC_PARAM Y643_EXLYR_ADDR_CALCPRM

#define YVC1_INTTBL_PLTTBL_SADDR    Y643_INTTBL_PLTTBL_SADDR
#define YVC1_INTTBL_PLTTBL_EADDR    Y643_INTTBL_PLTTBL_EADDR
#define YVC1_INTTBL_GAMTBL_SADDR    Y643_INTTBL_GAMTBL_SADDR
#define YVC1_INTTBL_GAMTBL_EADDR    Y643_INTTBL_GAMTBL_EADDR
#define YVC1_INTTBL_WRPTBL_SADDR    Y643_INTTBL_WRPTBL_SADDR
#define YVC1_INTTBL_WRPTBL_EADDR    Y643_INTTBL_WRPTBL_EADDR

/*------------------------------------------------------------------------------*/
/*                      S T R U C T - S E T                                     */
/*------------------------------------------------------------------------------*/

#if YVC_BIT_FIELD

	/********************************************************************************/
	/* Y643 PORT ACCESS STRUCT                                                      */
	/********************************************************************************/
	/*--- LSB priority allocation ---*/
	/*----------------------------------------*/
	/* Port number 2                          */
	/* Pattern memory read access port        */
	/*----------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08		STARES		:1;
			U08		RGN_7		:1;
			U08		FSS			:1;
			U08		FSAE		:1;
			U08		CSNEG		:1;
			U08		INIEND		:1;
			U08		PWBUSY		:1;
			U08		PMRREQ		:1;
		} BIT;
	} T_Y643_PORT_PTNMEM;

	/*----------------------------------------*/
	/* Port number 4                          */
	/* Register select port                   */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	RGN_0 		:7;
			U08 	AIRG		:1;
		} BIT;
	} T_Y643_PORT_REGSEL;

	/*----------------------------------------*/
	/* Port number 6                          */
	/* Flag & Status port                     */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	VB			:1;
			U08 	FB			:1;
			U08 	STALN		:1;
			U08 	FD			:1;
			U08 	FERR		:1;
			U08 	FV			:1;
			U08 	FP			:1;
			U08 	FR			:1;
		} BIT;
	} T_Y643_PORT_FLAG1;

	/*----------------------------------------*/
	/* Port number 7                          */
	/* Flag & Status port                     */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	FRSE		:1;
			U08 	FRRE		:1;
			U08 	FWE 		:1;
			U08 	FMCE		:1;
			U08 	FMF 		:1;
			U08 	FML 		:1;
			U08 	FME 		:1;
			U08 	FMC 		:1;
		} BIT;
	} T_Y643_PORT_FLAG2;

	/********************************************************************************/
	/* VC1W REGISTER STRUCT                                                         */
	/********************************************************************************/

	/*-------------------------------------
	 * Software Reset Version Information
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VER			:6;
			U08 dummy1		:1;
			U08 SR			:1;
		} BIT;
	} T_Y643_R00H;

	/*-------------------------------------
	 * PLL Divider Ratio Configuration
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DTSEL_1		:1;
			U08 dummy1		:2;
			U08 PLLR		:5;
		} BIT;
	} T_Y643_R01H;

	/*-------------------------------------
	 * PLL Divider Ratio Configuration
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PLLF		:7;
			U08 dummy1		:1;
		} BIT;
	} T_Y643_R02H;

	/*-------------------------------------
	 * Clock Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DTDV		:6;
			U08 REVCK		:1;
			U08 DTSEL_0		:1;
		} BIT;
	} T_Y643_R03H;

	/*-------------------------------------
	 * PLL Divider Ratio Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:3;
			U08 RPLLR		:5;
		} BIT;
	} T_Y643_R04H;
	
	/*-------------------------------------
	 * PLL Divider Ratio Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 RPLLF		:7;
			U08 dummy1		:1;
		} BIT;
	} T_Y643_R05H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VTL_8		:2;
			U08 REVBL		:1;
			U08 REVSY		:1;
			U08 VSTM		:1;
			U08 CSYOE		:1;
			U08 CSYPAL		:1;
			U08 INTL		:1;
		} BIT;
	} T_Y643_R06H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VTL_0		:8;
		} BIT;
	} T_Y643_R07H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HTL_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R08H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HTL_0		:8;
		} BIT;
	} T_Y643_R09H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLS_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R0AH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLS_0		:8;
		} BIT;
	} T_Y643_R0BH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLS_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R0CH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLS_0		:8;
		} BIT;
	} T_Y643_R0DH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLE_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R0EH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLE_0		:8;
		} BIT;
	} T_Y643_R0FH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLE_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R10H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLE_0		:8;
		} BIT;
	} T_Y643_R11H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDS_8		:2;
			U08 NMCMD		:1;
			U08 PDDS		:5;
		} BIT;
	} T_Y643_R12H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDS_0		:8;
		} BIT;
	} T_Y643_R13H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDS_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R14H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDS_0		:8;
		} BIT;
	} T_Y643_R15H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDE_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R16H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDE_0		:8;
		} BIT;
	} T_Y643_R17H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDE_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R18H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDE_0		:8;
		} BIT;
	} T_Y643_R19H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VSW			:4;
			U08 HSW_8		:2;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R1AH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HSW_0		:8;
		} BIT;
	} T_Y643_R1BH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DTEN		:1;
			U08 SYEN		:1;
			U08 dummy1		:1;
			U08 EVEN		:1;
			U08 dummy2		:2;
			U08 DMASEL		:2;
		} BIT;
	} T_Y643_R1CH;

	/*--------------------------------------
	 * Drawing Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DRTIM_8		:8;
		} BIT;
	} T_Y643_R1DH;

	/*-------------------------------------
	 * Drawing Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DRTIM_0		:8;
		} BIT;
	} T_Y643_R1EH;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CON			:8;
		} BIT;
	} T_Y643_R1FH;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BRI_8		:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_R20H;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BRI_0		:8;
		} BIT;
	} T_Y643_R21H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IEHNG		:1;
			U08 IEB			:1;
			U08 dummy1		:1;
			U08 IED			:1;
			U08 IERR		:1;
			U08 IEV			:1;
			U08 IEP			:1;
			U08 IER			:1;
		} BIT;
	} T_Y643_R22H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:1;
			U08 IESE		:1;
			U08 IESS		:1;
			U08 IESA		:1;
			U08 IEMF		:1;
			U08 IEML		:1;
			U08 IEME		:1;
			U08 IEMC		:1;
		} BIT;
	} T_Y643_R23H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IH_0		:8;
		} BIT;
	} T_Y643_R24H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IL_8		:2;
			U08 dummy1		:2;
			U08 IH_8		:3;
			U08 FPM			:1;
		} BIT;
	} T_Y643_R25H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IL_0		:8;
		} BIT;
	} T_Y643_R26H;

	/*-------------------------------------
	 * Address Increment Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 AICP		:1;
			U08 AIST		:1;
			U08 AIPM		:1;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_R27H;

	/*-------------------------------------
	 * General Table Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SA_8		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R28H;

	/*-------------------------------------
	 * General Table Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SA_0		:8;
		} BIT;
	} T_Y643_R29H;
	
	/*--------------------------------------
	 * Color Palette Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CP_8		:7;
			U08 dummy1		:1;
		} BIT;
	} T_Y643_R2AH;

	/*--------------------------------------
	 * Color Palette Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CP_0		:8;
		} BIT;
	} T_Y643_R2BH;
	
	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R2CH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_16		:8;
		} BIT;
	} T_Y643_R2DH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_8		:8;
		} BIT;
	} T_Y643_R2EH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_0		:8;
		} BIT;
	} T_Y643_R2FH;

	/*--------------------------------------
	 * Stand-Alone/Alpha Value Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 ALPBSEL		:1;
			U08 EXLY		:1;
			U08 dummy1		:1;
			U08 STAPLY		:1;
			U08 dummy2		:4;
		} BIT;
	} T_Y643_R30H;

	/*--------------------------------------
	 * Video Output Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VOBSEL			:1;
			U08 FRCM			:1;
			U08 FRCE			:1;
			U08 dummy1			:5;
		} BIT;
	} T_Y643_R31H;

	/*--------------------------------------
	 * Debug Flags
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STSEL		:2;
			U08 dummy1		:1;
			U08 FSE			:1;
			U08 FSA			:1;
			U08 FDDBUF		:1;
			U08 FDSREND		:1;
			U08 FDDEC		:1;
		} BIT;
	} T_Y643_R32H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CHS			:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_R33H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R34H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_16		:8;
		} BIT;
	} T_Y643_R35H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_8		:8;
		} BIT;
	} T_Y643_R36H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_0		:8;
		} BIT;
	} T_Y643_R37H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R38H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_16		:8;
		} BIT;
	} T_Y643_R39H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_8		:8;
		} BIT;
	} T_Y643_R3AH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_0		:8;
		} BIT;
	} T_Y643_R3BH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SUMD_8		:8;
		} BIT;
	} T_Y643_R3CH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SUMD_0		:8;
		} BIT;
	} T_Y643_R3DH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYSAC_8		:4;
			U08 dummy1		:2;
			U08 LYBSELC		:1;
			U08 LYDC		:1;
		} BIT;
	} T_Y643_R3EH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 LYSAC_2		:6;
		} BIT;
	} T_Y643_R3FH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYEAC_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_R40H;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 LYEAC_2		:6;
		} BIT;
	} T_Y643_R41H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYSAM_8		:4;
			U08 dummy1		:2;
			U08 LYBSELM		:1;
			U08 LYDM		:1;
		} BIT;
	} T_Y643_R42H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 LYSAM_2		:6;
		} BIT;
	} T_Y643_R43H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYEAM_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_R44H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 LYEAM_2		:6;
		} BIT;
	} T_Y643_R45H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSR		:8;
		} BIT;
	} T_Y643_R46H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSG		:8;
		} BIT;
	} T_Y643_R47H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSB		:8;
		} BIT;
	} T_Y643_R48H;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPR			:8;
		} BIT;
	} T_Y643_R49H;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPG			:8;
		} BIT;
	} T_Y643_R4AH;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPB			:8;
		} BIT;
	} T_Y643_R4BH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCR		:8;
		} BIT;
	} T_Y643_R4CH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCG		:8;
		} BIT;
	} T_Y643_R4DH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCB		:8;
		} BIT;
	} T_Y643_R4EH;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCR		:8;
		} BIT;
	} T_Y643_R4FH;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCG		:8;
		} BIT;
	} T_Y643_R50H;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCB		:8;
		} BIT;
	} T_Y643_R51H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDR			:8;
		} BIT;
	} T_Y643_R52H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDG			:8;
		} BIT;
	} T_Y643_R53H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDB			:8;
		} BIT;
	} T_Y643_R54H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 MMOD		:2;
			U08 CAP			:2;
			U08 SP			:1;
			U08 INSKIP		:1;
			U08 CSCAP		:2;
		} BIT;
	} T_Y643_R55H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DTTIM		:3;
			U08 CSTIM		:3;
			U08 NMBIT		:2;
		} BIT;
	} T_Y643_R56H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 MBTIM		:3;
			U08 ADTIM		:3;
			U08 PMOD		:2;
		} BIT;
	} T_Y643_R57H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 INTIM		:2;
			U08 DMTIM		:6;
		} BIT;
	} T_Y643_R58H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 INCOD		:8;
		} BIT;
	} T_Y643_R59H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMBIT_8		:8;
		} BIT;
	} T_Y643_R5AH;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMBIT_0		:8;
		} BIT;
	} T_Y643_R5BH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R5CH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_16		:8;
		} BIT;
	} T_Y643_R5DH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_8		:8;
		} BIT;
	} T_Y643_R5EH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 SSA_2		:6;
		} BIT;
	} T_Y643_R5FH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STEP		:1;
			U08 dummy1		:2;
			U08 MCRS		:1;
			U08 BREAK		:1;
			U08 STOP		:1;
			U08 PAUSE		:1;
			U08 PLAY		:1;
		} BIT;
	} T_Y643_R60H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R61H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_16		:8;
		} BIT;
	} T_Y643_R62H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_8		:8;
		} BIT;
	} T_Y643_R63H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_0		:8;
		} BIT;
	} T_Y643_R64H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LC			:8;
		} BIT;
	} T_Y643_R65H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IVC			:8;
		} BIT;
	} T_Y643_R66H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STHS_8		:3;
			U08 STHE_8		:3;
			U08 dummy1		:1;
			U08 REVSH		:1;
		} BIT;
	} T_Y643_R67H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STHE_0		:8;
		} BIT;
	} T_Y643_R68H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STHS_0		:8;
		} BIT;
	} T_Y643_R69H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LDHS_8		:3;
			U08 LDHE_8		:3;
			U08 dummy1		:1;
			U08 REVLH		:1;
		} BIT;
	} T_Y643_R6AH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LDHE_0		:8;
		} BIT;
	} T_Y643_R6BH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LDHS_0		:8;
		} BIT;
	} T_Y643_R6CH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CKVS_8		:3;
			U08 CKVE_8		:3;
			U08 dummy1		:1;
			U08 REVCV		:1;
		} BIT;
	} T_Y643_R6DH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CKVE_0		:8;
		} BIT;
	} T_Y643_R6EH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CKVS_0		:8;
		} BIT;
	} T_Y643_R6FH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSV_8		:2;
			U08 STVEV_8		:2;
			U08 dummy1		:2;
			U08 REVSVV		:1;
			U08 REVSV		:1;
		} BIT;
	} T_Y643_R70H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVEV_0		:8;	
		} BIT;
	} T_Y643_R71H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSV_0		:8;
		} BIT;
	} T_Y643_R72H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSH_8		:3;
			U08 STVEH_8		:3;
			U08 dummy1		:1;
			U08 REVSVH		:1;
		} BIT;
	} T_Y643_R73H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVEH_0		:8;
		} BIT;
	} T_Y643_R74H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSH_0		:8;
		} BIT;
	} T_Y643_R75H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESV_8		:2;
			U08 OEEV_8		:2;
			U08 dummy1		:2;
			U08 REVOEV		:1;
			U08 REVOE		:1;
		} BIT;
	} T_Y643_R76H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OEEV_0		:8;
		} BIT;
	} T_Y643_R77H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESV_0		:8;
		} BIT;
	} T_Y643_R78H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESH_8		:3;
			U08 OEEH_8		:3;
			U08 dummy1		:1;
			U08 REVOEH		:1;
		} BIT;
	} T_Y643_R79H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OEEH_0		:8;
		} BIT;
	} T_Y643_R7AH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESH_0		:8;
		} BIT;
	} T_Y643_R7BH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 POLP_8		:3;
			U08 POLM		:3;
			U08 dummy1		:1;
			U08 POLE		:1;
		} BIT;
	} T_Y643_R7CH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 POLP_0		:8;
		} BIT;
	} T_Y643_R7DH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R7EH;

	/*--------------------------------------
	 * Scratch Pad Register
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TMP			:8;
		} BIT;
	} T_Y643_R7FH;

	/*-------------------------------------
	 * Warping Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 WPEN		:1;
		} BIT;
	} T_Y643_R80H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSY_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R81H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSY_0		:8;
		} BIT;
	} T_Y643_R82H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSX_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R83H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSX_0		:8;
		} BIT;
	} T_Y643_R84H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEY_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R85H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEY_0		:8;
		} BIT;
	} T_Y643_R86H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEX_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R87H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEX_0		:8;
		} BIT;
	} T_Y643_R88H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBX			:8;
		} BIT;
	} T_Y643_R89H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8AH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8BH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8CH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8DH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8EH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8FH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R90H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R91H;

	/*-------------------------------------
	 * Capture Buffer Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LIDL_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R92H;

	/*-------------------------------------
	 * Capture Buffer Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LIDL_0		:8;
		} BIT;
	} T_Y643_R93H;

	/*-------------------------------------
	 * Video Output
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 CTC			:1;
			U08 GME			:1;
			U08 EEF			:1;
			U08 BLF			:1;
		} BIT;
	} T_Y643_R94H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DCON		:8;
		} BIT;
	} T_Y643_R95H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DBRI_8		:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_R96H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DBRI_0		:8;
		} BIT;
	} T_Y643_R97H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DSATU		:8;
		} BIT;
	} T_Y643_R98H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DHUE		:8;
		} BIT;
	} T_Y643_R99H;

	/*-------------------------------------
	 * Edge Enhancement Filter
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EEFC		:8;
		} BIT;
	} T_Y643_R9AH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R9BH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R9CH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 WCCOMP		:1;
			U08 WTBLSEL		:1;
			U08 WCIPMH		:1;
			U08 WCIPMV		:1;
			U08 dummy2		:1;
			U08 WCEN		:1;
		} BIT;
	} T_Y643_R9DH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSY_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_R9EH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSY_0		:8;
		} BIT;
	} T_Y643_R9FH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSX_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RA0H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSX_0		:8;
		} BIT;
	} T_Y643_RA1H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPY_8	:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RA2H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPY_0	:8;
		} BIT;
	} T_Y643_RA3H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPX_8	:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RA4H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPX_0	:8;
		} BIT;
	} T_Y643_RA5H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMY_8	:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RA6H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMY_0	:8;
		} BIT;
	} T_Y643_RA7H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMX_8	:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RA8H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMX_0	:8;
		} BIT;
	} T_Y643_RA9H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_24		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RAAH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_16		:8;
		} BIT;
	} T_Y643_RABH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_8		:8;
		} BIT;
	} T_Y643_RACH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_0		:8;
		} BIT;
	} T_Y643_RADH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RAEH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RAFH;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_24	:2;
			U08 dummy1		:5;
			U08 SDCOMP		:1;
		} BIT;
	} T_Y643_RB0H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_16	:8;
		} BIT;
	} T_Y643_RB1H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_8	:8;
		} BIT;
	} T_Y643_RB2H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_0	:8;
		} BIT;
	} T_Y643_RB3H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DESTAD_8	:5;
			U08 dummy1		:2;
			U08 DESTSEL		:1;
		} BIT;
	} T_Y643_RB4H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DESTAD_0	:8;
		} BIT;
	} T_Y643_RB5H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BYTECNT_8	:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_RB6H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BYTECNT_0	:8;
		} BIT;
	} T_Y643_RB7H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 DMACLR		:1;
			U08 dummy2		:1;
			U08 DSTART		:1;
		} BIT;
	} T_Y643_RB8H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:1;
			U08 DESTERR		:1;
			U08 SOCEERR		:1;
			U08 NODMAERR	:1;
			U08 DMAEND		:1;
			U08 dummy2		:3;
		} BIT;
	} T_Y643_RB9H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RBAH;

	/*-------------------------------------
	 * Warping parameter Correction
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 CODR		:3;
		} BIT;
	} T_Y643_RBBH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TY_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RBCH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TY_8		:8;
		} BIT;
	} T_Y643_RBDH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TY_0		:8;
		} BIT;
	} T_Y643_RBEH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TX_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RBFH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TX_8		:8;
		} BIT;
	} T_Y643_RC0H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TX_0		:8;
		} BIT;
	} T_Y643_RC1H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CA_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_RC2H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CA_0		:8;
		} BIT;
	} T_Y643_RC3H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CB_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_RC4H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CB_0		:8;
		} BIT;
	} T_Y643_RC5H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_RC6H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC_0		:8;
		} BIT;
	} T_Y643_RC7H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CD_8		:4;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_RC8H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CD_0		:8;
		} BIT;
	} T_Y643_RC9H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1SELSH	:1;
			U08 CC1SELX		:2;
			U08 CC1SELY		:2;
			U08 dummy1		:2;
			U08 CC1SELA		:1;
		} BIT;
	} T_Y643_RCAH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TY_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RCBH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TY_8		:8;
		} BIT;
	} T_Y643_RCCH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TY_0		:8;
		} BIT;
	} T_Y643_RCDH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TX_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RCEH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TX_8		:8;
		} BIT;
	} T_Y643_RCFH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TX_0		:8;
		} BIT;
	} T_Y643_RD0H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1KC_8		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_RD1H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1KC_0		:8;
		} BIT;
	} T_Y643_RD2H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2SELSH	:1;
			U08 CC2SELX		:2;
			U08 CC2SELY		:2;
			U08 CC2SELC		:1;
			U08 CC2SELB		:1;
			U08 CC2SELA		:1;
		} BIT;
	} T_Y643_RD3H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TY_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RD4H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TY_8		:8;
		} BIT;
	} T_Y643_RD5H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TY_0		:8;
		} BIT;
	} T_Y643_RD6H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TX_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RD7H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TX_8		:8;
		} BIT;
	} T_Y643_RD8H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TX_0		:8;
		} BIT;
	} T_Y643_RD9H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2KC_8		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_RDAH;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2KC_0		:8;
		} BIT;
	} T_Y643_RDBH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TY_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RDCH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TY_8		:8;
		} BIT;
	} T_Y643_RDDH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TY_0		:8;
		} BIT;
	} T_Y643_RDEH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TX_16	:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RDFH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TX_8		:8;
		} BIT;
	} T_Y643_RE0H;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TX_0		:8;
		} BIT;
	} T_Y643_RE1H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RE2H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RE3H;

	/*-------------------------------------
	 * External Sync
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VSADJ		:6;
			U08 dummy1		:1;
			U08 SYNCE		:1;
		} BIT;
	} T_Y643_RE4H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVIF		:2;
			U08 DVINTL		:1;
			U08 CRCBS		:1;
			U08 DVSP		:1;
			U08 DVPAL		:1;
			U08 EXGAM		:2;
		} BIT;
	} T_Y643_RE5H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHTL_8		:3;
			U08 dummy1		:3;
			U08 DVCSM		:1;
			U08 DVCSE		:1;
		} BIT;
	} T_Y643_RE6H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHTL_0		:8;
		} BIT;
	} T_Y643_RE7H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHSW_8		:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_RE8H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHSW_0		:8;
		} BIT;
	} T_Y643_RE9H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPAOFF		:1;
			U08 dummy1		:1;
			U08 BPHLFE		:1;
			U08 BPFLX		:1;
			U08 SFSEL		:1;
			U08 SCE			:1;
			U08 dummy2		:1;
			U08 BPD			:1;
		} BIT;
	} T_Y643_REAH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EISD_8		:5;
			U08 dummy1		:3;
		} BIT;
	} T_Y643_REBH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EISD_0		:8;
		} BIT;
	} T_Y643_RECH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSL_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_REDH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSL_0		:8;
		} BIT;
	} T_Y643_REEH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSD_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_REFH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSD_0		:8;
		} BIT;
	} T_Y643_RF0H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPEL_8		:2;
			U08 dummy1		:6;
		} BIT;
	} T_Y643_RF1H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPEL_0		:8;
		} BIT;
	} T_Y643_RF2H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPED_8		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_RF3H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPED_0		:8;
		} BIT;
	} T_Y643_RF4H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFY_8		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_RF5H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFY_0		:8;
		} BIT;
	} T_Y643_RF6H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFX_8		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_RF7H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFX_0		:8;
		} BIT;
	} T_Y643_RF8H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RF9H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RFAH;

	/*-------------------------------------
	 * Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EXADJHTL_8	:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_RFBH;

	/*-------------------------------------
	 * Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EXADJHTL_0	:8;
		} BIT;
	} T_Y643_RFCH;

	/*-------------------------------------
	 * Interrupt Enable
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IEDER		:1;
			U08 IEDEN		:1;
			U08 dummy1		:3;
			U08 IENME		:1;
			U08 IEDV		:1;
			U08 IENDS		:1;
		} BIT;
	} T_Y643_RFDH;

	/*-------------------------------------
	 * Flag and Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DEVEN		:1;
			U08 DVB			:1;
			U08 NDS			:1;
			U08 dummy1		:2;
			U08 FNME		:1;
			U08 DFV			:1;
			U08 FNDS		:1;
		} BIT;
	} T_Y643_RFEH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:8;
		} BIT;
	} T_Y643_RFFH;

	/********************************************************************************/
	/*	VC1W GENERAL TABLE ACCESS STRUCT                                            */
	/********************************************************************************/
	/*----------------------------------------
	 *	Layer structure for Sprite display
	 *----------------------------------------*/
	typedef union {
#if YVC1_EXLY
		U08 BYTE[13];
#else
		U08 BYTE[12];
#endif
		struct {
			U08 DOY_8	:3;				/* BYTE[0]						*/
			U08 MXSL	:1;
			U08 MX8		:1;
			U08 CLM		:3;
			U08 DOY_0	:8;				/* BYTE[1]						*/
			U08 DOX_8	:3;				/* BYTE[2]						*/
			U08 PLTI_8	:1;
			U08 MX_4	:4;
			U08 DOX_0	:8;				/* BYTE[3]						*/
			U08 SZY		:6;				/* BYTE[4]						*/
			U08 TP		:1;
			U08 ALPHA_1	:1;
			U08 SZX		:6;				/* BYTE[5]						*/
			U08 COMPM	:2;
			union {						/* BYTE[6]						*/
				struct {
					U08 LSYM	:3;
					U08 LSYE	:1;
					U08 DCM		:2;
					U08 LSYS	:1;
					U08 dummy1	:1;
				} RGB;
				struct {
					U08 PLTI_0	:8;
				} PLT;
			} COLOR;
			U08 PN_19	:7;				/* BYTE[7]						*/
			U08 ALPHA_0	:1;
			U08 PN_11	:8;				/* BYTE[8]						*/
			U08 PN_3	:8;				/* BYTE[9]						*/
			U08 MAGY	:8;				/* BYTE[10]						*/
			U08 MAGX	:8;				/* BYTE[11]						*/
#if YVC1_EXLY
			struct {					/* BYTE[12]						*/
				U08 MX_0	:4;
				U08 dummy1	:4;
			} EXLY;
#endif
		} BIT;
	} T_Y643_LYR_SPRTATTR;

	/*----------------------------------------
	 *	Layer structure for Text display
	 *----------------------------------------*/
	typedef union {
#if YVC1_EXLY
		U08 BYTE[13];
#else
		U08 BYTE[12];
#endif
		struct {
			U08 FDOY_8	:3;				/* BYTE[0]						*/
			U08 FMXSL	:1;
			U08 FMX8	:1;
			U08 LYSEL	:3;
			U08 FDOY_0	:8;				/* BYTE[1]						*/
			U08 FDOX_8	:3;				/* BYTE[2]						*/
			U08 FPLTI_8	:1;
			U08 FMX_4	:4;
			U08 FDOX_0	:8;				/* BYTE[3]						*/
			U08 FTYP_10	:2;				/* BYTE[4]						*/
			U08 FKNE	:1;
			U08 FDIR	:1;
			U08 FPFE	:1;
			U08 FPLTS	:1;
			U08 FTP		:1;
			U08 dummy1	:1;
			U08 FTYP_2	:8;				/* BYTE[5]						*/
			U08 FPLTI_0	:8;				/* BYTE[6]						*/
			U08 FCEA_9	:3;				/* BYTE[7]						*/
			U08 dummy3	:1;
			U08 FCSA_9	:3;
			U08 dummy2	:1;
			U08 FCSA_1	:8;				/* BYTE[8]						*/
			U08 FCEA_1	:8;				/* BYTE[9]						*/
			U08 dummy4	:8;				/* BYTE[10]						*/
			U08 dummy5	:8;				/* BYTE[11]						*/
#if YVC1_EXLY
			struct {					/* BYTE[12]						*/
				U08 FMX_0	:4;
				U08	dummy1	:4;
			} EXLY;
#endif
		} BIT;
	} T_Y643_LYR_FONTATTR;

	/*----------------------------------------
	 *	Font type attribute structure
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[8];
		struct {
			U08 FN_19		:7;			/* BYTE[0]						*/
			U08 CMODE		:1;
			U08 FN_11		:8;			/* BYTE[1]						*/
			U08 FN_3		:8;			/* BYTE[2]						*/
			U08 dummy1		:5;			/* BYTE[3]						*/
			U08 FCOMPV		:1;
			U08 FCOMPE		:1;
			U08 FHSZ		:1;
			U08 FSZX		:6;			/* BYTE[4]						*/
			U08 dummy2		:2;
			U08 FSZY		:6;			/* BYTE[5]						*/
			U08 dummy3		:2;
			U08 FDAB		:5;			/* BYTE[6]						*/
			U08 dummy4		:3;
			U08 FDLB		:5;			/* BYTE[7]						*/
			U08 dummy5		:3;
		} BIT;
	} T_Y643_FONT_TYPATTR;

	/*----------------------------------------
	 *	Proportional character code structure
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[4];
		struct {
			union {
				struct {
					U08 FCODE_8		:8;			/* BYTE[0]				*/
					U08 FCODE_0		:8;			/* BYTE[1]				*/
					U08 XS			:6;			/* BYTE[2]				*/
					U08 dummy1		:2;
					U08 XE			:6;			/* BYTE[3]				*/
					U08 dummy2		:2;
				} DOUBLE;
				struct {
					U08 FCODE_8		:7;			/* BYTE[0]				*/
					U08 dummy1		:1;
					U08 FCODE_0		:8;			/* BYTE[1]				*/
					U08 XS			:6;			/* BYTE[2]				*/
					U08 dummy2		:2;
					U08 XE			:6;			/* BYTE[3]				*/
					U08 dummy3		:2;
				} MIX;
			} GLYPH;
		} BIT;
	} T_Y643_TEXT_PFONT;

	/*----------------------------------------
	 * Layer structure for Clipping
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[12];
		struct {
			U08 dummy1	:5; 		   /* BYTE[0]					   */
			U08 LYSEL	:3;
			U08 CLPE	:2; 		   /* BYTE[1]					   */
			U08 dummy2	:6;
			U08 dummy3	:8; 		   /* BYTE[2]					   */
			U08 dummy4	:8; 		   /* BYTE[3]					   */
			U08 dummy5	:8; 		   /* BYTE[4]					   */
			U08 dummy6	:8; 		   /* BYTE[5]					   */
			U08 CLPSX_8 :3; 		   /* BYTE[6]					   */
			U08 dummy8	:1;
			U08 CLPSY_8 :3;
			U08 dummy7	:1;
			U08 CLPSY_0 :8; 		   /* BYTE[7]					   */
			U08 CLPSX_0 :8; 		   /* BYTE[8]					   */
			U08 CLPEX_8 :3; 		   /* BYTE[9]					   */
			U08 dummy10	:1;
			U08 CLPEY_8 :3;
			U08 dummy9  :1;
			U08 CLPEY_0 :8; 		   /* BYTE[10]					   */
			U08 CLPEX_0 :8; 		   /* BYTE[11]					   */
		} BIT;
	} T_Y643_LYR_CLIPATTR;

#else

	/********************************************************************************/
	/* Y643 PORT ACCESS STRUCT                                                      */
	/********************************************************************************/
	
	/*--- MSB priority allocation ---*/
	/*------------------------------------------*/
	/* Port number 2                            */
	/* Pattern memory read access port          */
	/*------------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08		PMRREQ		:1;
			U08		PWBUSY		:1;
			U08		INIEND		:1;
			U08		CSNEG		:1;
			U08		FSAE		:1;
			U08		FSS			:1;
			U08		RGN_7		:1;
			U08		STARES		:1;
		} BIT;
	} T_Y643_PORT_PTNMEM;

	/*----------------------------------------*/
	/* Port number 4                          */
	/* Register select port                   */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	AIRG		:1;
			U08 	RGN_0 		:7;
		} BIT;
	} T_Y643_PORT_REGSEL;

	/*----------------------------------------*/
	/* Port number 6                          */
	/* Flag & Status port                     */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	FR			:1;
			U08 	FP			:1;
			U08 	FV			:1;
			U08 	FERR		:1;
			U08 	FD			:1;
			U08 	STALN		:1;
			U08 	FB			:1;
			U08 	VB			:1;
		} BIT;
	} T_Y643_PORT_FLAG1;

	/*----------------------------------------*/
	/* Port number 7                          */
	/* Flag & Status port                     */
	/*----------------------------------------*/
	typedef union {
		U08 	BYTE;
		struct {
			U08 	FMC 		:1;
			U08 	FME 		:1;
			U08 	FML 		:1;
			U08 	FMF 		:1;
			U08 	FMCE		:1;
			U08 	FWE 		:1;
			U08 	FRRE		:1;
			U08 	FRSE		:1;
		} BIT;
	} T_Y643_PORT_FLAG2;

	/****************************************************************************
	 * VC1W REGISTER STRUCT
	 ****************************************************************************/
	/*-------------------------------------
	 * Software Reset Version Information
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SR			:1;
			U08 dummy1		:1;
			U08 VER			:6;
		} BIT;
	} T_Y643_R00H;

	/*-------------------------------------
	 * PLL Divider Ratio Configuration
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PLLR		:5;
			U08 dummy1		:2;
			U08 DTSEL_1		:1;
		} BIT;
	} T_Y643_R01H;

	/*-------------------------------------
	 * PLL Divider Ratio Configuration
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:1;
			U08 PLLF		:7;
		} BIT;
	} T_Y643_R02H;

	/*-------------------------------------
	 * Clock Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DTSEL_0		:1;
			U08 REVCK		:1;
			U08 DTDV		:6;
		} BIT;
	} T_Y643_R03H;

	/*-------------------------------------
	 * PLL Divider Ratio Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 RPLLR		:5;
			U08 dummy1		:3;
		} BIT;
	} T_Y643_R04H;

	/*-------------------------------------
	 * PLL Divider Ratio Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:1;
			U08 RPLLF		:7;
		} BIT;
	} T_Y643_R05H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 INTL		:1;
			U08 CSYPAL		:1;
			U08 CSYOE		:1;
			U08 VSTM		:1;
			U08 REVSY		:1;
			U08 REVBL		:1;
			U08 VTL_8		:2;
		} BIT;
	} T_Y643_R06H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VTL_0		:8;
		} BIT;
	} T_Y643_R07H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 HTL_8		:3;
		} BIT;
	} T_Y643_R08H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HTL_0		:8;
		} BIT;
	} T_Y643_R09H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 VBLS_8		:2;
		} BIT;
	} T_Y643_R0AH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLS_0		:8;
		} BIT;
	} T_Y643_R0BH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 HBLS_8		:3;
		} BIT;
	} T_Y643_R0CH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLS_0		:8;
		} BIT;
	} T_Y643_R0DH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 VBLE_8		:2;
		} BIT;
	} T_Y643_R0EH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VBLE_0		:8;
		} BIT;
	} T_Y643_R0FH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 HBLE_8		:3;
		} BIT;
	} T_Y643_R10H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HBLE_0		:8;
		} BIT;
	} T_Y643_R11H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PDDS		:5;
			U08 NMCMD		:1;
			U08 VDS_8		:2;
		} BIT;
	} T_Y643_R12H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDS_0		:8;
		} BIT;
	} T_Y643_R13H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 HDS_8		:3;
		} BIT;
	} T_Y643_R14H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDS_0		:8;
		} BIT;
	} T_Y643_R15H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 VDE_8		:2;
		} BIT;
	} T_Y643_R16H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 VDE_0		:8;
		} BIT;
	} T_Y643_R17H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 HDE_8		:3;
		} BIT;
	} T_Y643_R18H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HDE_0		:8;
		} BIT;
	} T_Y643_R19H;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 HSW_8		:2;
			U08 VSW			:4;
		} BIT;
	} T_Y643_R1AH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 HSW_0		:8;
		} BIT;
	} T_Y643_R1BH;

	/*-------------------------------------
	 * Display Scan Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMASEL		:2;
			U08 dummy2		:2;
			U08 EVEN		:1;
			U08 dummy1		:1;
			U08 SYEN		:1;
			U08 DTEN		:1;
		} BIT;
	} T_Y643_R1CH;

	/*--------------------------------------
	 * Drawing Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DRTIM_8		:8;
		} BIT;
	} T_Y643_R1DH;

	/*-------------------------------------
	 * Drawing Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DRTIM_0		:8;
		} BIT;
	} T_Y643_R1EH;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CON			:8;
		} BIT;
	} T_Y643_R1FH;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 BRI_8		:1;
		} BIT;
	} T_Y643_R20H;

	/*-------------------------------------
	 * Layer Image Quality Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BRI_0		:8;
		} BIT;
	} T_Y643_R21H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IER			:1;
			U08 IEP			:1;
			U08 IEV			:1;
			U08 IERR		:1;
			U08 IED			:1;
			U08 dummy1		:1;
			U08 IEB			:1;
			U08 IEHNG		:1;
		} BIT;
	} T_Y643_R22H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IEMC		:1;
			U08 IEME		:1;
			U08 IEML		:1;
			U08 IEMF		:1;
			U08 IESA		:1;
			U08 IESS		:1;
			U08 IESE		:1;
			U08 dummy1		:1;
		} BIT;
	} T_Y643_R23H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IH_0		:8;
		} BIT;
	} T_Y643_R24H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FPM			:1;
			U08 IH_8		:3;
			U08 dummy1		:2;
			U08 IL_8		:2;
		} BIT;
	} T_Y643_R25H;

	/*-------------------------------------
	 * Interrupt Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IL_0		:8;
		} BIT;
	} T_Y643_R26H;

	/*-------------------------------------
	 * Address Increment Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 AIPM		:1;
			U08 AIST		:1;
			U08 AICP		:1;
		} BIT;
	} T_Y643_R27H;

	/*-------------------------------------
	 * General Table Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 SA_8		:6;
		} BIT;
	} T_Y643_R28H;

	/*-------------------------------------
	 * General Table Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SA_0		:8;
		} BIT;
	} T_Y643_R29H;

	/*--------------------------------------
	 * Color Palette Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:1;
			U08 CP_8		:7;
		} BIT;
	} T_Y643_R2AH;

	/*--------------------------------------
	 * Color Palette Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CP_0		:8;
		} BIT;
	} T_Y643_R2BH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 PMA_24		:2;
		} BIT;
	} T_Y643_R2CH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_16		:8;
		} BIT;
	} T_Y643_R2DH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_8		:8;
		} BIT;
	} T_Y643_R2EH;

	/*--------------------------------------
	 * Pattern Memory Address
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMA_0		:8;
		} BIT;
	} T_Y643_R2FH;

	/*--------------------------------------
	 * Stand-Alone/Alpha Value Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy2		:4;
			U08 STAPLY		:1;
			U08 dummy1		:1;
			U08 EXLY		:1;
			U08 ALPBSEL		:1;
		} BIT;
	} T_Y643_R30H;

	/*--------------------------------------
	 * Video Output Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1			:5;
			U08 FRCE			:1;
			U08 FRCM			:1;
			U08 VOBSEL			:1;
		} BIT;
	} T_Y643_R31H;

	/*--------------------------------------
	 * Debug Flags
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FDDEC		:1;
			U08 FDSREND		:1;
			U08 FDDBUF		:1;
			U08 FSA			:1;
			U08 FSE			:1;
			U08 dummy1		:1;
			U08 STSEL		:2;
		} BIT;
	} T_Y643_R32H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CHS			:1;
		} BIT;
	} T_Y643_R33H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CSSA_24		:2;
		} BIT;
	} T_Y643_R34H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_16		:8;
		} BIT;
	} T_Y643_R35H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_8		:8;
		} BIT;
	} T_Y643_R36H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSSA_0		:8;
		} BIT;
	} T_Y643_R37H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CSEA_24		:2;
		} BIT;
	} T_Y643_R38H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_16		:8;
		} BIT;
	} T_Y643_R39H;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_8		:8;
		} BIT;
	} T_Y643_R3AH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSEA_0		:8;
		} BIT;
	} T_Y643_R3BH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SUMD_8		:8;
		} BIT;
	} T_Y643_R3CH;

	/*--------------------------------------
	 * Pattern Memory Checksum
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SUMD_0		:8;
		} BIT;
	} T_Y643_R3DH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYDC		:1;
			U08 LYBSELC		:1;
			U08 dummy1		:2;
			U08 LYSAC_8		:4;
		} BIT;
	} T_Y643_R3EH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYSAC_2		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R3FH;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 LYEAC_8		:4;
		} BIT;
	} T_Y643_R40H;

	/*--------------------------------------
	 * Host Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYEAC_2		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R41H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYDM		:1;
			U08 LYBSELM		:1;
			U08 dummy1		:2;
			U08 LYSAM_8		:4;
		} BIT;
	} T_Y643_R42H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYSAM_2		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R43H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 LYEAM_8		:4;
		} BIT;
	} T_Y643_R44H;

	/*--------------------------------------
	 * Macro Controlled Layer
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LYEAM_2		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R45H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSR		:8;
		} BIT;
	} T_Y643_R46H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSG		:8;
		} BIT;
	} T_Y643_R47H;

	/*--------------------------------------
	 * Blending Base Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SPSB		:8;
		} BIT;
	} T_Y643_R48H;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPR			:8;
		} BIT;
	} T_Y643_R49H;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPG			:8;
		} BIT;
	} T_Y643_R4AH;

	/*--------------------------------------
	 * Transparent Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TPB			:8;
		} BIT;
	} T_Y643_R4BH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 *  
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCR		:8;
		} BIT;
	} T_Y643_R4CH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 *  
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCG		:8;
		} BIT;
	} T_Y643_R4DH;

	/*--------------------------------------
	 * Font Color (Fore Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FFCB		:8;
		} BIT;
	} T_Y643_R4EH;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCR		:8;
		} BIT;
	} T_Y643_R4FH;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCG		:8;
		} BIT;
	} T_Y643_R50H;

	/*--------------------------------------
	 * Font Color (Back Color)
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBCB		:8;
		} BIT;
	} T_Y643_R51H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDR			:8;
		} BIT;
	} T_Y643_R52H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDG			:8;
		} BIT;
	} T_Y643_R53H;

	/*--------------------------------------
	 * Border Color
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BDB			:8;
		} BIT;
	} T_Y643_R54H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CSCAP		:2;
			U08 INSKIP		:1;
			U08 SP			:1;
			U08 CAP			:2;
			U08 MMOD		:2;
		} BIT;
	} T_Y643_R55H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 NMBIT		:2;
			U08 CSTIM		:3;
			U08 DTTIM		:3;
		} BIT;
	} T_Y643_R56H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PMOD		:2;
			U08 ADTIM		:3;
			U08 MBTIM		:3;
		} BIT;
	} T_Y643_R57H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMTIM		:6;
			U08 INTIM		:2;
		} BIT;
	} T_Y643_R58H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 INCOD		:8;
		} BIT;
	} T_Y643_R59H;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMBIT_8		:8;
		} BIT;
	} T_Y643_R5AH;

	/*--------------------------------------
	 * Pattern Memory Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DMBIT_0		:8;
		} BIT;
	} T_Y643_R5BH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 SSA_24		:2;
		} BIT;
	} T_Y643_R5CH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_16		:8;
		} BIT;
	} T_Y643_R5DH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_8		:8;
		} BIT;
	} T_Y643_R5EH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SSA_2		:6;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R5FH;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PLAY		:1;
			U08 PAUSE		:1;
			U08 STOP		:1;
			U08 BREAK		:1;
			U08 MCRS		:1;
			U08 dummy1		:2;
			U08 STEP		:1;
		} BIT;
	} T_Y643_R60H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 PC_24		:2;
		} BIT;
	} T_Y643_R61H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_16		:8;
		} BIT;
	} T_Y643_R62H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_8		:8;
		} BIT;
	} T_Y643_R63H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 PC_0		:8;
		} BIT;
	} T_Y643_R64H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LC			:8;
		} BIT;
	} T_Y643_R65H;

	/*--------------------------------------
	 * Macro Command
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IVC			:8;
		} BIT;
	} T_Y643_R66H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVSH		:1;
			U08 dummy1		:1;
			U08 STHE_8		:3;
			U08 STHS_8		:3;
		} BIT;
	} T_Y643_R67H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STHE_0		:8;
		} BIT;
	} T_Y643_R68H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STHS_0		:8;
		} BIT;
	} T_Y643_R69H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVLH		:1;
			U08 dummy1		:1;
			U08 LDHE_8		:3;
			U08 LDHS_8		:3;
		} BIT;
	} T_Y643_R6AH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LDHE_0		:8;
		} BIT;
	} T_Y643_R6BH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LDHS_0		:8;
		} BIT;
	} T_Y643_R6CH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVCV		:1;
			U08 dummy1		:1;
			U08 CKVE_8		:3;
			U08 CKVS_8		:3;
		} BIT;
	} T_Y643_R6DH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CKVE_0		:8;
		} BIT;
	} T_Y643_R6EH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CKVS_0		:8;
		} BIT;
	} T_Y643_R6FH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVSV		:1;
			U08 REVSVV		:1;
			U08 dummy1		:2;
			U08 STVEV_8		:2;
			U08 STVSV_8		:2;
		} BIT;
	} T_Y643_R70H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVEV_0		:8;
		} BIT;
	} T_Y643_R71H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSV_0		:8;
		} BIT;
	} T_Y643_R72H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVSVH		:1;
			U08 dummy1		:1;
			U08 STVEH_8		:3;
			U08 STVSH_8		:3;
		} BIT;
	} T_Y643_R73H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVEH_0		:8;
		} BIT;
	} T_Y643_R74H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 STVSH_0		:8;
		} BIT;
	} T_Y643_R75H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVOE		:1;
			U08 REVOEV		:1;
			U08 dummy1		:2;
			U08 OEEV_8		:2;
			U08 OESV_8		:2;
		} BIT;
	} T_Y643_R76H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OEEV_0		:8;
		} BIT;
	} T_Y643_R77H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESV_0		:8;
		} BIT;
	} T_Y643_R78H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 REVOEH		:1;
			U08 dummy1		:1;
			U08 OEEH_8		:3;
			U08 OESH_8		:3;
		} BIT;
	} T_Y643_R79H;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OEEH_0		:8;
		} BIT;
	} T_Y643_R7AH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 OESH_0		:8;
		} BIT;
	} T_Y643_R7BH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 POLE		:1;
			U08 dummy1		:1;
			U08 POLM		:3;
			U08 POLP_8		:3;
		} BIT;
	} T_Y643_R7CH;

	/*--------------------------------------
	 * Timing Controller
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 POLP_0		:8;
		} BIT;
	} T_Y643_R7DH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R7EH;

	/*--------------------------------------
	 * Scratch Pad Register
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 TMP			:8;
		} BIT;
	} T_Y643_R7FH;

	/*-------------------------------------
	 * Warping Control
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPEN		:1;
			U08 dummy1		:7;
		} BIT;
	} T_Y643_R80H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CAPSY_8		:2;
		} BIT;
	} T_Y643_R81H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSY_0		:8;
		} BIT;
	} T_Y643_R82H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CAPSX_8		:2;
		} BIT;
	} T_Y643_R83H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPSX_0		:8;
		} BIT;
	} T_Y643_R84H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CAPEY_8		:2;
		} BIT;
	} T_Y643_R85H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEY_0		:8;
		} BIT;
	} T_Y643_R86H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 CAPEX_8		:2;
		} BIT;
	} T_Y643_R87H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CAPEX_0		:8;
		} BIT;
	} T_Y643_R88H;

	/*-------------------------------------
	 * Capture Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FBX			:8;
		} BIT;
	} T_Y643_R89H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8AH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8BH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8CH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8DH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8EH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R8FH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R90H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R91H;

	/*-------------------------------------
	 * Capture Buffer Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 LIDL_8		:2;
		} BIT;
	} T_Y643_R92H;

	/*-------------------------------------
	 * Capture Buffer Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 LIDL_0		:8;
		} BIT;
	} T_Y643_R93H;

	/*-------------------------------------
	 * Video Output
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BLF			:1;
			U08 EEF			:1;
			U08 GME			:1;
			U08 CTC			:1;
			U08 dummy1		:4;
		} BIT;
	} T_Y643_R94H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DCON		:8;
		} BIT;
	} T_Y643_R95H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 DBRI_8		:1;
		} BIT;
	} T_Y643_R96H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DBRI_0		:8;
		} BIT;
	} T_Y643_R97H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DSATU		:8;
		} BIT;
	} T_Y643_R98H;

	/*-------------------------------------
	 * Color Tone Controls
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DHUE		:8;
		} BIT;
	} T_Y643_R99H;

	/*-------------------------------------
	 * Edge Enhancement Filter
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EEFC		:8;
		} BIT;
	} T_Y643_R9AH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R9BH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_R9CH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCEN		:1;
			U08 dummy2		:1;
			U08 WCIPMV		:1;
			U08 WCIPMH		:1;
			U08 WTBLSEL		:1;
			U08 WCCOMP		:1;
			U08 dummy1		:2;
		} BIT;
	} T_Y643_R9DH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCSY_8		:2;
		} BIT;
	} T_Y643_R9EH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSY_0		:8;
		} BIT;
	} T_Y643_R9FH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCSX_8		:2;
		} BIT;
	} T_Y643_RA0H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSX_0		:8;
		} BIT;
	} T_Y643_RA1H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCSTPY_8	:2;
		} BIT;
	} T_Y643_RA2H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPY_0	:8;
		} BIT;
	} T_Y643_RA3H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCSTPX_8	:2;
		} BIT;
	} T_Y643_RA4H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCSTPX_0	:8;
		} BIT;
	} T_Y643_RA5H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCNUMY_8	:2;
		} BIT;
	} T_Y643_RA6H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMY_0	:8;
		} BIT;
	} T_Y643_RA7H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WCNUMX_8	:2;
		} BIT;
	} T_Y643_RA8H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WCNUMX_0	:8;
		} BIT;
	} T_Y643_RA9H;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 WPBA_24		:2;
		} BIT;
	} T_Y643_RAAH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_16		:8;
		} BIT;
	} T_Y643_RABH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_8		:8;
		} BIT;
	} T_Y643_RACH;

	/*-------------------------------------
	 * Warping parameters
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 WPBA_0		:8;
		} BIT;
	} T_Y643_RADH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RAEH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RAFH;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SDCOMP		:1;
			U08 dummy1		:5;
			U08 SOCEAD_24	:2;
		} BIT;
	} T_Y643_RB0H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_16	:8;
		} BIT;
	} T_Y643_RB1H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_8	:8;
		} BIT;
	} T_Y643_RB2H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SOCEAD_0	:8;
		} BIT;
	} T_Y643_RB3H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DESTSEL		:1;
			U08 dummy1		:2;
			U08 DESTAD_8	:5;
		} BIT;
	} T_Y643_RB4H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DESTAD_0	:8;
		} BIT;
	} T_Y643_RB5H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 BYTECNT_8	:6;
		} BIT;
	} T_Y643_RB6H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BYTECNT_0	:8;
		} BIT;
	} T_Y643_RB7H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DSTART		:1;
			U08 dummy2		:1;
			U08 DMACLR		:1;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_RB8H;

	/*-------------------------------------
	 * DMAC
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy2		:3;
			U08 DMAEND		:1;
			U08 NODMAERR	:1;
			U08 SOCEERR		:1;
			U08 DESTERR		:1;
			U08 dummy1		:1;
		} BIT;
	} T_Y643_RB9H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RBAH;

	/*-------------------------------------
	 * Warping parameter Correction
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CODR		:3;
			U08 dummy1		:5;
		} BIT;
	} T_Y643_RBBH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC0TY_16	:1;
		} BIT;
	} T_Y643_RBCH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TY_8		:8;
		} BIT;
	} T_Y643_RBDH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TY_0		:8;
		} BIT;
	} T_Y643_RBEH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC0TX_16	:1;
		} BIT;
	} T_Y643_RBFH;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TX_8		:8;
		} BIT;
	} T_Y643_RC0H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC0TX_0		:8;
		} BIT;
	} T_Y643_RC1H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 CA_8		:4;
		} BIT;
	} T_Y643_RC2H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CA_0		:8;
		} BIT;
	} T_Y643_RC3H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 CB_8		:4;
		} BIT;
	} T_Y643_RC4H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CB_0		:8;
		} BIT;
	} T_Y643_RC5H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 CC_8		:4;
		} BIT;
	} T_Y643_RC6H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC_0		:8;
		} BIT;
	} T_Y643_RC7H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:4;
			U08 CD_8		:4;
		} BIT;
	} T_Y643_RC8H;

	/*-------------------------------------
	 * Warping parameter Processing 0
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CD_0		:8;
		} BIT;
	} T_Y643_RC9H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1SELA		:1;
			U08 dummy1		:2;
			U08 CC1SELY		:2;
			U08 CC1SELX		:2;
			U08 CC1SELSH	:1;
		} BIT;
	} T_Y643_RCAH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC1TY_16	:1;
		} BIT;
	} T_Y643_RCBH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TY_8		:8;
		} BIT;
	} T_Y643_RCCH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TY_0		:8;
		} BIT;
	} T_Y643_RCDH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC1TX_16	:1;
		} BIT;
	} T_Y643_RCEH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TX_8		:8;
		} BIT;
	} T_Y643_RCFH;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1TX_0		:8;
		} BIT;
	} T_Y643_RD0H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 CC1KC_8		:6;
		} BIT;
	} T_Y643_RD1H;

	/*-------------------------------------
	 * Warping parameter Processing 1
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC1KC_0		:8;
		} BIT;
	} T_Y643_RD2H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2SELA		:1;
			U08 CC2SELB		:1;
			U08 CC2SELC		:1;
			U08 CC2SELY		:2;
			U08 CC2SELX		:2;
			U08 CC2SELSH	:1;
		} BIT;
	} T_Y643_RD3H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC2TY_16	:1;
		} BIT;
	} T_Y643_RD4H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TY_8		:8;
		} BIT;
	} T_Y643_RD5H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TY_0		:8;
		} BIT;
	} T_Y643_RD6H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC2TX_16	:1;
		} BIT;
	} T_Y643_RD7H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TX_8		:8;
		} BIT;
	} T_Y643_RD8H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2TX_0		:8;
		} BIT;
	} T_Y643_RD9H;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 CC2KC_8		:6;
		} BIT;
	} T_Y643_RDAH;

	/*-------------------------------------
	 * Warping parameter Processing 2
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC2KC_0		:8;
		} BIT;
	} T_Y643_RDBH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC3TY_16	:1;
		} BIT;
	} T_Y643_RDCH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TY_8		:8;
		} BIT;
	} T_Y643_RDDH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TY_0		:8;
		} BIT;
	} T_Y643_RDEH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 CC3TX_16	:1;
		} BIT;
	} T_Y643_RDFH;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TX_8		:8;
		} BIT;
	} T_Y643_RE0H;

	/*-------------------------------------
	 * Warping parameter Processing 3
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 CC3TX_0		:8;
		} BIT;
	} T_Y643_RE1H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RE2H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RE3H;

	/*-------------------------------------
	 * External Sync
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SYNCE		:1;
			U08 dummy1		:1;
			U08 VSADJ		:6;
		} BIT;
	} T_Y643_RE4H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EXGAM		:2;
			U08 DVPAL		:1;
			U08 DVSP		:1;
			U08 CRCBS		:1;
			U08 DVINTL		:1;
			U08 DVIF		:2;
		} BIT;
	} T_Y643_RE5H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVCSE		:1;
			U08 DVCSM		:1;
			U08 dummy1		:3;
			U08 DVHTL_8		:3;
		} BIT;
	} T_Y643_RE6H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHTL_0		:8;
		} BIT;
	} T_Y643_RE7H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:7;
			U08 DVHSW_8		:1;
		} BIT;
	} T_Y643_RE8H;

	/*-------------------------------------
	 * External Video Input
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 DVHSW_0		:8;
		} BIT;
	} T_Y643_RE9H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPD			:1;
			U08 dummy2		:1;
			U08 SCE			:1;
			U08 SFSEL		:1;
			U08 BPFLX		:1;
			U08 BPHLFE		:1;
			U08 dummy1		:1;
			U08 BPAOFF		:1;
		} BIT;
	} T_Y643_REAH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:3;
			U08 EISD_8		:5;
		} BIT;
	} T_Y643_REBH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EISD_0		:8;
		} BIT;
	} T_Y643_RECH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 BPSL_8		:2;
		} BIT;
	} T_Y643_REDH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSL_0		:8;
		} BIT;
	} T_Y643_REEH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 BPSD_8		:3;
		} BIT;
	} T_Y643_REFH;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPSD_0		:8;
		} BIT;
	} T_Y643_RF0H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:6;
			U08 BPEL_8		:2;
		} BIT;
	} T_Y643_RF1H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPEL_0		:8;
		} BIT;
	} T_Y643_RF2H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 BPED_8		:3;
		} BIT;
	} T_Y643_RF3H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 BPED_0		:8;
		} BIT;
	} T_Y643_RF4H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 SCFY_8		:6;
		} BIT;
	} T_Y643_RF5H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFY_0		:8;
		} BIT;
	} T_Y643_RF6H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:2;
			U08 SCFX_8		:6;
		} BIT;
	} T_Y643_RF7H;

	/*-------------------------------------
	 * Backdrop Plane
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 SCFX_0		:8;
		} BIT;
	} T_Y643_RF8H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RF9H;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
	} T_Y643_RFAH;

	/*-------------------------------------
	 * Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:5;
			U08 EXADJHTL_8	:3;
		} BIT;
	} T_Y643_RFBH;

	/*-------------------------------------
	 * Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 EXADJHTL_0	:8;
		} BIT;
	} T_Y643_RFCH;

	/*-------------------------------------
	 * Interrupt Enable
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 IENDS		:1;
			U08 IEDV		:1;
			U08 IENME		:1;
			U08 dummy1		:3;
			U08 IEDEN		:1;
			U08 IEDER		:1;
		} BIT;
	} T_Y643_RFDH;

	/*-------------------------------------
	 * Flag and Status
	 * 
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 FNDS		:1;
			U08 DFV			:1;
			U08 FNME		:1;
			U08 dummy1		:2;
			U08 NDS			:1;
			U08 DVB			:1;
			U08 DEVEN		:1;
		} BIT;
	} T_Y643_RFEH;

	/*--------------------------------------
	 *	Reserved
	 *-------------------------------------*/
	typedef union {
		U08		BYTE;
		struct {
			U08 dummy1		:8;
		} BIT;
	} T_Y643_RFFH;

	/********************************************************************************/
	/*	VC1W GENERAL TABLE ACCESS STRUCT                                            */
	/********************************************************************************/
	/*----------------------------------------
	 *	Layer structure for Sprite display
	 *----------------------------------------*/
	typedef union {
#if YVC1_EXLY
		U08 BYTE[13];
#else
		U08 BYTE[12];
#endif
		struct {
			U08 CLM		:3;				/* BYTE[0]						*/
			U08 MX8		:1;
			U08 MXSL	:1;
			U08 DOY_8	:3;
			U08 DOY_0	:8;				/* BYTE[1]						*/
			U08 MX_4	:4;				/* BYTE[2]						*/
			U08 PLTI_8	:1;
			U08 DOX_8	:3;
			U08 DOX_0	:8;				/* BYTE[3]						*/
			U08 ALPHA_1	:1;				/* BYTE[4]						*/
			U08 TP		:1;
			U08 SZY		:6;
			U08 COMPM	:2;				/* BYTE[5]						*/
			U08 SZX		:6;
			union {						/* BYTE[6]						*/
				struct {
					U08 dummy1	:1;
					U08 LSYS	:1;
					U08 DCM		:2;
					U08 LSYE	:1;
					U08 LSYM	:3;
				} RGB;
				struct {
					U08 PLTI_0	:8;
				} PLT;
			} COLOR;
			U08 ALPHA_0	:1;				/* BYTE[7]						*/
			U08 PN_19	:7;
			U08 PN_11	:8;				/* BYTE[8]						*/
			U08 PN_3	:8;				/* BYTE[9]						*/
			U08 MAGY	:8;				/* BYTE[10]						*/
			U08 MAGX	:8;				/* BYTE[11]						*/
#if YVC1_EXLY
			struct {					/* BYTE[12]						*/
				U08 dummy1	:4;
				U08 MX_0	:4;
			} EXLY;
#endif
		} BIT;
	} T_Y643_LYR_SPRTATTR;

	/*----------------------------------------
	 *	Layer structure for Text display
	 *----------------------------------------*/
	typedef union {
#if YVC1_EXLY
		U08 BYTE[13];
#else
		U08 BYTE[12];
#endif
		struct {
			U08 LYSEL	:3;				/* BYTE[0]						*/
			U08 FMX8	:1;
			U08 FMXSL	:1;
			U08 FDOY_8	:3;
			U08 FDOY_0	:8;				/* BYTE[1]						*/
			U08 FMX_4	:4;				/* BYTE[2]						*/
			U08 FPLTI_8	:1;
			U08 FDOX_8	:3;
			U08 FDOX_0	:8;				/* BYTE[3]						*/
			U08 dummy1	:1;				/* BYTE[4]						*/
			U08 FTP		:1;
			U08 FPLTS	:1;
			U08 FPFE	:1;
			U08 FDIR	:1;
			U08 FKNE	:1;
			U08 FTYP_10	:2;
			U08 FTYP_2	:8;				/* BYTE[5]						*/
			U08 FPLTI_0	:8;				/* BYTE[6]						*/
			U08 dummy2	:1;				/* BYTE[7]						*/
			U08 FCSA_9	:3;
			U08 dummy3	:1;
			U08 FCEA_9	:3;
			U08 FCSA_1	:8;				/* BYTE[8]						*/
			U08 FCEA_1	:8;				/* BYTE[9]						*/
			U08 dummy4	:8;				/* BYTE[10]						*/
			U08 dummy5	:8;				/* BYTE[11]						*/
#if YVC1_EXLY
			struct {					/* BYTE[12]						*/
				U08	dummy1	:4;
				U08 FMX_0	:4;
			} EXLY;
#endif
		} BIT;
	} T_Y643_LYR_FONTATTR;

	/*----------------------------------------
	 *	Font type attribute structure
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[8];
		struct {
			U08 CMODE		:1;			/* BYTE[0]						*/
			U08 FN_19		:7;
			U08 FN_11		:8;			/* BYTE[1]						*/
			U08 FN_3		:8;			/* BYTE[2]						*/
			U08 FHSZ		:1;			/* BYTE[3]						*/
			U08 FCOMPE		:1;
			U08 FCOMPV		:1;
			U08 dummy1		:5;
			U08 dummy2		:2;			/* BYTE[4]						*/
			U08 FSZX		:6;
			U08 dummy3		:2;			/* BYTE[5]						*/
			U08 FSZY		:6;
			U08 dummy4		:3;			/* BYTE[6]						*/
			U08 FDAB		:5;
			U08 dummy5		:3;			/* BYTE[7]						*/
			U08 FDLB		:5;
		} BIT;
	} T_Y643_FONT_TYPATTR;

	/*----------------------------------------
	 *	Proportional character code structure 
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[4];
		struct {
			union {
				struct {
					U08 FCODE_8		:8;			/* BYTE[0]				*/
					U08 FCODE_0		:8;			/* BYTE[1]				*/
					U08 dummy1		:2;			/* BYTE[2]				*/
					U08 XS			:6;
					U08 dummy2		:2;			/* BYTE[3]				*/
					U08 XE			:6;
				} DOUBLE;
				struct {
					U08 dummy1		:1;			/* BYTE[0]				*/
					U08 FCODE_8		:7;
					U08 FCODE_0		:8;			/* BYTE[1]				*/
					U08 dummy2		:2;			/* BYTE[2]				*/
					U08 XS			:6;
					U08 dummy3		:2;			/* BYTE[3]				*/
					U08 XE			:6;
				} MIX;
			} GLYPH;
		} BIT;
	} T_Y643_TEXT_PFONT;
	
	/*----------------------------------------
	 * Layer structure for Clipping
	 *----------------------------------------*/
	typedef union {
		U08 BYTE[12];
		struct {
			U08 LYSEL   :3;            /* BYTE[0]                      */
			U08 dummy1  :5;
			U08 dummy2  :6;            /* BYTE[1]                      */
			U08 CLPE    :2;            
			U08 dummy3  :8;            /* BYTE[2]                      */
			U08 dummy4  :8;            /* BYTE[3]                      */
			U08 dummy5  :8;            /* BYTE[4]                      */
			U08 dummy6  :8;            /* BYTE[5]                      */
			U08 dummy7  :1;            /* BYTE[6]                      */
			U08 CLPSY_8 :3;            
			U08 dummy8  :1;
			U08 CLPSX_8 :3;            
			U08 CLPSY_0 :8;            /* BYTE[7]                      */
			U08 CLPSX_0 :8;            /* BYTE[8]                      */
			U08 dummy9  :1;            /* BYTE[9]                      */
			U08 CLPEY_8 :3;            
			U08 dummy10 :1;
			U08 CLPEX_8 :3;            
			U08 CLPEY_0 :8;            /* BYTE[10]                      */
			U08 CLPEX_0 :8;            /* BYTE[11]                      */
		} BIT;
	} T_Y643_LYR_CLIPATTR;
	
#endif	/* YVC_BIT_FIELD	*/

#endif /* _def_YVC643_H_	*/
/*------------------------------ E O F -----------------------------------------*/

