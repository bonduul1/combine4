/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright(c) 2011-2015 Yamaha Corporation. All rights reserved.             */
/*  Module          : $Workfile: Yvc1Ddrv.h $                                   */
/*  Description     : XFL Driver include header                                 */
/*  Version         : $Rev: 55 $                                                */
/*  Last UpDate Time: $Date:: 2015-02-27 13:15:26#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/*                  : H.Nakahara                                                */
/********************************************************************************/
#ifndef _def_YVC1_DDRV_H_
#define _def_YVC1_DDRV_H_

/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc643.h"
#include "Yvc1_HAL.h"
/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
/*----------------------------------*/
/*  Version                         */
/*----------------------------------*/
/*--- Driver Version	---*/
#define YVC1_DRIVER_VERSION             ("0.8.0.1")

#define YVC1_LYBSELC_MASK               (0x40U)
#define YVC1_LYBSELC_TBL1               (0)
#define YVC1_PLLR_MASK                  (0xF8U)
#define YVC1_PLLF_MASK                  (0x7FU)

/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/
/*----------------------------------------
 *	Device initialization
 *----------------------------------------*/
typedef struct {
	U08 Clk[3];							/* Clock data							*/
	U08 Disp[23];						/* Display scan data					*/
	U08 VideoOut;						/* Video Output Control					*/
	U08 TCON[23];						/* Timing controller data				*/
} T_YVC1_DATA;

/*----------------------------------------
 *	DMA Initialization
 *----------------------------------------*/
typedef struct {
	U08 SDCOMP;							/* Source-data compression selection	*/
	U08 DESTSEL;						/* Destination selection				*/
	U32 SOCEAD;							/* Source address						*/
	U16 DESTAD;							/* Destination address					*/
	U16 BYTECNT;						/* Transmission number of bytes			*/
} T_YVC1_DMA_INIT;


/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
BOOL YVC1_PmemSet(void);
BOOL YVC1_WriteReg(U08 RegNo, U08 Data);
BOOL YVC1_ReadReg(U08 RegNo, U08 *Data);
BOOL YVC1_WriteRegs(U08 RegNo, const U08 *Buff, U08 Num);
BOOL YVC1_ReadTbls(U16 Addr, U08 *Buff, U16 Num);
BOOL YVC1_WriteTbls(U16 Addr, const U08 *Buff, U16 Num);
BOOL YVC1_ReadPlts(U16 Addr, U08 *Buff, U16 Num);
BOOL YVC1_WritePlts(U16 Addr, const U08 *Buff, U16 Num);
BOOL YVC1_ReadGamTbls(U16 Addr, U08 *Buff, U16 Num);
BOOL YVC1_WriteGamTbls(U16 Addr, const U08 *Buff, U16 Num);
BOOL YVC1_ReadWrpTbls(U16 Addr, U08 *Buff, U16 Num);
BOOL YVC1_WriteWrpTbls(U16 Addr, const U08 *Buff, U16 Num);
BOOL YVC1_DmaInit(const T_YVC1_DMA_INIT *tYvc1DmaInit);
BOOL YVC1_DmaCtrl(BOOL Enable);
BOOL YVC1_Init(const T_YVC1_DATA *tYvc1Data);
BOOL YVC1_GetVer(U08 *DevVer, C08 *DrvVer);
BOOL YVC1_FlipTbl(void);
BOOL YVC1_VBWait(U32 Count);

BOOL YVC1_GetChecksum(U32 startAddr, U32 endAddr , U16 *Checksum);
BOOL Vsync_Filp_Check(void);
U16 YVC1_ReadFlag(void );
#endif /* _def_YVC1_DDRV_H_ */

/*------------------------------ E O F -----------------------------------------*/

