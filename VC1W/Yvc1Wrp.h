/********************************************************************************
 *
 *		CONFIDENTIAL
 *
 *		Copyright (C) 2014-2015 Yamaha Corporation. All rights reserved.
 *
 *		Module			: $Workfile: Yvc1Wrp.h $
 *
 *		Description		: VC1W Device Driver Warping control include header
 *
 *		Version			: $Rev: 40 $
 *
 *		Last UpDate Time: $Date:: 2015-01-08 18:57:09#$
 *
 *		FOOT NOTE		: adjust 4tab
 *
 *		AUTHOR			: H.Katayama
 *
 ********************************************************************************/

#ifndef YVC1_WRP_H
#define YVC1_WRP_H

/*------------------------------------------------------------------------------
 *						I N C L U D E
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						D E F I N E
 *------------------------------------------------------------------------------*/
#define CAPTURE_BUFF_SIZE_DOT			(36864)
/*=== Capture Buff X Size = Capture X Size(8n) =================================*/
/*    input a          = Capture Width											*/
#define CPTR_BUFFSIZE_X_CALC(a)			((U16)(((S16)(((S16)a - 1) / 8) + 1) * 8))
/*=== LineDelay = Cptr Buff Y Size / 2 =========================================*/
/*    Cptr Buff Y Size = 36864 / (Cptr Buff X Size / 4)							*/
/*    input a          = Capture Width											*/
#define CPTR_LINE_DELAY_CALC(a)			((U16)((CAPTURE_BUFF_SIZE_DOT / (CPTR_BUFFSIZE_X_CALC(a) / 4)) / 2))

/*------------------------------------------------------------------------------
 *						S T R U C T
 *------------------------------------------------------------------------------*/
/*----------------------------------------
 *	Capture Buffer Controls setup
 *----------------------------------------*/
typedef struct {
	U16 Sx;						/* Capture area start X coordinates			*/
	U16 Sy;						/* Capture area start Y coordinates			*/
	U16 Width;					/* Capture area width coordinates			*/
	U16 Height;					/* Capture area height coordinates			*/
	U16 BuffSizeX;				/* Capture Buffer X size					*/
	U16 LineDelay;				/* Capture lead delay timing (line)			*/
} T_YVC1_WRP_CPTR;

/*----------------------------------------
 *	Hardware Image Warping setup
 *----------------------------------------*/
typedef struct {
	U16 WCSX;					/* Warping parameter start X coordinate				*/
	U16 WCSY;					/* Warping parameter start Y coordinate				*/
	U16 WCSTPX;					/* Warping parameter X direction intervals			*/
	U16 WCSTPY;					/* Warping parameter Y direction intervals			*/
	U16 WCNUMX;					/* Number of X-direction warping parameters			*/
	U16 WCNUMY;					/* Number of Y-direction warping parameters			*/
	U08 WCIPMH;					/* X-direction interpolation mode					*/
	U08 WCIPMV;					/* Y-direction interpolation mode 					*/
	U08 WTBLSEL;				/* Space to store warping parameters to use			*/
	U32 WPBA;					/* First address of warping parameters stored		*/
	U08 WCCOMP;					/* Compresses or Uncompresses warping parameters	*/
} T_YVC1_WRP_COF_INIT;

/*----------------------------------------
 *	Warping Parameters setup
 *----------------------------------------*/
typedef struct {
	U08 CODR;					/* Correction calculation circuit -  Computation sequence selection				*/
	U32 CC0TY;					/* Coefficient calculation circuit 0 - Coefficient Ty							*/
	U32 CC0TX;					/* Coefficient calculation circuit 0 - Coefficient Tx							*/
	U16 CA;						/* Coefficient calculation circuit 0 - Coefficient Ca							*/
	U16 CB;						/* Coefficient calculation circuit 0 - Coefficient Cb							*/
	U16 CC;						/* Coefficient calculation circuit 0 - Coefficient Cc							*/
	U16 CD;						/* Coefficient calculation circuit 0 - Coefficient Cd							*/
	U08 CC1SELA;				/* Coefficient calculation circuit 1 - Multiplier input A selection				*/
	U08 CC1SELY;				/* Coefficient calculation circuit 1 - Y output selection						*/
	U08 CC1SELX;				/* Coefficient calculation circuit 1 - X output selection						*/
	U08 CC1SELSH;				/* Coefficient calculation circuit 1 - Output multiplier shift count selection	*/
	U32 CC1TY;					/* Coefficient calculation circuit 1 - Coefficient Ty							*/
	U32 CC1TX;					/* Coefficient calculation circuit 1 - Coefficient Tx							*/
	U16 CC1KC;					/* Coefficient calculation circuit 1 - Coefficient Kc							*/
	U08 CC2SELA;				/* Coefficient calculation circuit 2 - Multiplier input A selection				*/
	U08 CC2SELB;				/* Coefficient calculation circuit 2 - Multiplier input B selection				*/
	U08 CC2SELC;				/* Coefficient calculation circuit 2 - Adding machine input selection			*/
	U08 CC2SELY;				/* Coefficient calculation circuit 2 - Y output selection						*/
	U08 CC2SELX;				/* Coefficient calculation circuit 2 - X output selection						*/
	U08 CC2SELSH;				/* Coefficient calculation circuit 2 - Output multiplier shift count selection	*/
	U32 CC2TY;					/* Coefficient calculation circuit 2 - Coefficient Ty							*/
	U32 CC2TX;					/* Coefficient calculation circuit 2 - Coefficient Tx							*/
	U16 CC2KC;					/* Coefficient calculation circuit 2 - Coefficient Kc							*/
	U32 CC3TY;					/* Coefficient calculation circuit 3 - Coefficient Ty							*/
	U32 CC3TX;					/* Coefficient calculation circuit 3 - Coefficient Tx							*/
} T_YVC1_WRP_COF_CMPS;


/*------------------------------------------------------------------------------
 *						P R O T O T Y P E
 *------------------------------------------------------------------------------*/
BOOL YVC1_WrpEnable(BOOL Enable);
BOOL YVC1_WrpCptr(const T_YVC1_WRP_CPTR *tYvc1WrpCptr);
BOOL YVC1_WrpDispSettings(U08 BLF, U08 EEF, U08 GME, U08 CTC);
BOOL YVC1_WrpEdgFlt(U08 EEFC);
BOOL YVC1_WrpCont(U08 Cont);
BOOL YVC1_WrpBright(S16 Bright);
BOOL YVC1_WrpSat(U08 Sat);
BOOL YVC1_WrpHue(S08 Hue);
BOOL YVC1_WrpCofEnable(BOOL Enable);
BOOL YVC1_WrpCofInit(const T_YVC1_WRP_COF_INIT *tYvc1WrpCofInit);
BOOL YVC1_WrpCofCmps(const T_YVC1_WRP_COF_CMPS *tYvc1WrpCofCmps);

#endif /* YVC1_WRP_H	*/


/*------------------------------ E O F -----------------------------------------*/

