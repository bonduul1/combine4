/********************************************************************************
 *
 *		CONFIDENTIAL
 *
 *		Copyright (C) 2014-2015 Yamaha Corporation. All rights reserved.
 *
 *		Module			: $Workfile: YVC1Vin.h $
 *
 *		Description		: VC1W External Image Input function include header
 *
 *		Version			: $Rev: 37 $
 *
 *		Last UpDate Time: $Date:: 2015-01-07 17:44:06#$
 *
 *		FOOT NOTE		: adjust 4tab
 *
 *		AUTHOR			: H.Katayama
 *
 ********************************************************************************/

#ifndef YVC1_VIN_H
#define YVC1_VIN_H

/*------------------------------------------------------------------------------
 *						I N C L U D E
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						D E F I N E
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *						S T R U C T
 *------------------------------------------------------------------------------*/
/*----------------------------------------
 *	Digital video input
 *	Digital video signal setup
 *----------------------------------------*/
typedef struct {
	U08 DVIF;						/* Digital video input format			*/
	U08 DVINTL;						/* Digital video input scan mode		*/
	U08 DVPAL;						/* Digital video PAL selection			*/
	U08 DVSP;						/* Synchronized signal polarity selection */
	U08 CRCBS;						/* Color-difference-signal swap			*/
} T_YVC1_VIN_DIGITAL_IN;

/*----------------------------------------
 *	Digital video input
 *	Composite synchronizing signal setup
 *----------------------------------------*/
typedef struct {
	U08 DVCSE;				/* Composite synchronizing signal permission	*/
	U08 DVCSM;				/* Synchronizing separation mode				*/
	U16 DVHTL;				/* Horizontal cycle								*/
	U16 DVHSW;				/* Horizontal synchronizing pulse width 		*/
} T_YVC1_VIN_DIGITAL_SYNC;

/*----------------------------------------
 *	Backdrop plane
 *	Viewing-area specification
 *----------------------------------------*/
typedef struct {
	U16 Sx;							/* Backdrop plane start dot				*/
	U16 Sy;							/* Backdrop plane side start line		*/
	U16 Width;						/* Backdrop plane display width			*/
	U16 Height;						/* Backdrop plane display height		*/
} T_YVC1_VIN_BCD_AREA;

/*----------------------------------------
 *	Backdrop plane
 *	Scaling setup
 *----------------------------------------*/
typedef struct {
	BOOL Enable;					/* Scaling function ON/OFF				*/
	U08 Mode;						/* The interpolation method selection	*/
	F32 Fx;							/* The rate of X scaling 				*/
	F32 Fy;							/* The rate of Y scaling 				*/
} T_YVC1_VIN_BCD_SCALE;


/*------------------------------------------------------------------------------
 *						P R O T O T Y P E
 *------------------------------------------------------------------------------*/
BOOL YVC1_VinGamma(U08 GAM);
BOOL YVC1_VinDigitalIn(const T_YVC1_VIN_DIGITAL_IN *tYvc1VinDigitalIn);
BOOL YVC1_VinDigitalSync(const T_YVC1_VIN_DIGITAL_SYNC *tYvc1VinDigitalSync);
BOOL YVC1_VinBcdExtSync(BOOL Enable);
BOOL YVC1_VinBcdDisp(BOOL Disp, BOOL AutoOff);
BOOL YVC1_VinBcdXPos(F32 StaDot);
BOOL YVC1_VinBcdArea(const T_YVC1_VIN_BCD_AREA *tYvc1VinBcdArea);
BOOL YVC1_VinBcdScale(const T_YVC1_VIN_BCD_SCALE *tYvc1VinBcdScale);
BOOL YVC1_VinBcdXFlip(BOOL Enable);
BOOL YVC1_VinBcdVSA(S08 VsAdj);
BOOL YVC1_VinBcdHLFE(BOOL Enable);

#endif /* YVC1_VIN_H		*/


/*------------------------------- E O F ----------------------------------------*/

