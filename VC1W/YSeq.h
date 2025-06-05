/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright (C) 2011-2015 Yamaha Corporation. All rights reserved.            */
/*  Module          : $Workfile: YSeq.h $                                       */
/*  Description     : Sequence control Function include header                  */
/*  Version         : $Rev: 39 $                                               */
/*  Last UpDate Time: $Date:: 2015-01-08 15:38:06#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/********************************************************************************/

#ifndef _YSEQ_H_
#define _YSEQ_H_

/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "Yvc1Ddrv.h"
#include "YSeqConfig.h"

#ifdef YSEQ_SOUND_ENABLE
#ifdef YSEQ_SOUND_SD1
	#define _Y825TYPES_H_
	#include "y825devdrv.h"
#endif /* YSEQ_SOUND_SD1 */
#ifdef YSEQ_SOUND_APL5
	#define _APL5TYPES_H_
	#include "apl5drv.h"
#endif /*YSEQ_SOUND_APL5 */
#endif /* YSEQ_SOUND_ENABLE */


/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
#define YSEQ_BUTTONID_INVALID       (-1)


/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      E X T E R N                                             */
/*------------------------------------------------------------------------------*/
#ifdef YSEQ_SOUND_ENABLE
#ifdef YSEQ_SOUND_SD1
	extern const T_SD1_SOUND_INFO tSd1SoundInfo[];
#endif /* YSEQ_SOUND_SD1 */
#ifdef YSEQ_SOUND_APL5
	/* Unmounted. */
#endif /*YSEQ_SOUND_APL5 */
#endif /* YSEQ_SOUND_ENABLE */


/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/
BOOL YSEQ_Init(const T_YSEQ_DATA *tYSeqData);
BOOL YSEQ_Sequencer(S16 X, S16 Y, S16 Event, S16 *NextYSeqID, S16 *ButtonId);
BOOL YSEQ_SetIndex(S16 YSeqID);

#ifdef YSEQ_SOUND_ENABLE
	BOOL YSEQ_SoundInit(void);
	BOOL YSEQ_SetSound(U16 SoundID);
	BOOL YSEQ_SoundIrqReq(void);
#endif /* YSEQ_SOUND_ENABLE */

#endif /* _YSEQ_H_	*/

/*------------------------------ E O F -----------------------------------------*/

