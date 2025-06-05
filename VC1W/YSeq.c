/********************************************************************************/
/*  CONFIDENTIAL                                                                */
/*  Copyright (C) 2011-2015 Yamaha Corporation. All rights reserved.            */
/*  Module          : $Workfile: YSeq.c $                                       */
/*  Description     : Sequence control Function                                 */
/*  Version         : $Rev: 39 $                                               */
/*  Last UpDate Time: $Date:: 2015-01-08 15:38:06#$                             */
/*  FOOT NOTE       : adjust 4tab                                               */
/*  AUTHOR          : H.Katayama                                                */
/********************************************************************************/


/*------------------------------------------------------------------------------*/
/*                      I N C L U D E                                           */
/*------------------------------------------------------------------------------*/
#include "YSeq.h"

/*------------------------------------------------------------------------------*/
/*                      D E F I N E                                             */
/*------------------------------------------------------------------------------*/
#define YSEQ_INI_YSEQ_ID		(0)
#define YSEQ_MCR_PLAY			(0x80)
#define YSEQ_MCR_PAUSE			(0x40)
#define YSEQ_MCR_STOP			(0x20)
#define YSEQ_MCR_BREAK			(0x10)
#define YSEQ_MCR_MCRS			(0x08)
#define YSEQ_STATUS_FMC			(0x80)

#define YSEQ_ACTION_UNLOCK		(0)
#define YSEQ_ACTION_LOCK		(1)


/*------------------------------------------------------------------------------*/
/*                      S T R U C T                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      V A R I A B L E S                                       */
/*------------------------------------------------------------------------------*/
static const T_YSEQ_DATA *g_pYSeqData       = (const T_YSEQ_DATA*)NULL;
static const T_YSEQ_INFO *g_pYSeqInfo       = (const T_YSEQ_INFO*)NULL;
static const T_YSEQ_BUTTON *g_ptYSeqButton0 = (const T_YSEQ_BUTTON*)NULL;
static S16 g_s16YSeqID       = 0;
static U08 g_u08ButtonAction = YSEQ_ACTION_NONE;

/*------------------------------------------------------------------------------*/
/*                      E X T E R N                                             */
/*------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------*/
/*                      P R O T O T Y P E                                       */
/*------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_Init                                                 */
/* Contents         : Sequence initialization                                   */
/* Argument         : T_YSEQ_DATA *tYSeqData    (W) Sequence Data               */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YSEQ_Init(const T_YSEQ_DATA *tYSeqData)
{
	BOOL bRet    = TRUE;
	U16 u16Cnt   = 0;
	BOOL bGetFlg = FALSE;
	const T_YSEQ_INFO *ptYSeqInfo = (const T_YSEQ_INFO*)NULL;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if (tYSeqData == (const T_YSEQ_DATA*)NULL)
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		g_pYSeqData = tYSeqData;
		g_pYSeqInfo = (const T_YSEQ_INFO*)NULL;

		/*--- Get tYSeqButton[0] pointer --*/
		ptYSeqInfo  = (const T_YSEQ_INFO*)(tYSeqData->tYSeqInfo);
		for(u16Cnt=0;(u16Cnt<(tYSeqData->YSeqInfoNum)) && (bGetFlg==FALSE);u16Cnt++)
		{
			if(ptYSeqInfo[u16Cnt].YSeqButtonNum != 0)
			{
				g_ptYSeqButton0 = ptYSeqInfo[u16Cnt].tYSeqButton;
				bGetFlg=TRUE;
			}
		}

		/*--- Sound Set ---*/
#ifdef YSEQ_SOUND_ENABLE
		bRet = YSEQ_SoundInit();
#endif /* YSEQ_SOUND_ENABLE	*/
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_Sequencer                                            */
/* Contents         : Sequence Data ID acquisition                              */
/* Argument         : S16   X                   (W)   X Position                */
/*                    S16   Y                   (W)   Y Position                */
/*                    S16   Event               (W)   Event                     */
/*                    S16   *NextYSeqID         (R)   Next Sequence ID          */
/*                    S16   *ButtonId           (R/W) Button ID                 */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YSEQ_Sequencer(S16 X, S16 Y, S16 Event, S16 *NextYSeqID, S16 *ButtonId)
{
	const T_YSEQ_BUTTON *ptYSeqButton = (const T_YSEQ_BUTTON*)NULL;
	BOOL bRet                         = TRUE;
	BOOL bExe                         = FALSE;
	BOOL bDataSetFlg                  = FALSE;
	U08  u08YSeqButtonNum             = 0;
	U16  u16YSeqButtonId              = 0;
	S16  s16YSeqButtonEvent           = 0;
	S16  s16CompButtonId              = 0;
	U08  u08Action                    = YSEQ_ACTION_NONE;
	U08  u08ButtonAction              = g_u08ButtonAction;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if ( (NextYSeqID == (S16*)NULL) ||
		 (g_pYSeqData == (const T_YSEQ_DATA*)NULL) ||
		 (ButtonId == (S16*)NULL) )
	{
		bRet = FALSE;
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- Initialize Data Setting ---*/
		s16CompButtonId = *ButtonId;
		*NextYSeqID     = YSEQ_INVALID;
		if (Event == YSEQ_INVALID)			/* case of Event=YSEQ_INVALID	*/
		{
			*ButtonId = YSEQ_BUTTONID_INVALID;
		}

		/*--- Data Setting ---*/
		if (g_pYSeqInfo == (const T_YSEQ_INFO*)NULL)
		{
			/*--- 1st Action ---*/
			*NextYSeqID = YSEQ_INI_YSEQ_ID;
		}
		else
		{
			/*--- Button Area Check ---*/
			if (g_pYSeqInfo->YSeqButtonNum != 0)
			{
				/* It updates, only when there is valid data. */
				ptYSeqButton     = g_pYSeqInfo->tYSeqButton;
				u08YSeqButtonNum = g_pYSeqInfo->YSeqButtonNum;
				for (u16YSeqButtonId=0; (u16YSeqButtonId<u08YSeqButtonNum) && (bExe==FALSE); u16YSeqButtonId++)
				{
					if ( (ptYSeqButton[u16YSeqButtonId].X1 <= X) &&
						 (ptYSeqButton[u16YSeqButtonId].X2 >= X) &&
						 (ptYSeqButton[u16YSeqButtonId].Y1 <= Y) &&
						 (ptYSeqButton[u16YSeqButtonId].Y2 >= Y) )
					{
						/*--- ButtonId Set ---*/
						*ButtonId = (S16)((ptYSeqButton - g_ptYSeqButton0) + u16YSeqButtonId);

						s16YSeqButtonEvent = (S16)ptYSeqButton[u16YSeqButtonId].Event;
						switch(Event)
						{
						case YSEQ_EVENT_CLICK:
						case YSEQ_EVENT_MOUSEUP:
							if ( (s16YSeqButtonEvent == YSEQ_EVENT_CLICK  ) ||
								 (s16YSeqButtonEvent == YSEQ_EVENT_MOUSEUP) )
							{
								if (s16YSeqButtonEvent == YSEQ_EVENT_CLICK)
								{
									if (s16CompButtonId == *ButtonId)
									{
										bDataSetFlg = TRUE;
									}
								}
								else
								{
									bDataSetFlg = TRUE;
								}
							}
							break;
						case YSEQ_EVENT_MOUSEDOWN:
						case YSEQ_EVENT_UNKNOWN:
							if (s16YSeqButtonEvent == Event)
							{
								bDataSetFlg = TRUE;
							}
							break;
						case YSEQ_INVALID:
							bDataSetFlg = TRUE;
							break;
						default:
							bRet = FALSE;
							break;
						}

						if ( (bDataSetFlg == TRUE) && (bRet == TRUE) )
						{
							/*--- Next Data Set ---*/
							*NextYSeqID = ptYSeqButton[u16YSeqButtonId].NextSeqID;

							/*--- Action Backup ---*/
							g_u08ButtonAction = ptYSeqButton[u16YSeqButtonId].Action;

							/*--- Sound Set ---*/
							#ifdef YSEQ_SOUND_ENABLE
							bRet = YSEQ_SetSound(ptYSeqButton[u16YSeqButtonId].SoundID);
							#endif /* YSEQ_SOUND_ENABLE	*/

							bExe = TRUE;
						}
					}
				}
			}

			/*--- Processing in NextYSeqID = YSEQ_INVALID & Macro Command END ---*/
			if ( (*NextYSeqID == YSEQ_INVALID) && 
				 ((YVC_READ_PORT(YVC1_PORT_FLG_STATUS2) & YSEQ_STATUS_FMC) != 0) )
			{
				/*--- Next Sequence Id calculation ---*/
				u08Action = g_pYSeqInfo->Action;
				if (u08Action == YSEQ_ACTION_NONE)
				{
					u08Action = u08ButtonAction;
				}
				else
				{
					g_u08ButtonAction = YSEQ_ACTION_NONE;
				}

				switch(u08Action)
				{
				case YSEQ_ACTION_STOP:
					break;
				default:							/* YSEQ_ACTION_NONE */
													/* YSEQ_ACTION_PLAY */
					*NextYSeqID = g_pYSeqInfo->NextSeqID;
					if (*NextYSeqID == YSEQ_INVALID)
					{
						*NextYSeqID = g_s16YSeqID + 1;
						if (*NextYSeqID >= g_pYSeqData->YSeqInfoNum)
						{
							*NextYSeqID = YSEQ_INI_YSEQ_ID;
						}
					}
					break;
				}
			}
		}

		/*--- Sound Set ---*/
		#ifdef YSEQ_SOUND_ENABLE
		if (bRet == TRUE)
		{
			bRet = YSEQ_SoundIrqReq();
		}
		#endif /* YSEQ_SOUND_ENABLE	*/
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_SetIndex                                             */
/* Contents         : Sequence Data setup                                       */
/* Argument         : S16   YSeqID              (W) YSeq ID                     */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
BOOL YSEQ_SetIndex(S16 YSeqID)
{
	BOOL bRet         = TRUE;
	U32  u32SceneAddr = 0;
	U32  u32Count     = 0;
	U08  u08SSA[4]    = {0};
	U08  u08RegR60    = 0;

	/*--------------------------------------*/
	/*  Input Check                         */
	/*--------------------------------------*/
	if (g_pYSeqData == (const T_YSEQ_DATA*)NULL)
	{
		bRet = FALSE;
	}
	else
	{
		if (YSeqID >= g_pYSeqData->YSeqInfoNum)
		{
			bRet = FALSE;
		}
	}

	/*--------------------------------------*/
	/*  Setup                               */
	/*--------------------------------------*/
	if (bRet == TRUE)
	{
		/*--- Macro Command (MCRS) ---*/
		bRet = YVC1_WriteReg(REG_R60H, YSEQ_MCR_MCRS);
		if (bRet == TRUE)
		{
			do {
				bRet = YVC1_ReadReg(REG_R60H, &u08RegR60);
				u32Count++;
				if(u32Count > YVC1_POLLING_MAX)
				{
					bRet = FALSE;
				}
			} while (((u08RegR60 & YSEQ_MCR_MCRS) != 0) && (bRet == TRUE));
		}

		if (bRet == TRUE)
		{
			/* Sequence Infomation setup */
			g_pYSeqInfo = &g_pYSeqData->tYSeqInfo[YSeqID];

			/* Scene data setup */
			u32SceneAddr = g_pYSeqInfo->SceneAddr;
			u08SSA[0]    = (U08)((u32SceneAddr >> SHIFT_24BIT) & MASK_03H);
			u08SSA[1]    = (U08)((u32SceneAddr >> SHIFT_16BIT) & MASK_FFH);
			u08SSA[2]    = (U08)((u32SceneAddr >> SHIFT_08BIT) & MASK_FFH);
			u08SSA[3]    = (U08)((u32SceneAddr               ) & MASK_FCH);
			bRet = YVC1_WriteRegs(REG_R5CH, &u08SSA[0], 4);

			/*--- Frame Action setup ---*/
			if (bRet == TRUE)
			{
				/*--- Macro Command ---*/
				YVC_WRITE_PORT(YVC1_PORT_FLG_STATUS2, YSEQ_STATUS_FMC);
				bRet = YVC1_WriteReg(REG_R60H, YSEQ_MCR_PLAY);

				/*--- YSeqID under execution is held. ---*/
				g_s16YSeqID = YSeqID;

				/*--- Sound Set ---*/
				#ifdef YSEQ_SOUND_ENABLE
				bRet = YSEQ_SetSound(g_pYSeqInfo->SoundID);
				#endif /* YSEQ_SOUND_ENABLE	*/
			}
		}
	}

	return bRet;
}



/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_SoundInit                                            */
/* Contents         : Sequence Sound initialization                             */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
#ifdef YSEQ_SOUND_ENABLE
BOOL YSEQ_SoundInit(void)
{
	BOOL bRet = TRUE;

	/*--------------------------------------*/
	/*  YMF825(SD-1) initialization         */
	/*--------------------------------------*/
	#ifdef YSEQ_SOUND_SD1
	Y825DD_Init();
	#endif

	/*--------------------------------------*/
	/*  YMF827(APL-5) initialization        */
	/*--------------------------------------*/
	#ifdef YSEQ_SOUND_APL5
	
		/* Unmounted. */
		#if 0
		APL5Drv_Init();
		#endif

	#endif

	return bRet;
}
#endif /* YSEQ_SOUND_ENABLE	*/



/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_SetSound                                             */
/* Contents         : Sequence Sound Data setup                                 */
/* Argument         : U16   SoundID             (W) Sound ID                    */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
#ifdef YSEQ_SOUND_ENABLE
BOOL YSEQ_SetSound(U16 SoundID)
{
	BOOL bRet                = TRUE;
	const U08 *u08pSoundData = NULL;
	U16  u16SoundDataSize    = 0;
	S16  s16Ret_SD1          = 0;
	S16  s16SoundID          = SoundID & (~YSEQ_SOUND_STOP);
	S16  s16Status           = 0;

	if (s16SoundID != YSEQ_SOUND_NONE)
	{
		if ((SoundID & YSEQ_SOUND_STOP) != YSEQ_SOUND_STOP)
		{
			/*----------------------------------------------*/
			/*                  Sound Play                  */
			/*----------------------------------------------*/
			/*------------------------------*/
			/*          SD-1 Driver         */
			/*------------------------------*/
			#ifdef YSEQ_SOUND_SD1
			if ( (0 < SoundID) && (SoundID <= tSd1SoundInfo->Sd1SoundTblNum) )
			{
				SoundID--;				/* Sequence Data Sound ID : 1 Origin	*/
				u08pSoundData    = &tSd1SoundInfo->SoundData[tSd1SoundInfo->tSd1SoundTbl[SoundID].SoundDataIndex];
				u16SoundDataSize = tSd1SoundInfo->tSd1SoundTbl[SoundID].SoundDataSize;

				/*--- Status ---*/
				s16Status = Y825DD_Status();
				if ((s16Status == Y825DD_OPENED) || (s16Status == Y825DD_PLAYING))
				{
					/*--- Close ---*/
					s16Ret_SD1 = Y825DD_Close();
				}

				/*--- Open ---*/
				if (s16Ret_SD1 == Y825DD_SUCCESS)
				{
					s16Ret_SD1 = Y825DD_Open(u08pSoundData, (S16)u16SoundDataSize);
				}

				/*--- Start ---*/
				if (s16Ret_SD1 == Y825DD_SUCCESS)
				{
					s16Ret_SD1 = Y825DD_Start();
				}
			}
			else
			{
				s16Ret_SD1 = Y825DD_ERROR;
			}
			#endif /* YSEQ_SOUND_SD1	*/

			/*------------------------------*/
			/*          APL-5 Driver        */
			/*------------------------------*/
			#ifdef YSEQ_SOUND_APL5

			/* Unmounted. */

			#endif
		}
		else
		{
			/*----------------------------------------------*/
			/*                  Sound Stop                  */
			/*----------------------------------------------*/
			/*------------------------------*/
			/*          SD-1 Driver         */
			/*------------------------------*/
			#ifdef YSEQ_SOUND_SD1
			/*--- Close ---*/
			s16Status = Y825DD_Status();
			if ((s16Status == Y825DD_OPENED) || (s16Status == Y825DD_PLAYING))
			{
				s16Ret_SD1 = Y825DD_Close();
			}
			#endif /* YSEQ_SOUND_SD1	*/

			/*------------------------------*/
			/*          APL-5 Driver        */
			/*------------------------------*/
			#ifdef YSEQ_SOUND_APL5

			/* Unmounted. */

			#endif
		}
	}

	/*--- Result ---*/
	#ifdef YSEQ_SOUND_SD1
	if (s16Ret_SD1 == Y825DD_ERROR)
	{
		bRet = FALSE;
	}
	#endif /* YSEQ_SOUND_SD1	*/

	#ifdef YSEQ_SOUND_APL5

	/* Unmounted. */

	#endif

	return bRet;
}
#endif /* YSEQ_SOUND_ENABLE	*/



/*------------------------------------------------------------------------------*/
/* Function name    : YSEQ_SoundIrqReq                                          */
/* Contents         : Sequence Sound Interrupt ReQuest                          */
/* Argument         : void                                                      */
/*                                                                              */
/* Return value     : TRUE or FALSE                                             */
/*------------------------------------------------------------------------------*/
#ifdef YSEQ_SOUND_ENABLE
BOOL YSEQ_SoundIrqReq(void)
{
	BOOL bRet        = FALSE;
	S16  s16Ret_APL5 = 0;

	#ifdef YSEQ_SOUND_SD1
	/*------------------------------*/
	/*          SD-1 Driver         */
	/*------------------------------*/
	machdep_IrqProc();
	bRet = TRUE;
	#endif /* YSEQ_SOUND_SD1	*/

	#ifdef YSEQ_SOUND_APL5
	/*------------------------------*/
	/*          APL-5 Driver        */
	/*------------------------------*/

	/* Unmounted. */

		#if 0
		s16Ret_APL5 = APL5Drv_IntHandler();
		if (s16Ret_APL5 >= 0)
		{
			bRet = TRUE;
		}
		#endif

	#endif

	return bRet;
}
#endif /* YSEQ_SOUND_ENABLE	*/


/*------------------------------ E O F -----------------------------------------*/

