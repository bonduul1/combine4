/******************************
	Sequencer Configuration Data
	Filename	: YSeqConfig.h
	Format Ver	: 1.4.0.60
	Date		: 2025-05-29 오후 4:00:38 +09:00
*******************************/

#ifndef _YSEQ_CONFIG_H_
#define _YSEQ_CONFIG_H_

/* #define YSEQ_SOUND_ENABLE */
/* #define YSEQ_SOUND_SD1 */

#define YSEQ_INVALID			(-1)

#define YSEQ_ACTION_NONE		(0)
#define YSEQ_ACTION_STOP		(1)
#define YSEQ_ACTION_PLAY		(2)

#define YSEQ_EVENT_UNKNOWN		(0)
#define YSEQ_EVENT_CLICK		(1)
#define YSEQ_EVENT_MOUSEDOWN	(2)
#define YSEQ_EVENT_MOUSEUP		(3)

#define YSEQ_SOUND_NONE			(0)
#define YSEQ_SOUND_STOP			(0x8000)
#define YSEQ_SOUND_PLAY			(0x0000)

/*----------------------------------------
 *	Sequence Button Structure
 *----------------------------------------*/
typedef struct {
	S16						X1;				/* Left										*/
	S16						Y1;				/* Top										*/
	S16						X2;				/* Right									*/
	S16						Y2;				/* Bottom									*/
	S16						NextSeqID;		/* Next Sequence number						*/
	U08						Action;			/* None, Stop or Play						*/
	U08						Event;			/* Unknown, Click, MouseDown or MouseUp		*/
#ifdef YSEQ_SOUND_ENABLE
	U16						SoundID;		/* Sound data index number					*/
#endif
} T_YSEQ_BUTTON;

/*----------------------------------------
 *	Sequence Information Structure
 *----------------------------------------*/
typedef struct {
	U32						SceneAddr;		/* Scene data start address					*/
	U08						Action;			/* None, Stop or Play						*/
	U08						YSeqButtonNum;	/* The number of button informations		*/
	const T_YSEQ_BUTTON		*tYSeqButton;	/* Pointer to the top of button information	*/
	S16						NextSeqID;		/* Next sequence index number				*/
#ifdef YSEQ_SOUND_ENABLE
	U16						SoundID;		/* Sound data index number with action		*/
#endif
} T_YSEQ_INFO;

/*----------------------------------------
 *	Sequence Data Structure
 *----------------------------------------*/
typedef struct {
	const T_YSEQ_INFO		*tYSeqInfo;		/* Pointer to the sequence information		*/
	U16						YSeqInfoNum;	/* The number of sequence informations		*/
} T_YSEQ_DATA;

/*----------------------------------------
 *	Sound Data Table Structure
 *----------------------------------------*/
typedef struct {
	U16						SoundDataIndex;	/* Index number of sound data				*/
	U16						SoundDataSize;	/* Byte size of sound data					*/
} T_SD1_SOUND_TBL;

/*----------------------------------------
 *	Sound Data Access Table Structure
 *----------------------------------------*/
typedef struct {
	const T_SD1_SOUND_TBL	*tSd1SoundTbl;	/* Pointer to the sound data table			*/
	U16						Sd1SoundTblNum;	/* The number of sound data table			*/
	const U08				*SoundData;		/* Pointer to the sound data				*/
} T_SD1_SOUND_INFO;

#endif /* _YSEQ_CONFIG_H_ */
