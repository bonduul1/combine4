/******************************
	Layout Data
	Filename	: layoutData.h
	Format Ver	: 1.4.0.60
	Date		: 2025-05-29 오후 4:00:47 +09:00
*******************************/

#ifndef _YLAYOUT_DATA_H_
#define _YLAYOUT_DATA_H_


#if 0

/*----------------------------------------
 *	BASIC - DEFINITION
 *----------------------------------------*/
typedef signed char		S08;				/* signed  8 bit integer	*/
typedef signed short	S16;				/* signed 16 bit integer	*/
typedef signed long		S32;				/* signed 32 bit integer	*/
typedef unsigned char	U08;				/* unsigned  8 bit integer	*/
typedef unsigned short	U16;				/* unsigned 16 bit integer	*/
typedef unsigned long	U32;				/* unsigned 32 bit integer	*/

/*----------------------------------------
 *	YGF Proportional Data
 *----------------------------------------*/
typedef struct {
	U08						XS;	/* Proportional data on the font left side	*/
	U08						XE;	/* Proportional data on the font right side	*/
} T_YGFONT_PDATA;

#endif

/*----------------------------------------
 *	Text Align X Type
 *----------------------------------------*/
#define YTEXTALIGNX_LEFT				(0)
#define YTEXTALIGNX_CENTER				(1)
#define YTEXTALIGNX_RIGHT				(2)

/*----------------------------------------
 *	YGF Search Table Data
 *----------------------------------------*/
typedef struct {
	U16						StartCode;	/* Top character code in the block	*/
	U16						EndCode;	/* End character code in the block	*/
	U16						SeqOffset;	/* Search character code offset		*/
} T_YGFONT_TBL_LYT;

/*----------------------------------------
 *	Font Data
 *----------------------------------------*/
typedef struct {
	const U08				FontTypAttr[8];	/* Font type attribute data		*/
	const U08				YgfontTblSize;	/* Number of Search table data	*/
	const T_YGFONT_TBL_LYT	*tYgfontTbl;	/* YGF Search table data		*/
	const T_YGFONT_PDATA	*tYgfontPData;	/* YGF Proportional data		*/
} T_YGFONT_DATA_LYT;

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
typedef struct t_fontctrl {
	const U08				*FontAttr;		/* Font attribute data pointer	*/
	const U08				*FCode;			/* FCode data pointer			*/
	const T_YGFONT_DATA_LYT	*tYgfontData;	/* Font data pointer			*/
	U16						Width;			/* Display width				*/
	U08						AlignX;			/* X align						*/
	U08						FontAttrNum;	/* Number of font attribute data*/
	U16						MaxCharNum;		/* Number of max character		*/
	U16						LayerIndex;		/* Layer index					*/
	const struct t_fontctrl	*tRelation;		/* Related font control data	*/
} T_YGFONTCTRL;



/*----------------------------------------
 *	Extended Layer
 *----------------------------------------*/
#define YVCX_EXLY					(1)

/*----------------------------------------
 *	Language Type
 *----------------------------------------*/
#define YLANGUAGE_EN				(0)
#define YLANGUAGE_KO				(1)

/*----------------------------------------
 *	Font Data
 *----------------------------------------*/
#define FontTypeAttr_ADDR		(0x0FD8)	/* Start address of Font Type Attr	*/
#define FontTypeAttr_SIZE		(0x0028)	/* Data size of Font Type Attr		*/
#define FCodeData_ADDR			(0x0C70)	/* Start address of Font Code Data	*/
#define FCodeData_SIZE			(0x0368)	/* Data size of Font Code Data		*/

/*----------------------------------------
 *	Palette Data
 *----------------------------------------*/
#define  PaletteData_ADDR		(0x0000)	/* Start address of Palette Table	*/
#define  PaletteData_SIZE		(0x0008)	/* Data size of Palettes			*/
extern const U08 PaletteData_LYT[8];



enum E_WARNING_1 {		/* <CPU>WARNING_1 */
	warning_1_bg_001 = 0,				/* warning_1_bg_001 */
	Warning_fuel_alarm,					/* Warning_fuel_alarm */
	Warning_engine_oil,					/* Warning_engine_oil */
	Warning_mission_oil,					/* Warning_mission_oil */
	Warning_center_error,					/* Warning_center_error */
	Warning_starting_safety,					/* Warning_starting_safety */
	Warning_starting_safety_threshing,					/* Warning_starting_safety_threshing */
	Warning_preheater,					/* Warning_preheater */
	Warning_grain_emission_complete,					/* Warning_grain_emission_complete */
	Warning_grain_emission_blockage,					/* Warning_grain_emission_blockage */
	Warning_can_error_1,					/* Warning_can_error_1 */
	Warning_can_error_2,					/* Warning_can_error_2 */
	Warning_can_error_3,					/* Warning_can_error_3 */
	Warning_can_error_4,					/* Warning_can_error_4 */
	Warning_can_error_5,					/* Warning_can_error_5 */
	Warning_can_error_6,					/* Warning_can_error_6 */
	Warning_can_error_7,					/* Warning_can_error_7 */
	Warning_can_error_8,					/* Warning_can_error_8 */
	Warning_can_error_9,					/* Warning_can_error_9 */
	Warning_chuyan,					/* Warning_chuyan */
	Warning_auger_motor,					/* Warning_auger_motor */
	Warning_threshing_depth,					/* Warning_threshing_depth */
	Warning_chuhen_motor,					/* Warning_chuhen_motor */
	Warning_chuyan_motor,					/* Warning_chuyan_motor */
	Warning_battery,					/* Warning_battery */
	Warning_chuhen_cont,					/* Warning_chuhen_cont */
	WARNING_1_NUM				/* number of layers of '<CPU>WARNING_1' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  WARNING_1_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  WARNING_1_GT_SIZE		(0x0138)	/* Data size of General Table */
extern const U08 WARNING_1_GT[26][12];		/* <CPU>WARNING_1 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  WARNING_1_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  WARNING_1_EL_SIZE		(0x0026)	/* Data size of Extended Layer */
extern const U08 WARNING_1_EL[38];		/* <CPU>WARNING_1 */
#endif



enum E_CHECK_MODE {		/* <CPU>CHECK_MODE */
	bg_new_1 = 0,				/* bg_new_1 */
	bg_new_2,					/* bg_new_2 */
	fuel_logo,					/* fuel_logo */
	fuel_level_1,					/* fuel_level_1 */
	fuel_level_2,					/* fuel_level_2 */
	fuel_level_3,					/* fuel_level_3 */
	fuel_level_4,					/* fuel_level_4 */
	fuel_level_5,					/* fuel_level_5 */
	fuel_level_frame,					/* fuel_level_frame */
	diagnol,					/* diagnol */
	horizontal1,					/* horizontal1 */
	horizontal2,					/* horizontal2 */
	conveyer1,					/* conveyer - 1 */
	conveyer2,					/* conveyer - 2 */
	conveyer3,					/* conveyer - 3 */
	conveyer4,					/* conveyer - 4 */
	conveyer5,					/* conveyer - 5 */
	conveyer6,					/* conveyer - 6 */
	conveyer7,					/* conveyer - 7 */
	conveyer8,					/* conveyer - 8 */
	conveyer9,					/* conveyer - 9 */
	conveyer10,					/* conveyer - 10 */
	conveyer11,					/* conveyer - 11 */
	conveyer12,					/* conveyer - 12 */
	conveyer13,					/* conveyer - 13 */
	conveyer14,					/* conveyer - 14 */
	conveyer15,					/* conveyer - 15 */
	tank_level1,					/* tank_level1 */
	tank_level2,					/* tank_level2 */
	tank_level3,					/* tank_level3 */
	tank_level4,					/* tank_level4 */
	tank_box,					/* tank_box */
	bank_level11,					/* bank_level11 */
	bank_level12,					/* bank_level12 */
	bank_level13,					/* bank_level13 */
	bank_level14,					/* bank_level14 */
	bank_level15,					/* bank_level15 */
	bank_box_1,					/* bank_box_1 */
	bank_level21,					/* bank_level21 */
	bank_level22,					/* bank_level22 */
	bank_level23,					/* bank_level23 */
	bank_level24,					/* bank_level24 */
	bank_level25,					/* bank_level25 */
	bank_box_2,					/* bank_box_2 */
	unitf,					/* unitf */
	unite,					/* unite */
	unit_half,					/* unit_half */
	dash1,					/* dash1 */
	dash2,					/* dash2 */
	dash_block,					/* dash_block */
	check_mode_text1,					/* check_mode_text1 */
	check_mode_text2,					/* check_mode_text2 */
	check_mode_text3,					/* check_mode_text3 */
	check_mode_text4,					/* check_mode_text4 */
	check_mode_text5,					/* check_mode_text5 */
	CHECK_MODE_NUM				/* number of layers of '<CPU>CHECK_MODE' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  CHECK_MODE_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  CHECK_MODE_GT_SIZE		(0x0294)	/* Data size of General Table */
extern const U08 CHECK_MODE_GT[55][12];		/* <CPU>CHECK_MODE */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  CHECK_MODE_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  CHECK_MODE_EL_SIZE		(0x0052)	/* Data size of Extended Layer */
extern const U08 CHECK_MODE_EL[82];		/* <CPU>CHECK_MODE */
#endif



enum E_WARNING_2 {		/* <CPU>WARNING_2 */
	warning_2_bg_001 = 0,				/* warning_2_bg_001 */
	warning_2_bg_002,					/* warning_2_bg_002 */
	combine_pic,					/* combine_pic */
	combine_pic_info,					/* combine_pic_info */
	Title_bg,					/* Title_bg */
	warning_stop_logo,					/* warning_stop - _logo */
	warning_stop_message,					/* warning_stop - _message */
	warning_stop_body,					/* warning_stop - _body */
	warning_stop_arrow,					/* warning_stop - _arrow */
	warning_stop_title,					/* warning_stop - _title */
	warning_battery_logo,					/* warning_battery - _logo */
	warning_battery_title,					/* warning_battery - _title */
	warning_battery_message,					/* warning_battery - _message */
	warning_battery_body,					/* warning_battery - _body */
	warning_battery_arrow,					/* warning_battery - _arrow */
	warning_oil_logo,					/* warning_oil - _logo */
	warning_oil_message,					/* warning_oil - _message */
	warning_oil_body,					/* warning_oil - _body */
	warning_oil_arrow,					/* warning_oil - _arrow */
	warning_oil_title,					/* warning_oil - _title */
	warning_temperature_logo,					/* warning_temperature - _logo */
	warning_temperature_message,					/* warning_temperature - _message */
	warning_temperature_body,					/* warning_temperature - _body */
	warning_temperature_arrow,					/* warning_temperature - _arrow */
	warning_temperature_title,					/* warning_temperature - _title */
	warning_flow_logo,					/* warning_flow - _logo */
	warning_flow_message,					/* warning_flow - _message */
	warning_flow_body,					/* warning_flow - _body */
	warning_flow_arrow,					/* warning_flow - _arrow */
	warning_flow_title,					/* warning_flow - _title */
	warning_leaf1_logo,					/* warning_leaf1 - _logo */
	warning_leaf1_message,					/* warning_leaf1 - _message */
	warning_leaf1_body,					/* warning_leaf1 - _body */
	warning_leaf1_arrow,					/* warning_leaf1 - _arrow */
	warning_leaf1_title,					/* warning_leaf1 - _title */
	warning_leaf2_logo,					/* warning_leaf2 - _logo */
	warning_leaf2_message,					/* warning_leaf2 - _message */
	warning_leaf2_body,					/* warning_leaf2 - _body */
	warning_leaf2_arrow,					/* warning_leaf2 - _arrow */
	warning_leaf2_title,					/* warning_leaf2 - _title */
	warning_rotation_logo,					/* warning_rotation - _logo */
	warning_rotation_message,					/* warning_rotation - _message */
	warning_rotation_body,					/* warning_rotation - _body */
	warning_rotation_arrow,					/* warning_rotation - _arrow */
	warning_rotation_title,					/* warning_rotation - _title */
	warning_gear_logo,					/* warning_gear - _logo */
	warning_gear_message,					/* warning_gear - _message */
	warning_gear_body,					/* warning_gear - _body */
	warning_gear_arrow,					/* warning_gear - _arrow */
	warning_gear_title,					/* warning_gear - _title */
	warning_leaf3_logo,					/* warning_leaf3 - _logo */
	warning_leaf3_message,					/* warning_leaf3 - _message */
	warning_leaf3_body,					/* warning_leaf3 - _body */
	warning_leaf3_arrow,					/* warning_leaf3 - _arrow */
	warning_leaf3_title,					/* warning_leaf3 - _title */
	warning_drop_title,					/* warning_drop - _title */
	warning_drop_logo,					/* warning_drop - _logo */
	warning_drop_message,					/* warning_drop - _message */
	warning_drop_body,					/* warning_drop - _body */
	warning_drop_arrow,					/* warning_drop - _arrow */
	warning_gugmuli_arrow,					/* warning_gugmuli - _arrow */
	warning_gugmuli_message,					/* warning_gugmuli - _message */
	warning_gugmuli_logo,					/* warning_gugmuli - _logo */
	warning_cut_arrow,					/* warning_cut - _arrow */
	warning_cut_body,					/* warning_cut - _body */
	warning_cut_message,					/* warning_cut - _message */
	warning_cut_logo,					/* warning_cut - _logo */
	warning_cut_title,					/* warning_cut - _title */
	warning_eh_arrow,					/* warning_eh - _arrow */
	warning_eh_body,					/* warning_eh - _body */
	warning_eh_message,					/* warning_eh - _message */
	warning_eh_logo,					/* warning_eh - _logo */
	warning_eh_title,					/* warning_eh - _title */
	warning_2nd_error_arrow,					/* warning_2nd_error - _arrow */
	warning_2nd_error_body,					/* warning_2nd_error - _body */
	warning_2nd_error_message,					/* warning_2nd_error - _message */
	warning_2nd_error_logo,					/* warning_2nd_error - _logo */
	warning_2nd_error_title,					/* warning_2nd_error - _title */
	warning_2nd_sw_arrow,					/* warning_2nd_sw - _arrow */
	warning_2nd_sw_body,					/* warning_2nd_sw - _body */
	warning_2nd_sw_message,					/* warning_2nd_sw - _message */
	warning_2nd_sw_logo,					/* warning_2nd_sw - _logo */
	warning_2nd_sw_title,					/* warning_2nd_sw - _title */
	warning_yangug_error_arrow,					/* warning_yangug_error - _arrow */
	warning_yangug_error_body,					/* warning_yangug_error - _body */
	warning_yangug_error_message,					/* warning_yangug_error - _message */
	warning_yangug_error_logo,					/* warning_yangug_error - _logo */
	warning_yangug_error_title,					/* warning_yangug_error - _title */
	WARNING_2_NUM				/* number of layers of '<CPU>WARNING_2' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  WARNING_2_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  WARNING_2_GT_SIZE		(0x0420)	/* Data size of General Table */
extern const U08 WARNING_2_GT[88][12];		/* <CPU>WARNING_2 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  WARNING_2_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  WARNING_2_EL_SIZE		(0x0083)	/* Data size of Extended Layer */
extern const U08 WARNING_2_EL[131];		/* <CPU>WARNING_2 */
#endif



enum E_WARNING_3 {		/* <CPU>WARNING_3 */
	warning_3_bg_002 = 0,				/* warning_3_bg_002 */
	Data1_value,					/* <GRP2><TXT5>Data1_value */
	Data2_value,					/* <GRP3><TXT5>Data2_value */
	Data3_value,					/* <GRP4><TXT5>Data3_value */
	Data4_value,					/* <GRP5><TXT5>Data4_value */
	unit_percent,					/* unit_percent */
	unit_voltage,					/* unit_voltage */
	unit_on,					/* unit_on */
	unit_off,					/* unit_off */
	MH1_title,					/* MH1 - _title */
	MH1_sub_title,					/* MH1 - _sub_title */
	MH1_message_1,					/* MH1 - _message_1 */
	MH2_sub_title,					/* MH2 - _sub_title */
	MH2_title,					/* MH2 - _title */
	MH2_message_1,					/* MH2 - _message_1 */
	MH3_sub_title,					/* MH3 - _sub_title */
	MH3_title,					/* MH3 - _title */
	MH3_message_1,					/* MH3 - _message_1 */
	MH4_title,					/* MH4 - _title */
	MH4_sub_title,					/* MH4 - _sub_title */
	MH4_message_1,					/* MH4 - _message_1 */
	MH4_message_2,					/* MH4 - _message_2 */
	MH4_message_3,					/* MH4 - _message_3 */
	MH4_message_4,					/* MH4 - _message_4 */
	MH5_title,					/* MH5 - _title */
	MH5_sub_title,					/* MH5 - _sub_title */
	MH5_message_1,					/* MH5 - _message_1 */
	MH5_message_2,					/* MH5 - _message_2 */
	MH5_message_3,					/* MH5 - _message_3 */
	MH5_message_4,					/* MH5 - _message_4 */
	MH6_title,					/* MH6 - _title */
	MH6_sub_title,					/* MH6 - _sub_title */
	MH6_message_1,					/* MH6 - _message_1 */
	MH6_message_2,					/* MH6 - _message_2 */
	MH7_title,					/* MH7 - _title */
	MH7_sub_title,					/* MH7 - _sub_title */
	MH7_message_1,					/* MH7 - _message_1 */
	MH7_message_2,					/* MH7 - _message_2 */
	MH8_title,					/* MH8 - _title */
	MH8_sub_title,					/* MH8 - _sub_title */
	MH8_message_1,					/* MH8 - _message_1 */
	MH8_message_2,					/* MH8 - _message_2 */
	MH9_title,					/* MH9 - _title */
	MH9_sub_title,					/* MH9 - _sub_title */
	MH9_message_1,					/* MH9 - _message_1 */
	MH10_title,					/* MH10 - _title */
	MH10_sub_title,					/* MH10 - _sub_title */
	MH10_message_1,					/* MH10 - _message_1 */
	MH11_title,					/* MH11 - _title */
	MH11_sub_title,					/* MH11 - _sub_title */
	MH11_message_1,					/* MH11 - _message_1 */
	MH12_title,					/* MH12 - _title */
	MH12_sub_title,					/* MH12 - _sub_title */
	MH12_message_1,					/* MH12 - _message_1 */
	MH13_title,					/* MH13 - _title */
	MH13_sub_title,					/* MH13 - _sub_title */
	MH13_message_1,					/* MH13 - _message_1 */
	MH13_message_2,					/* MH13 - _message_2 */
	MH14_title,					/* MH14 - _title */
	MH14_sub_title,					/* MH14 - _sub_title */
	MH14_message_1,					/* MH14 - _message_1 */
	MH14_message_2,					/* MH14 - _message_2 */
	MH15_title,					/* MH15 - _title */
	MH15_sub_title,					/* MH15 - _sub_title */
	MH15_message_1,					/* MH15 - _message_1 */
	MH15_message_2,					/* MH15 - _message_2 */
	MH16_title,					/* MH16 - _title */
	MH16_sub_title,					/* MH16 - _sub_title */
	MH16_message_1,					/* MH16 - _message_1 */
	MH16_message_2,					/* MH16 - _message_2 */
	WARNING_3_NUM				/* number of layers of '<CPU>WARNING_3' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  WARNING_3_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  WARNING_3_GT_SIZE		(0x0348)	/* Data size of General Table */
extern const U08 WARNING_3_GT[70][12];		/* <CPU>WARNING_3 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  WARNING_3_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  WARNING_3_EL_SIZE		(0x0068)	/* Data size of Extended Layer */
extern const U08 WARNING_3_EL[104];		/* <CPU>WARNING_3 */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL Data1_value_FC;		/* <GRP2><TXT5>Data1_value */

extern const T_YGFONTCTRL Data2_value_FC;		/* <GRP3><TXT5>Data2_value */

extern const T_YGFONTCTRL Data3_value_FC;		/* <GRP4><TXT5>Data3_value */

extern const T_YGFONTCTRL Data4_value_FC;		/* <GRP5><TXT5>Data4_value */




enum E_UNITS {		/* <CPU>UNITS */
	UNITS_BG_1 = 0,				/* UNITS_BG_1 */
	Layer_1,					/* Layer_1 */
	Layer_2,					/* Layer_2 */
	Layer_3,					/* Layer_3 */
	Layer_4,					/* Layer_4 */
	Layer_5,					/* Layer_5 */
	Layer_6,					/* Layer_6 */
	Layer_7,					/* Layer_7 */
	Layer_8,					/* Layer_8 */
	Layer_9,					/* Layer_9 */
	Layer_10,					/* Layer_10 */
	Layer_11,					/* Layer_11 */
	Layer_12,					/* Layer_12 */
	Layer_13,					/* Layer_13 */
	Layer_14,					/* Layer_14 */
	Layer_15,					/* Layer_15 */
	Layer_16,					/* Layer_16 */
	Layer_17,					/* Layer_17 */
	Layer_18,					/* Layer_18 */
	Layer_19,					/* Layer_19 */
	Layer_20,					/* Layer_20 */
	Layer_21,					/* Layer_21 */
	Layer_22,					/* Layer_22 */
	Layer_23,					/* Layer_23 */
	Layer_24,					/* Layer_24 */
	Layer_25,					/* Layer_25 */
	Layer_26,					/* Layer_26 */
	Layer_27,					/* Layer_27 */
	Layer_28,					/* Layer_28 */
	Layer_29,					/* Layer_29 */
	Layer_30,					/* Layer_30 */
	Layer_31,					/* Layer_31 */
	Layer_32,					/* Layer_32 */
	Layer_33,					/* Layer_33 */
	Layer_34,					/* Layer_34 */
	Layer_35,					/* Layer_35 */
	Layer_36,					/* Layer_36 */
	Layer_37,					/* Layer_37 */
	Layer_38,					/* Layer_38 */
	Layer_39,					/* Layer_39 */
	Layer_40,					/* Layer_40 */
	Layer_41,					/* Layer_41 */
	Layer_42,					/* Layer_42 */
	Layer_43,					/* Layer_43 */
	Layer_44,					/* Layer_44 */
	Layer_45,					/* Layer_45 */
	Layer_46,					/* Layer_46 */
	Layer_47,					/* Layer_47 */
	Layer_48,					/* Layer_48 */
	Layer_49,					/* Layer_49 */
	Layer_50,					/* Layer_50 */
	Layer_51,					/* Layer_51 */
	Layer_52,					/* Layer_52 */
	Layer_53,					/* Layer_53 */
	Layer_54,					/* Layer_54 */
	Layer_55,					/* Layer_55 */
	Layer_56,					/* Layer_56 */
	Layer_57,					/* Layer_57 */
	Layer_58,					/* Layer_58 */
	Layer_59,					/* Layer_59 */
	Layer_60,					/* Layer_60 */
	Layer_61,					/* Layer_61 */
	Layer_62,					/* Layer_62 */
	Layer_63,					/* Layer_63 */
	Layer_64,					/* Layer_64 */
	Layer_65,					/* Layer_65 */
	Layer_66,					/* Layer_66 */
	Layer_67,					/* Layer_67 */
	Layer_68,					/* Layer_68 */
	units_a,					/* units_ - a */
	units_Cellius,					/* units_ - Cellius */
	units_cm,					/* units_ - cm */
	units_comm_ok,					/* units_ - comm_ok */
	units_gradius,					/* units_ - gradius */
	units_hz,					/* units_ - hz */
	units_msec,					/* units_ - msec */
	units_off,					/* units_ - off */
	units_on,					/* units_ - on */
	units_percent,					/* units_ - percent */
	units_rpm,					/* units_ - rpm */
	units_s,					/* units_ - s */
	units_sec,					/* units_ - sec */
	units_v,					/* units_ - v */
	units_val,					/* units_ - val */
	UNITS_NUM				/* number of layers of '<CPU>UNITS' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  UNITS_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  UNITS_GT_SIZE		(0x03F0)	/* Data size of General Table */
extern const U08 UNITS_GT[84][12];		/* <CPU>UNITS */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  UNITS_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  UNITS_EL_SIZE		(0x007D)	/* Data size of Extended Layer */
extern const U08 UNITS_EL[125];		/* <CPU>UNITS */
#endif



enum E_BTN_PAGE {		/* <CPU>BTN_PAGE */
	Btn_selected_1 = 0,				/* Btn_selected_1 */
	Btn_selected_2,					/* Btn_selected_2 */
	Btn_selected_3,					/* Btn_selected_3 */
	Btn_selected_4,					/* Btn_selected_4 */
	Btn_unselect_1,					/* Btn_unselect_1 */
	Btn_unselect_2,					/* Btn_unselect_2 */
	Btn_unselect_3,					/* Btn_unselect_3 */
	Btn_unselect_4,					/* Btn_unselect_4 */
	Btn_unselect_5,					/* Btn_unselect_5 */
	Btn_huagin,					/* Btn_huagin */
	Btn_ok,					/* Btn_ok */
	Btn_select,					/* Btn_select */
	Btn_complete,					/* Btn_complete */
	Btn_agree,					/* Btn_agree */
	Btn_change_section,					/* Btn_change_section */
	Btn_previous,					/* Btn_previous */
	Btn_cancel,					/* Btn_cancel */
	Btn_next_item,					/* Btn_next_item */
	Btn_forward,					/* Btn_forward */
	Btn_down,					/* Btn_down */
	Btn_decrease,					/* Btn_decrease */
	Btn_previous_item,					/* Btn_previous_item */
	Btn_used,					/* Btn_used */
	Btn_up,					/* Btn_up */
	Btn_increase,					/* Btn_increase */
	Btn_back,					/* Btn_back */
	Btn_not_used,					/* Btn_not_used */
	Btn_menu,					/* Btn_menu */
	Btn_menu_1,					/* Btn_menu_1 */
	Btn_menu_2,					/* Btn_menu_2 */
	Btn_help,					/* Btn_help */
	Btn_exchange,					/* Btn_exchange */
	Btn_forward_text,					/* Btn_forward_text */
	Btn_home,					/* Btn_home */
	Btn_pgup,					/* Btn_pgup */
	Btn_pgdn,					/* Btn_pgdn */
	Btn_mode,					/* Btn_mode */
	Btn_escape,					/* Btn_escape */
	Btn_next_eng,					/* Btn_next_eng */
	Btn_camera,					/* Btn_camera */
	Btn_camera_2,					/* Btn_camera_2 */
	Btn_camera_1,					/* Btn_camera_1 */
	Btn_engine,					/* Btn_engine */
	Btn_mission,					/* Btn_mission */
	Btn_next_han,					/* Btn_next_han */
	Btn_mode_han,					/* Btn_mode_han */
	SelectRow,					/* SelectRow */
	BTN_PAGE_NUM				/* number of layers of '<CPU>BTN_PAGE' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  BTN_PAGE_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  BTN_PAGE_GT_SIZE		(0x0234)	/* Data size of General Table */
extern const U08 BTN_PAGE_GT[47][12];		/* <CPU>BTN_PAGE */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  BTN_PAGE_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  BTN_PAGE_EL_SIZE		(0x0046)	/* Data size of Extended Layer */
extern const U08 BTN_PAGE_EL[70];		/* <CPU>BTN_PAGE */
#endif



enum E_MAIN {		/* <CPU>MAIN */
	BG_002 = 0,				/* BG_002 */
	BG_001,					/* BG_001 */
	warningkukmul,					/* warning - kukmul */
	warningtalkuk,					/* warning - talkuk */
	warning2bon,					/* warning - 2bon */
	warningchunjon,					/* warning - chunjon */
	warningengineOil,					/* warning - engineOil */
	warningkitaanjon,					/* warning - kitaanjon */
	warningyusu,					/* warning - yusu */
	BG_kukmul,					/* BG_kukmul */
	clock_box,					/* clock_box */
	error_box,					/* error_box */
	Text,					/* <GRP0><TXT9>Text */
	Error_message_2,					/* <GRP28><TXT20>Error_message_2 */
	Error_message_1,					/* <GRP29><TXT11>Error_message_1 */
	oil_lamp,					/* oil_lamp */
	charge_lamp,					/* charge_lamp */
	turn_right,					/* turn_right */
	turn_left,					/* turn_left */
	front_light,					/* front_light */
	clock1,					/* clock1 */
	clock2,					/* clock2 */
	clock3,					/* clock3 */
	clock4,					/* clock4 */
	clock5,					/* clock5 */
	clock6,					/* clock6 */
	clock7,					/* clock7 */
	clock8,					/* clock8 */
	fuel_red,					/* fuel_red */
	load_rate001,					/* load_rate - 001 */
	load_rate002,					/* load_rate - 002 */
	load_rate003,					/* load_rate - 003 */
	load_rate004,					/* load_rate - 004 */
	load_rate005,					/* load_rate - 005 */
	load_rate006,					/* load_rate - 006 */
	load_rate007,					/* load_rate - 007 */
	load_rate008,					/* load_rate - 008 */
	load_rate009,					/* load_rate - 009 */
	load_rate010,					/* load_rate - 010 */
	load_rate011,					/* load_rate - 011 */
	load_rate012,					/* load_rate - 012 */
	load_rate013,					/* load_rate - 013 */
	load_rate014,					/* load_rate - 014 */
	load_rate015,					/* load_rate - 015 */
	load_rate016,					/* load_rate - 016 */
	load_rate017,					/* load_rate - 017 */
	load_rate018,					/* load_rate - 018 */
	load_rate019,					/* load_rate - 019 */
	load_rate020,					/* load_rate - 020 */
	load_rate021,					/* load_rate - 021 */
	load_rate022,					/* load_rate - 022 */
	load_rate023,					/* load_rate - 023 */
	load_rate024,					/* load_rate - 024 */
	load_rate025,					/* load_rate - 025 */
	gokmul001,					/* gokmul - 001 */
	gokmul002,					/* gokmul - 002 */
	gokmul003,					/* gokmul - 003 */
	gokmul004,					/* gokmul - 004 */
	fuelorange001,					/* fuel - orange001 */
	fuelwhite001,					/* fuel - white001 */
	fuelwhite002,					/* fuel - white002 */
	fuelwhite003,					/* fuel - white003 */
	fuelwhite004,					/* fuel - white004 */
	fuelwhite005,					/* fuel - white005 */
	fuelwhite006,					/* fuel - white006 */
	fuelwhite007,					/* fuel - white007 */
	fuelwhite008,					/* fuel - white008 */
	fuelwhite009,					/* fuel - white009 */
	check001warning1,					/* check001 - warning1 */
//	check001Desox_inhibit_by_switch copy,					/* check001 - Desox_inhibit_by_switch copy */
	check001Emission_failure_indicator,					/* check001 - Emission_failure_indicator */
	check001Engine_Warning,					/* check001 - Engine_Warning */
	check002forced_Desox,					/* check002 - forced_Desox */
	check002DeSox_active,					/* check002 - DeSox_active */
	check002regen_request,					/* check002 - regen_request */
	MAIN_NUM				/* number of layers of '<CPU>MAIN' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MAIN_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MAIN_GT_SIZE		(0x0384)	/* Data size of General Table */
extern const U08 MAIN_GT[75][12];		/* <CPU>MAIN */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MAIN_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MAIN_EL_SIZE		(0x0070)	/* Data size of Extended Layer */
extern const U08 MAIN_EL[112];		/* <CPU>MAIN */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL Text_FC;		/* <GRP0><TXT9>Text */

extern const T_YGFONTCTRL Error_message_2_FC;		/* <GRP28><TXT20>Error_message_2 */

extern const T_YGFONTCTRL Error_message_1_FC;		/* <GRP29><TXT11>Error_message_1 */




enum E_MAIN_RPM {		/* <CPU>MAIN_RPM */
	rpm001 = 0,				/* rpm - 001 */
	rpm002,					/* rpm - 002 */
	rpm003,					/* rpm - 003 */
	rpm004,					/* rpm - 004 */
	rpm005,					/* rpm - 005 */
	rpm006,					/* rpm - 006 */
	rpm007,					/* rpm - 007 */
	rpm008,					/* rpm - 008 */
	rpm009,					/* rpm - 009 */
	rpm010,					/* rpm - 010 */
	rpm011,					/* rpm - 011 */
	rpm012,					/* rpm - 012 */
	rpm013,					/* rpm - 013 */
	rpm014,					/* rpm - 014 */
	rpm015,					/* rpm - 015 */
	rpm016,					/* rpm - 016 */
	rpm017,					/* rpm - 017 */
	rpm018,					/* rpm - 018 */
	rpm019,					/* rpm - 019 */
	rpm020,					/* rpm - 020 */
	rpm021,					/* rpm - 021 */
	rpm022,					/* rpm - 022 */
	rpm023,					/* rpm - 023 */
	rpm024,					/* rpm - 024 */
	rpm025,					/* rpm - 025 */
	rpm026,					/* rpm - 026 */
	rpm027,					/* rpm - 027 */
	rpm028,					/* rpm - 028 */
	rpm029,					/* rpm - 029 */
	rpm030,					/* rpm - 030 */
	rpm031,					/* rpm - 031 */
	rpm032,					/* rpm - 032 */
	rpm033,					/* rpm - 033 */
	rpm034,					/* rpm - 034 */
	rpm035,					/* rpm - 035 */
	rpm036,					/* rpm - 036 */
	rpm037,					/* rpm - 037 */
	rpm038,					/* rpm - 038 */
	rpm039,					/* rpm - 039 */
	rpm040,					/* rpm - 040 */
	rpm041,					/* rpm - 041 */
	rpm042,					/* rpm - 042 */
	rpm043,					/* rpm - 043 */
	rpm044,					/* rpm - 044 */
	rpm045,					/* rpm - 045 */
	rpm046,					/* rpm - 046 */
	rpm047,					/* rpm - 047 */
	rpm048,					/* rpm - 048 */
	rpm049,					/* rpm - 049 */
	rpm050,					/* rpm - 050 */
	rpm051,					/* rpm - 051 */
	rpm052,					/* rpm - 052 */
	rpm053,					/* rpm - 053 */
	rpm054,					/* rpm - 054 */
	rpm055,					/* rpm - 055 */
	rpm056,					/* rpm - 056 */
	rpm057,					/* rpm - 057 */
	rpm058,					/* rpm - 058 */
	rpm059,					/* rpm - 059 */
	rpm060,					/* rpm - 060 */
	rpm061,					/* rpm - 061 */
	rpm062,					/* rpm - 062 */
	rpm063,					/* rpm - 063 */
	rpm064,					/* rpm - 064 */
	rpm065,					/* rpm - 065 */
	rpm066,					/* rpm - 066 */
	rpm067,					/* rpm - 067 */
	rpm068,					/* rpm - 068 */
	rpm069,					/* rpm - 069 */
	rpm070,					/* rpm - 070 */
	rpm071,					/* rpm - 071 */
	rpm072,					/* rpm - 072 */
	rpm073,					/* rpm - 073 */
	rpm074,					/* rpm - 074 */
	rpm075,					/* rpm - 075 */
	rpm076,					/* rpm - 076 */
	rpm077,					/* rpm - 077 */
	rpm078,					/* rpm - 078 */
	rpm079,					/* rpm - 079 */
	rpm080,					/* rpm - 080 */
	rpm081,					/* rpm - 081 */
	rpm082,					/* rpm - 082 */
	rpm083,					/* rpm - 083 */
	rpm084,					/* rpm - 084 */
	rpm085,					/* rpm - 085 */
	rpm086,					/* rpm - 086 */
	rpm087,					/* rpm - 087 */
	rpm088,					/* rpm - 088 */
	rpm089,					/* rpm - 089 */
	rpm090,					/* rpm - 090 */
	rpm091,					/* rpm - 091 */
	rpm092,					/* rpm - 092 */
	rpm093,					/* rpm - 093 */
	rpm094,					/* rpm - 094 */
	rpm095,					/* rpm - 095 */
	rpm096,					/* rpm - 096 */
	rpm097,					/* rpm - 097 */
	rpm098,					/* rpm - 098 */
	rpm099,					/* rpm - 099 */
	rpm100,					/* rpm - 100 */
	rpm101,					/* rpm - 101 */
	rpm102,					/* rpm - 102 */
	rpm103,					/* rpm - 103 */
	rpm104,					/* rpm - 104 */
	rpm105,					/* rpm - 105 */
	rpm106,					/* rpm - 106 */
	rpm107,					/* rpm - 107 */
	rpm108,					/* rpm - 108 */
	rpm109,					/* rpm - 109 */
	rpm110,					/* rpm - 110 */
	rpm111,					/* rpm - 111 */
	rpm112,					/* rpm - 112 */
	rpm113,					/* rpm - 113 */
	rpm114,					/* rpm - 114 */
	rpm115,					/* rpm - 115 */
	rpm116,					/* rpm - 116 */
	rpm117,					/* rpm - 117 */
	rpm118,					/* rpm - 118 */
	rpm119,					/* rpm - 119 */
	rpm120,					/* rpm - 120 */
	rpm121,					/* rpm - 121 */
	rpm122,					/* rpm - 122 */
	rpm123,					/* rpm - 123 */
	rpm124,					/* rpm - 124 */
	rpm125,					/* rpm - 125 */
	rpm126,					/* rpm - 126 */
	rpm127,					/* rpm - 127 */
	rpm128,					/* rpm - 128 */
	rpm129,					/* rpm - 129 */
	rpm130,					/* rpm - 130 */
	rpm131,					/* rpm - 131 */
	rpm132,					/* rpm - 132 */
	rpm133,					/* rpm - 133 */
	rpm134,					/* rpm - 134 */
	rpm135,					/* rpm - 135 */
	rpm136,					/* rpm - 136 */
	rpm137,					/* rpm - 137 */
	rpm138,					/* rpm - 138 */
	rpm139,					/* rpm - 139 */
	rpm140,					/* rpm - 140 */
	rpm141,					/* rpm - 141 */
	rpm142,					/* rpm - 142 */
	rpm143,					/* rpm - 143 */
	rpm144,					/* rpm - 144 */
	rpm145,					/* rpm - 145 */
	rpm146,					/* rpm - 146 */
	rpm147,					/* rpm - 147 */
	rpm148,					/* rpm - 148 */
	rpm149,					/* rpm - 149 */
	rpm150,					/* rpm - 150 */
	rpm151,					/* rpm - 151 */
	rpm152,					/* rpm - 152 */
	rpm153,					/* rpm - 153 */
	rpm154,					/* rpm - 154 */
	rpm155,					/* rpm - 155 */
	rpm156,					/* rpm - 156 */
	rpm157,					/* rpm - 157 */
	rpm158,					/* rpm - 158 */
	rpm159,					/* rpm - 159 */
	rpm160,					/* rpm - 160 */
	rpm161,					/* rpm - 161 */
	rpm162,					/* rpm - 162 */
	rpm163,					/* rpm - 163 */
	rpm164,					/* rpm - 164 */
	rpm165,					/* rpm - 165 */
	rpm166,					/* rpm - 166 */
	rpm167,					/* rpm - 167 */
	rpm168,					/* rpm - 168 */
	rpm169,					/* rpm - 169 */
	rpm170,					/* rpm - 170 */
	rpm171,					/* rpm - 171 */
	rpm172,					/* rpm - 172 */
	rpm173,					/* rpm - 173 */
	rpm174,					/* rpm - 174 */
	rpm175,					/* rpm - 175 */
	rpm176,					/* rpm - 176 */
	rpm177,					/* rpm - 177 */
	rpm178,					/* rpm - 178 */
	rpm179,					/* rpm - 179 */
	rpm180,					/* rpm - 180 */
	rpm181,					/* rpm - 181 */
	rpm182,					/* rpm - 182 */
	rpm183,					/* rpm - 183 */
	rpm184,					/* rpm - 184 */
	rpm185,					/* rpm - 185 */
	rpm186,					/* rpm - 186 */
	rpm187,					/* rpm - 187 */
	rpm188,					/* rpm - 188 */
	rpm189,					/* rpm - 189 */
	rpm190,					/* rpm - 190 */
	rpm191,					/* rpm - 191 */
	rpm192,					/* rpm - 192 */
	rpm193,					/* rpm - 193 */
	rpm194,					/* rpm - 194 */
	rpm195,					/* rpm - 195 */
	rpm196,					/* rpm - 196 */
	rpm197,					/* rpm - 197 */
	rpm198,					/* rpm - 198 */
	rpm199,					/* rpm - 199 */
	MAIN_RPM_NUM				/* number of layers of '<CPU>MAIN_RPM' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MAIN_RPM_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MAIN_RPM_GT_SIZE		(0x0954)	/* Data size of General Table */
extern const U08 MAIN_RPM_GT[199][12];		/* <CPU>MAIN_RPM */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MAIN_RPM_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MAIN_RPM_EL_SIZE		(0x012A)	/* Data size of Extended Layer */
extern const U08 MAIN_RPM_EL[298];		/* <CPU>MAIN_RPM */
#endif



enum E_MODE000 {		/* <CPU>MODE000 */
	MODE000_BG_1 = 0,				/* MODE000_BG_1 */
	MODE000_TITLE2,					/* MODE000_TITLE2 */
	MODE000_TITLE1,					/* MODE000_TITLE1 */
	Main_1_1_0,					/* Main - _1_1_0 */
	Main_1_1_1,					/* Main - _1_1_1 */
	Main_1_1_2,					/* Main - _1_1_2 */
	Main_1_1_3,					/* Main - _1_1_3 */
	Main_1_1_4,					/* Main - _1_1_4 */
	Main_1_1_5,					/* Main - _1_1_5 */
	Main_1_1_6,					/* Main - _1_1_6 */
	MODE000_NUM				/* number of layers of '<CPU>MODE000' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE000_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE000_GT_SIZE		(0x0078)	/* Data size of General Table */
extern const U08 MODE000_GT[10][12];		/* <CPU>MODE000 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE000_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE000_EL_SIZE		(0x000E)	/* Data size of Extended Layer */
extern const U08 MODE000_EL[14];		/* <CPU>MODE000 */
#endif



enum E_MODE1xx {		/* <CPU>MODE1xx */
	MODEbg_003 = 0,				/* MODEbg_003 */
	page_normal_text,					/* page_normal_text */
	page_abnormal_text,					/* page_abnormal_text */
	page_11_0,					/* page_11_ - 0 */
	page_11_1,					/* page_11_ - 1 */
	page_11_2,					/* page_11_ - 2 */
	page_11_3,					/* page_11_ - 3 */
	page_11_4,					/* page_11_ - 4 */
	page_11_5,					/* page_11_ - 5 */
	page_11_6,					/* page_11_ - 6 */
	MODE1xx_NUM				/* number of layers of '<CPU>MODE1xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1xx_GT_SIZE		(0x0078)	/* Data size of General Table */
extern const U08 MODE1xx_GT[10][12];		/* <CPU>MODE1xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1xx_EL_SIZE		(0x000E)	/* Data size of Extended Layer */
extern const U08 MODE1xx_EL[14];		/* <CPU>MODE1xx */
#endif



enum E_MODE1111 {		/* <CPU>MODE1111 */
	MODE1111_BG_1 = 0,				/* MODE1111_BG_1 */
	page_111_0,					/* page_111_ - 0 */
	page_111_1,					/* page_111_ - 1 */
	page_111_2,					/* page_111_ - 2 */
	page_111_3,					/* page_111_ - 3 */
	MODE1111_NUM				/* number of layers of '<CPU>MODE1111' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1111_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1111_GT_SIZE		(0x003C)	/* Data size of General Table */
extern const U08 MODE1111_GT[5][12];		/* <CPU>MODE1111 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1111_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1111_EL_SIZE		(0x0007)	/* Data size of Extended Layer */
extern const U08 MODE1111_EL[7];		/* <CPU>MODE1111 */
#endif



enum E_MODE1112 {		/* <CPU>MODE1112 */
	MODE1112_BG_1 = 0,				/* MODE1112_BG_1 */
	page_112_0,					/* page_112_ - 0 */
	page_112_1,					/* page_112_ - 1 */
	page_112_2,					/* page_112_ - 2 */
	MODE1112_NUM				/* number of layers of '<CPU>MODE1112' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1112_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1112_GT_SIZE		(0x0030)	/* Data size of General Table */
extern const U08 MODE1112_GT[4][12];		/* <CPU>MODE1112 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1112_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1112_EL_SIZE		(0x0005)	/* Data size of Extended Layer */
extern const U08 MODE1112_EL[5];		/* <CPU>MODE1112 */
#endif



enum E_MODE1113 {		/* <CPU>MODE1113 */
	MODE1113_BG_1 = 0,				/* MODE1113_BG_1 */
	page_113_0,					/* page_113_ - 0 */
	page_113_1,					/* page_113_ - 1 */
	page_113_2,					/* page_113_ - 2 */
	page_113_3,					/* page_113_ - 3 */
	MODE1113_NUM				/* number of layers of '<CPU>MODE1113' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1113_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1113_GT_SIZE		(0x003C)	/* Data size of General Table */
extern const U08 MODE1113_GT[5][12];		/* <CPU>MODE1113 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1113_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1113_EL_SIZE		(0x0007)	/* Data size of Extended Layer */
extern const U08 MODE1113_EL[7];		/* <CPU>MODE1113 */
#endif



enum E_MODE1114 {		/* <CPU>MODE1114 */
	MODE1114_BG_1 = 0,				/* MODE1114_BG_1 */
	page114_text,					/* <GRP30><TXT5>page114_text */
	page_114_0,					/* page_114_ - 0 */
	page_114_1,					/* page_114_ - 1 */
	page_114_2,					/* page_114_ - 2 */
	MODE1114_NUM				/* number of layers of '<CPU>MODE1114' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1114_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1114_GT_SIZE		(0x003C)	/* Data size of General Table */
extern const U08 MODE1114_GT[5][12];		/* <CPU>MODE1114 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1114_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1114_EL_SIZE		(0x0007)	/* Data size of Extended Layer */
extern const U08 MODE1114_EL[7];		/* <CPU>MODE1114 */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL page114_text_FC;		/* <GRP30><TXT5>page114_text */




enum E_MODE1115 {		/* <CPU>MODE1115 */
	MODE1115_BG_1 = 0,				/* MODE1115_BG_1 */
	page115_text2,					/* <GRP32><TXT5>page115_text2 */
	page115_text1,					/* <GRP31><TXT5>page115_text1 */
	page_115_0,					/* page_115_ - 0 */
	page_115_1,					/* page_115_ - 1 */
	page_115_2,					/* page_115_ - 2 */
	page_115_1_1,					/* page_115_1_ - 1 */
	page_115_1_2,					/* page_115_1_ - 2 */
	page_115_1_3,					/* page_115_1_ - 3 */
	page_115_2_1,					/* page_115_2_ - 1 */
	page_115_2_2,					/* page_115_2_ - 2 */
	page_115_2_3,					/* page_115_2_ - 3 */
	page_115_3_1,					/* page_115_3_ - 1 */
	page_115_3_2,					/* page_115_3_ - 2 */
	page_115_3_3,					/* page_115_3_ - 3 */
	page_115_4_1,					/* page_115_4_ - 1 */
	page_115_4_2,					/* page_115_4_ - 2 */
	page_115_4_3,					/* page_115_4_ - 3 */
	page_115_5_1,					/* page_115_5_ - 1 */
	page_115_5_2,					/* page_115_5_ - 2 */
	page_115_5_3,					/* page_115_5_ - 3 */
	page_115_6_1,					/* page_115_6_ - 1 */
	page_115_6_2,					/* page_115_6_ - 2 */
	page_115_6_3,					/* page_115_6_ - 3 */
	page_115_7_1,					/* page_115_7_ - 1 */
	page_115_7_2,					/* page_115_7_ - 2 */
	page_115_7_3,					/* page_115_7_ - 3 */
	page_115_8_1,					/* page_115_8_ - 1 */
	page_115_8_2,					/* page_115_8_ - 2 */
	page_115_8_3,					/* page_115_8_ - 3 */
	page_115_9_1,					/* page_115_9_ - 1 */
	page_115_9_2,					/* page_115_9_ - 2 */
	page_115_9_3,					/* page_115_9_ - 3 */
	page_115_10_1,					/* page_115_10_ - 1 */
	page_115_10_2,					/* page_115_10_ - 2 */
	page_115_10_3,					/* page_115_10_ - 3 */
	page_115_11_1,					/* page_115_11_ - 1 */
	page_115_11_2,					/* page_115_11_ - 2 */
	page_115_11_3,					/* page_115_11_ - 3 */
	page_115_12_1,					/* page_115_12_ - 1 */
	MODE1115_NUM				/* number of layers of '<CPU>MODE1115' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1115_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1115_GT_SIZE		(0x01E0)	/* Data size of General Table */
extern const U08 MODE1115_GT[40][12];		/* <CPU>MODE1115 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1115_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1115_EL_SIZE		(0x003B)	/* Data size of Extended Layer */
extern const U08 MODE1115_EL[59];		/* <CPU>MODE1115 */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL page115_text2_FC;		/* <GRP32><TXT5>page115_text2 */

extern const T_YGFONTCTRL page115_text1_FC;		/* <GRP31><TXT5>page115_text1 */




enum E_MODE1116 {		/* <CPU>MODE1116 */
	MODE1116_BG_1 = 0,				/* MODE1116_BG_1 */
	page_116_0,					/* page_116_ - 0 */
	page_116_1,					/* page_116_ - 1 */
	page_116_1_1,					/* page_116_1_ - 1 */
	page_116_2_1,					/* page_116_2_ - 1 */
	page_116_3_1,					/* page_116_3_ - 1 */
	MODE1116_NUM				/* number of layers of '<CPU>MODE1116' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE1116_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE1116_GT_SIZE		(0x0048)	/* Data size of General Table */
extern const U08 MODE1116_GT[6][12];		/* <CPU>MODE1116 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE1116_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE1116_EL_SIZE		(0x0008)	/* Data size of Extended Layer */
extern const U08 MODE1116_EL[8];		/* <CPU>MODE1116 */
#endif



enum E_MODE2xx {		/* <CPU>MODE2xx */
	MODE2xx_BG_1 = 0,				/* MODE2xx_BG_1 */
	Setting_value_change_completed,					/* Setting_value_change_completed */
	value,					/* <GRP1><TXT5>value */
	page_12_0,					/* page_12_ - 0 */
	page_12_1,					/* page_12_ - 1 */
	page_12_2,					/* page_12_ - 2 */
	page_12_3,					/* page_12_ - 3 */
	page_12_4,					/* page_12_ - 4 */
	page_12_5,					/* page_12_ - 5 */
	page_12_6,					/* page_12_ - 6 */
	page_121_1_text,					/* page_121_ - 1_ - text */
	page_121_2_text,					/* page_121_ - 2_ - text */
	page_121_3_text,					/* page_121_ - 3_ - text */
	page_121_4_text,					/* page_121_ - 4_ - text */
	page_121_4_2,					/* page_121_ - 4_ - 2 */
	page_121_4_3,					/* page_121_ - 4_ - 3 */
	page_121_5_text,					/* page_121_ - 5_ - text */
	page_121_5_2,					/* page_121_ - 5_ - 2 */
	page_121_5_3,					/* page_121_ - 5_ - 3 */
	page_121_6_text,					/* page_121_ - 6_ - text */
	page_121_6_2,					/* page_121_ - 6_ - 2 */
	page_121_6_3,					/* page_121_ - 6_ - 3 */
	page_121_7_text,					/* page_121_ - 7_ - text */
	page_121_8_text,					/* page_121_ - 8_ - text */
	page_121_8_2,					/* page_121_ - 8_ - 2 */
	page_121_8_3,					/* page_121_ - 8_ - 3 */
	page_121_0,					/* page_121_ - 0 */
	page_122_1_text,					/* page_122_ - 1_ - text */
	page_122_2_text,					/* page_122_ - 2_ - text */
	page_122_3_text,					/* page_122_ - 3_ - text */
	page_122_4_text,					/* page_122_ - 4_ - text */
	page_122_5_text,					/* page_122_ - 5_ - text */
	page_122_6_text,					/* page_122_ - 6_ - text */
	page_122_6_2,					/* page_122_ - 6_ - 2 */
	page_122_6_3,					/* page_122_ - 6_ - 3 */
	page_122_7_text,					/* page_122_ - 7_ - text */
	page_122_7_2,					/* page_122_ - 7_ - 2 */
	page_122_7_3,					/* page_122_ - 7_ - 3 */
	page_122_0,					/* page_122_ - 0 */
	page_123_1_text,					/* page_123_ - 1_ - text */
	page_123_2_text,					/* page_123_ - 2_ - text */
	page_123_3_text,					/* page_123_ - 3_ - text */
	page_123_4_text,					/* page_123_ - 4_ - text */
	page_123_5_text,					/* page_123_ - 5_ - text */
	page_123_6_text,					/* page_123_ - 6_ - text */
	page_123_7_text,					/* page_123_ - 7_ - text */
	page_123_8_text,					/* page_123_ - 8_ - text */
	page_123_0,					/* page_123_ - 0 */
	page_124_1_text,					/* page_124_ - 1_ - text */
	page_124_0,					/* page_124_ - 0 */
	page_125_10_0,					/* page_125_ - 10_ - 0 */
	page_125_10_1,					/* page_125_ - 10_ - 1 */
	page_125_10_2,					/* page_125_ - 10_ - 2 */
	page_125_9_0,					/* page_125_ - 9_ - 0 */
	page_125_8_0,					/* page_125_ - 8_ - 0 */
	page_125_7_0,					/* page_125_ - 7_ - 0 */
	page_125_6_0,					/* page_125_ - 6_ - 0 */
	page_125_6_1,					/* page_125_ - 6_ - 1 */
	page_125_6_2,					/* page_125_ - 6_ - 2 */
	page_125_5_0,					/* page_125_ - 5_ - 0 */
	page_125_4_0,					/* page_125_ - 4_ - 0 */
	page_125_3_0,					/* page_125_ - 3_ - 0 */
	page_125_3_1,					/* page_125_ - 3_ - 1 */
	page_125_3_2,					/* page_125_ - 3_ - 2 */
	page_125_2_0,					/* page_125_ - 2_ - 0 */
	page_125_2_1,					/* page_125_ - 2_ - 1 */
	page_125_2_2,					/* page_125_ - 2_ - 2 */
	page_125_1_0,					/* page_125_ - 1_ - 0 */
	page_125_1_1,					/* page_125_ - 1_ - 1 */
	page_125_1_2,					/* page_125_ - 1_ - 2 */
	page_125_1_3,					/* page_125_ - 1_ - 3 */
	page_125_1_4,					/* page_125_ - 1_ - 4 */
	page_125_0,					/* page_125_ - 0 */
	page_125_1,					/* page_125_ - 1 */
	page_126_1_text,					/* page_126_ - 1_ - text */
	page_126_2_text,					/* page_126_ - 2_ - text */
	page_126_0,					/* page_126_ - 0 */
	MODE2xx_NUM				/* number of layers of '<CPU>MODE2xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE2xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE2xx_GT_SIZE		(0x039C)	/* Data size of General Table */
extern const U08 MODE2xx_GT[77][12];		/* <CPU>MODE2xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE2xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE2xx_EL_SIZE		(0x0073)	/* Data size of Extended Layer */
extern const U08 MODE2xx_EL[115];		/* <CPU>MODE2xx */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL value_FC;		/* <GRP1><TXT5>value */




enum E_MODE3xx {		/* <CPU>MODE3xx */
	MODE3xx_BG_1 = 0,				/* MODE3xx_BG_1 */
	page_13_0,					/* page_13_ - 0 */
	page_13_1,					/* page_13_ - 1 */
	page_13_2,					/* page_13_ - 2 */
	page_13_3,					/* page_13_ - 3 */
	page_13_4,					/* page_13_ - 4 */
	page_13_5,					/* page_13_ - 5 */
	page_13_6,					/* page_13_ - 6 */
	MODE3xx_NUM				/* number of layers of '<CPU>MODE3xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE3xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE3xx_GT_SIZE		(0x0060)	/* Data size of General Table */
extern const U08 MODE3xx_GT[8][12];		/* <CPU>MODE3xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE3xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE3xx_EL_SIZE		(0x000B)	/* Data size of Extended Layer */
extern const U08 MODE3xx_EL[11];		/* <CPU>MODE3xx */
#endif



enum E_MODE31x {		/* <CPU>MODE31x */
	MODE31x_BG_1 = 0,				/* MODE31x_BG_1 */
	can_raw_data_1,					/* can_raw_data_ - <GRP6><TXT5>1 */
	can_raw_data_2,					/* can_raw_data_ - <GRP7><TXT5>2 */
	can_raw_data_3,					/* can_raw_data_ - <GRP8><TXT5>3 */
	can_raw_data_4,					/* can_raw_data_ - <GRP9><TXT5>4 */
	can_raw_data_5,					/* can_raw_data_ - <GRP10><TXT5>5 */
	can_raw_data_6,					/* can_raw_data_ - <GRP11><TXT5>6 */
	can_raw_data_7,					/* can_raw_data_ - <GRP12><TXT5>7 */
	can_raw_data_8,					/* can_raw_data_ - <GRP13><TXT5>8 */
	can_raw_data_9,					/* can_raw_data_ - <GRP14><TXT5>9 */
	can_raw_data_10,					/* can_raw_data_ - <GRP15><TXT5>10 */
	can_raw_data_11,					/* can_raw_data_ - <GRP16><TXT5>11 */
	can_raw_data_12,					/* can_raw_data_ - <GRP17><TXT5>12 */
	page_131_0,					/* page_131_ - 0 */
	page_131_1,					/* page_131_ - 1 */
	page_131_2,					/* page_131_ - 2 */
	page_131_3,					/* page_131_ - 3 */
	page_131_4,					/* page_131_ - 4 */
	page_131_5,					/* page_131_ - 5 */
	MODE31x_NUM				/* number of layers of '<CPU>MODE31x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE31x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE31x_GT_SIZE		(0x00E4)	/* Data size of General Table */
extern const U08 MODE31x_GT[19][12];		/* <CPU>MODE31x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE31x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE31x_EL_SIZE		(0x001C)	/* Data size of Extended Layer */
extern const U08 MODE31x_EL[28];		/* <CPU>MODE31x */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL can_raw_data_1_FC;		/* can_raw_data_ - <GRP6><TXT5>1 */

extern const T_YGFONTCTRL can_raw_data_2_FC;		/* can_raw_data_ - <GRP7><TXT5>2 */

extern const T_YGFONTCTRL can_raw_data_3_FC;		/* can_raw_data_ - <GRP8><TXT5>3 */

extern const T_YGFONTCTRL can_raw_data_4_FC;		/* can_raw_data_ - <GRP9><TXT5>4 */

extern const T_YGFONTCTRL can_raw_data_5_FC;		/* can_raw_data_ - <GRP10><TXT5>5 */

extern const T_YGFONTCTRL can_raw_data_6_FC;		/* can_raw_data_ - <GRP11><TXT5>6 */

extern const T_YGFONTCTRL can_raw_data_7_FC;		/* can_raw_data_ - <GRP12><TXT5>7 */

extern const T_YGFONTCTRL can_raw_data_8_FC;		/* can_raw_data_ - <GRP13><TXT5>8 */

extern const T_YGFONTCTRL can_raw_data_9_FC;		/* can_raw_data_ - <GRP14><TXT5>9 */

extern const T_YGFONTCTRL can_raw_data_10_FC;		/* can_raw_data_ - <GRP15><TXT5>10 */

extern const T_YGFONTCTRL can_raw_data_11_FC;		/* can_raw_data_ - <GRP16><TXT5>11 */

extern const T_YGFONTCTRL can_raw_data_12_FC;		/* can_raw_data_ - <GRP17><TXT5>12 */




enum E_MODE311 {		/* <CPU>MODE311 */
	MODE311_BG_1 = 0,				/* MODE311_BG_1 */
	page_1311_0,					/* page_1311_ - 0 */
	page_1311_1,					/* page_1311_ - 1 */
	page_1311_2,					/* page_1311_ - 2 */
	page_1311_3,					/* page_1311_ - 3 */
	page_13111_0,					/* page_13111_ - 0 */
	page_13111_1,					/* page_13111_ - 1 */
	page_13111_2,					/* page_13111_ - 2 */
	page_13111_3,					/* page_13111_ - 3 */
	page_13111_4,					/* page_13111_ - 4 */
	page_13111_5,					/* page_13111_ - 5 */
	page_13111_6,					/* page_13111_ - 6 */
	page_13111_7,					/* page_13111_ - 7 */
	page_13111_8,					/* page_13111_ - 8 */
	page_13111_9,					/* page_13111_ - 9 */
	page_13111_10,					/* page_13111_ - 10 */
	page_13111_11,					/* page_13111_ - 11 */
	page_13111_12,					/* page_13111_ - 12 */
	page_13111_13,					/* page_13111_ - 13 */
	page_13111_14,					/* page_13111_ - 14 */
	page_13111_15,					/* page_13111_ - 15 */
	page_13111_16,					/* page_13111_ - 16 */
	page_13111_17,					/* page_13111_ - 17 */
	page_13111_18,					/* page_13111_ - 18 */
	page_13111_19,					/* page_13111_ - 19 */
	page_13111_20,					/* page_13111_ - 20 */
	page_13111_21,					/* page_13111_ - 21 */
	page_13111_22,					/* page_13111_ - 22 */
	page_13111_23,					/* page_13111_ - 23 */
	page_13111_24,					/* page_13111_ - 24 */
	page_13111_25,					/* page_13111_ - 25 */
	page_13111_26,					/* page_13111_ - 26 */
	page_13111_27,					/* page_13111_ - 27 */
	page_13111_28,					/* page_13111_ - 28 */
	page_13111_29,					/* page_13111_ - 29 */
	page_13111_30,					/* page_13111_ - 30 */
	page_13111_31,					/* page_13111_ - 31 */
	page_13111_32,					/* page_13111_ - 32 */
	page_13111_33,					/* page_13111_ - 33 */
	page_13111_34,					/* page_13111_ - 34 */
	page_13111_35,					/* page_13111_ - 35 */
	page_13111_36,					/* page_13111_ - 36 */
	page_13111_37,					/* page_13111_ - 37 */
	page_13111_38,					/* page_13111_ - 38 */
	page_13111_39,					/* page_13111_ - 39 */
	page_13111_40,					/* page_13111_ - 40 */
	page_13111_41,					/* page_13111_ - 41 */
	page_13111_42,					/* page_13111_ - 42 */
	page_13111_43,					/* page_13111_ - 43 */
	page_13111_44,					/* page_13111_ - 44 */
	page_13111_45,					/* page_13111_ - 45 */
	page_13111_46,					/* page_13111_ - 46 */
	page_13111_47,					/* page_13111_ - 47 */
	page_13111_48,					/* page_13111_ - 48 */
	page_13111_49,					/* page_13111_ - 49 */
	page_13111_50,					/* page_13111_ - 50 */
	page_13111_51,					/* page_13111_ - 51 */
	page_13111_52,					/* page_13111_ - 52 */
	page_13111_53,					/* page_13111_ - 53 */
	page_13111_54,					/* page_13111_ - 54 */
	page_13111_55,					/* page_13111_ - 55 */
	page_13111_56,					/* page_13111_ - 56 */
	page_13111_total,					/* page_13111_ - total */
	page_13112_0,					/* page_13112_ - 0 */
	page_13112_1,					/* page_13112_ - 1 */
	page_13112_2,					/* page_13112_ - 2 */
	page_13112_3,					/* page_13112_ - 3 */
	page_13112_4,					/* page_13112_ - 4 */
	page_13112_5,					/* page_13112_ - 5 */
	page_13112_6,					/* page_13112_ - 6 */
	page_13112_7,					/* page_13112_ - 7 */
	page_13112_8,					/* page_13112_ - 8 */
	page_13112_9,					/* page_13112_ - 9 */
	page_13112_10,					/* page_13112_ - 10 */
	page_13112_11,					/* page_13112_ - 11 */
	page_13112_12,					/* page_13112_ - 12 */
	page_13112_13,					/* page_13112_ - 13 */
	page_13112_14,					/* page_13112_ - 14 */
	page_13112_15,					/* page_13112_ - 15 */
	page_13112_16,					/* page_13112_ - 16 */
	page_13112_17,					/* page_13112_ - 17 */
	page_13112_18,					/* page_13112_ - 18 */
	page_13112_19,					/* page_13112_ - 19 */
	page_13112_total,					/* page_13112_ - total */
	page_13113_0,					/* page_13113_ - 0 */
	page_13113_1,					/* page_13113_ - 1 */
	page_13113_2,					/* page_13113_ - 2 */
	page_13113_3,					/* page_13113_ - 3 */
	page_13113_4,					/* page_13113_ - 4 */
	page_13113_5,					/* page_13113_ - 5 */
	page_13113_6,					/* page_13113_ - 6 */
	page_13113_total,					/* page_13113_ - total */
	MODE311_NUM				/* number of layers of '<CPU>MODE311' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE311_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE311_GT_SIZE		(0x0450)	/* Data size of General Table */
extern const U08 MODE311_GT[92][12];		/* <CPU>MODE311 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE311_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE311_EL_SIZE		(0x0089)	/* Data size of Extended Layer */
extern const U08 MODE311_EL[137];		/* <CPU>MODE311 */
#endif



enum E_MODE312 {		/* <CPU>MODE312 */
	MODE312_BG_1 = 0,				/* MODE312_BG_1 */
	page_1312_0,					/* page_1312_ - 0 */
	page_1312_1,					/* page_1312_ - 1 */
	page_1312_2,					/* page_1312_ - 2 */
	page_1312_3,					/* page_1312_ - 3 */
	page_1312_4,					/* page_1312_ - 4 */
	page_1312_5,					/* page_1312_ - 5 */
	page_1312_6,					/* page_1312_ - 6 */
	page_1312_7,					/* page_1312_ - 7 */
	page_1312_8,					/* page_1312_ - 8 */
	page_1312_9,					/* page_1312_ - 9 */
	page_1312_total,					/* page_1312_ - total */
	MODE312_NUM				/* number of layers of '<CPU>MODE312' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE312_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE312_GT_SIZE		(0x0090)	/* Data size of General Table */
extern const U08 MODE312_GT[12][12];		/* <CPU>MODE312 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE312_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE312_EL_SIZE		(0x0011)	/* Data size of Extended Layer */
extern const U08 MODE312_EL[17];		/* <CPU>MODE312 */
#endif



enum E_MODE313 {		/* <CPU>MODE313 */
	MODE313_BG_1 = 0,				/* MODE313_BG_1 */
	page_1313_0,					/* page_1313_ - 0 */
	page_1313_1,					/* page_1313_ - 1 */
	page_1313_2,					/* page_1313_ - 2 */
	page_1313_3,					/* page_1313_ - 3 */
	page_1313_4,					/* page_1313_ - 4 */
	page_1313_5,					/* page_1313_ - 5 */
	page_1313_6,					/* page_1313_ - 6 */
	page_1313_7,					/* page_1313_ - 7 */
	page_1313_8,					/* page_1313_ - 8 */
	page_1313_total,					/* page_1313_ - total */
	MODE313_NUM				/* number of layers of '<CPU>MODE313' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE313_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE313_GT_SIZE		(0x0084)	/* Data size of General Table */
extern const U08 MODE313_GT[11][12];		/* <CPU>MODE313 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE313_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE313_EL_SIZE		(0x0010)	/* Data size of Extended Layer */
extern const U08 MODE313_EL[16];		/* <CPU>MODE313 */
#endif



enum E_MODE314 {		/* <CPU>MODE314 */
	MODE314_BG_1 = 0,				/* MODE314_BG_1 */
	page_1314_0,					/* page_1314_ - 0 */
	page_1314_1,					/* page_1314_ - 1 */
	page_1314_2,					/* page_1314_ - 2 */
	page_1314_3,					/* page_1314_ - 3 */
	page_13143_0,					/* page_13143_ - 0 */
	page_13143_1,					/* page_13143_ - 1 */
	page_13143_2,					/* page_13143_ - 2 */
	page_13143_3,					/* page_13143_ - 3 */
	page_13143_4,					/* page_13143_ - 4 */
	page_13142_0,					/* page_13142_ - 0 */
	page_13142_1,					/* page_13142_ - 1 */
	page_13142_2,					/* page_13142_ - 2 */
	page_13142_3,					/* page_13142_ - 3 */
	page_13142_4,					/* page_13142_ - 4 */
	page_13142_total,					/* page_13142_ - total */
	page_13141_0,					/* page_13141_ - 0 */
	page_13141_1,					/* page_13141_ - 1 */
	page_13141_2,					/* page_13141_ - 2 */
	page_13141_3,					/* page_13141_ - 3 */
	page_13141_4,					/* page_13141_ - 4 */
	page_13141_5,					/* page_13141_ - 5 */
	page_13141_6,					/* page_13141_ - 6 */
	page_13141_7,					/* page_13141_ - 7 */
	page_13141_8,					/* page_13141_ - 8 */
	page_13141_9,					/* page_13141_ - 9 */
	page_13141_10,					/* page_13141_ - 10 */
	page_13141_11,					/* page_13141_ - 11 */
	page_13141_12,					/* page_13141_ - 12 */
	page_13141_13,					/* page_13141_ - 13 */
	page_13141_14,					/* page_13141_ - 14 */
	page_13141_15,					/* page_13141_ - 15 */
	page_13141_16,					/* page_13141_ - 16 */
	page_13141_17,					/* page_13141_ - 17 */
	page_13141_18,					/* page_13141_ - 18 */
	page_13141_19,					/* page_13141_ - 19 */
	page_13141_20,					/* page_13141_ - 20 */
	page_13141_21,					/* page_13141_ - 21 */
	page_13141_22,					/* page_13141_ - 22 */
	page_13141_23,					/* page_13141_ - 23 */
	page_13141_24,					/* page_13141_ - 24 */
	page_13141_25,					/* page_13141_ - 25 */
	page_13141_26,					/* page_13141_ - 26 */
	page_13141_27,					/* page_13141_ - 27 */
	page_13141_28,					/* page_13141_ - 28 */
	page_13141_29,					/* page_13141_ - 29 */
	page_13141_30,					/* page_13141_ - 30 */
	page_13141_31,					/* page_13141_ - 31 */
	page_13141_32,					/* page_13141_ - 32 */
	page_13141_33,					/* page_13141_ - 33 */
	page_13141_34,					/* page_13141_ - 34 */
	page_13141_35,					/* page_13141_ - 35 */
	page_13141_total,					/* page_13141_ - total */
	MODE314_NUM				/* number of layers of '<CPU>MODE314' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE314_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE314_GT_SIZE		(0x027C)	/* Data size of General Table */
extern const U08 MODE314_GT[53][12];		/* <CPU>MODE314 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE314_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE314_EL_SIZE		(0x004F)	/* Data size of Extended Layer */
extern const U08 MODE314_EL[79];		/* <CPU>MODE314 */
#endif



enum E_MODE315 {		/* <CPU>MODE315 */
	MODE315_BG_1 = 0,				/* MODE315_BG_1 */
	page_1315_0,					/* page_1315_ - 0 */
	page_1315_1,					/* page_1315_ - 1 */
	page_1315_2,					/* page_1315_ - 2 */
	page_1315_3,					/* page_1315_ - 3 */
	page_1315_4,					/* page_1315_ - 4 */
	page_1315_5,					/* page_1315_ - 5 */
	page_1315_6,					/* page_1315_ - 6 */
	page_1315_7,					/* page_1315_ - 7 */
	page_1315_8,					/* page_1315_ - 8 */
	page_1315_9,					/* page_1315_ - 9 */
	page_1315_10,					/* page_1315_ - 10 */
	page_1315_11,					/* page_1315_ - 11 */
	page_1315_12,					/* page_1315_ - 12 */
	page_1315_13,					/* page_1315_ - 13 */
	page_1315_14,					/* page_1315_ - 14 */
	page_1315_15,					/* page_1315_ - 15 */
	page_1315_16,					/* page_1315_ - 16 */
	page_1315_17,					/* page_1315_ - 17 */
	page_1315_18,					/* page_1315_ - 18 */
	page_1315_19,					/* page_1315_ - 19 */
	page_1315_20,					/* page_1315_ - 20 */
	page_1315_21,					/* page_1315_ - 21 */
	page_1315_22,					/* page_1315_ - 22 */
	page_1315_23,					/* page_1315_ - 23 */
	page_1315_24,					/* page_1315_ - 24 */
	page_1315_25,					/* page_1315_ - 25 */
	page_1315_26,					/* page_1315_ - 26 */
	page_1315_27,					/* page_1315_ - 27 */
	page_1315_28,					/* page_1315_ - 28 */
	page_1315_29,					/* page_1315_ - 29 */
	page_1315_30,					/* page_1315_ - 30 */
	page_1315_31,					/* page_1315_ - 31 */
	page_1315_32,					/* page_1315_ - 32 */
	page_1315_33,					/* page_1315_ - 33 */
	page_1315_34,					/* page_1315_ - 34 */
	page_1315_35,					/* page_1315_ - 35 */
	page_1315_36,					/* page_1315_ - 36 */
	page_1315_37,					/* page_1315_ - 37 */
	page_1315_38,					/* page_1315_ - 38 */
	page_1315_39,					/* page_1315_ - 39 */
	page_1315_40,					/* page_1315_ - 40 */
	page_1315_41,					/* page_1315_ - 41 */
	page_1315_42,					/* page_1315_ - 42 */
	page_1315_43,					/* page_1315_ - 43 */
	page_1315_44,					/* page_1315_ - 44 */
	page_1315_45,					/* page_1315_ - 45 */
	page_1315_46,					/* page_1315_ - 46 */
	page_1315_47,					/* page_1315_ - 47 */
	page_1315_48,					/* page_1315_ - 48 */
	page_1315_49,					/* page_1315_ - 49 */
	page_1315_50,					/* page_1315_ - 50 */
	page_1315_51,					/* page_1315_ - 51 */
	page_1315_52,					/* page_1315_ - 52 */
	page_1315_53,					/* page_1315_ - 53 */
	page_1315_54,					/* page_1315_ - 54 */
	page_1315_55,					/* page_1315_ - 55 */
	page_1315_56,					/* page_1315_ - 56 */
	page_1315_57,					/* page_1315_ - 57 */
	page_1315_58,					/* page_1315_ - 58 */
	page_1315_59,					/* page_1315_ - 59 */
	page_1315_60,					/* page_1315_ - 60 */
	page_1315_61,					/* page_1315_ - 61 */
	page_1315_62,					/* page_1315_ - 62 */
	page_1315_63,					/* page_1315_ - 63 */
	page_1315_64,					/* page_1315_ - 64 */
	page_1315_65,					/* page_1315_ - 65 */
	page_1315_66,					/* page_1315_ - 66 */
	page_1315_67,					/* page_1315_ - 67 */
	page_1315_68,					/* page_1315_ - 68 */
	page_1315_69,					/* page_1315_ - 69 */
	page_1315_70,					/* page_1315_ - 70 */
	page_1315_71,					/* page_1315_ - 71 */
	page_1315_72,					/* page_1315_ - 72 */
	page_1315_73,					/* page_1315_ - 73 */
	page_1315_74,					/* page_1315_ - 74 */
	page_1315_75,					/* page_1315_ - 75 */
	page_1315_76,					/* page_1315_ - 76 */
	page_1315_77,					/* page_1315_ - 77 */
	page_1315_78,					/* page_1315_ - 78 */
	page_1315_79,					/* page_1315_ - 79 */
	page_1315_80,					/* page_1315_ - 80 */
	page_1315_81,					/* page_1315_ - 81 */
	page_1315_82,					/* page_1315_ - 82 */
	page_1315_83,					/* page_1315_ - 83 */
	page_1315_84,					/* page_1315_ - 84 */
	page_1315_85,					/* page_1315_ - 85 */
	page_1315_86,					/* page_1315_ - 86 */
	page_1315_87,					/* page_1315_ - 87 */
	page_1315_88,					/* page_1315_ - 88 */
	page_1315_89,					/* page_1315_ - 89 */
	page_1315_90,					/* page_1315_ - 90 */
	page_1315_91,					/* page_1315_ - 91 */
	page_1315_92,					/* page_1315_ - 92 */
	page_1315_93,					/* page_1315_ - 93 */
	page_1315_94,					/* page_1315_ - 94 */
	page_1315_95,					/* page_1315_ - 95 */
	page_1315_96,					/* page_1315_ - 96 */
	page_1315_97,					/* page_1315_ - 97 */
	page_1315_98,					/* page_1315_ - 98 */
	page_1315_99,					/* page_1315_ - 99 */
	page_1315_100,					/* page_1315_ - 100 */
	page_1315_101,					/* page_1315_ - 101 */
	page_1315_102,					/* page_1315_ - 102 */
	page_1315_103,					/* page_1315_ - 103 */
	page_1315_104,					/* page_1315_ - 104 */
	page_1315_105,					/* page_1315_ - 105 */
	page_1315_106,					/* page_1315_ - 106 */
	page_1315_107,					/* page_1315_ - 107 */
	page_1315_108,					/* page_1315_ - 108 */
	page_1315_109,					/* page_1315_ - 109 */
	page_1315_110,					/* page_1315_ - 110 */
	page_1315_111,					/* page_1315_ - 111 */
	page_1315_112,					/* page_1315_ - 112 */
	page_1315_113,					/* page_1315_ - 113 */
	page_1315_114,					/* page_1315_ - 114 */
	page_1315_115,					/* page_1315_ - 115 */
	page_1315_116,					/* page_1315_ - 116 */
	page_1315_117,					/* page_1315_ - 117 */
	page_1315_118,					/* page_1315_ - 118 */
	page_1315_119,					/* page_1315_ - 119 */
	page_1315_120,					/* page_1315_ - 120 */
	page_1315_121,					/* page_1315_ - 121 */
	page_1315_122,					/* page_1315_ - 122 */
	page_1315_123,					/* page_1315_ - 123 */
	page_1315_124,					/* page_1315_ - 124 */
	page_1315_125,					/* page_1315_ - 125 */
	page_1315_126,					/* page_1315_ - 126 */
	page_1315_127,					/* page_1315_ - 127 */
	page_1315_128,					/* page_1315_ - 128 */
	page_1315_129,					/* page_1315_ - 129 */
	page_1315_130,					/* page_1315_ - 130 */
	page_1315_131,					/* page_1315_ - 131 */
	page_1315_132,					/* page_1315_ - 132 */
	page_1315_133,					/* page_1315_ - 133 */
	page_1315_134,					/* page_1315_ - 134 */
	page_1315_135,					/* page_1315_ - 135 */
	page_1315_136,					/* page_1315_ - 136 */
	page_1315_137,					/* page_1315_ - 137 */
	page_1315_138,					/* page_1315_ - 138 */
	page_1315_139,					/* page_1315_ - 139 */
	page_1315_140,					/* page_1315_ - 140 */
	page_1315_141,					/* page_1315_ - 141 */
	page_1315_total,					/* page_1315_ - total */
	MODE315_NUM				/* number of layers of '<CPU>MODE315' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE315_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE315_GT_SIZE		(0x06C0)	/* Data size of General Table */
extern const U08 MODE315_GT[144][12];		/* <CPU>MODE315 */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE315_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE315_EL_SIZE		(0x00D7)	/* Data size of Extended Layer */
extern const U08 MODE315_EL[215];		/* <CPU>MODE315 */
#endif



enum E_MODE32x {		/* <CPU>MODE32x */
	MODE32x_BG_1 = 0,				/* MODE32x_BG_1 */
	can_data_ck1,					/* <LYR1><GRP18><TXT5>can_data_ck1 */
	can_data_ck2,					/* <LYR1><GRP19><TXT6>can_data_ck2 */
	can_data_ck3,					/* <LYR1><GRP20><TXT5>can_data_ck3 */
	can_data_ck4,					/* <LYR1><GRP21><TXT6>can_data_ck4 */
	can_data_ck5,					/* <LYR1><GRP22><TXT6>can_data_ck5 */
	page_132_0,					/* page_132_ - 0 */
	page_132_1,					/* page_132_ - 1 */
	page_132_2,					/* page_132_ - 2 */
	page_132_3,					/* page_132_ - 3 */
	page_132_4,					/* page_132_ - 4 */
	page_132_5,					/* page_132_ - 5 */
	page_132_6,					/* page_132_ - 6 */
	page_132_7,					/* page_132_ - 7 */
	page_132_8,					/* page_132_ - 8 */
	page_132_9,					/* page_132_ - 9 */
	page_132_10,					/* page_132_ - 10 */
	page_132_11,					/* page_132_ - 11 */
	page_132_12,					/* page_132_ - 12 */
	page_132_13,					/* page_132_ - 13 */
	page_132_14,					/* page_132_ - 14 */
	page_132_16,					/* page_132_ - 16 */
	page_132_17,					/* page_132_ - 17 */
	page_132_18,					/* page_132_ - 18 */
	page_132_19,					/* page_132_ - 19 */
	page_132_20,					/* page_132_ - 20 */
	page_132_21,					/* page_132_ - 21 */
	page_132_22,					/* page_132_ - 22 */
	page_132_23,					/* page_132_ - 23 */
	page_132_24,					/* page_132_ - 24 */
	MODE32x_NUM				/* number of layers of '<CPU>MODE32x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE32x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE32x_GT_SIZE		(0x0168)	/* Data size of General Table */
extern const U08 MODE32x_GT[30][12];		/* <CPU>MODE32x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE32x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE32x_EL_SIZE		(0x002C)	/* Data size of Extended Layer */
extern const U08 MODE32x_EL[44];		/* <CPU>MODE32x */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL can_data_ck1_FC;		/* <LYR1><GRP18><TXT5>can_data_ck1 */

extern const T_YGFONTCTRL can_data_ck2_FC;		/* <LYR1><GRP19><TXT6>can_data_ck2 */

extern const T_YGFONTCTRL can_data_ck3_FC;		/* <LYR1><GRP20><TXT5>can_data_ck3 */

extern const T_YGFONTCTRL can_data_ck4_FC;		/* <LYR1><GRP21><TXT6>can_data_ck4 */

extern const T_YGFONTCTRL can_data_ck5_FC;		/* <LYR1><GRP22><TXT6>can_data_ck5 */




enum E_MODE33x {		/* <CPU>MODE33x */
	MODE33x_BG_1 = 0,				/* MODE33x_BG_1 */
	can_data_eg4,					/* <LYR1><GRP23><TXT5>can_data_eg4 */
	can_data_eg3,					/* <LYR1><GRP24><TXT5>can_data_eg3 */
	can_data_eg2,					/* <LYR1><GRP25><TXT5>can_data_eg2 */
	can_data_eg1,					/* <LYR1><GRP26><TXT5>can_data_eg1 */
	page_1332_1,					/* page_1332_ - 1 */
	page_1331_1,					/* page_1331_ - 1 */
	page_133_0,					/* page_133_ - 0 */
	page_133_1,					/* page_133_ - 1 */
	page_133_2,					/* page_133_ - 2 */
	page_133_3,					/* page_133_ - 3 */
	page_133_4,					/* page_133_ - 4 */
	page_133_5,					/* page_133_ - 5 */
	page_133_6,					/* page_133_ - 6 */
	page_133_7,					/* page_133_ - 7 */
	page_133_8,					/* page_133_ - 8 */
	MODE33x_NUM				/* number of layers of '<CPU>MODE33x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE33x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE33x_GT_SIZE		(0x00C0)	/* Data size of General Table */
extern const U08 MODE33x_GT[16][12];		/* <CPU>MODE33x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE33x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE33x_EL_SIZE		(0x0017)	/* Data size of Extended Layer */
extern const U08 MODE33x_EL[23];		/* <CPU>MODE33x */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL can_data_eg4_FC;		/* <LYR1><GRP23><TXT5>can_data_eg4 */

extern const T_YGFONTCTRL can_data_eg3_FC;		/* <LYR1><GRP24><TXT5>can_data_eg3 */

extern const T_YGFONTCTRL can_data_eg2_FC;		/* <LYR1><GRP25><TXT5>can_data_eg2 */

extern const T_YGFONTCTRL can_data_eg1_FC;		/* <LYR1><GRP26><TXT5>can_data_eg1 */




enum E_MODE34x {		/* <CPU>MODE34x */
	MODE34x_BG_1 = 0,				/* MODE34x_BG_1 */
	page_134_0,					/* page_134_ - 0 */
	page_134_1,					/* page_134_ - 1 */
	page_134_2,					/* page_134_ - 2 */
	page_134_3,					/* page_134_ - 3 */
	page_134_4,					/* page_134_ - 4 */
	page_134_5,					/* page_134_ - 5 */
	page_134_6,					/* page_134_ - 6 */
	page_134_7,					/* page_134_ - 7 */
	page_134_8,					/* page_134_ - 8 */
	page_134_9,					/* page_134_ - 9 */
	page_134_10,					/* page_134_ - 10 */
	page_134_11,					/* page_134_ - 11 */
	page_134_12,					/* page_134_ - 12 */
	MODE34x_NUM				/* number of layers of '<CPU>MODE34x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE34x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE34x_GT_SIZE		(0x00A8)	/* Data size of General Table */
extern const U08 MODE34x_GT[14][12];		/* <CPU>MODE34x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE34x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE34x_EL_SIZE		(0x0014)	/* Data size of Extended Layer */
extern const U08 MODE34x_EL[20];		/* <CPU>MODE34x */
#endif



enum E_MODE35x {		/* <CPU>MODE35x */
	MODE35x_BG_1 = 0,				/* MODE35x_BG_1 */
	page_135_0,					/* page_135_ - 0 */
	page_135_1,					/* page_135_ - 1 */
	page_135_2,					/* page_135_ - 2 */
	page_135_3,					/* page_135_ - 3 */
	page_135_4,					/* page_135_ - 4 */
	page_135_5,					/* page_135_ - 5 */
	page_135_6,					/* page_135_ - 6 */
	page_135_7,					/* page_135_ - 7 */
	page_135_8,					/* page_135_ - 8 */
	MODE35x_NUM				/* number of layers of '<CPU>MODE35x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE35x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE35x_GT_SIZE		(0x0078)	/* Data size of General Table */
extern const U08 MODE35x_GT[10][12];		/* <CPU>MODE35x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE35x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE35x_EL_SIZE		(0x000E)	/* Data size of Extended Layer */
extern const U08 MODE35x_EL[14];		/* <CPU>MODE35x */
#endif



enum E_MODE36x {		/* <CPU>MODE36x */
	MODE36x_BG_1 = 0,				/* MODE36x_BG_1 */
	page_1361_0,					/* page_1361_ - 0 */
	page_1361_1,					/* page_1361_ - 1 */
	page_1361_2,					/* page_1361_ - 2 */
	page_136_0,					/* page_136_ - 0 */
	MODE36x_NUM				/* number of layers of '<CPU>MODE36x' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE36x_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE36x_GT_SIZE		(0x003C)	/* Data size of General Table */
extern const U08 MODE36x_GT[5][12];		/* <CPU>MODE36x */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE36x_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE36x_EL_SIZE		(0x0007)	/* Data size of Extended Layer */
extern const U08 MODE36x_EL[7];		/* <CPU>MODE36x */
#endif



enum E_MODE4xx {		/* <CPU>MODE4xx */
	MODE4xx_BG_1 = 0,				/* MODE4xx_BG_1 */
	page_14_1,					/* page_14_ - 1 */
	MODE4xx_NUM				/* number of layers of '<CPU>MODE4xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE4xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE4xx_GT_SIZE		(0x0018)	/* Data size of General Table */
extern const U08 MODE4xx_GT[2][12];		/* <CPU>MODE4xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE4xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE4xx_EL_SIZE		(0x0002)	/* Data size of Extended Layer */
extern const U08 MODE4xx_EL[2];		/* <CPU>MODE4xx */
#endif



enum E_MODE5xx {		/* <CPU>MODE5xx */
	MODE5xx_BG_1 = 0,				/* MODE5xx_BG_1 */
	Time,					/* <GRP27><TXT11>Time */
	digits_0,					/* digits_ - 0 */
	digits_1,					/* digits_ - 1 */
	digits_2,					/* digits_ - 2 */
	digits_3,					/* digits_ - 3 */
	digits_4,					/* digits_ - 4 */
	digits_5,					/* digits_ - 5 */
	digits_6,					/* digits_ - 6 */
	digits_7,					/* digits_ - 7 */
	digits_8,					/* digits_ - 8 */
	digits_9,					/* digits_ - 9 */
	digits_bg,					/* digits_ - bg */
	page_15_0,					/* page_15_ - 0 */
	page_15_1,					/* page_15_ - 1 */
	page_15_1_1,					/* page_15_1_ - 1 */
	page_15_1_2,					/* page_15_1_ - 2 */
	page_15_1_3,					/* page_15_1_ - 3 */
	page_15_1_4,					/* page_15_1_ - 4 */
	page_15_1_5,					/* page_15_1_ - 5 */
	page_15_2_0,					/* page_15_2_ - 0 */
	page_15_2_1,					/* page_15_2_ - 1 */
	page_15_2_2,					/* page_15_2_ - 2 */
	page_15_3_0,					/* page_15_3_ - 0 */
	page_15_3_1,					/* page_15_3_ - 1 */
	page_15_3_2,					/* page_15_3_ - 2 */
	page_15_4_0,					/* page_15_4_ - 0 */
	page_15_5_0,					/* page_15_5_ - 0 */
	page_15_6_0,					/* page_15_6_ - 0 */
	MODE5xx_NUM				/* number of layers of '<CPU>MODE5xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE5xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE5xx_GT_SIZE		(0x015C)	/* Data size of General Table */
extern const U08 MODE5xx_GT[29][12];		/* <CPU>MODE5xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE5xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE5xx_EL_SIZE		(0x002B)	/* Data size of Extended Layer */
extern const U08 MODE5xx_EL[43];		/* <CPU>MODE5xx */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL Time_FC;		/* <GRP27><TXT11>Time */




enum E_MODE6xx {		/* <CPU>MODE6xx */
	MODE6xx_BG_1 = 0,				/* MODE6xx_BG_1 */
	lcd3_value,					/* <GRP35><TXT5>lcd3_value */
	lcd2_value,					/* <GRP34><TXT7>lcd2_value */
	lcd1_value,					/* <GRP33><TXT7>lcd1_value */
	page6_lcd_1,					/* page6_lcd_ - 1 */
	page6_lcd_4,					/* page6_lcd_ - 4 */
	page6_lcd_0,					/* page6_lcd_ - 0 */
	page6_lcd_2,					/* page6_lcd_ - 2 */
	page6_lcd_5,					/* page6_lcd_ - 5 */
	page6_lcd_select_1,					/* page6_lcd_ - select_1 */
	page6_lcd_select_2,					/* page6_lcd_ - select_2 */
	page6_lcd_select_3,					/* page6_lcd_ - select_3 */
	MODE6xx_NUM				/* number of layers of '<CPU>MODE6xx' */
};

/*----------------------------------------
 *	Layout Attribute Data
 *----------------------------------------*/
#define  MODE6xx_GT_ADDR		(0x0210)	/* Start address of General Table */
#define  MODE6xx_GT_SIZE		(0x0090)	/* Data size of General Table */
extern const U08 MODE6xx_GT[12][12];		/* <CPU>MODE6xx */

#if YVCX_EXLY
/*----------------------------------------
 *	Extended Layer Attribute Data
 *----------------------------------------*/
#define  MODE6xx_EL_ADDR		(0x2042)	/* Start address of Extended Layer */
#define  MODE6xx_EL_SIZE		(0x0011)	/* Data size of Extended Layer */
extern const U08 MODE6xx_EL[17];		/* <CPU>MODE6xx */
#endif

/*----------------------------------------
 *	Font Control Data
 *----------------------------------------*/
extern const T_YGFONTCTRL lcd3_value_FC;		/* <GRP35><TXT5>lcd3_value */

extern const T_YGFONTCTRL lcd2_value_FC;		/* <GRP34><TXT7>lcd2_value */

extern const T_YGFONTCTRL lcd1_value_FC;		/* <GRP33><TXT7>lcd1_value */



#endif /* _YLAYOUT_DATA_H_ */
