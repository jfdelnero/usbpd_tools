///////////////////////////////////////////////////////////////////////////////////
// File : pd_defs.c
// Contains: Power delivery messages strings definitions
//
// Written by: Jean-François DEL NERO
///////////////////////////////////////////////////////////////////////////////////

#include "pd_defs.h"

const char * pd_ctrl_msg[]=
{
	"Reserved_00",
	"GoodCRC",
	"GotoMin",
	"Accept",
	"Reject",
	"Ping",
	"PS_RDY",
	"Get_Source_Cap",
	"Get_Sink_Cap",
	"DR_Swap",
	"PR_Swap",
	"VCONN_Swap",
	"Wait",
	"Soft_Reset",
	"Reserved_14",
	"Reserved_15",
	"Not_Supported",
	"Get_Source_Cap_Extended",
	"Get_Status",
	"FR_Swap",
	"Get_PPS_Status",
	"Get_Country_Codes",
	"Get_Sink_Cap_Extented",
	"Get_Source_Info",
	"Get_Revision",
	"Reserved_25",
	"Reserved_26",
	"Reserved_27",
	"Reserved_28",
	"Reserved_29",
	"Reserved_30",
	"Reserved_31"
};

const char * pd_data_msg[]=
{
	"Reserved_00",
	"Source_Capabilities",
	"Request",
	"BIST",
	"Sink_Capabilities",
	"Battery_Status",
	"Alert",
	"Get_Country_Info",
	"Enter_USB",
	"EPR_Request",
	"EPR_Mode",
	"Source_Info",
	"Revision",
	"Reserved_13",
	"Reserved_14",
	"Vendor_Defined",
	"Reserved_16",
	"Reserved_17",
	"Reserved_18",
	"Reserved_19",
	"Reserved_20",
	"Reserved_21",
	"Reserved_22",
	"Reserved_23",
	"Reserved_24",
	"Reserved_25",
	"Reserved_26",
	"Reserved_27",
	"Reserved_28",
	"Reserved_29",
	"Reserved_30",
	"Reserved_31"
};

const char * pd_extended_msg[]=
{
	"Reserved_00",
	"SOURCE_CAPABILITIES",
	"STATUS",
	"GET_BATTERY_CAP",
	"GET_BATTERY_STATUS",
	"BATTERY_CAPABILITIES",
	"GET_MANUFACTURER_INFO",
	"MANUFACTURER_INFO",
	"SECURITY_REQUEST",
	"SECURITY_RESPONSE",
	"FIRMWARE_UPDATE_REQUEST",
	"FIRMWARE_UPDATE_RESPONSE",
	"PPS_STATUS",
	"COUNTRY_INFO",
	"COUNTRY_CODES",
	"Reserved_15",
	"Reserved_16",
	"Reserved_17",
	"Reserved_18",
	"Reserved_19",
	"Reserved_20",
	"Reserved_21",
	"Reserved_22",
	"Reserved_23",
	"Reserved_24",
	"Reserved_25",
	"Reserved_26",
	"Reserved_27",
	"Reserved_28",
	"Reserved_29",
	"Reserved_30",
	"Reserved_31"
};

// --------------------------------------
// SOP
// --------------------------------------

// SOP   : CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_2 : Communication to UFP
// SOP'  : CODE_SYNC_1->CODE_SYNC_1->CODE_SYNC_3->CODE_SYNC_3 : Communication to USB Type-C Plug Side A
// SOP'' : CODE_SYNC_1->CODE_SYNC_3->CODE_SYNC_1->CODE_SYNC_3 : Communication to USB Type-C Plug Side B

// Hard Reset  : CODE_RST_1  CODE_RST_1  CODE_RST_1  CODE_RST_2   : Resets logic in all connected PD devices (UFP and/or Active/Electronically Marked Cable)
// Cable Reset : CODE_RST_1  CODE_SYNC_1 CODE_RST_1  CODE_SYNC_3  : Reset for only Active/Electronically Marked Cable.
// SOP’_Debug  : CODE_SYNC_1 CODE_RST_2  CODE_RST_2  CODE_SYNC_3  : Used for debug of USB Type-C Plug Side A
// SOP’’_Debug : CODE_SYNC_1 CODE_RST_2  CODE_SYNC_3 CODE_SYNC_2  : Used for debug of USB Type-C Plug Side B

