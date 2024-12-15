/*******************************************************************************
*
* TITLE:	Error Handler for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	12/15/2024
*
* NOTES
* 1. Summary of LED status below.
* 		- Green		= no error
* 		- Yellow	= initialization error
* 		- Orange	= EPOS4 abort error
* 		- Red		= EPOS4 fault error
*
*******************************************************************************/

#include "error_handler.h"
#include "main.h"

typedef enum
{
	Green,
	Red
} LED_Color_e;

typedef enum
{
	NoError,
	Ankle_AS5145B_InitError,
	Ankle_EPOS4_InitError,
	Ankle_EPOS4_FaultError,
	Ankle_EPOS4_AbortError,
	Ankle_MCP25625_InitError,
	Knee_AS5145B_InitError,
	Knee_EPOS4_InitError,
	Knee_EPOS4_FaultError,
	Knee_EPOS4_AbortError,
	Knee_MCP25625_InitError,
	MPU925x_InitError
} LED_Code_e;

static LED_Color_e CM_ledColor = Green;
static LED_Code_e CM_ledCode = NoError;

void ErrorHandler_AS5145B(EncoderIndex_e deviceIndex, AS5145B_Error_e error)
{
	CM_ledColor = Red;

	if(deviceIndex == AnkleEncoderIndex)
		CM_ledCode = Ankle_AS5145B_InitError;
	else
		CM_ledCode = Knee_AS5145B_InitError;

	while(1);
}

void ErrorHandler_EPOS4(MotorControllerIndex_e deviceIndex, EPOS4_Error_e error)
{
	CM_ledColor = Red;

	if(deviceIndex == AnkleMotorControllerIndex)
	{
		if(error == EPOS4_FaultError)
			CM_ledCode = Ankle_EPOS4_FaultError;
		else if(error == EPOS4_AbortError)
			CM_ledCode = Ankle_EPOS4_AbortError;
		else
			CM_ledCode = Ankle_EPOS4_InitError;
	}
	else
	{
		if(error == EPOS4_FaultError)
			CM_ledCode = Knee_EPOS4_FaultError;
		else if(error == EPOS4_AbortError)
			CM_ledCode = Knee_EPOS4_AbortError;
		else
			CM_ledCode = Knee_EPOS4_InitError;
	}

	while(1);
}

void ErrorHandler_MCP25625(uint8_t deviceIndex, MCP25625_Error_e error)
{
	CM_ledColor = Red;

	if(deviceIndex == AnkleCAN_ControllerIndex)
		CM_ledCode = Ankle_MCP25625_InitError;
	else
		CM_ledCode = Knee_MCP25625_InitError;

	while(1);
}

void ErrorHandler_MPU925x(MPU925x_Error_e error)
{
	CM_ledColor = Red;
	CM_ledCode = MPU925x_InitError;
	while(1);
}


/*******************************************************************************
* END
*******************************************************************************/
