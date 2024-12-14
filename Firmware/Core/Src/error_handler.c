/*******************************************************************************
*
* TITLE:	Error Handler for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	??
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
	Yellow,
	Orange,
	Red
} LED_Color_e;

typedef enum
{
	NoError,
	AS5145B__InitError,
	EPOS4__FaultError,
	EPOS4__AbortError,
	EPOS4__InitError,
	MCP25625__InitError,
	MPU925x__InitError
} LED_Code_e;

static LED_Color_e CM_ledColor = Green;
static LED_Code_e CM_ledCode = NoError;

void ErrorHandler_AS5145B(uint8_t deviceIndex, AS5145B_Error_e error)
{
	CM_ledColor = Yellow;
	CM_ledCode = AS5145B__InitError;
	while(1);
}

void ErrorHandler_EPOS4(uint8_t deviceIndex, EPOS4_Error_e error)
{
	if(error == EPOS4_FaultError)
	{
		CM_ledColor = Red;
		CM_ledCode = EPOS4__FaultError;

		if((Prosthesis_Init.Joint == Ankle) || (Prosthesis_Init.Joint == Combined))
			EPOS4_DisableVoltage(AnkleMotorControllerIndex);

		if((Prosthesis_Init.Joint == Knee) || (Prosthesis_Init.Joint == Combined))
			EPOS4_DisableVoltage(KneeMotorControllerIndex);
	}
	else if(error == EPOS4_AbortError)
	{
		CM_ledColor = Orange;
		CM_ledCode = EPOS4__AbortError;

		if((Prosthesis_Init.Joint == Ankle) || (Prosthesis_Init.Joint == Combined))
			EPOS4_DisableVoltage(AnkleMotorControllerIndex);

		if((Prosthesis_Init.Joint == Knee) || (Prosthesis_Init.Joint == Combined))
			EPOS4_DisableVoltage(KneeMotorControllerIndex);
	}
	else
	{
		CM_ledColor = Yellow;
		CM_ledCode = EPOS4__InitError;
	}

	while(1);
}

void ErrorHandler_MCP25625(uint8_t deviceIndex, MCP25625_Error_e error)
{
	CM_ledColor = Yellow;
	CM_ledCode = MCP25625__InitError;
	while(1);
}

void ErrorHandler_MPU925x(uint8_t deviceIndex, MPU925x_Error_e error)
{
	CM_ledColor = Yellow;
	CM_ledCode = MPU925x__InitError;
	while(1);
}


/*******************************************************************************
* END
*******************************************************************************/
