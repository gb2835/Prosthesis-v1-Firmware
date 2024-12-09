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
#include "prosthesis_v1.h"

typedef enum
{
	Green,
	Yellow,
	Orange,
	Red
} LED_Status_e;

LED_Status_e CM_led = Green;

void ErrorHandler_AS5145B(uint8_t deviceIndex, AS5145B_Error_e error)
{
	CM_led = Yellow;
	while(1);
}

void ErrorHandler_EPOS4(uint8_t deviceIndex, EPOS4_Error_e error)
{
	if(error == EPOS4_FaultError)
	{
		CM_led = Red;
		EPOS4_DisableVoltage(AnkleMotorControllerIndex);
		EPOS4_DisableVoltage(KneeMotorControllerIndex);
	}
	else if(error == EPOS4_AbortError)
	{
		CM_led = Orange;
		EPOS4_DisableVoltage(AnkleMotorControllerIndex);
		EPOS4_DisableVoltage(KneeMotorControllerIndex);
	}
	else
		CM_led = Yellow;

	while(1);
}

void ErrorHandler_MCP25625(uint8_t deviceIndex, MCP25625_Error_e error)
{
	CM_led = Yellow;
	while(1);
}

void ErrorHandler_MPU925x(uint8_t deviceIndex, MPU925x_Error_e error)
{
	CM_led = Yellow;
	while(1);
}


/*******************************************************************************
* END
*******************************************************************************/
