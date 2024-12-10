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

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

#include "as5145b.h"
#include "epos4.h"
#include "mcp25625.h"
#include "mpu925x_spi.h"

void ErrorHandler_AS5145B(uint8_t deviceIndex, AS5145B_Error_e error);
void ErrorHandler_EPOS4(uint8_t deviceIndex, EPOS4_Error_e error);
void ErrorHandler_MCP25625(uint8_t deviceIndex, MCP25625_Error_e error);
void ErrorHandler_MPU925x(uint8_t deviceIndex, MPU925x_Error_e error);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_ERROR_HANDLER_H_ */
