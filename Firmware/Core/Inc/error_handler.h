/*******************************************************************************
*
* See source file for more information.
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
void ErrorHandler_MPU925x(MPU925x_Error_e error);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_ERROR_HANDLER_H_ */
