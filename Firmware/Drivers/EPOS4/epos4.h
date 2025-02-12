/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#ifndef INC_EPOS4_H_
#define INC_EPOS4_H_

#include "mcp25625.h"
#include <stdint.h>

#define EPOS4_NUMBER_OF_DEVICES	2

#define ERROR_REG_INDEX				0x1001
#define ERROR_HISTORY_INDEX			0x1003
#define NUMBER_OF_ERRORS_SUBINDEX	0
#define ERROR_HISTORY_1_SUBINDEX	1
#define ERROR_HISTORY_2_SUBINDEX	2
#define ERROR_HISTORY_3_SUBINDEX	3
#define ERROR_HISTORY_4_SUBINDEX	4
#define ERROR_HISTORY_5_SUBINDEX	5

typedef enum
{
	EPOS4_NoError,
	EPOS4_TimeoutError,
	EPOS4_ProductCodeError,
	EPOS4_InitFaultDetected,
	EPOS4_DisableVoltageError,
	EPOS4_FirstStepError,
	EPOS4_OptionCodesError,
	EPOS4_ModeOfOperationError,
	EPOS4_FaultError,
	EPOS4_AbortError
} EPOS4_Error_e;

typedef enum
{
	EPOS4_CyclicSynchronousTorqueMode,
} EPOS4_ModeOfOperation_e;

typedef struct
{
	enum
	{
		EPOS4_Rate1000Kbps = 0,
		EPOS4_Rate800Kbps,
		EPOS4_Rate500Kbps,
		EPOS4_Rate250Kbps,
		EPOS4_Rate125Kbps,
		EPOS4_Rate50Kbps = 6,
		EPOS4_Rate20Kbps,
		EPOS4_AutomaticBitRateDetection = 9
	} CAN_BitRate;
	enum
	{
		EPOS4_PhaseModulatedDcMotor = 1,
		EPOS4_SinusoidalPmBlMotor = 10,
		EPOS4_TrapezoidalPmBlMotor
	} MotorType;
	uint32_t nominalCurrent;
	uint32_t outputCurrentLimit;
	uint8_t numberOfPolePairs;
	uint16_t thermalTimeConstantWinding;
	uint32_t torqueConstant;
	uint32_t maxMotorSpeed;
	uint32_t maxGearInputSpeed;
	uint32_t sensorsConfiguration;
	uint32_t controlStructure;
	uint32_t commutationSensors;
	uint32_t axisConfigMiscellaneous;
	uint32_t currentControllerP_Gain;
	uint32_t currentControllerI_Gain;
} EPOS4_FirstStep_t;

typedef struct
{
	uint8_t isFirstStepRequired;
	uint8_t isModeOfOperationRequired;
} EPOS4_Requirements_t;

typedef struct
{
	uint8_t nodeId;
	uint8_t mcpIndex;
	EPOS4_Requirements_t Requirements;
	EPOS4_FirstStep_t FirstStep;
	EPOS4_ModeOfOperation_e ModeOfOperation;
} EPOS4_Init_t;

EPOS4_Error_e EPOS4_Init(uint8_t deviceIndex, EPOS4_Init_t *Device_Init);
EPOS4_Error_e EPOS4_WriteTargetTorqueValue(uint8_t deviceIndex, int16_t torque);
EPOS4_Error_e EPOS4_DisableVoltage(uint8_t deviceIndex);
EPOS4_Error_e EPOS4_ReadObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t *value);
EPOS4_Error_e EPOS4_WriteObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value);
EPOS4_Error_e EPOS4_CheckForFault(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx);
EPOS4_Error_e EPOS4_CheckForAbort(uint8_t deviceIndex, uint8_t *data);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_EPOS4_H_ */
