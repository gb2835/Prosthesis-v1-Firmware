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

typedef enum
{
	EPOS4_NoError,
	EPOS4_TimeoutError,
	EPOS4_ProductCodeError,
	EPOS4_InitFaultDetected,
	EPOS4_DisableVoltageError,
	EPOS4_FirstStepError,
	EPOS4_ModeOfOperationError,
	EPOS4_FaultError,
	EPOS4_AbortError
} EPOS4_Error_e;

typedef enum
{
	CyclicSynchronousTorqueMode
} EPOS4_ModeOfOperation_e;

// take enums out of structs??
typedef struct
{
	enum
	{
		Rate1000Kbps,
		Rate800Kbps,
		Rate500Kbps,
		Rate250Kbps,
		Rate125Kbps,
		__Reserved,
		Rate50Kbps,
		Rate20Kbps,
		__NotSupported10Kbps,
		AutomaticBitRateDetection
	} CAN_BitRate; 							// 0x2001-00, should these all be EPOS4_??
	enum
	{
		__NoOption0,
		PhaseModulatedDcMotor,
		__NoOption2,
		__NoOption3,
		__NoOption4,
		__NoOption5,
		__NoOption6,
		__NoOption7,
		__NoOption8,
		__NoOption9,
		SinusoidalPmBlMotor,
		TrapezoidalPmBlMotor
	} MotorType; 							// 0x6402-00, should these all be EPOS4_??
	uint32_t nominalCurrent;				// 0x3001-01, units in mA
	uint32_t outputCurrentLimit;			// 0x3001-02, units in mA
	uint8_t numberOfPolePairs;				// 0x3001-03
	uint16_t thermalTimeConstantWinding;	// 0x3001-04, units in 0.1 seconds (400 = 40.0 seconds)
	uint32_t torqueConstant;				// 0x3001-05, units in mNm/A
	uint32_t maxMotorSpeed;					// 0x6080-00, units in rpm
	uint32_t maxGearInputSpeed;				// 0x3003-03, units in rpm
	uint32_t sensorsConfiguration;			// 0x3000-01
	uint32_t controlStructure;				// 0x3000-02
	uint32_t commutationSensors;			// 0x3000-03
	uint32_t axisConfigMiscellaneous;		// 0x3000-04
	uint32_t currentControllerP_Gain;		// 0x30A0-01, units in uV/A
	uint32_t currentControllerI_Gain;		// 0x30A0-02, units in uV/(A*s)
} EPOS4_FirstStep_t;

typedef struct
{
	uint8_t isFirstStepRequired;
	uint8_t isModeOfOperationRequired;
} EPOS4_Requirements_t ;

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
EPOS4_Error_e EPOS4_CheckForError(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx);
EPOS4_Error_e EPOS4_CheckForAbort(uint8_t deviceIndex, uint8_t *data);


/*******************************************************************************
* END
*******************************************************************************/

#endif /* INC_EPOS4_H_ */
