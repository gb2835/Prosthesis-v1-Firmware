/*******************************************************************************
*
* See source file for more information.
*
*******************************************************************************/

#include "mcp25625.h"
#include <stdint.h>

typedef enum
{
	cyclicSynchronousTorqueMode
} EPOS4_ModeOfOperation_t;

typedef struct
{
	enum
	{
		rate1000Kbps,
		rate800Kbps,
		rate500Kbps,
		rate250Kbps,
		rate125Kbps,
		__reserved,
		rate50Kbps,
		rate20Kbps,
		__notSupported10Kbps,
		automaticBitRateDetection
	} CAN_BitRate; 							// 0x2001-00, should these all be EPOS4_??
	enum
	{
		__noOption0,
		phaseModulatedDcMotor,
		__noOption2,
		__noOption3,
		__noOption4,
		__noOption5,
		__noOption6,
		__noOption7,
		__noOption8,
		__noOption9,
		sinusoidalPmBlMotor,
		trapezoidalPmBlMotor
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
	EPOS4_Requirements_t Requirements;
	EPOS4_FirstStep_t FirstStep;
	EPOS4_ModeOfOperation_t ModeOfOperation;
} EPOS4_t;

void EPOS4_Init(uint8_t nodeId, EPOS4_t *Device_Init);
int32_t EPOS4_ReadPositionActualValue(uint8_t nodeId);
int32_t EPOS4_ReadVelocityActualValue(uint8_t nodeId);
int32_t EPOS4_ReadVelocityActualValueAveraged(uint8_t nodeId);
int16_t EPOS4_ReadTargetTorqueValue(uint8_t nodeId);
int16_t EPOS4_ReadTorqueActualValue(uint8_t nodeId);
int16_t EPOS4_ReadTorqueActualValueAveraged(uint8_t nodeId);
void EPOS4_WriteTargetTorqueValue(uint8_t nodeId, int16_t torque);


/*******************************************************************************
* END
*******************************************************************************/
