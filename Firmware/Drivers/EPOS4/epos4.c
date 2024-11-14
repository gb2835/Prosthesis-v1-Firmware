/*******************************************************************************
*
* TITLE		Driver for Maxon EPOS4 Motor Controller with Microchip MCP25625 CAN Controller
* AUTHOR	Greg Berkeley
* RELEASE	??
*
* NOTES
* 1. This driver is based on:
*     - EPOS4 Application Notes
*        - Document Number: rel8760
*        - Revision: 2019-11
*     - EPOS4 Positioning Controllers Firmware Specification
*        - Document Number: rel8234
*      	 - Edition: November 2018
* 2. STO pins are not controlled in this driver.
* 3. Only CST mode is provided.
*     - Quick stop function is not provided, thus deceleration parameters are not set for CST mode.
*     - Only parameters for CST are included in EPOS4_FirstStep_t.
* 4. FirstStep can be used to either initialize the device or check parameters already set by EPOS Studio.
*    CAN_BitRate in FirstStep cannot be initialized from this driver, only checked.
*
*******************************************************************************/

#include "epos4.h"
#include "mcp25625.h"
#include "string.h"


/*******************************************************************************
* PRIVATE DEFINTIONS
*******************************************************************************/

#define ERROR_REG_INDEX							0x1001
#define ERROR_HISTORY_INDEX						0x1003
#define NUMBER_OF_ERRORS_SUBINDEX				0
#define ERROR_HISTORY_1_SUBINDEX				1
#define ERROR_HISTORY_2_SUBINDEX				2
#define ERROR_HISTORY_3_SUBINDEX				3
#define ERROR_HISTORY_4_SUBINDEX				4
#define ERROR_HISTORY_5_SUBINDEX				5
#define IDENTITY_OBJECT_INDEX					0x1018
#define PRODUCT_CODE_SUBINDEX					2
#define NODE_ID_INDEX							0x2000
#define CAN_BITRATE_INDEX						0x2001
#define AXIS_CONFIGURATION_INDEX				0x3000
#define SENSOR_CONFIGURATION_SUBINDEX			1
#define CONTROL_STRUCTURE_SUBINDEX				2
#define COMMUTATION_SENSORS_SUBINDEX			3
#define AXIS_CONFIG_MISCELLANEOUS_SUBINDEX		4
#define MOTOR_DATA_INDEX						0x3001
#define NOMINAL_CURRENT_SUBINDEX				1
#define OUTPUT_CURRENT_LIMIT_SUBINDEX			2
#define NUMBER_OF_POLE_PAIRS_SUBINDEX			3
#define THERMAL_TIME_CONSTANT_WINDING_SUBINDEX	4
#define TORQUE_CONSTANT_SUBINDEX				5
#define GEAR_CONFIGURATION_INDEX				0x3003
#define MAX_GEAR_INPUT_SPEED_SUBINDEX			3
#define CURRENT_CTRL_PARAMETER_SET_INDEX		0x30A0
#define CURRENT_CONTROLLER_P_GAIN_SUBINDEX		1
#define CURRENT_CONTROLLER_I_GAIN_SUBINDEX		2
#define TORQUE_ACTUAL_VALUE_AVERAGED_INDEX		0x30D2
#define TORQUE_ACTUAL_VALUE_AVERAGED_SUBINDEX	1
#define VELOCITY_ACTUAL_VALUE_AVERAGED_INDEX	0x30D3
#define VELOCITY_ACTUAL_VALUE_AVERAGED_SUBINDEX	1
#define CONTROLWORD_INDEX						0x6040
#define STATUSWORD_INDEX						0x6041
#define MODES_OF_OPERATION_INDEX				0x6060
#define POSITION_ACTUAL_VALUE_INDEX				0x6064
#define VELOCITY_ACTUAL_VALUE_INDEX				0x606C
#define TARGET_TORQUE_INDEX						0x6071
#define TORQUE_ACTUAL_VALUE_INDEX				0x6077
#define MAX_MOTOR_SPEED_INDEX					0x6080
#define TORQUE_OFFSET_INDEX						0x60B2
#define MOTOR_TYPE_INDEX						0x6402

#define	CTRLCMD_DISABLE_VOLTAGE			0b0000
#define CTRLCMD_SHUTDOWN				0b0110
#define CTRLCMD_SWITCH_ON_AND_ENABLE	0b1111

#define STATE_FAULT					0b1000
#define STATE_FAULT_REACTION_ACTIVE	0b00001111
#define STATE_MASK					0b01101111
#define STATE_OPERATION_ENABLED		0b00100111
#define STATE_READY_TO_SWITCH_ON	0b00100001
#define STATE_SWITCH_ON_DISABLED	0b01000000

#define CLIENT_UPLOAD				0b01000000
#define CST_MODE					0x0A
#define EXPEDITED_CLIENT_DOWNLOAD	0b00100010

// i can see this in prosthesis_control.h??
typedef enum
{
	noError,
	timeoutError,
	nodeIdError,
	productCodeError,
	initFaultDetected,
	disableVoltageError,
	firstStepError,
	modeOfOperationError,
	deviceError,
	abortError
} Errors_t;

typedef struct
{
	EPOS4_Requirements_t Requirements;
	EPOS4_FirstStep_t FirstStep;
	EPOS4_ModeOfOperation_t ModeOfOperation;
	uint8_t isInit;
} Device_t;

static Device_t Device;

static uint8_t errorHasOccurred = 0;

uint8_t CM_epos4_abortLowByte = 0, CM_epos4_abortHighByte = 0, CM_epos4_abortSubindex = 0;
uint8_t CM_epos4_error = noError, CM_epos4_numOfErrors = 0;
uint16_t CM_epos4_state = 0;
uint32_t CM_epos4_abortCode = 0;
uint32_t CM_epos4_errorHistory1 = 0, CM_epos4_errorHistory2 = 0, CM_epos4_errorHistory3 = 0, CM_epos4_errorHistory4 = 0, CM_epos4_errorHistory5 = 0;

static uint32_t ReadObjectValue(uint16_t cobId, uint16_t index, uint8_t subindex);
static void WriteObjectValue(uint16_t cobId, uint16_t index, uint8_t subindex, uint32_t value);
static uint32_t ParseValueFromData(uint8_t *data);
static void SDO_Upload(uint16_t cobId, uint16_t index, uint8_t subindex, MCP25625_RXBx_t *RXBx);
static void SDO_Download(uint16_t cobId, uint16_t index, uint8_t subindex, uint32_t value, MCP25625_RXBx_t *RXBx);
static void CheckForError(uint8_t nodeId, MCP25625_RXBx_t *RXBx);
static void CheckForAbort(uint16_t cobId, uint8_t *data);
static void FrameData(uint8_t *data, uint8_t byte0, uint16_t index, uint8_t subindex, uint32_t value);
static uint8_t WriteFirstStepObjects(uint16_t cobId, EPOS4_FirstStep_t FirstStep);
static uint8_t WriteModeOfOperation(uint16_t cobId, EPOS4_ModeOfOperation_t modeOfOperation);
static void ErrorHandler(uint16_t cobId, Errors_t error);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void EPOS4_Init(uint8_t nodeId, EPOS4_Inits_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(EPOS4_Inits_t));

	uint16_t cobId = nodeId + 0x600;

	if(ReadObjectValue(cobId, NODE_ID_INDEX, 0) != nodeId)	// timeout if turned off??
		ErrorHandler(cobId, nodeIdError);

	uint8_t epos4ProductCodeError = 1;
	uint16_t hwVersions[6] = {0x6050, 0x6150, 0x6551, 0x6552, 0x6350, 0x6450};
	uint16_t productCode = ReadObjectValue(cobId, IDENTITY_OBJECT_INDEX, PRODUCT_CODE_SUBINDEX) >> 16;
	for(uint8_t i = 0; i < 6; i++)
	{
		if(productCode == hwVersions[i])
		{
			epos4ProductCodeError = 0;
			break;
		}
	}
	if(epos4ProductCodeError)
		ErrorHandler(cobId, productCodeError);

	uint16_t state = ReadObjectValue(cobId, STATUSWORD_INDEX, 0) & STATE_MASK;
	if((state == STATE_FAULT) || (state == STATE_FAULT_REACTION_ACTIVE))
		ErrorHandler(cobId, initFaultDetected);

	WriteObjectValue(cobId, CONTROLWORD_INDEX, 0, CTRLCMD_DISABLE_VOLTAGE);
	while((ReadObjectValue(cobId, STATUSWORD_INDEX, 0) & STATE_MASK) != STATE_SWITCH_ON_DISABLED); // timeout??

	if(Device.Requirements.isFirstStepRequired)
		if(WriteFirstStepObjects(cobId, Device.FirstStep))
			ErrorHandler(cobId, firstStepError);

	if(Device.Requirements.isModeOfOperationRequired)
		if(WriteModeOfOperation(cobId, Device.ModeOfOperation))
			ErrorHandler(cobId, modeOfOperationError);

	Device.isInit = 1;
}

int32_t EPOS4_ReadPositionActualValue(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int32_t) ReadObjectValue(cobId, POSITION_ACTUAL_VALUE_INDEX, 0);
}

int32_t EPOS4_ReadVelocityActualValue(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int32_t) ReadObjectValue(cobId, VELOCITY_ACTUAL_VALUE_INDEX, 0);
}

int32_t EPOS4_ReadVelocityActualValueAveraged(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int32_t) ReadObjectValue(cobId, VELOCITY_ACTUAL_VALUE_AVERAGED_INDEX, VELOCITY_ACTUAL_VALUE_AVERAGED_SUBINDEX);
}

int16_t EPOS4_ReadTargetTorqueValue(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int16_t) ReadObjectValue(cobId, TARGET_TORQUE_INDEX, 0);
}

int16_t EPOS4_ReadTorqueActualValueAveraged(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int16_t) ReadObjectValue(cobId, TORQUE_ACTUAL_VALUE_AVERAGED_INDEX, TORQUE_ACTUAL_VALUE_AVERAGED_SUBINDEX);
}

int16_t EPOS4_ReadTorqueActualValue(uint8_t nodeId)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	return (int16_t) ReadObjectValue(cobId, TORQUE_ACTUAL_VALUE_INDEX, 0);
}

void EPOS4_WriteTargetTorqueValue(uint8_t nodeId, int16_t torque)
{
	if(!Device.isInit)
		__NOP(); // add assert??

	uint16_t cobId = nodeId + 0x600;

	WriteObjectValue(cobId, TARGET_TORQUE_INDEX, 0, torque);
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static uint32_t ReadObjectValue(uint16_t cobId, uint16_t index, uint8_t subindex)
{
	MCP25625_RXBx_t RXBx;
	SDO_Upload(cobId, index, subindex, &RXBx);

	if(!errorHasOccurred)
	{
		uint8_t nodeId = cobId - 0x600;
		CheckForError(nodeId, &RXBx);
	}

	CheckForAbort(cobId, RXBx.Struct.RXBxDn_Reg);

	return ParseValueFromData(RXBx.Struct.RXBxDn_Reg);
}

static void WriteObjectValue(uint16_t cobId, uint16_t index, uint8_t subindex, uint32_t value)
{
	MCP25625_RXBx_t	RXBx;
	SDO_Download(cobId, index, subindex, value, &RXBx);

	if(!errorHasOccurred)
	{
		uint8_t nodeId = cobId - 0x600;
		CheckForError(nodeId, &RXBx);
	}

	CheckForAbort(cobId, RXBx.Struct.RXBxDn_Reg);
}

static uint32_t ParseValueFromData(uint8_t *data)
{
	return (uint32_t) ((data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4]);
}

static void SDO_Upload(uint16_t cobId, uint16_t index, uint8_t subindex, MCP25625_RXBx_t *RXBx)
{
	uint8_t data[8];
	FrameData(data, CLIENT_UPLOAD, index, subindex, 0);
	while(MCP25625_LoadTxBufferAtSIDH(cobId, data, 8));
	while(MCP25625_ReadRxBufferAtSIDH(RXBx, 8));
}

static void SDO_Download(uint16_t cobId, uint16_t index, uint8_t subindex, uint32_t value, MCP25625_RXBx_t *RXBx)
{
	uint8_t data[8];
	FrameData(data, EXPEDITED_CLIENT_DOWNLOAD, index, subindex, value);
	while(MCP25625_LoadTxBufferAtSIDH(cobId, data, 8));
	while(MCP25625_ReadRxBufferAtSIDH(RXBx, 8));
}

static void CheckForError(uint8_t nodeId, MCP25625_RXBx_t *RXBx)
{
	uint8_t cobIdEmcy = nodeId + 0x80;
	uint16_t cobId = (uint16_t) ((RXBx->Struct.RXBxSIDH_Reg << 3) + (RXBx->Struct.RXBxSIDL_Reg.value >> 5));
	if(cobId == cobIdEmcy)
		ErrorHandler(cobId, deviceError);
}

static void CheckForAbort(uint16_t cobId, uint8_t *data)
{
	if(data[0] >> 7)
	{
		CM_epos4_abortLowByte = data[1];
		CM_epos4_abortHighByte = data[2];
		CM_epos4_abortSubindex = data[3];
		CM_epos4_abortCode = ParseValueFromData(data);

		ErrorHandler(cobId, abortError);
	}
}

static void FrameData(uint8_t *data, uint8_t byte0, uint16_t index, uint8_t subindex, uint32_t value)
{
	data[0] = byte0;
	data[1] = (0x00 | index);
	data[2] = (0x00 | index >> 8);
	data[3] = subindex;
	data[4] = (0x00 | value);
	data[5] = (0x00 | value >> 8);
	data[6] = (0x00 | value >> 16);
	data[7] = (0x00 | value >> 24);
}

static uint8_t WriteFirstStepObjects(uint16_t cobId, EPOS4_FirstStep_t FirstStep)
{
//	WriteObjectValue(cobId, CAN_BITRATE_INDEX, 0, FirstStep.CAN_BitRate);
//	if(ReadObjectValue(cobId, CAN_BITRATE_INDEX, 0) != FirstStep.CAN_BitRate)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_TYPE_INDEX, 0, FirstStep.MotorType);
//	if(ReadObjectValue(cobId, MOTOR_TYPE_INDEX, 0) != FirstStep.MotorType)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX, FirstStep.nominalCurrent);
//	if(ReadObjectValue(cobId, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX) != FirstStep.nominalCurrent)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX, FirstStep.outputCurrentLimit);
//	if(ReadObjectValue(cobId, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX) != FirstStep.outputCurrentLimit)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_DATA_INDEX, NUMBER_OF_POLE_PAIRS_SUBINDEX, FirstStep.numberOfPolePairs);
//	if(ReadObjectValue(cobId, MOTOR_DATA_INDEX, NUMBER_OF_POLE_PAIRS_SUBINDEX) != FirstStep.numberOfPolePairs)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_DATA_INDEX, THERMAL_TIME_CONSTANT_WINDING_SUBINDEX, FirstStep.thermalTimeConstantWinding);
//	if(ReadObjectValue(cobId, MOTOR_DATA_INDEX, THERMAL_TIME_CONSTANT_WINDING_SUBINDEX) != FirstStep.thermalTimeConstantWinding)
//		return 1;
//
//	WriteObjectValue(cobId, MOTOR_DATA_INDEX,TORQUE_CONSTANT_SUBINDEX, FirstStep.torqueConstant);
//	if(ReadObjectValue(cobId, MOTOR_DATA_INDEX,TORQUE_CONSTANT_SUBINDEX) != FirstStep.torqueConstant)
//		return 1;
//
//	WriteObjectValue(cobId, MAX_MOTOR_SPEED_INDEX, 0, FirstStep.maxMotorSpeed);
//	if(ReadObjectValue(cobId, MAX_MOTOR_SPEED_INDEX, 0) != FirstStep.maxMotorSpeed)
//		return 1;
//
//	WriteObjectValue(cobId, GEAR_CONFIGURATION_INDEX, MAX_GEAR_INPUT_SPEED_SUBINDEX, FirstStep.maxGearInputSpeed);
//	if(ReadObjectValue(cobId, GEAR_CONFIGURATION_INDEX, MAX_GEAR_INPUT_SPEED_SUBINDEX) != FirstStep.maxGearInputSpeed)
//		return 1;
//
//	WriteObjectValue(cobId, AXIS_CONFIGURATION_INDEX, SENSOR_CONFIGURATION_SUBINDEX, FirstStep.sensorsConfiguration);
//	if(ReadObjectValue(cobId, AXIS_CONFIGURATION_INDEX, SENSOR_CONFIGURATION_SUBINDEX) != FirstStep.sensorsConfiguration)
//		return 1;
//
//	WriteObjectValue(cobId, AXIS_CONFIGURATION_INDEX, CONTROL_STRUCTURE_SUBINDEX, FirstStep.controlStructure);
//	if(ReadObjectValue(cobId, AXIS_CONFIGURATION_INDEX, CONTROL_STRUCTURE_SUBINDEX) != FirstStep.controlStructure)
//		return 1;
//
//	WriteObjectValue(cobId, AXIS_CONFIGURATION_INDEX, COMMUTATION_SENSORS_SUBINDEX, FirstStep.commutationSensors);
//	if(ReadObjectValue(cobId, AXIS_CONFIGURATION_INDEX, COMMUTATION_SENSORS_SUBINDEX) != FirstStep.commutationSensors)
//		return 1;
//
//	WriteObjectValue(cobId, AXIS_CONFIGURATION_INDEX, AXIS_CONFIG_MISCELLANEOUS_SUBINDEX, FirstStep.axisConfigMiscellaneous);
//	if(ReadObjectValue(cobId, AXIS_CONFIGURATION_INDEX, AXIS_CONFIG_MISCELLANEOUS_SUBINDEX) != FirstStep.axisConfigMiscellaneous)
//		return 1;
//
//	WriteObjectValue(cobId, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_P_GAIN_SUBINDEX, FirstStep.currentControllerP_Gain);
//	if(ReadObjectValue(cobId, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_P_GAIN_SUBINDEX) != FirstStep.currentControllerP_Gain)
//		return 1;
//
//	WriteObjectValue(cobId, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_I_GAIN_SUBINDEX, FirstStep.currentControllerI_Gain);
//	if(ReadObjectValue(cobId, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_I_GAIN_SUBINDEX) != FirstStep.currentControllerI_Gain)
//		return 1;

	return 0;
}

static uint8_t WriteModeOfOperation(uint16_t cobId, EPOS4_ModeOfOperation_t modeOfOperation)
{
	switch (modeOfOperation)
	{
	case cyclicSynchronousTorqueMode:
	{
		WriteObjectValue(cobId, TARGET_TORQUE_INDEX, 0, 0);
		if(ReadObjectValue(cobId, TARGET_TORQUE_INDEX, 0) != 0)
			return 1;

		WriteObjectValue(cobId, MODES_OF_OPERATION_INDEX, 0, CST_MODE);
		if(ReadObjectValue(cobId, MODES_OF_OPERATION_INDEX, 0) != CST_MODE)
			return 1;

		WriteObjectValue(cobId, CONTROLWORD_INDEX, 0, CTRLCMD_SHUTDOWN);
		while((ReadObjectValue(cobId, STATUSWORD_INDEX, 0) & STATE_MASK) != STATE_READY_TO_SWITCH_ON); // timeout??

		WriteObjectValue(cobId, CONTROLWORD_INDEX, 0, CTRLCMD_SWITCH_ON_AND_ENABLE);
		while((ReadObjectValue(cobId, STATUSWORD_INDEX, 0) & STATE_MASK) != STATE_OPERATION_ENABLED); // timeout??

		WriteObjectValue(cobId, TORQUE_OFFSET_INDEX, 0, 0);
		if(ReadObjectValue(cobId, TORQUE_OFFSET_INDEX, 0) != 0)
			return 1;

		return 0;
	}
	}

	return 1;
}

// right way to do this??
static void ErrorHandler(uint16_t cobId, Errors_t error)
{
	errorHasOccurred = 1;

	CM_epos4_error = error;
	CM_epos4_numOfErrors = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, NUMBER_OF_ERRORS_SUBINDEX);
	CM_epos4_state = ReadObjectValue(cobId, STATUSWORD_INDEX, 0) & STATE_MASK;
	CM_epos4_errorHistory1 = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, ERROR_HISTORY_1_SUBINDEX);
	CM_epos4_errorHistory2 = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, ERROR_HISTORY_2_SUBINDEX);
	CM_epos4_errorHistory3 = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, ERROR_HISTORY_3_SUBINDEX);
	CM_epos4_errorHistory4 = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, ERROR_HISTORY_4_SUBINDEX);
	CM_epos4_errorHistory5 = ReadObjectValue(cobId, ERROR_HISTORY_INDEX, ERROR_HISTORY_5_SUBINDEX);

	while(1)
	{
		WriteObjectValue(cobId, CONTROLWORD_INDEX, 0, CTRLCMD_DISABLE_VOLTAGE);
	}
}


/*******************************************************************************
* END
*******************************************************************************/
