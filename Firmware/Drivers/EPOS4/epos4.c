/*******************************************************************************
*
* TITLE: Driver for Maxon EPOS4 Motor Controller with Microchip MCP25625 CAN Controller
*
* NOTES
* 1. This driver is based on:
*		- EPOS4 Application Notes
*			- Document Number: rel8760
*			- Revision: 2019-11
*		- EPOS4 Positioning Controllers Firmware Specification
*			- Document Number: rel8234
*			- Edition: November 2018
* 2. Unless otherwise specified, units are
*		Torque	= N*mm
* 2. #define EPOS4_NUMBER_OF_DEVICES must be updated to (at least) the number of devices used.
* 3. STO pins are not controlled in this driver.
* 4. Only CST mode is provided.
*     - Quick stop function is not used, thus deceleration parameters are not set for CST mode.
*     - Only parameters for CST are included in EPOS4_FirstStep_t.
*     - CST mode sets target torque and torque offset to zero.
* 5. FirstStep can be used to either initialize the device or check parameters already set by EPOS Studio.
*    CAN_BitRate in FirstStep cannot be initialized from this driver, only checked.
* 6. All option codes are configured to disable drive function (quick stop option code not used).
*
*******************************************************************************/

#include "epos4.h"
#include "mcp25625.h"
#include <string.h>


/*******************************************************************************
* PRIVATE DEFINTIONS
*******************************************************************************/

#define IDENTITY_OBJECT_INDEX					0x1018
#define PRODUCT_CODE_SUBINDEX					2
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
#define ABORT_CONNECTION_OPTION_CODE_INDEX		0x6007
#define SHUTDOWN_OPTION_CODE_INDEX				0x605B
#define DISABLE_OPERATION_OPTION_CODE_INDEX		0x605C
#define FAULT_REACTION_OPTION_CODE_INDEX		0x605E
#define CONTROLWORD_INDEX						0x6040
#define STATUSWORD_INDEX						0x6041
#define MODES_OF_OPERATION_INDEX				0x6060
#define TARGET_TORQUE_INDEX						0x6071
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

typedef struct
{
	uint8_t nodeId;
	uint8_t mcpIndex;
	EPOS4_Requirements_t Requirements;
	EPOS4_FirstStep_t FirstStep;
	EPOS4_ModeOfOperation_e ModeOfOperation;
	uint16_t cobId;
	uint32_t isInit;
} Device_t;

static Device_t Device[EPOS4_NUMBER_OF_DEVICES];

static EPOS4_Error_e ReadObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t *value);
static EPOS4_Error_e WriteObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value);
static void ParseValueFromData(uint32_t *value, uint8_t *data);
static void SDO_Upload(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, MCP25625_RXBx_t *RXBx);
static void SDO_Download(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value, MCP25625_RXBx_t *RXBx);
static void FrameData(uint8_t *data, uint8_t byte0, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value);
static EPOS4_Error_e DisableVoltage(uint8_t deviceIndex);
static EPOS4_Error_e CheckForFault(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx);
static EPOS4_Error_e CheckForAbort(uint8_t deviceIndex, uint8_t *data);
static uint8_t WriteFirstStepObjects(uint8_t deviceIndex, EPOS4_FirstStep_t FirstStep);
static uint8_t WriteOptionCodes(uint8_t deviceIndex);
static uint8_t WriteModeOfOperation(uint8_t deviceIndex, EPOS4_ModeOfOperation_e modeOfOperation);

/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

EPOS4_Error_e EPOS4_Init(uint8_t deviceIndex, EPOS4_Init_t *Device_Init)
{
	if(deviceIndex + 1 > EPOS4_NUMBER_OF_DEVICES)
		while(1);

	memcpy(&Device[deviceIndex], Device_Init, sizeof(EPOS4_Init_t));

	Device[deviceIndex].cobId = Device[deviceIndex].nodeId + 0x0600;

	uint8_t epos4ProductCodeError = 1;
	uint16_t hwVersions[6] = {0x6050, 0x6150, 0x6551, 0x6552, 0x6350, 0x6450};
	uint32_t value;
	ReadObjectValue(deviceIndex, IDENTITY_OBJECT_INDEX, PRODUCT_CODE_SUBINDEX, &value);
	value = value >> 16;
	for(uint8_t i = 0; i < 6; i++)
	{
		if(value == hwVersions[i])
		{
			epos4ProductCodeError = 0;
			break;
		}
	}
	if(epos4ProductCodeError)
		return EPOS4_ProductCodeError;

	ReadObjectValue(deviceIndex, STATUSWORD_INDEX, 0, &value);
	value = value & STATE_MASK;
	if((value == STATE_FAULT) || (value == STATE_FAULT_REACTION_ACTIVE))
		return EPOS4_InitFaultDetected;

	DisableVoltage(deviceIndex);
	do
	{
		ReadObjectValue(deviceIndex, STATUSWORD_INDEX, 0, &value);
		value = value & STATE_MASK;
	} while(value != STATE_SWITCH_ON_DISABLED);

	if(Device[deviceIndex].Requirements.isFirstStepRequired)
		if(WriteFirstStepObjects(deviceIndex, Device[deviceIndex].FirstStep))
			return EPOS4_FirstStepError;

	if(WriteOptionCodes(deviceIndex))
		return EPOS4_OptionCodesError;

	if(Device[deviceIndex].Requirements.isModeOfOperationRequired)
		if(WriteModeOfOperation(deviceIndex, Device[deviceIndex].ModeOfOperation))
			return EPOS4_ModeOfOperationError;

	Device[deviceIndex].isInit = 1;

	return EPOS4_NoError;
}

EPOS4_Error_e EPOS4_WriteTargetTorqueValue(uint8_t deviceIndex, int16_t torque)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	EPOS4_Error_e error = WriteObjectValue(deviceIndex, TARGET_TORQUE_INDEX, 0, torque);
	if(error)
		return error;

	return EPOS4_NoError;
}

EPOS4_Error_e EPOS4_DisableVoltage(uint8_t deviceIndex)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return DisableVoltage(deviceIndex);
}

EPOS4_Error_e EPOS4_ReadObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t *value)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return ReadObjectValue(deviceIndex, objectIndex, objectSubindex, value);
}

EPOS4_Error_e EPOS4_WriteObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return WriteObjectValue(deviceIndex, objectIndex, objectSubindex, value);
}

EPOS4_Error_e EPOS4_CheckForFault(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return CheckForFault(deviceIndex, RXBx);
}

EPOS4_Error_e EPOS4_CheckForAbort(uint8_t deviceIndex, uint8_t *data)
{
	if(!Device[deviceIndex].isInit)
		while(1);

	return CheckForAbort(deviceIndex, data);
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static EPOS4_Error_e ReadObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t *value)
{
	MCP25625_RXBx_t RXBx;
	SDO_Upload(deviceIndex, objectIndex, objectSubindex, &RXBx);

	if(CheckForFault(deviceIndex, &RXBx))
		return EPOS4_FaultError;
	if(CheckForAbort(deviceIndex, RXBx.Struct.RXBxDn_Reg))
		return EPOS4_AbortError;

	ParseValueFromData(value, RXBx.Struct.RXBxDn_Reg);

	return EPOS4_NoError;
}

static EPOS4_Error_e WriteObjectValue(uint8_t deviceIndex, uint16_t objectIndex, uint8_t objectSubindex, uint32_t value)
{
	MCP25625_RXBx_t	RXBx;
	SDO_Download(deviceIndex, objectIndex, objectSubindex, value, &RXBx);

	if(CheckForFault(deviceIndex, &RXBx))
		return EPOS4_FaultError;
	if(CheckForAbort(deviceIndex, RXBx.Struct.RXBxDn_Reg))
		return EPOS4_AbortError;

	return EPOS4_NoError;
}

static void ParseValueFromData(uint32_t *value, uint8_t *data)
{
	value[0] = ((data[7] << 24) + (data[6] << 16) + (data[5] << 8) + data[4]);
}

static void SDO_Upload(uint8_t deviceIndex, uint16_t index, uint8_t subindex, MCP25625_RXBx_t *RXBx)
{
	uint8_t data[8];
	FrameData(data, CLIENT_UPLOAD, index, subindex, 0);
	while(MCP25625_LoadTxBufferAtSIDH(Device[deviceIndex].mcpIndex, Device[deviceIndex].cobId, data, 8));
	while(MCP25625_ReadRxBufferAtSIDH(Device[deviceIndex].mcpIndex, RXBx, 8));
}

static void SDO_Download(uint8_t deviceIndex, uint16_t index, uint8_t subindex, uint32_t value, MCP25625_RXBx_t *RXBx)
{
	uint8_t data[8];
	FrameData(data, EXPEDITED_CLIENT_DOWNLOAD, index, subindex, value);
	while(MCP25625_LoadTxBufferAtSIDH(Device[deviceIndex].mcpIndex, Device[deviceIndex].cobId, data, 8));
	while(MCP25625_ReadRxBufferAtSIDH(Device[deviceIndex].mcpIndex, RXBx, 8));
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

static EPOS4_Error_e DisableVoltage(uint8_t deviceIndex)
{
	EPOS4_Error_e error = WriteObjectValue(deviceIndex, CONTROLWORD_INDEX, 0, CTRLCMD_DISABLE_VOLTAGE);
	if(error)
		return error;

	return EPOS4_NoError;
}

static EPOS4_Error_e CheckForFault(uint8_t deviceIndex, MCP25625_RXBx_t *RXBx)
{
	uint8_t cobIdEmcy = Device[deviceIndex].nodeId + 0x80;
	uint16_t cobId = (uint16_t) ((RXBx->Struct.RXBxSIDH_Reg << 3) + (RXBx->Struct.RXBxSIDL_Reg.value >> 5));
	if(cobId == cobIdEmcy)
		return EPOS4_FaultError;

	return EPOS4_NoError;
}

static EPOS4_Error_e CheckForAbort(uint8_t deviceIndex, uint8_t *data)
{
	if(data[0] >> 7)
		return EPOS4_AbortError;

	return EPOS4_NoError;
}

static uint8_t WriteFirstStepObjects(uint8_t deviceIndex, EPOS4_FirstStep_t FirstStep)
{
	uint32_t value;

	WriteObjectValue(deviceIndex, CAN_BITRATE_INDEX, 0, FirstStep.CAN_BitRate);
	ReadObjectValue(deviceIndex, CAN_BITRATE_INDEX, 0, &value);
	if(value != FirstStep.CAN_BitRate)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_TYPE_INDEX, 0, FirstStep.MotorType);
	ReadObjectValue(deviceIndex, MOTOR_TYPE_INDEX, 0, &value);
	if(value != FirstStep.MotorType)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX, FirstStep.nominalCurrent);
	ReadObjectValue(deviceIndex, MOTOR_DATA_INDEX, NOMINAL_CURRENT_SUBINDEX, &value);
	if(value != FirstStep.nominalCurrent)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX, FirstStep.outputCurrentLimit);
	ReadObjectValue(deviceIndex, MOTOR_DATA_INDEX, OUTPUT_CURRENT_LIMIT_SUBINDEX, &value);
	if(value != FirstStep.outputCurrentLimit)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_DATA_INDEX, NUMBER_OF_POLE_PAIRS_SUBINDEX, FirstStep.numberOfPolePairs);
	ReadObjectValue(deviceIndex, MOTOR_DATA_INDEX, NUMBER_OF_POLE_PAIRS_SUBINDEX, &value);
	if(value != FirstStep.numberOfPolePairs)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_DATA_INDEX, THERMAL_TIME_CONSTANT_WINDING_SUBINDEX, FirstStep.thermalTimeConstantWinding);
	ReadObjectValue(deviceIndex, MOTOR_DATA_INDEX, THERMAL_TIME_CONSTANT_WINDING_SUBINDEX, &value);
	if(value != FirstStep.thermalTimeConstantWinding)
		return 1;

	WriteObjectValue(deviceIndex, MOTOR_DATA_INDEX,TORQUE_CONSTANT_SUBINDEX, FirstStep.torqueConstant);
	ReadObjectValue(deviceIndex, MOTOR_DATA_INDEX,TORQUE_CONSTANT_SUBINDEX, &value);
	if(value != FirstStep.torqueConstant)
		return 1;

	WriteObjectValue(deviceIndex, MAX_MOTOR_SPEED_INDEX, 0, FirstStep.maxMotorSpeed);
	ReadObjectValue(deviceIndex, MAX_MOTOR_SPEED_INDEX, 0, &value);
	if(value != FirstStep.maxMotorSpeed)
		return 1;

	WriteObjectValue(deviceIndex, GEAR_CONFIGURATION_INDEX, MAX_GEAR_INPUT_SPEED_SUBINDEX, FirstStep.maxGearInputSpeed);
	ReadObjectValue(deviceIndex, GEAR_CONFIGURATION_INDEX, MAX_GEAR_INPUT_SPEED_SUBINDEX, &value);
	if(value != FirstStep.maxGearInputSpeed)
		return 1;

	WriteObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, SENSOR_CONFIGURATION_SUBINDEX, FirstStep.sensorsConfiguration);
	ReadObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, SENSOR_CONFIGURATION_SUBINDEX, &value);
	if(value != FirstStep.sensorsConfiguration)
		return 1;

	WriteObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, CONTROL_STRUCTURE_SUBINDEX, FirstStep.controlStructure);
	ReadObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, CONTROL_STRUCTURE_SUBINDEX, &value);
	if(value != FirstStep.controlStructure)
		return 1;

	WriteObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, COMMUTATION_SENSORS_SUBINDEX, FirstStep.commutationSensors);
	ReadObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, COMMUTATION_SENSORS_SUBINDEX, &value);
	if(value != FirstStep.commutationSensors)
		return 1;

	WriteObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, AXIS_CONFIG_MISCELLANEOUS_SUBINDEX, FirstStep.axisConfigMiscellaneous);
	ReadObjectValue(deviceIndex, AXIS_CONFIGURATION_INDEX, AXIS_CONFIG_MISCELLANEOUS_SUBINDEX, &value);
	if(value != FirstStep.axisConfigMiscellaneous)
		return 1;

	WriteObjectValue(deviceIndex, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_P_GAIN_SUBINDEX, FirstStep.currentControllerP_Gain);
	ReadObjectValue(deviceIndex, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_P_GAIN_SUBINDEX, &value);
	if(value != FirstStep.currentControllerP_Gain)
		return 1;

	WriteObjectValue(deviceIndex, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_I_GAIN_SUBINDEX, FirstStep.currentControllerI_Gain);
	ReadObjectValue(deviceIndex, CURRENT_CTRL_PARAMETER_SET_INDEX, CURRENT_CONTROLLER_I_GAIN_SUBINDEX, &value);
	if(value != FirstStep.currentControllerI_Gain)
		return 1;

	return 0;
}

static uint8_t WriteOptionCodes(uint8_t deviceIndex)
{
	uint32_t value;
	WriteObjectValue(deviceIndex, ABORT_CONNECTION_OPTION_CODE_INDEX, 0, 2);
	ReadObjectValue(deviceIndex, ABORT_CONNECTION_OPTION_CODE_INDEX, 0, &value);
	if(value != 2)
		return 1;

	WriteObjectValue(deviceIndex, SHUTDOWN_OPTION_CODE_INDEX, 0, 0);
	ReadObjectValue(deviceIndex, SHUTDOWN_OPTION_CODE_INDEX, 0, &value);
	if(value != 0)
		return 1;

	WriteObjectValue(deviceIndex, DISABLE_OPERATION_OPTION_CODE_INDEX, 0, 0);
	ReadObjectValue(deviceIndex, DISABLE_OPERATION_OPTION_CODE_INDEX, 0, &value);
	if(value != 0)
		return 1;

	WriteObjectValue(deviceIndex, FAULT_REACTION_OPTION_CODE_INDEX, 0, 0);
	ReadObjectValue(deviceIndex, FAULT_REACTION_OPTION_CODE_INDEX, 0, &value);
	if(value != 0)
		return 1;

	return 0;
}

static uint8_t WriteModeOfOperation(uint8_t deviceIndex, EPOS4_ModeOfOperation_e modeOfOperation)
{
	uint32_t value;
	switch (modeOfOperation)
	{
	case EPOS4_CyclicSynchronousTorqueMode:
		WriteObjectValue(deviceIndex, TARGET_TORQUE_INDEX, 0, 0);
		ReadObjectValue(deviceIndex, TARGET_TORQUE_INDEX, 0, &value);
		if(value != 0)
			return 1;

		WriteObjectValue(deviceIndex, MODES_OF_OPERATION_INDEX, 0, CST_MODE);
		ReadObjectValue(deviceIndex, MODES_OF_OPERATION_INDEX, 0, &value);
		if(value != CST_MODE)
			return 1;

		WriteObjectValue(deviceIndex, CONTROLWORD_INDEX, 0, CTRLCMD_SHUTDOWN);
		do
		{
			ReadObjectValue(deviceIndex, STATUSWORD_INDEX, 0, &value);
			value = value & STATE_MASK;
		} while(value != STATE_READY_TO_SWITCH_ON);

		WriteObjectValue(deviceIndex, CONTROLWORD_INDEX, 0, CTRLCMD_SWITCH_ON_AND_ENABLE);
		do
		{
			ReadObjectValue(deviceIndex, STATUSWORD_INDEX, 0, &value);
			value = value & STATE_MASK;
		} while(value != STATE_OPERATION_ENABLED);

		WriteObjectValue(deviceIndex, TORQUE_OFFSET_INDEX, 0, 0);
		ReadObjectValue(deviceIndex, TORQUE_OFFSET_INDEX, 0, &value);
		if(value != 0)
			return 1;

		return 0;
	}

	return 1;
}


/*******************************************************************************
* END
*******************************************************************************/
