/*******************************************************************************
*
* TITLE: Application for Prosthesis v1
*
* NOTES
* 1. IMPORTANT: A new encoder bias position must be found and defined whenever the magnet is reassembled into the device.
*    A test program is provided to find the bias.
*    The biases are defined in Prosthesis_Init() below.
* 2. Unless otherwise specified, units are
* 		- Accelerometer	= g's
* 		- Angle			= degrees
* 		- Current		= Amperes
* 		- Gyroscope		= degrees/second
* 		- Load Cell		= ADC
* 		- Torque		= N*m
* 		- Speed			= degrees/second
*
*******************************************************************************/

#include "as5145b.h"
#include "epos4.h"
#include "error_handler.h"
#include "main.h"
#include <math.h>
#include "mcp25625.h"
#include "mpu925x_spi.h"
#include "prosthesis_v1.h"
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_ll_adc.h"
#include "utilities.h"


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

#define CURRENT_LIMIT		29.3f
#define DT					1 / 512.0										// Sample time
#define GEAR_RATIO			40.0f
#define NOMINAL_CURRENT		CURRENT_LIMIT / 2
#define TAU					1.0 / (2 * 3.1416 * 10)							// Time constant for practical differentiator (fc = 10 Hz)
#define TORQUE_CONSTANT		60.0f / (2 * 3.1416f * 100)						// For Kv = 100 rpm/V
#define MAX_JOINT_TORQUE	CURRENT_LIMIT * TORQUE_CONSTANT * GEAR_RATIO

typedef enum
{
	EarlyStance,
	MidStance,
	LateStance,
	SwingFlexion,
	SwingExtension,
	SwingDescension
} StateMachine_e;

typedef struct
{
	float eqPoint;
	float kd;
	float kp;
} ControlParams_t;

typedef struct
{
	double jointAngle[2];				// [0] = k-0, [1] = k-1
	double jointSpeed;
	double limbSpeed;
	float encoderBias;
	float jointTargetTorque;
	float jointTorqueActual;
	ControlParams_t ProsCtrl;
	ControlParams_t EarlyStanceCtrl;
	ControlParams_t MidStanceCtrl;
	ControlParams_t LateStanceCtrl;
	ControlParams_t SwingFlexCtrl;
	ControlParams_t SwingExtCtrl;
	ControlParams_t SwingDescCtrl;
} Joint_t;

typedef struct
{
	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Raw;
	struct
	{
		float bot[3];	// [0] = k-0, [1] = k-1, [2] = k-2
		float top[3];	// [0] = k-0, [1] = k-1, [2] = k-2
	} Filtered;
	float outOfStanceThreshold;
	float intoStanceThreshold;
} LoadCell_t;

static MPU925x_IMU_Data_t IMU_Data;
static Prosthesis_Init_t Device;
static TestProgram_e testProgram;

static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static float CM_footSpeedThreshold = 0.0f;
static int8_t CM_state_angles, CM_state_torques;
static int16_t CM_state_speeds;
static Joint_t CM_Ankle, CM_Knee;
static LoadCell_t CM_LoadCell;
static MPU925x_IMU_Data_t CM_IMU_Data;
static uint16_t CM_ankleRawEncoderBias, CM_kneeRawEncoderBias;
static uint16_t CM_state_loadCells;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void RunStateMachine(void);
static void RunImpedanceControl(void);
static void RunTestProgram(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(&Device_Init));

	float startEqPoint = -11.0f;
	float startKd = 0.00f;
	float startKp = 5.0f;

	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		CM_Ankle.encoderBias = 1357 * AS5145B_RAW2DEG;

		CM_Ankle.EarlyStanceCtrl.eqPoint = startEqPoint;
		CM_Ankle.EarlyStanceCtrl.kd = startKd;
		CM_Ankle.EarlyStanceCtrl.kp = startKp;

		CM_Ankle.MidStanceCtrl.eqPoint = startEqPoint;
		CM_Ankle.MidStanceCtrl.kd = startKd;
		CM_Ankle.MidStanceCtrl.kp = startKp;

		CM_Ankle.LateStanceCtrl.eqPoint = startEqPoint;
		CM_Ankle.LateStanceCtrl.kd = startKd;

		CM_Ankle.SwingFlexCtrl.eqPoint = startEqPoint;
		CM_Ankle.SwingFlexCtrl.kd = startKd;
		CM_Ankle.SwingFlexCtrl.kp = startKp;

		CM_Ankle.SwingExtCtrl.eqPoint = startEqPoint;
		CM_Ankle.SwingExtCtrl.kd = startKd;
		CM_Ankle.SwingExtCtrl.kp = startKp;

		CM_Ankle.SwingDescCtrl.eqPoint = startEqPoint;
		CM_Ankle.SwingDescCtrl.kd = startKd;
		CM_Ankle.SwingDescCtrl.kp = startKp;
	}
	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		CM_Knee.encoderBias = 2244 * AS5145B_RAW2DEG;

		CM_Knee.EarlyStanceCtrl.eqPoint = 2.0f;
		CM_Knee.EarlyStanceCtrl.kd = 0.00f;
		CM_Knee.EarlyStanceCtrl.kp = 0.00f;

		CM_Knee.MidStanceCtrl.eqPoint = 0.0f;
		CM_Knee.MidStanceCtrl.kd = 0.00f;
		CM_Knee.MidStanceCtrl.kp = 0.00f;

		CM_Knee.LateStanceCtrl.eqPoint = 8.0f;
		CM_Knee.LateStanceCtrl.kd = 0.00f;
		CM_Knee.LateStanceCtrl.kp = 0.00f;

		CM_Knee.SwingFlexCtrl.eqPoint = 65.0f;
		CM_Knee.SwingFlexCtrl.kd = 0.00f;
		CM_Knee.SwingFlexCtrl.kp = 0.12f;

		CM_Knee.SwingExtCtrl.eqPoint = 22.0f;
		CM_Knee.SwingExtCtrl.kd = 0.00f;
		CM_Knee.SwingExtCtrl.kp = 0.13f;
	}

	CM_LoadCell.intoStanceThreshold = 1300;
	CM_LoadCell.outOfStanceThreshold = 1300 + 50;
}

void RequireTestProgram(TestProgram_e option)
{
	testProgram = option;
	if(testProgram != None)
		isTestProgramRequired = 1;
}

void RunProsthesisControl(void)
{
	GetInputs();
	ProcessInputs();

	if(isTestProgramRequired)
		RunTestProgram();

	RunStateMachine();
	RunImpedanceControl();

	// Check for first and second executions, needed for derivatives, filters, etc.
	if(isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if(isSecond)
		isSecond = 0;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs(void)
{
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition(AnkleEncoderIndex) - CM_Ankle.encoderBias;

	if((Device.Joint == Knee) || (Device.Joint == Combined))
		CM_Knee.jointAngle[0] = - (AS5145B_ReadPosition(KneeEncoderIndex) - CM_Knee.encoderBias);

	CM_LoadCell.Raw.bot[0] = ReadLoadCell(ADC1);
	CM_LoadCell.Raw.top[0] = ReadLoadCell(ADC2);

	IMU_Data = MPU925x_ReadIMU(0);
}

static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while (!LL_ADC_IsActiveFlag_EOC(ADCx));
	uint16_t data = LL_ADC_REG_ReadConversionData12(ADCx);
	return data;
}

static void ProcessInputs(void)
{
	// Derivative of joint angle (joint speed) and filtering of load cells
	if(isFirst)
	{
		CM_Ankle.jointSpeed = 0.0;
		CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];

		CM_Knee.jointSpeed = 0.0;
		CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];

		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[0];
	}
	else if(isSecond)
	{
		// Practical differentiator (bilinear transformation used)
		CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*TAU - DT)*CM_Ankle.jointSpeed) / (DT + 2*TAU);
		CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];

		// Practical differentiator (bilinear transformation used)
		CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*TAU - DT)*CM_Knee.jointSpeed) / (DT + 2*TAU);
		CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];

		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}
	else
	{
		// Practical differentiator (bilinear transformation used)
		CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*TAU - DT)*CM_Ankle.jointSpeed) / (DT + 2*TAU);
		CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];

		// Practical differentiator (bilinear transformation used)
		CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*TAU - DT)*CM_Knee.jointSpeed) / (DT + 2*TAU);
		CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_LoadCell.Filtered.bot[0] =   1.6556 * CM_LoadCell.Filtered.bot[1] - 0.7068 * CM_LoadCell.Filtered.bot[2]
									  + 0.0128 * CM_LoadCell.Raw.bot[0] + 0.0256 * CM_LoadCell.Raw.bot[1] + 0.0128 * CM_LoadCell.Raw.bot[2];
		CM_LoadCell.Filtered.top[0] =   1.6556 * CM_LoadCell.Filtered.top[1] - 0.7068 * CM_LoadCell.Filtered.top[2]
									  + 0.0128 * CM_LoadCell.Raw.top[0] + 0.0256 * CM_LoadCell.Raw.top[1] + 0.0128 * CM_LoadCell.Raw.top[2];

		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[1];
		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[1];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[1];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[1];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}

	if(Device.Side == Left)
	{
		CM_IMU_Data.Struct.ax = -IMU_Data.Struct.ax;
		CM_IMU_Data.Struct.ay = IMU_Data.Struct.ay;
		CM_IMU_Data.Struct.az = -IMU_Data.Struct.az;
		CM_IMU_Data.Struct.gx = -IMU_Data.Struct.gx;
		CM_IMU_Data.Struct.gy = IMU_Data.Struct.gy;
		CM_IMU_Data.Struct.gz = -IMU_Data.Struct.gz;
	}
	else
		memcpy(&CM_IMU_Data, &IMU_Data, sizeof(IMU_Data));

	CM_Ankle.limbSpeed = CM_IMU_Data.Struct.gz + CM_Ankle.jointSpeed;
}

static void RunStateMachine(void)
{
	static StateMachine_e state = EarlyStance;
	static uint8_t isFirstCallForLateStance = 1;
	switch(state)
	{
	case EarlyStance:
		CM_state_angles = -10;
		CM_state_loadCells = 1100;
		CM_state_torques = -30;
		CM_state_speeds = -200;

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.EarlyStanceCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.EarlyStanceCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.EarlyStanceCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.EarlyStanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.EarlyStanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.EarlyStanceCtrl.kp;
		}

		if(CM_Ankle.limbSpeed > CM_footSpeedThreshold)
			state = MidStance;

		break;

	case MidStance:
		CM_state_angles = 5;
		CM_state_loadCells = 1200;
		CM_state_torques = -20;
		CM_state_speeds = -120;
		isFirstCallForLateStance = 1;

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.MidStanceCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.MidStanceCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.MidStanceCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.MidStanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.MidStanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.MidStanceCtrl.kp;
		}

		if(CM_Ankle.jointSpeed < 0)
			state = LateStance;

		break;

	case LateStance:
		CM_state_angles = 20;
		CM_state_loadCells = 1300;
		CM_state_torques = -10;
		CM_state_speeds = -40;

		// Compute kp to start with previous torque when first called
		if(isFirstCallForLateStance)
		{
			CM_Ankle.LateStanceCtrl.kp = (CM_Ankle.jointTargetTorque + CM_Ankle.jointSpeed*CM_Ankle.LateStanceCtrl.kd) / (CM_Ankle.LateStanceCtrl.eqPoint - CM_Ankle.jointAngle[0]);
			if(CM_Ankle.LateStanceCtrl.kp < 0)
			{
				state = MidStance;
				break;
			}
			else
				isFirstCallForLateStance = 0;
		}

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.LateStanceCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.LateStanceCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.LateStanceCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.LateStanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.LateStanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.LateStanceCtrl.kp;
		}

		if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.outOfStanceThreshold)
			state = SwingFlexion;

		break;

	case SwingFlexion:
		CM_state_angles = 35;
		CM_state_loadCells = 1400;
		CM_state_torques = 0;
		CM_state_speeds = 40;
		isFirstCallForLateStance = 1;

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.SwingFlexCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.SwingFlexCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.SwingFlexCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingFlexCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingFlexCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingFlexCtrl.kp;
		}

		if(CM_Knee.jointSpeed < 0)
			state = SwingExtension;

		break;

	case SwingExtension:
		CM_state_angles = 50;
		CM_state_loadCells = 1500;
		CM_state_torques = 10;
		CM_state_speeds = 120;

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.SwingExtCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.SwingExtCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.SwingExtCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingExtCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingExtCtrl.kp;
		}

		if(CM_Ankle.limbSpeed < 0)
			state = SwingDescension;

		break;

	case SwingDescension:
		CM_state_angles = 65;
		CM_state_loadCells = 1600;
		CM_state_torques = 20;
		CM_state_speeds = 200;

		if(testProgram != ImpedanceControl)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.SwingDescCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.SwingDescCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.SwingDescCtrl.kp;

			CM_Knee.SwingDescCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
			CM_Knee.SwingDescCtrl.kd = CM_Knee.SwingExtCtrl.kd;
			CM_Knee.SwingDescCtrl.kp = CM_Knee.SwingExtCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingDescCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingDescCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingDescCtrl.kp;
		}

		if(CM_LoadCell.Filtered.bot[0] < CM_LoadCell.intoStanceThreshold)
			state = EarlyStance;

		break;
	}
}

static void RunImpedanceControl(void)
{
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		float errorPos = CM_Ankle.ProsCtrl.eqPoint - CM_Ankle.jointAngle[0];

		float jointTargetTorque = (CM_Ankle.ProsCtrl.kp*errorPos - CM_Ankle.ProsCtrl.kd*CM_Ankle.jointSpeed);
		if(jointTargetTorque > MAX_JOINT_TORQUE)
			CM_Ankle.jointTargetTorque = MAX_JOINT_TORQUE;
		else if(jointTargetTorque < -MAX_JOINT_TORQUE)
			CM_Ankle.jointTargetTorque = -MAX_JOINT_TORQUE;
		else
			CM_Ankle.jointTargetTorque = jointTargetTorque;

		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			int16_t targetTorque = CM_Ankle.jointTargetTorque / (TORQUE_CONSTANT * GEAR_RATIO * NOMINAL_CURRENT) * 1000;
			EPOS4_Error_e error = EPOS4_WriteTargetTorqueValue(AnkleMotorControllerIndex, targetTorque);
			if(error)
				ErrorHandler_EPOS4(AnkleMotorControllerIndex, error);

			int16_t torqueActual;
			error = EPOS4_ReadTorqueActualValue(AnkleMotorControllerIndex, &torqueActual);
			if(error)
				ErrorHandler_EPOS4(AnkleMotorControllerIndex, error);

			CM_Ankle.jointTorqueActual = (float) torqueActual * (TORQUE_CONSTANT * GEAR_RATIO * NOMINAL_CURRENT) / 1000;
		}
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		float errorPos = CM_Knee.ProsCtrl.eqPoint - CM_Knee.jointAngle[0];

		float jointTargetTorque = (CM_Knee.ProsCtrl.kp*errorPos - CM_Knee.ProsCtrl.kd*CM_Knee.jointSpeed);
		if(jointTargetTorque > MAX_JOINT_TORQUE)
			CM_Knee.jointTargetTorque = MAX_JOINT_TORQUE;
		else if(jointTargetTorque < -MAX_JOINT_TORQUE)
			CM_Knee.jointTargetTorque = -MAX_JOINT_TORQUE;
		else
			CM_Knee.jointTargetTorque = jointTargetTorque;

		if((testProgram == None) || (testProgram == ImpedanceControl))
		{
			int16_t targetTorque = CM_Knee.jointTargetTorque / (TORQUE_CONSTANT * GEAR_RATIO * NOMINAL_CURRENT) * 1000;
			EPOS4_Error_e error = EPOS4_WriteTargetTorqueValue(KneeMotorControllerIndex, targetTorque);
			if(error)
				ErrorHandler_EPOS4(KneeMotorControllerIndex, error);

			int16_t torqueActual;
			error = EPOS4_ReadTorqueActualValue(KneeMotorControllerIndex, &torqueActual);
			if(error)
				ErrorHandler_EPOS4(KneeMotorControllerIndex, error);

			CM_Knee.jointTorqueActual = (float) torqueActual * (TORQUE_CONSTANT * GEAR_RATIO * NOMINAL_CURRENT) / 1000;
		}
	}
}

static void RunTestProgram(void)
{
	switch (testProgram)
	{
	case None:
		break;

	case EncoderBias:
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			static uint32_t sum = 0;
			static uint16_t count = 0;
			sum += AS5145B_ReadPosition_Raw(AnkleEncoderIndex);
			count++;
			if(count == 10)
			{
				CM_ankleRawEncoderBias = sum/count;
				sum = 0;
				count = 0;
			}
		}

		if((Device.Joint == Knee) || (Device.Joint == Combined))
		{
			static uint32_t sum = 0;
			static uint16_t count = 0;
			sum += AS5145B_ReadPosition_Raw(KneeEncoderIndex);
			count++;
			if(count == 10)
			{
				CM_kneeRawEncoderBias = sum/count;
				sum = 0;
				count = 0;
			}
		}

		break;

	case ImpedanceControl:
		if(isFirst)
		{
			if((Device.Joint == Ankle) || (Device.Joint == Combined))
			{
				uint16_t i;
				float sum = 0.0f;
				for(i = 0; i < 1000; i++)
				{
					float position = AS5145B_ReadPosition(AnkleEncoderIndex);
					sum += position;
				}

				CM_Ankle.ProsCtrl.eqPoint = sum / i - CM_Ankle.encoderBias;
			}

			if((Device.Joint == Knee) || (Device.Joint == Combined))
			{
				uint16_t i;
				float sum = 0.0f;
				for(i = 0; i < 1000; i++)
				{
					float position = AS5145B_ReadPosition(KneeEncoderIndex);
					sum += position;
				}

				CM_Knee.ProsCtrl.eqPoint = sum / i - CM_Knee.encoderBias;
			}
		}

		break;
	}
}


/*******************************************************************************
* END
*******************************************************************************/
