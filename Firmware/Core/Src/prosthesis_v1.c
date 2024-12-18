/*******************************************************************************
*
* TITLE:	Application for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	12/15/2024
*
* NOTES
* 1. Unless otherwise specified, units are
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

typedef enum
{
	EarlyStance,
	MidStance,
	LateStance,
	SwingFlexion,
	SwingExtension
} StateMachine_e;

typedef struct
{
	float eqPoint;
	float kd;
	float kp;
} ControlParams_t;

typedef struct
{
	double jointAngle[2];			// [0] = k-0, [1] = k-1
	double jointSpeed;
	double limbSpeed;
	float encoderBias;
	float jointTorque;
	float speedThreshold;
	ControlParams_t ProsCtrl;
	ControlParams_t EarlyStanceCtrl;
	ControlParams_t MidStanceCtrl;
	ControlParams_t LateStanceCtrl;
	ControlParams_t SwingFlexCtrl;
	ControlParams_t SwingExtCtrl;
} Joint_t;

typedef struct
{
	struct
	{
		float bot[3];
		float top[3];
	} Raw;
	struct
	{
		float bot[3];
		float top[3];
	} Filtered;
	float outOfStanceThreshold;
	float intoStanceThreshold;
} LoadCell_t;

static Prosthesis_Init_t Device;
static TestProgram_e testProgram;

static double dt = 1 / 512.0;
static MPU925x_IMU_Data_t IMU_Data;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static Joint_t CM_Ankle, CM_Knee;
static LoadCell_t CM_LoadCell;			// [0] = k-0, [1] = k-1, [2] = k-2
static uint16_t CM_ankleRawEncoderBias, CM_kneeRawEncoderBias;
static Utils_IMU_Data_t CM_IMU_Data;

static uint16_t CM_state = 1200;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void RunStateMachine(void);
static void RunImpedanceControl(void);
static void RunTestProgram(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

// Includes variables that are subject to change during testing for convenience
void InitProsthesisControl(Prosthesis_Init_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(&Device_Init));

	memset(&CM_Ankle, 0, sizeof(CM_Ankle));
	memset(&CM_Knee, 0, sizeof(CM_Knee));

	CM_Ankle.encoderBias = 1325 * AS5145B_RAW2DEG;
	CM_Ankle.speedThreshold = -5.0f;

	CM_Knee.encoderBias = 2244 * AS5145B_RAW2DEG;

	CM_LoadCell.intoStanceThreshold = 1347;
	CM_LoadCell.outOfStanceThreshold = CM_LoadCell.intoStanceThreshold + 50;
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
	else
	{
		RunStateMachine();
		RunImpedanceControl();
	}

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
	{
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition(AnkleEncoderIndex) - CM_Ankle.encoderBias;
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		CM_Knee.jointAngle[0] = AS5145B_ReadPosition(KneeEncoderIndex) - CM_Knee.encoderBias;
	}

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
	double tau = 1.0 / (2 * M_PI * 10);	// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of joint angle (joint speed) and filtering of load cells
	if(isFirst)
	{
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			CM_Ankle.jointSpeed = 0.0;
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == Knee) || (Device.Joint == Combined))
		{
			CM_Knee.jointSpeed = 0.0;
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

		CM_LoadCell.Raw.bot[2] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[2] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[2] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[2] = CM_LoadCell.Filtered.top[0];
	}
	else if(isSecond)
	{
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == Knee) || (Device.Joint == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*tau - dt)*CM_Knee.jointSpeed) / (dt + 2*tau);
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

		CM_LoadCell.Raw.bot[1] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Raw.top[1] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[0] = CM_LoadCell.Raw.bot[0];
		CM_LoadCell.Filtered.top[0] = CM_LoadCell.Raw.top[0];
		CM_LoadCell.Filtered.bot[1] = CM_LoadCell.Filtered.bot[0];
		CM_LoadCell.Filtered.top[1] = CM_LoadCell.Filtered.top[0];
	}
	else
	{
		if((Device.Joint == Ankle) || (Device.Joint == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == Knee) || (Device.Joint == Combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Knee.jointSpeed = (2*(CM_Knee.jointAngle[0] - CM_Knee.jointAngle[1]) + (2*tau - dt)*CM_Knee.jointSpeed) / (dt + 2*tau);
			CM_Knee.jointAngle[1] = CM_Knee.jointAngle[0];
		}

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

	double cosines[3];
	if(Device.Side == Left)
	{
		cosines[0] = -1;
		cosines[1] = -1;
		cosines[2] = 1;
	}
	else
	{
		cosines[0] = 1;
		cosines[1] = 1;
		cosines[2] = 1;
	}

	double biases[6] = {0,0,0,0,0,0};
	double sines[3] = {0,0,0};
	CM_IMU_Data = CalibrateIMU(IMU_Data.array, biases, 1, cosines, sines);

	CM_Ankle.limbSpeed = CM_IMU_Data.gz + CM_Ankle.jointSpeed;
}

static void RunStateMachine(void)
{
	static StateMachine_e state = EarlyStance;
	switch(state)
	{
	case EarlyStance:
		CM_state = 1200;

		if(testProgram == None)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.EarlyStanceCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.EarlyStanceCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.EarlyStanceCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.EarlyStanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.EarlyStanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.EarlyStanceCtrl.kp;
		}

		if(CM_Ankle.limbSpeed > CM_Ankle.speedThreshold)
			state = MidStance;

		break;

	case MidStance:
		CM_state = 1300;

		if(testProgram == None)
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
		CM_state = 1400;

		if(testProgram == None)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.LateStanceCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.LateStanceCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.LateStanceCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.LateStanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.LateStanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.LateStanceCtrl.kp;
		}

		if(CM_LoadCell.Filtered.bot[0] > CM_LoadCell.outOfStanceThreshold)
			state = SwingExtension;

		break;

	case SwingFlexion:
		CM_state = 1500;

		if(testProgram == None)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.SwingFlexCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.SwingFlexCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.SwingFlexCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingFlexCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingFlexCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingFlexCtrl.kp;
		}

		if(CM_Knee.jointSpeed > CM_Knee.speedThreshold)
			state = SwingExtension;

		break;

	case SwingExtension:
		CM_state = 1600;

		if(testProgram == None)
		{
			CM_Ankle.ProsCtrl.eqPoint = CM_Ankle.SwingExtCtrl.eqPoint;
			CM_Ankle.ProsCtrl.kd = CM_Ankle.SwingExtCtrl.kd;
			CM_Ankle.ProsCtrl.kp = CM_Ankle.SwingExtCtrl.kp;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingExtCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingExtCtrl.kp;
		}

		if(CM_LoadCell.Filtered.bot[0] < CM_LoadCell.intoStanceThreshold)
			state = EarlyStance;

		break;
	}
}

static void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent = 8.0f;
	float torqueConst = 60.0f / (2 * M_PI * 100);	// For Kv = 100 rpm/V
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		float errorPos = CM_Ankle.ProsCtrl.eqPoint - CM_Ankle.jointAngle[0];
		CM_Ankle.jointTorque = (CM_Ankle.ProsCtrl.kp*errorPos - CM_Ankle.ProsCtrl.kd*CM_Ankle.jointSpeed);
		int16_t motorTorque = CM_Ankle.jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_Error_e error = EPOS4_WriteTargetTorqueValue(AnkleMotorControllerIndex, motorTorque);
		if(error)
			ErrorHandler_EPOS4(AnkleMotorControllerIndex, error);

	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		float errorPos = CM_Knee.ProsCtrl.eqPoint - CM_Knee.jointAngle[0];
		CM_Knee.jointTorque = (CM_Knee.ProsCtrl.kp*errorPos - CM_Knee.ProsCtrl.kd*CM_Knee.jointSpeed);
		int16_t motorTorque = CM_Knee.jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_Error_e error = EPOS4_WriteTargetTorqueValue(KneeMotorControllerIndex, -motorTorque);		// Knee joint rotates opposite of coordinate system
		if(error)
			ErrorHandler_EPOS4(AnkleMotorControllerIndex, error);
	}
}

static void RunTestProgram(void)
{
	switch (testProgram)
	{
	case None:
		break;

	case ReadOnly:
		break;

	case EncoderBias:
		if(Device.Joint == Ankle || Device.Joint == Combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(AnkleEncoderIndex);

			CM_ankleRawEncoderBias = sum / i;
		}

		if(Device.Joint == Knee || Device.Joint == Combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(KneeEncoderIndex);

			CM_kneeRawEncoderBias = sum / i;
		}

		break;

	case ImpedanceControl:
		if(isFirst)
		{
			if(Device.Joint == Ankle || Device.Joint == Combined)
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

			if(Device.Joint == Knee || Device.Joint == Combined)
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


		RunImpedanceControl();

		break;
	}
}


/*******************************************************************************
* END
*******************************************************************************/
