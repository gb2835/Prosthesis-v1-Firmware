/*******************************************************************************
*
* TITLE:	Application for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	??
*
* NOTES
* 1. Unless otherwise specified, units are
* 		- Accelerometer = g's
* 		- Angle         = degrees
* 		- Current       = Amperes
* 		- Gyroscope     = degress/second
* 		- Load Cell     = ADC
* 		- Torque        = N*m
* 		- Speed         = degrees/second
*
*******************************************************************************/

#include "as5145b.h"
#include "epos4.h"
#include "main.h"
#include <math.h>
#include "mcp25625.h"
#include "mpu925x_spi.h"
#include "prosthesis_v1.h"
#include <stdint.h>
#include <string.h>
#include "stm32l4xx_ll_adc.h"


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
	double limbAngle;
	double limbSpeed;
	float jointTorque;
	ControlParams_t ProsCtrl;
	ControlParams_t StanceCtrl;
	ControlParams_t SwingFlexCtrl;
	ControlParams_t SwingExtCtrl;
} DeviceParams_t;

typedef struct
{
	double bot[3];
	double top[3];
} LoadCell_Data_t;

static TestPrograms_e testProgram;
static float ankleEncBias, kneeEncBias;
static Prosthesis_Init_t Device;
static LoadCell_Data_t LoadCell[3];					// [0] = k-0, [1] = k-1, [2] = k-2
static MPU925x_IMU_Data_t IMU_Data;

static double dt = 1 / 512.0;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static float CM_lcBot_upperBound, CM_lcBot_lowerBound;
static float CM_lcTop_upperBound, CM_lcTop_lowerBound;
static float CM_kneeSpeedThreshold, CM_ankleSpeedThreshold;
static DeviceParams_t CM_Ankle, CM_Knee;
static MPU925x_IMU_Data_t CM_IMU_Data;
static LoadCell_Data_t CM_LoadCell_Filtered[3];				// [0] = k-0, [1] = k-1, [2] = k-2
static uint16_t CM_ankleEncBias, CM_kneeEncBias;

static uint16_t CM_state = 1200;

static void GetInputs(void);
static uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
static void ProcessInputs(void);
static void CalibrateIMU(void);
static void ComputeLimbAngle(void);
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

	CM_Ankle.ProsCtrl.kp = 1;

	ankleEncBias = 1325 * AS5145B_RAW2DEG;
	kneeEncBias = 2244 * AS5145B_RAW2DEG;

	CM_ankleSpeedThreshold = 0.0f;
	CM_kneeSpeedThreshold = 0.0f;

	CM_lcBot_lowerBound = 1398.0f;
	CM_lcBot_upperBound = 1425.0f;
	CM_lcTop_lowerBound = 1415.0f;
	CM_lcTop_upperBound = 1451.0f;
}

void RequireTestProgram(TestPrograms_e option)
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
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition(AnkleEncoderIndex) - ankleEncBias;
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		CM_Knee.jointAngle[0] = AS5145B_ReadPosition(KneeEncoderIndex) - kneeEncBias;
	}

	LoadCell->bot[0] = ReadLoadCell(ADC1);
	LoadCell->top[0] = ReadLoadCell(ADC2);

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
	double tau = 1.0 / (2 * M_PI * 10);		// Time constant for practical differentiator (fc = 10 Hz)

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

		LoadCell->bot[2] = LoadCell->bot[0];
		LoadCell->top[2] = LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = LoadCell->top[0];
		CM_LoadCell_Filtered->bot[2] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[2] = CM_LoadCell_Filtered->top[0];
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

		LoadCell->bot[1] = LoadCell->bot[0];
		LoadCell->top[1] = LoadCell->top[0];
		CM_LoadCell_Filtered->bot[0] = LoadCell->bot[0];
		CM_LoadCell_Filtered->top[0] = LoadCell->top[0];
		CM_LoadCell_Filtered->bot[1] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[1] = CM_LoadCell_Filtered->top[0];
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
		CM_LoadCell_Filtered->bot[0] =   1.6556 * CM_LoadCell_Filtered->bot[1] - 0.7068 * CM_LoadCell_Filtered->bot[2]
									   + 0.0128 * LoadCell->bot[0] + 0.0256 * LoadCell->bot[1] + 0.0128 * LoadCell->bot[2];
		CM_LoadCell_Filtered->top[0] =   1.6556 * CM_LoadCell_Filtered->top[1] - 0.7068 * CM_LoadCell_Filtered->top[2]
									   + 0.0128 * LoadCell->top[0] + 0.0256 * LoadCell->top[1] + 0.0128 * LoadCell->top[2];

		LoadCell->bot[2] = LoadCell->bot[1];
		LoadCell->bot[1] = LoadCell->bot[0];
		LoadCell->top[2] = LoadCell->top[1];
		LoadCell->top[1] = LoadCell->top[0];
		CM_LoadCell_Filtered->bot[2] = CM_LoadCell_Filtered->bot[1];
		CM_LoadCell_Filtered->bot[1] = CM_LoadCell_Filtered->bot[0];
		CM_LoadCell_Filtered->top[2] = CM_LoadCell_Filtered->top[1];
		CM_LoadCell_Filtered->top[1] = CM_LoadCell_Filtered->top[0];
	}

	CalibrateIMU();
	ComputeLimbAngle();
}

static void CalibrateIMU(void)
{
	double axBias = 0.0;
	double ayBias = 0.0;
	double azBias = 0.0;
	double gxBias = 0.0;
	double gyBias = 0.0;
	double gzBias = 0.0;
	double n = 1.0;

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double c1, c2, c3, s1, s2, s3;
	if(Device.Side == Left)
	{
		c1 = -1.0;
		c2 = -1.0;
		c3 = 1.0;
		s1 = 0.0;
		s2 = 0.0;
		s3 = 0.0;
	}
	else
	{
		c1 = 1.0;
		c2 = 1.0;
		c3 = 1.0;
		s1 = 0.0;
		s2 = 0.0;
		s3 = 0.0;
	}

	CM_IMU_Data.ax = n * (IMU_Data.ax*(c1*c3 - c2*s1*s3) + IMU_Data.ay*(-c3*s1    - c1*c2*s3) + IMU_Data.az*( s2*s3)) - axBias;
	CM_IMU_Data.ay = n * (IMU_Data.ax*(c1*s3 + c2*c3*s1) + IMU_Data.ay*( c1*c2*c3 - s1*s3   ) + IMU_Data.az*(-c3*s2)) - ayBias;
	CM_IMU_Data.az = n * (IMU_Data.ax*(s1*s2           ) + IMU_Data.ay*( c1*s2              ) + IMU_Data.az*( c2   )) - azBias;
	CM_IMU_Data.gx = n * (IMU_Data.gx*(c1*c3 - c2*s1*s3) + IMU_Data.gy*(-c3*s1    - c1*c2*s3) + IMU_Data.gz*( s2*s3)) - gxBias;
	CM_IMU_Data.gy = n * (IMU_Data.gx*(c1*s3 + c2*c3*s1) + IMU_Data.gy*( c1*c2*c3 - s1*s3   ) + IMU_Data.gz*(-c3*s2)) - gyBias;
	CM_IMU_Data.gz = n * (IMU_Data.gx*(s1*s2           ) + IMU_Data.gy*( c1*s2              ) + IMU_Data.gz*( c2   )) - gzBias;
}

static void ComputeLimbAngle(void)
{
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		double accelAngle = (atan(CM_IMU_Data.ax / sqrt(pow(CM_IMU_Data.ay, 2) + pow(CM_IMU_Data.az, 2)))) * 180.0 / M_PI;
		static double compFiltAngle = 0.0;
		static double dGyroAngle = 0.0;

		dGyroAngle = dt/2 * (CM_IMU_Data.gz + dGyroAngle);	// Change in angle from gyro (trapezoidal used)

		// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
		double alpha = 0.002;
		compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

		CM_Ankle.limbAngle = compFiltAngle - CM_Ankle.jointAngle[0];
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		double accelAngle = (atan(CM_IMU_Data.ax / sqrt(pow(CM_IMU_Data.ay, 2) + pow(CM_IMU_Data.az, 2)))) * 180.0 / M_PI;
		static double compFiltAngle = 0.0;
		static double dGyroAngle = 0.0;

		dGyroAngle = dt/2 * (CM_IMU_Data.gz + dGyroAngle);	// Change in angle from gyro (trapezoidal used)

		// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
		double alpha = 0.002;
		compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

		CM_Knee.limbAngle = compFiltAngle - CM_Knee.jointAngle[0];
	}
}

static void RunStateMachine(void)
{
	if((Device.Joint == Ankle) || (Device.Joint == Combined))
	{
		CM_Ankle.ProsCtrl.eqPoint = 0.0f;
		CM_Ankle.ProsCtrl.kd = 0.0f;
		CM_Ankle.ProsCtrl.kp = 0.0f;
	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		static StateMachine_e State = EarlyStance;
		static uint8_t isCheckBoundsRequired = 0;

		switch(State)
		{
		case EarlyStance:
			CM_state = 1200;

			CM_Knee.ProsCtrl.eqPoint = CM_Knee.StanceCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.StanceCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.StanceCtrl.kp;

	        if(CM_LoadCell_Filtered->bot[0] < CM_lcBot_lowerBound && CM_LoadCell_Filtered->top[0] > CM_lcTop_upperBound)
	            isCheckBoundsRequired = 1;

			if(isCheckBoundsRequired)
			{
				uint8_t lcBotWithinBounds = (CM_LoadCell_Filtered->bot[0] < CM_lcBot_upperBound) && (CM_LoadCell_Filtered->bot[0] > CM_lcBot_lowerBound);
				uint8_t lcTopWithinBounds = (CM_LoadCell_Filtered->top[0] < CM_lcTop_upperBound) && (CM_LoadCell_Filtered->top[0] > CM_lcTop_lowerBound);
				if(lcBotWithinBounds && lcTopWithinBounds)
				{
					isCheckBoundsRequired = 0;
					State = SwingFlexion;
				}
			}

			break;

		case MidStance:
			CM_state = 1300;

			break;

		case LateStance:
			CM_state = 1400;

			break;

		case SwingFlexion:
			CM_state = 1500;
			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingFlexCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingFlexCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingFlexCtrl.kp;

			if(CM_Knee.jointSpeed > CM_kneeSpeedThreshold)
				State = SwingExtension;

			break;

		case SwingExtension:
			CM_state = 1600;
			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingExtCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingExtCtrl.kp;

			if(CM_LoadCell_Filtered->top[0] < CM_lcBot_lowerBound)
				State = EarlyStance;

			break;
		}
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
		int16_t motorTorque = CM_Ankle.jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;			// Units are N*mm
		EPOS4_WriteTargetTorqueValue(AnkleMotorControllerIndex, motorTorque);								// Ankle joint rotates opposite of coordinate system

	}

	if((Device.Joint == Knee) || (Device.Joint == Combined))
	{
		float errorPos = CM_Knee.ProsCtrl.eqPoint - CM_Knee.jointAngle[0];
		CM_Knee.jointTorque = (CM_Knee.ProsCtrl.kp*errorPos - CM_Knee.ProsCtrl.kd*CM_Knee.jointSpeed);
		int16_t motorTorque = CM_Knee.jointTorque / (torqueConst * gearRatio * nomCurrent) * 1000;		// Units are N*mm
		EPOS4_WriteTargetTorqueValue(KneeMotorControllerIndex, -motorTorque);							// Knee joint rotates opposite of coordinate system
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

	case ConstantMotorTorque100Nmm:
		if(Device.Joint == Ankle || Device.Joint == Combined)
			EPOS4_WriteTargetTorqueValue(AnkleMotorControllerIndex, -100);	// Ankle joint rotates opposite of coordinate system

		if(Device.Joint == Knee || Device.Joint == Combined)
			EPOS4_WriteTargetTorqueValue(KneeMotorControllerIndex, -100);	// Knee joint rotates opposite of coordinate system

		break;

	case EncoderBias:
		if(Device.Joint == Ankle || Device.Joint == Combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(AnkleEncoderIndex);

			CM_ankleEncBias = sum / i;
		}

		if(Device.Joint == Knee || Device.Joint == Combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(KneeEncoderIndex);

			CM_kneeEncBias = sum / i;
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

				CM_Ankle.ProsCtrl.eqPoint = sum / i - ankleEncBias;
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

				CM_Knee.ProsCtrl.eqPoint = sum / i - kneeEncBias;
			}
		}

		RunImpedanceControl();

		break;
	}
}


/*******************************************************************************
* END
*******************************************************************************/
