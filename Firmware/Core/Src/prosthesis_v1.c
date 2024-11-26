/*******************************************************************************
*
* TITLE:	Application for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	??
*
* NOTES
* 1. Unless otherwise specified, units are
* 		- Angle     = degrees
* 		- Torque    = N*m
* 		- Speed     = degrees/second
* 		- Load Cell = ADC
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
	stance,
	swingFlexion,
	swingExtension
} StateMachine_t;

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
	float jointTorque;
	ControlParams_t ProsCtrl;
	ControlParams_t StanceCtrl;
	ControlParams_t SwingFlexCtrl;
	ControlParams_t SwingExtCtrl;
	MPU925x_IMUData_t IMUData;
	uint8_t motorId;
} DeviceParams_t;

typedef struct
{
	double bot[3];
	double top[3];
} LoadCell_Data_t;

static TestPrograms_t testProgram;
static float ankleEncBias, kneeEncBias;
static Prosthesis_t Device;
static LoadCell_Data_t LoadCell[3];					// [0] = k-0, [1] = k-1, [2] = k-2
static MPU925x_IMUData_t AnkleIMUData, KneeIMUData;

static double dt = 1 / 512.0;
static uint8_t isFirst = 1;
static uint8_t isSecond = 0;
static uint8_t isTestProgramRequired = 0;

static float CM_lcBot_upperBound, CM_lcBot_lowerBound;
static float CM_lcTop_upperBound, CM_lcTop_lowerBound;
static float CM_kneeSpeedThreshold, CM_ankleSpeedThreshold;
static DeviceParams_t CM_Ankle, CM_Knee;
static LoadCell_Data_t CM_LoadCell_Filtered[3];				// [0] = k-0, [1] = k-1, [2] = k-2
static uint16_t CM_ankleEncBias, CM_kneeEncBias;
static uint16_t CM_state;

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
void InitProsthesisControl(Prosthesis_t *Device_Init)
{
	memcpy(&Device, Device_Init, sizeof(&Device_Init));

	memset(&CM_Ankle, 0, sizeof(CM_Ankle)); // check this??
	memset(&CM_Knee, 0, sizeof(CM_Knee));

	CM_Ankle.motorId = Device.ankleMotorId;
	CM_Knee.motorId = Device.kneeMotorId;

	ankleEncBias = 1325 * AS5145B_RAW2DEG;
	kneeEncBias = 2244 * AS5145B_RAW2DEG;

	CM_ankleSpeedThreshold = 0.0f;
	CM_kneeSpeedThreshold = 0.0f;

	CM_lcBot_lowerBound = 1398.0f;
	CM_lcBot_upperBound = 1425.0f;
	CM_lcTop_lowerBound = 1415.0f;
	CM_lcTop_upperBound = 1451.0f;
}

void RequireTestProgram(TestPrograms_t testProgram)
{
	if(testProgram != none)
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
	if((Device.Joint == ankle) || (Device.Joint == combined))
	{
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition(AnkleEncoderIndex) - ankleEncBias;
		AnkleIMUData = MPU925x_ReadIMU();
	}

	if((Device.Joint == knee) || (Device.Joint == combined))
	{
		CM_Knee.jointAngle[0] = AS5145B_ReadPosition(KneeEncoderIndex) - kneeEncBias;
		KneeIMUData = MPU925x_ReadIMU();
	}

	LoadCell->bot[0] = ReadLoadCell(ADC1);
	LoadCell->top[0] = ReadLoadCell(ADC2);
}

static uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t data = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return data;
}

static void ProcessInputs(void)
{
	double tau = 1.0 / (2 * M_PI * 10);		// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of joint angle (joint speed) and filtering of load cells
	if(isFirst)
	{
		if((Device.Joint == ankle) || (Device.Joint == combined))
		{
			CM_Ankle.jointSpeed = 0.0;
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == knee) || (Device.Joint == combined))
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
		if((Device.Joint == ankle) || (Device.Joint == combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == knee) || (Device.Joint == combined))
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
		if((Device.Joint == ankle) || (Device.Joint == combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Device.Joint == knee) || (Device.Joint == combined))
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
	double axBias;
	double ayBias;
	double azBias;
	double gxBias;
	double gyBias;
	double gzBias;
	double n;

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double c1, c2, c3, s1, s2, s3;

	if((Device.Joint == ankle) || (Device.Joint == combined))
	{
		axBias = 0.0;
		ayBias = 0.0;
		azBias = 0.0;
		gxBias = 0.0;
		gyBias = 0.0;
		gzBias = 0.0;
		n = 1.0;

		if(Device.Side == left)
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

		CM_Ankle.IMUData.ax = n * (AnkleIMUData.ax*(c1*c3 - c2*s1*s3) + AnkleIMUData.ay*(-c3*s1    - c1*c2*s3) + AnkleIMUData.az*( s2*s3)) - axBias;
		CM_Ankle.IMUData.ay = n * (AnkleIMUData.ax*(c1*s3 + c2*c3*s1) + AnkleIMUData.ay*( c1*c2*c3 - s1*s3   ) + AnkleIMUData.az*(-c3*s2)) - ayBias;
		CM_Ankle.IMUData.az = n * (AnkleIMUData.ax*(s1*s2           ) + AnkleIMUData.ay*( c1*s2              ) + AnkleIMUData.az*( c2   )) - azBias;
		CM_Ankle.IMUData.gx = n * (AnkleIMUData.gx*(c1*c3 - c2*s1*s3) + AnkleIMUData.gy*(-c3*s1    - c1*c2*s3) + AnkleIMUData.gz*( s2*s3)) - gxBias;
		CM_Ankle.IMUData.gy = n * (AnkleIMUData.gx*(c1*s3 + c2*c3*s1) + AnkleIMUData.gy*( c1*c2*c3 - s1*s3   ) + AnkleIMUData.gz*(-c3*s2)) - gyBias;
		CM_Ankle.IMUData.gz = n * (AnkleIMUData.gx*(s1*s2           ) + AnkleIMUData.gy*( c1*s2              ) + AnkleIMUData.gz*( c2   )) - gzBias;
	}

	if((Device.Joint == knee) || (Device.Joint == combined))
	{
		axBias = 0.0;
		ayBias = 0.0;
		azBias = 0.0;
		gxBias = 0.0;
		gyBias = 0.0;
		gzBias = 0.0;
		n = 1.0;

		if(Device.Side == left)
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

		CM_Knee.IMUData.ax = n * (KneeIMUData.ax*(c1*c3 - c2*s1*s3) + KneeIMUData.ay*(-c3*s1 - c1*c2*s3) + KneeIMUData.az*( s2*s3)) - axBias;
		CM_Knee.IMUData.ay = n * (KneeIMUData.ax*(c1*s3 + c2*c3*s1) + KneeIMUData.ay*( c1*c2*c3 - s1*s3) + KneeIMUData.az*(-c3*s2)) - ayBias;
		CM_Knee.IMUData.az = n * (KneeIMUData.ax*(s1*s2           ) + KneeIMUData.ay*( c1*s2           ) + KneeIMUData.az*( c2   )) - azBias;
		CM_Knee.IMUData.gx = n * (KneeIMUData.gx*(c1*c3 - c2*s1*s3) + KneeIMUData.gy*(-c3*s1 - c1*c2*s3) + KneeIMUData.gz*( s2*s3)) - gxBias;
		CM_Knee.IMUData.gy = n * (KneeIMUData.gx*(c1*s3 + c2*c3*s1) + KneeIMUData.gy*( c1*c2*c3 - s1*s3) + KneeIMUData.gz*(-c3*s2)) - gyBias;
		CM_Knee.IMUData.gz = n * (KneeIMUData.gx*(s1*s2           ) + KneeIMUData.gy*( c1*s2           ) + KneeIMUData.gz*( c2   )) - gzBias;
	}
}

static void ComputeLimbAngle(void)
{
	if((Device.Joint == ankle) || (Device.Joint == combined))
	{
		double accelAngle = (atan(CM_Ankle.IMUData.ax / sqrt(pow(CM_Ankle.IMUData.ay, 2) + pow(CM_Ankle.IMUData.az, 2)))) * 180.0 / M_PI;
		static double compFiltAngle = 0.0;
		static double dGyroAngle = 0.0;

		dGyroAngle = dt/2 * (CM_Ankle.IMUData.gz + dGyroAngle);	// Change in angle from gyro (trapezoidal used)

		// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
		double alpha = 0.002;
		compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

		CM_Ankle.limbAngle = compFiltAngle - CM_Ankle.jointAngle[0];
	}

	if((Device.Joint == knee) || (Device.Joint == combined))
	{
		double accelAngle = (atan(CM_Knee.IMUData.ax / sqrt(pow(CM_Knee.IMUData.ay, 2) + pow(CM_Knee.IMUData.az, 2)))) * 180.0 / M_PI;
		static double compFiltAngle = 0.0;
		static double dGyroAngle = 0.0;

		dGyroAngle = dt/2 * (CM_Knee.IMUData.gz + dGyroAngle);	// Change in angle from gyro (trapezoidal used)

		// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
		double alpha = 0.002;
		compFiltAngle = accelAngle*alpha + (1 - alpha) * (dGyroAngle + compFiltAngle);

		CM_Knee.limbAngle = compFiltAngle - CM_Knee.jointAngle[0];
	}
}

static void RunStateMachine(void)
{
	if((Device.Joint == ankle) || (Device.Joint == combined))
	{
		CM_Ankle.ProsCtrl.eqPoint = 0.0f;
		CM_Ankle.ProsCtrl.kd = 0.0f;
		CM_Ankle.ProsCtrl.kp = 0.0f;
	}

	if((Device.Joint == knee) || (Device.Joint == combined))
	{
		static StateMachine_t state = stance;
		static uint8_t isCheckBoundsRequired = 0;

		switch(state)
		{
		case stance:
			CM_state = 1120;

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
					state = swingFlexion;
				}
			}

			break;

		case swingFlexion:
			CM_state = 1345;
			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingFlexCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingFlexCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingFlexCtrl.kp;

			if(CM_Knee.jointSpeed > CM_kneeSpeedThreshold)
				state = swingExtension;

			break;

		case swingExtension:
			CM_state = 1570;
			CM_Knee.ProsCtrl.eqPoint = CM_Knee.SwingExtCtrl.eqPoint;
			CM_Knee.ProsCtrl.kd = CM_Knee.SwingExtCtrl.kd;
			CM_Knee.ProsCtrl.kp = CM_Knee.SwingExtCtrl.kp;

			if(CM_LoadCell_Filtered->top[0] < CM_lcBot_lowerBound)
				state = stance;

			break;
		}
	}
}

static void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent = 8.0f;						// is this number accurate??
	float torqueConst = 60.0f / (2 * M_PI * 100);	// Units in N*m/A, for Kv = 100 rpm/V

	if((Device.Joint == ankle) || (Device.Joint == combined))
	{
		float errorPos = CM_Ankle.ProsCtrl.eqPoint - CM_Ankle.jointAngle[0];

		CM_Ankle.jointTorque = (CM_Ankle.ProsCtrl.kp*errorPos - CM_Ankle.ProsCtrl.kd*CM_Ankle.jointSpeed);
		float correctedTorque = -CM_Ankle.jointTorque;														// Ankle motor rotates opposite of coordinate system

		int16_t motorTorque = correctedTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_WriteTargetTorqueValue(CM_Ankle.motorId, motorTorque);
	}

	if((Device.Joint == knee) || (Device.Joint == combined))
	{
		float errorPos = CM_Knee.ProsCtrl.eqPoint - CM_Knee.jointAngle[0];

		CM_Knee.jointTorque = (CM_Knee.ProsCtrl.kp*errorPos - CM_Knee.ProsCtrl.kd*CM_Knee.jointSpeed);
		float correctedTorque = -CM_Knee.jointTorque;													// Knee motor rotates opposite of coordinate system

		int16_t motorTorque = correctedTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_WriteTargetTorqueValue(CM_Knee.motorId, motorTorque);
	}
}

static void RunTestProgram(void)
{
	switch (testProgram)
	{
	case none:
		break;

	case readOnly:
		break;

	case constantMotorTorque100Nmm:
		if(Device.Joint == ankle || Device.Joint == combined)
			EPOS4_WriteTargetTorqueValue(CM_Ankle.motorId, -100);	// Ankle motor rotates opposite of coordinate system
		else if(Device.Joint == knee || Device.Joint == combined)
			EPOS4_WriteTargetTorqueValue(CM_Knee.motorId, -100);	// Knee motor rotates opposite of coordinate system

		break;

	case magneticEncoderBias:
		if(Device.Joint == ankle || Device.Joint == combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(AnkleEncoderIndex);

			CM_ankleEncBias = sum / i;
		}
		else if(Device.Joint == knee || Device.Joint == combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
				sum += AS5145B_ReadPosition_Raw(KneeEncoderIndex);

			CM_kneeEncBias = sum / i;
		}

		break;

	case impedanceControl:
		if(Device.Joint == ankle || Device.Joint == combined)
		{
			uint16_t i;
			float sum = 0.0f;
			for(i = 0; i < 1000; i++)
			{
				float position = AS5145B_ReadPosition(AnkleEncoderIndex);
				sum += position;
			}

			CM_Ankle.ProsCtrl.eqPoint = sum / i - ankleEncBias; // (float)??
		}
		else if(Device.Joint == knee || Device.Joint == combined)
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

		RunImpedanceControl();

		break;
	}
}


/*******************************************************************************
* END
*******************************************************************************/
