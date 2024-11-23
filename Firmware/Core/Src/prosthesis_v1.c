/*******************************************************************************
*
* TITLE:	Application for Prosthesis v1
* AUTHOR:	Greg Berkeley
* RELEASE:	XX/XX/XXXX
*
* NOTES
* 1. None.
*
*******************************************************************************/

#include "as5145b.h" // use DelayUs()??
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

enum StateMachine_e
{
	stance,
	swingFlexion,
	swingExtension
};

struct ControlParams_s
{
	float eqPoint;	// Units in degrees
	float kd;		// Units in N*m/(deg/s)
	float kp;		// Units in N*m/deg
};

struct DeviceParams_s
{
	double jointAngle[2];					// [0] = k-0, [1] = k-1
	double jointSpeed;
	double limbAngle;
	float jointTorque;
	struct ControlParams_s ProsCtrl;
	struct ControlParams_s StanceCtrl;
	struct ControlParams_s SwingFlexCtrl;
	struct ControlParams_s SwingExtCtrl;
	struct MPU925x_IMUData_s IMUData;
	uint8_t motorId;
};

struct LoadCell_Data_s
{
	double bot[3];
	double top[3];
};

enum TestPrograms_e testProgram;
float ankleEncBias, kneeEncBias;
struct Configuration_s Config;
struct LoadCell_Data_s LoadCell[3];					// [0] = k-0, [1] = k-1, [2] = k-2
struct MPU925x_IMUData_s AnkleIMUData, KneeIMUData;

double dt = 1 / 512.0;
uint8_t isFirst = 1;
uint8_t isSecond = 0;
uint8_t isTestProgramRequired = 0;

float CM_lcBot_upperBound, CM_lcBot_lowerBound;
float CM_lcTop_upperBound, CM_lcTop_lowerBound;
float CM_kneeSpeedThreshold;
struct DeviceParams_s CM_Ankle, CM_Knee;
struct LoadCell_Data_s CM_LoadCell_Filtered[3];	// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_ankleEncBias, CM_kneeEncBias;
uint16_t CM_state;

void GetInputs(void);
uint16_t ReadLoadCell(ADC_TypeDef *ADCx);
void ProcessInputs(void);
void CalibrateIMU(void);
void ComputeLimbAngle(void);
void RunStateMachine(void);
void RunImpedanceControl(void);
void RunTestProgram(void);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

// Includes variables that are subject to change during testing for convenience
void InitProsthesisControl(struct Configuration_s *Options)
{
	memcpy(&Config, Options, sizeof(struct Configuration_s));

	if((Config.Device == ankle) || (Config.Device == combined))
	{
		CM_Ankle.motorId = Config.ankleMotorId;
		ankleEncBias = 1325 * AS5145B_RAW2DEG;

		CM_Ankle.ProsCtrl.eqPoint = 0.0f;
		CM_Ankle.ProsCtrl.kd = 0.0f;
		CM_Ankle.ProsCtrl.kp = 0.0f;
		CM_Ankle.StanceCtrl.eqPoint = 0.0f;
		CM_Ankle.StanceCtrl.kd = 0.0f;
		CM_Ankle.StanceCtrl.kp = 0.0f;
		CM_Ankle.SwingFlexCtrl.eqPoint = 0.0f;
		CM_Ankle.SwingFlexCtrl.kd = 0.0f;
		CM_Ankle.SwingFlexCtrl.kp = 0.0f;
		CM_Ankle.SwingExtCtrl.eqPoint = 0.0f;
		CM_Ankle.SwingExtCtrl.kd = 0.0f;
		CM_Ankle.SwingExtCtrl.kp = 0.0f;
	}

	if((Config.Device == knee) || (Config.Device == combined))
	{
		CM_Knee.motorId = Config.kneeMotorId;
		kneeEncBias = 2244 * AS5145B_RAW2DEG;

		CM_Knee.ProsCtrl.eqPoint = 0.0f;
		CM_Knee.ProsCtrl.kd = 0.0f;
		CM_Knee.ProsCtrl.kp = 0.0f;
		CM_Knee.StanceCtrl.eqPoint = 0.0f;
		CM_Knee.StanceCtrl.kd = 0.0f;
		CM_Knee.StanceCtrl.kp = 0.0f;
		CM_Knee.SwingFlexCtrl.eqPoint = 0.0f;
		CM_Knee.SwingFlexCtrl.kd = 0.0f;
		CM_Knee.SwingFlexCtrl.kp = 0.0f;
		CM_Knee.SwingExtCtrl.eqPoint = 0.0f;
		CM_Knee.SwingExtCtrl.kd = 0.0f;
		CM_Knee.SwingExtCtrl.kp = 0.0f;
	}

	CM_lcBot_lowerBound = 1398.0f;
	CM_lcBot_upperBound = 1425.0f;
	CM_lcTop_lowerBound = 1415.0f;
	CM_lcTop_upperBound = 1451.0f;

	CM_kneeSpeedThreshold = 0.0f;
}

void RequireTestProgram(enum TestPrograms_e testProgram)
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

void GetInputs(void)
{
	// Differentiate between two IMUs??
	if((Config.Device == ankle) || (Config.Device == combined))
	{
		CM_Ankle.jointAngle[0] = AS5145B_ReadPosition_Deg() - ankleEncBias;
		AnkleIMUData = MPU925x_ReadIMU();
	}

	if((Config.Device == knee) || (Config.Device == combined))
	{
		CM_Knee.jointAngle[0] = AS5145B_ReadPosition_Deg() - kneeEncBias;
		KneeIMUData = MPU925x_ReadIMU();
	}

	LoadCell->bot[0] = ReadLoadCell(ADC1);
	LoadCell->top[0] = ReadLoadCell(ADC2);
}

uint16_t ReadLoadCell(ADC_TypeDef *ADCx)
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t data = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return data;
}

void ProcessInputs(void)
{
	double tau = 1.0 / (2 * M_PI * 10);		// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of joint angle (joint speed) and filtering of load cells
	if(isFirst)
	{
		if((Config.Device == ankle) || (Config.Device == combined))
		{
			CM_Ankle.jointSpeed = 0.0;
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Config.Device == knee) || (Config.Device == combined))
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
		if((Config.Device == ankle) || (Config.Device == combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Config.Device == knee) || (Config.Device == combined))
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
		if((Config.Device == ankle) || (Config.Device == combined))
		{
			// Practical differentiator (bilinear transformation used)
			CM_Ankle.jointSpeed = (2*(CM_Ankle.jointAngle[0] - CM_Ankle.jointAngle[1]) + (2*tau - dt)*CM_Ankle.jointSpeed) / (dt + 2*tau);
			CM_Ankle.jointAngle[1] = CM_Ankle.jointAngle[0];
		}

		if((Config.Device == knee) || (Config.Device == combined))
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

void CalibrateIMU(void)
{
	double axBias = 0.0;
	double ayBias = 0.0;
	double azBias = 0.0;
	double gxBias = 0.0;
	double gyBias = 0.0;
	double gzBias = 0.0;
	double n = 1.0;			// For normalization

	// Sine and cosine of Euler angles (1 = z angle, 2 = x' angle, 3 = z'' angle)
	double c1, c2, c3, s1, s2, s3;

	if((Config.Device == ankle) || (Config.Device == combined))
	{
		if(Config.Side == left)
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

	if((Config.Device == knee) || (Config.Device == combined))
	{
		if(Config.Side == left)
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

void ComputeLimbAngle(void)
{
	if((Config.Device == ankle) || (Config.Device == combined))
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

	if((Config.Device == knee) || (Config.Device == combined))
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

void RunStateMachine(void)
{
	if((Config.Device == ankle) || (Config.Device == combined))
	{
		CM_Ankle.ProsCtrl.eqPoint = 0.0f;
		CM_Ankle.ProsCtrl.kd = 0.0f;
		CM_Ankle.ProsCtrl.kp = 0.0f;
	}

	if((Config.Device == knee) || (Config.Device == combined))
	{
		static enum StateMachine_e state = stance;
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

void RunImpedanceControl(void)
{
	float gearRatio = 40.0f;
	float nomCurrent = 8.0f;						// is this number accurate??
	float torqueConst = 60.0f / (2 * M_PI * 100);	// Units in N*m/A, for Kv = 100 rpm/V

	if((Config.Device == ankle) || (Config.Device == combined))
	{
		float errorPos = CM_Ankle.ProsCtrl.eqPoint - CM_Ankle.jointAngle[0];

		CM_Ankle.jointTorque = (CM_Ankle.ProsCtrl.kp*errorPos - CM_Ankle.ProsCtrl.kd*CM_Ankle.jointSpeed);
		float correctedTorque = -CM_Ankle.jointTorque;														// Ankle motor rotates opposite of coordinate system

		int16_t motorTorque = correctedTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_WriteTargetTorqueValue(CM_Ankle.motorId, motorTorque);
	}

	if((Config.Device == knee) || (Config.Device == combined))
	{
		float errorPos = CM_Knee.ProsCtrl.eqPoint - CM_Knee.jointAngle[0];

		CM_Knee.jointTorque = (CM_Knee.ProsCtrl.kp*errorPos - CM_Knee.ProsCtrl.kd*CM_Knee.jointSpeed);
		float correctedTorque = -CM_Knee.jointTorque;													// Knee motor rotates opposite of coordinate system

		int16_t motorTorque = correctedTorque / (torqueConst * gearRatio * nomCurrent) * 1000;
		EPOS4_WriteTargetTorqueValue(CM_Knee.motorId, motorTorque);
	}
}

void RunTestProgram(void)
{
	switch (testProgram)
	{
	case none:
		break;

	case readOnly:
		break;

	case constantMotorTorque100Nmm:
		if(Config.Device == ankle || Config.Device == combined)
			EPOS4_WriteTargetTorqueValue(CM_Ankle.motorId, 100);
		else if(Config.Device == knee || Config.Device == combined)
			EPOS4_WriteTargetTorqueValue(CM_Knee.motorId, -100);	// Knee motor rotates opposite of coordinate system

		break;

	case magneticEncoderBias:
		if(Config.Device == ankle || Config.Device == combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
			{
				uint16_t bias = AS5145B_ReadPosition_Raw();
				sum += bias;
			}

			CM_ankleEncBias = sum / i;
		}
		else if(Config.Device == knee || Config.Device == combined)
		{
			uint16_t i;
			uint32_t sum = 0;
			for(i = 0; i < 1000; i++)
			{
				uint16_t bias = AS5145B_ReadPosition_Raw();
				sum += bias;
			}

			CM_kneeEncBias = sum / i;
		}

		break;

	case impedanceControl:
		if(Config.Device == ankle || Config.Device == combined)
		{
			uint16_t i;
			float sum = 0.0f;
			for(i = 0; i < 1000; i++)
			{
				float pos = AS5145B_ReadPosition_Deg();
				sum += pos;
			}

			CM_Ankle.ProsCtrl.eqPoint = sum / i - ankleEncBias;
		}
		else if(Config.Device == knee || Config.Device == combined)
		{
			uint16_t i;
			float sum = 0.0f;
			for(i = 0; i < 1000; i++)
			{
				float pos = AS5145B_ReadPosition_Deg();
				sum += pos;
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
