/*******************************************************************************
 *
 * TITLE   Prosthesis Control
 * AUTHOR  Greg Berkeley
 * RELEASE XX/XX/XXXX
 *
 * NOTES
 * 1. None.
 *
 * ABSTRACT
 * The below code provides the functionality for prosthesis control.
 *
 ******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "prosthesis_control.h"
#include <math.h>
#include "mcp25625.h"
#include "mpu9255.h"
#include <stdint.h>
#include "stm32l4xx_ll_adc.h"


/*******************************************************************************
* PUBLIC DEFINITIONS
*******************************************************************************/

uint8_t isProsthesisControlRequired = 0;
uint16_t CAN_ID = 0x601;


/*******************************************************************************
* PRIVATE DEFINITIONS
*******************************************************************************/

static enum TestPrograms_e testProgram;
float kp;
float kd;
float equilibriumPoint_deg;

float dt = 1 / 512.0f;					// Sample time (programmatically find??)
uint8_t isFirst = 1;
uint8_t isSecond = 0;
uint8_t isTestProgramRequired = 0;

// For CubeMonitor
double CM_hipAngle_deg;
float CM_jointAngle_deg[2];											// [0] = k-0, [1] = k-1
float CM_jointTorque_nm;
float CM_jointSpeed_dps = 0.0f;
uint16_t CM_magEncBias;
uint16_t CM_loadCell_bot[3], CM_loadCell_top[3];					// [0] = k-0, [1] = k-1, [2] = k-2
uint16_t CM_loadCell_bot_filtered[3], CM_loadCell_top_filtered[3];	// [0] = k-0, [1] = k-1, [2] = k-2 (float or uint??)

static void GetInputs (void);
static uint16_t ReadLoadCell ( ADC_TypeDef *ADCx );
static void ProcessInputs (void);
static void CalibrateIMU (void);
static void ComputeHipAngle (void);
static void RunStateMachine (void);
static void RunImpedanceControl (void);
static void RunTestProgram (void);


/*******************************************************************************
* Deal with this after we figure out DMP??
*******************************************************************************/

#define MPUREG_I2C_SLV0_ADDR 0x25
#define AK8963_I2C_ADDR 0x0c
#define READ_FLAG 0x80
#define MPUREG_I2C_SLV0_REG 0x26
#define AK8963_HXL 0x03
#define MPUREG_I2C_SLV0_CTRL 0x27
#define MPUREG_ACCEL_XOUT_H 0x3B

struct imu_data_s
{
   double	ax_g;
   double	ay_g;
   double	az_g;
   double	gx_dps;
   double	gy_dps;
   double	gz_dps;
};

struct dmp_data_s
{
   short	ax_g;
   short	ay_g;
   short	az_g;
   short	gx_dps;
   short	gy_dps;
   short  	gz_dps;
   long		qw;
   long		qx;
   long		qy;
   long		qz;
};

struct imu_data_s imu_data;

struct imu_data_s CM_imu_data;
struct dmp_data_s CM_dmp_data;

struct imu_data_s IMU_read(void);
unsigned int WriteReg(uint8_t adress, uint8_t data);
void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes);


/*******************************************************************************
* PUBLIC FUNCTIONS
*******************************************************************************/

void RunProsthesisControl (void)
{
	GetInputs();
	ProcessInputs();

	// If test program is required, run test program
	// Otherwise continue prosthesis control
	if (isTestProgramRequired)
	{
		RunTestProgram();
	}
	else
	{
		RunStateMachine();
		RunImpedanceControl();
	}

	// Check for first or second executions, needed for derivatives, filters, etc.
	if (isFirst)
	{
		isFirst = 0;
		isSecond = 1;
	}
	else if (isSecond)
	{
		isSecond = 0;
	}
}

void RequireTestProgram ( enum TestPrograms_e option )
{
	testProgram = option;

	if ( testProgram != None )
		isTestProgramRequired = 1;
}


/*******************************************************************************
* PRIVATE FUNCTIONS
*******************************************************************************/

static void GetInputs (void)
{
	float encBias_deg = 1325.0f * 360.0f/4096.0f;	// Bias found using RunTestProgram below

	CM_jointAngle_deg[0] = AS5145B_ReadPosition_Deg() - encBias_deg;
	CM_loadCell_bot[0] = ReadLoadCell(ADC2);
	CM_loadCell_top[0] = ReadLoadCell(ADC1);
	imu_data = IMU_read();

	// DMP??
	mpu9255_process();
	dmp_data_t *dmp_data = mpu9255_getLast();
	CM_dmp_data.ax_g = dmp_data->acceleration.data.x;
	CM_dmp_data.ay_g = dmp_data->acceleration.data.y;
	CM_dmp_data.az_g = dmp_data->acceleration.data.z;
	CM_dmp_data.gx_dps = dmp_data->gyro.data.x;
	CM_dmp_data.gy_dps = dmp_data->gyro.data.y;
	CM_dmp_data.gz_dps = dmp_data->gyro.data.z;
	CM_dmp_data.qw = dmp_data->quaternarion.data.w;
	CM_dmp_data.qx = dmp_data->quaternarion.data.x;
	CM_dmp_data.qy = dmp_data->quaternarion.data.y;
	CM_dmp_data.qz = dmp_data->quaternarion.data.z;
}

static uint16_t ReadLoadCell ( ADC_TypeDef *ADCx )
{
	LL_ADC_REG_StartConversion(ADCx);
	while ( !LL_ADC_IsActiveFlag_EOC(ADCx) );
	LL_ADC_ClearFlag_EOC(ADCx);								// remove this??
	uint16_t val = LL_ADC_REG_ReadConversionData12(ADCx);	// Change resolution??
	return val;
}

static void ProcessInputs (void)
{
	float tau = 1 / ( 2 * 3.1416f * 10 );	// Time constant for practical differentiator (fc = 10 Hz)

	// Derivative of angle and filtering of load cells
	// No derivative of angle (angular speed) on first execution
	// No filtering of load cells on first or second execution
	if (isFirst)
	{
		CM_jointSpeed_dps = 0;

		// Shift values in arrays
		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
		CM_loadCell_bot[2] = CM_loadCell_bot[0];
		CM_loadCell_top[2] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[0] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[0] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[2] = CM_loadCell_top_filtered[0];
	}
	else if (isSecond)
	{
		// Practical differentiator (bilinear transformation used)
		CM_jointSpeed_dps = ( 2*( CM_jointAngle_deg[0] - CM_jointAngle_deg[1] ) + ( 2*tau - dt )*CM_jointSpeed_dps ) / ( dt + 2*tau );

		// Shift values in arrays
		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
		CM_loadCell_bot[1] = CM_loadCell_bot[0];
		CM_loadCell_top[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[0] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[0] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top_filtered[0];
	}
	else
	{
		// Practical differentiator (bilinear transformation used)
		CM_jointSpeed_dps = ( 2*( CM_jointAngle_deg[0] - CM_jointAngle_deg[1] ) + ( 2*tau - dt )*CM_jointSpeed_dps ) / ( dt + 2*tau );

		// 2nd order low-pass Butterworth (fc = 20 Hz)
		CM_loadCell_bot_filtered[0] =   1.6556f * CM_loadCell_bot_filtered[1] - 0.7068f * CM_loadCell_bot_filtered[2]
									  + 0.0128f * CM_loadCell_bot[0] + 0.0256f * CM_loadCell_bot[1] + 0.0128f * CM_loadCell_bot[2];
		CM_loadCell_top_filtered[0] =   1.6556f * CM_loadCell_top_filtered[1] - 0.7068f * CM_loadCell_top_filtered[2]
									  + 0.0128f * CM_loadCell_top[0] + 0.0256f * CM_loadCell_top[1] + 0.0128f * CM_loadCell_top[2];

		// Shift values in arrays
		CM_jointAngle_deg[1] = CM_jointAngle_deg[0];
		CM_loadCell_bot[2] = CM_loadCell_bot[1];
		CM_loadCell_top[2] = CM_loadCell_top[1];
		CM_loadCell_bot[1] = CM_loadCell_bot[0];
		CM_loadCell_top[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot[1];
		CM_loadCell_top_filtered[2] = CM_loadCell_top[1];
		CM_loadCell_bot_filtered[2] = CM_loadCell_bot_filtered[1];
		CM_loadCell_top_filtered[2] = CM_loadCell_top_filtered[1];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top[0];
		CM_loadCell_bot_filtered[1] = CM_loadCell_bot_filtered[0];
		CM_loadCell_top_filtered[1] = CM_loadCell_top_filtered[0];
	}

	CalibrateIMU();
	ComputeHipAngle();
}

static void CalibrateIMU (void)
{
	double gxBias_dps = 60 / 32.8;
	double gyBias_dps = 29 / 32.8;
	double gzBias_dps = 16 / 32.8;

	CM_imu_data.ax_g = imu_data.ax_g;
	CM_imu_data.ay_g = imu_data.ay_g;
	CM_imu_data.az_g = imu_data.az_g;

	CM_imu_data.gx_dps = imu_data.gx_dps - gxBias_dps;
	CM_imu_data.gy_dps = imu_data.gy_dps - gyBias_dps;
	CM_imu_data.gz_dps = imu_data.gz_dps - gzBias_dps;
}

static void ComputeHipAngle (void)
{
//	float gyroAngle = 0.0;	// Value not needed but used to suppress initialization warning
//
//	double gz_rad = CM_imu_data.gz_dps * 3.1416 / 180.0;
//	double accelAngle = atan(CM_imu_data.ax_g / sqrt(pow(CM_imu_data.ay_g,2) + pow(CM_imu_data.az_g,2)));
//
//	// Compute change in angle from gyro (trapezoidal used)
//	if (isFirst)
//	{
//		gyroAngle = 0.0f;
//	}
//	else
//	{
//		gyroAngle = dt/2 * (gz_rad + gyroAngle);
//	}
//
//	// Complementary filter (optimal alpha value found from trial and error experiment of MSE)
//	double alpha = 0.002;
//	CM_hipAngle = accelAngle*alpha + ( 1 - alpha ) * ( gyroAngle + CM_hipAngle );
}

static void RunStateMachine (void)
{
	switch (0)
	{
	case 0:
		kp = 2.5f;
		kd = 0;
		equilibriumPoint_deg = 0;
		break;
	}
}

static void RunImpedanceControl (void)
{
	float gearRatio = 40;
	float torqueConst_nmpa = 0.095f;	// is this number accurate??
	float nomCurrent_amp = 8.;			// is this number accurate??

	float errorPos_deg = equilibriumPoint_deg - CM_jointAngle_deg[0];
	CM_jointTorque_nm = kp*errorPos_deg - kd*CM_jointSpeed_dps;
	int32_t motorTorque = CM_jointTorque_nm / ( torqueConst_nmpa * gearRatio * nomCurrent_amp ) * 1000;
	EPOS4_SetTorque( CAN_ID, motorTorque );
}

static void RunTestProgram (void)
{
	switch (testProgram)
	{
	case None:
		break;
	case ReadOnly:
		break;
	case ConstantTorque:
	{
		int32_t torque = 150;
		EPOS4_SetTorque( CAN_ID, torque );
		break;
	}
	case MagneticEncoderBias:
	{
		uint16_t i;
		uint32_t sum = 0;

		for ( i = 0; i < 1000; i++ )
		{
			struct AS5145B_Data_s data = AS5145B_ReadData();
			sum += data.pos_raw;
		}

		CM_magEncBias = sum / i;
		(void) CM_magEncBias;

		while (1);	// Halt program
	}
	case ImpedanceControl:
	{
		// First compute average of current position and use as equilibrium point
		// Then run impedance control
		if (isFirst)
		{
			uint16_t i;

			kp = 2.5;
			kd = 0;
			uint32_t sum = 0;

			for ( i = 0; i < 1000; i++ )
			{
				struct AS5145B_Data_s data = AS5145B_ReadData();
				sum += data.pos_raw;
			}

			equilibriumPoint_deg = (float) sum/i * 360/4096;
		}
		else
		{
			RunImpedanceControl();
		}

		break;
	}
	}
}


/*******************************************************************************
* Deal with this after we figure out DMP??
*******************************************************************************/

struct imu_data_s IMU_read(void)
{
	struct imu_data_s IMU;
	uint8_t response[21];
	WriteReg(MPUREG_I2C_SLV0_ADDR, AK8963_I2C_ADDR | READ_FLAG); // Set the I2C slave addres of AK8963 and set for read.
	WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL); // I2C slave 0 register address from where to begin data transfer
	WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87); // Read 7 bytes from the magnetometer

	ReadRegs(MPUREG_ACCEL_XOUT_H, response, 21);

	int16_t AX = ((int16_t) response[0] << 8) | response[1];
	int16_t AY = ((int16_t) response[2] << 8) | response[3];
	int16_t AZ = ((int16_t) response[4] << 8) | response[5];
	int16_t GX = ((int16_t) response[8] << 8) | response[9];
	int16_t GY = ((int16_t) response[10] << 8) | response[11];
	int16_t GZ = ((int16_t) response[12] << 8) | response[13];

	IMU.ax_g = AX / 4096.0;
	IMU.ay_g = AY / 4096.0;
	IMU.az_g = AZ / 4096.0;
	IMU.gx_dps = GX / 32.8;
	IMU.gy_dps = GY / 32.8;
	IMU.gz_dps = GZ / 32.8;
	return IMU;
}

unsigned int WriteReg(uint8_t adress, uint8_t data)
{
	unsigned int temp_val;
	LL_GPIO_ResetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);

	while (!(SPI1->SR & SPI_SR_TXE)); //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, adress);
	while (!(SPI1->SR & SPI_SR_RXNE)); //data received?
	LL_SPI_ReceiveData8(SPI1);

	while (!(SPI1->SR & SPI_SR_TXE)); //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, data);
	while (!(SPI1->SR & SPI_SR_RXNE)); //data received?
	temp_val = LL_SPI_ReceiveData8(SPI1);

	LL_GPIO_SetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin);
	return temp_val;
}

void ReadRegs(uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes)
{
	unsigned int i = 0;
	LL_GPIO_ResetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin); // PA4 CS RESET Active Low

	while (!(SPI1->SR & SPI_SR_TXE)); //transmit buffer empty?
	LL_SPI_TransmitData8(SPI1, (ReadAddr | 0x80)); // (Starting Address 0x22 | 0x80); MSB is '1' for 0x80, next 7 bit Address of register to write 0x22
	while (!(SPI1->SR & SPI_SR_RXNE)); //data received?
	LL_SPI_ReceiveData8(SPI1);

	for (i = 0; i < Bytes; i++)
	{
		while (!(SPI1->SR & SPI_SR_TXE)); //transmit buffer empty?
		LL_SPI_TransmitData8(SPI1, 0x00);
		while (!(SPI1->SR & SPI_SR_RXNE)); //data received?
		ReadBuf[i] = LL_SPI_ReceiveData8(SPI1);
	}

	LL_GPIO_SetOutputPin(IMU_CS_GPIO_Port, IMU_CS_Pin); // PC4 CS SET Active Low
}


/*******************************************************************************
* END
*******************************************************************************/
