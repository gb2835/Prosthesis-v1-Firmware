/* USER CODE BEGIN Header */

/*******************************************************************************
*
* TITLE: Prosthesis v1 Firmware
*
* NOTES
* 1. IMPORTANT: A new encoder bias position must be found and defined whenever the magnet is reassembled into the device.
*    A test program is provided to find the bias.
*    See prosthesis_v1.c for more information.
* 2. The below lines can be used to measure PB2 on oscilloscope (#include main.h may need to be added to certain files):
*		- LL_GPIO_SetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_ResetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
* 3. Search -> File on "* USER ADDED " will show code added to MX auto-generated files.
*
*******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "lptim.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*******************************************************************************
* USER ADDED MAIN.C
*******************************************************************************/

#include "as5145b.h"
#include "epos4.h"
#include "error_handler.h"
#include "mcp25625.h"
#include "mpu925x_spi.h"
#include "stm32l4xx_ll_tim.h"
#include <string.h>

#define LPTIM2_PERIOD 0x3F	// Timer frequency = timer clock frequency / (prescaler * (period + 1))

Prosthesis_Init_t Prosthesis_Init;


/******************************************************************************/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_LPTIM2_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/*******************************************************************************
* USER ADDED DEFINITIONS
*******************************************************************************/

	AS5145B_Init_t Encoder_Init[AS5145B_NUMBER_OF_DEVICES];
	Encoder_Init[AnkleEncoderIndex].DO_GPIOx = ANKLE_ENCODER_DO_GPIO_Port;
	Encoder_Init[AnkleEncoderIndex].CLK_GPIOx = ANKLE_ENCODER_CLK_GPIO_Port;
	Encoder_Init[AnkleEncoderIndex].CSn_GPIOx = ANKLE_ENCODER_CSn_GPIO_Port;
	Encoder_Init[AnkleEncoderIndex].DO_Pin = ANKLE_ENCODER_DO_Pin;
	Encoder_Init[AnkleEncoderIndex].CLK_Pin = ANKLE_ENCODER_CLK_Pin;
	Encoder_Init[AnkleEncoderIndex].CSn_Pin = ANKLE_ENCODER_CSn_Pin;
	Encoder_Init[AnkleEncoderIndex].TIMx = TIM6;
	Encoder_Init[AnkleEncoderIndex].timerRateMHz = 10;

	memcpy(&Encoder_Init[KneeEncoderIndex], &Encoder_Init[AnkleEncoderIndex], sizeof(Encoder_Init[AnkleEncoderIndex]));
	Encoder_Init[KneeEncoderIndex].DO_GPIOx = KNEE_ENCODER_DO_GPIO_Port;
	Encoder_Init[KneeEncoderIndex].CLK_GPIOx = KNEE_ENCODER_CLK_GPIO_Port;
	Encoder_Init[KneeEncoderIndex].CSn_GPIOx = KNEE_ENCODER_CSn_GPIO_Port;
	Encoder_Init[KneeEncoderIndex].DO_Pin = KNEE_ENCODER_DO_Pin;
	Encoder_Init[KneeEncoderIndex].CLK_Pin = KNEE_ENCODER_CLK_Pin;
	Encoder_Init[KneeEncoderIndex].CSn_Pin = KNEE_ENCODER_CSn_Pin;

	EPOS4_Init_t MotorController_Init[EPOS4_NUMBER_OF_DEVICES];
	MotorController_Init[AnkleMotorControllerIndex].nodeId = 2;
	MotorController_Init[AnkleMotorControllerIndex].mcpIndex = AnkleCAN_ControllerIndex;
	MotorController_Init[AnkleMotorControllerIndex].Requirements.isFirstStepRequired = 1;
	MotorController_Init[AnkleMotorControllerIndex].Requirements.isModeOfOperationRequired = 1;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.CAN_BitRate = EPOS4_Rate1000Kbps;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.MotorType = EPOS4_TrapezoidalPmBlMotor;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.nominalCurrent = 8000;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.outputCurrentLimit = 29300;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.numberOfPolePairs = 21;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.thermalTimeConstantWinding = 400;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.torqueConstant = 60 / (2 * 3.1416f * 100) * 1000000;	// For Kv = 100 rpm/V
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.maxMotorSpeed = 2384;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.maxGearInputSpeed = 100000;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.sensorsConfiguration = 0x00100000;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.controlStructure = 0x00030111;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.commutationSensors = 0x00000030;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.axisConfigMiscellaneous = 0x00000000;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.currentControllerP_Gain = 643609;
	MotorController_Init[AnkleMotorControllerIndex].FirstStep.currentControllerI_Gain = 2791837;
	MotorController_Init[AnkleMotorControllerIndex].ModeOfOperation = EPOS4_CyclicSynchronousTorqueMode;

	memcpy(&MotorController_Init[KneeMotorControllerIndex], &MotorController_Init[AnkleMotorControllerIndex], sizeof(MotorController_Init[AnkleMotorControllerIndex]));
	MotorController_Init[KneeMotorControllerIndex].nodeId = 1;
	MotorController_Init[KneeMotorControllerIndex].mcpIndex = KneeCAN_ControllerIndex;

  	MCP25625_Init_t CAN_Controller_Init[MCP25625_NUMBER_OF_DEVICES];
  	memset(&CAN_Controller_Init, 0, sizeof(CAN_Controller_Init));
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].SPIx = SPI3;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CS_Port = ANKLE_CAN_CONTROLLER_CS_GPIO_Port;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].csPin = ANKLE_CAN_CONTROLLER_CS_Pin;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CANCTRL_Reg.Bits.CLKEN = MCP25625_ClockoutDisabled;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CANCTRL_Reg.Bits.OSM = MCP25625_OneShotModeEnabled;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CANCTRL_Reg.Bits.ABAT = MCP25625_AbortAllTransmissions;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CANCTRL_Reg.Bits.REQOP = MCP25625_NormalOperationMode;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF1_Reg.Bits.BRP = 0;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF1_Reg.Bits.SJW = MCP25625_Length1xT_Q;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF2_Reg.Bits.PRSEG = 4;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF2_Reg.Bits.PHSEG1 = 1;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF2_Reg.Bits.SAM = MCP25625_BusSampledOnceAtSamplePoint;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF2_Reg.Bits.BLTMODE = MCP25625_PS2LengthDeterminedByCNF3;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF3_Reg.Bits.PHSEG2 = 1;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF3_Reg.Bits.WAKFIL = MCP25625_WakeUpFilterIsDisabled;
  	CAN_Controller_Init[AnkleCAN_ControllerIndex].CNF3_Reg.Bits.SOF = MCP25625_ClockoutPinIsEnabledForClockOutFunction;

  	memcpy(&CAN_Controller_Init[KneeCAN_ControllerIndex], &CAN_Controller_Init[AnkleCAN_ControllerIndex], sizeof(CAN_Controller_Init[AnkleCAN_ControllerIndex]));
  	CAN_Controller_Init[KneeCAN_ControllerIndex].SPIx = SPI2;
  	CAN_Controller_Init[KneeCAN_ControllerIndex].CS_Port = KNEE_CAN_CONTROLLER_CS_GPIO_Port;
  	CAN_Controller_Init[KneeCAN_ControllerIndex].csPin = KNEE_CAN_CONTROLLER_CS_Pin;

  	MPU925x_Init_t IMU_Init;
  	IMU_Init.SPI_Handle = SPI1;
  	IMU_Init.CS_GPIOx = IMU_CS_GPIO_Port;
  	IMU_Init.csPin = IMU_CS_Pin;

  	Prosthesis_Init.Joint = Combined;
  	Prosthesis_Init.Side = Left;


/*******************************************************************************
* USER ADDED INITIALIZATIONS
*******************************************************************************/

	LL_SYSTICK_EnableIT();

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_SetAutoReload(LPTIM2, LPTIM2_PERIOD);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	LL_TIM_EnableCounter(TIM6);

	LL_SPI_Enable(SPI1);
	LL_SPI_Enable(SPI2);
	LL_SPI_Enable(SPI3);
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	LL_mDelay(4000);	// Significant delay when powering on EPOS4

	MPU925x_Error_e imuError = MPU925x_Init(0, &IMU_Init);
	if(imuError)
		ErrorHandler_MPU925x(imuError);
	MPU925x_SetAccelSensitivity(0, MPU925x_AccelSensitivity_8g);
	MPU925x_SetGyroSensitivity(0, MPU925x_GyroSensitivity_1000dps);

	if((Prosthesis_Init.Joint == Ankle) || (Prosthesis_Init.Joint == Combined))
	{
		AS5145B_Error_e encoderError = AS5145B_Init(AnkleEncoderIndex, &Encoder_Init[AnkleEncoderIndex]);
		if(encoderError)
			ErrorHandler_AS5145B(AnkleEncoderIndex, encoderError);

		MCP25625_Error_e canControllerError = MCP25625_Init(AnkleCAN_ControllerIndex, &CAN_Controller_Init[AnkleCAN_ControllerIndex]);
		if(canControllerError)
			ErrorHandler_MCP25625(AnkleCAN_ControllerIndex, canControllerError);

		EPOS4_Error_e motorControllerError = EPOS4_Init(AnkleMotorControllerIndex, &MotorController_Init[AnkleMotorControllerIndex]);
		if(motorControllerError)
			ErrorHandler_EPOS4(AnkleMotorControllerIndex, motorControllerError);
	}

	if((Prosthesis_Init.Joint == Knee) || (Prosthesis_Init.Joint == Combined))
	{
		AS5145B_Error_e encoderError = AS5145B_Init(KneeEncoderIndex, &Encoder_Init[KneeEncoderIndex]);
		if(encoderError)
			ErrorHandler_AS5145B(KneeEncoderIndex, encoderError);

		MCP25625_Error_e canControllerError = MCP25625_Init(KneeCAN_ControllerIndex, &CAN_Controller_Init[KneeCAN_ControllerIndex]);
		if(canControllerError)
			ErrorHandler_MCP25625(KneeCAN_ControllerIndex, canControllerError);

		EPOS4_Error_e motorControllerError = EPOS4_Init(KneeMotorControllerIndex, &MotorController_Init[KneeMotorControllerIndex]);
		if(motorControllerError)
			ErrorHandler_EPOS4(KneeMotorControllerIndex, motorControllerError);
	}

	InitProsthesisControl(&Prosthesis_Init);


/*******************************************************************************
* USER ADDED TEST PROGRAMS
*******************************************************************************/

	RequireTestProgram(ImpedanceControl);


/*******************************************************************************
* USER ADDED MAIN LOOP
*******************************************************************************/

  while(1)
  {
	  if(isProsthesisControlRequired)
	  {
		  RunProsthesisControl();
		  isProsthesisControlRequired = 0;
	  }


/******************************************************************************/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_PWR_EnableBkUpAccess();
  LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
  LL_RCC_LSE_Enable();

   /* Wait till LSE is ready */
  while(LL_RCC_LSE_IsReady() != 1)
  {

  }
  LL_RCC_MSI_EnablePLLMode();
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_6, 40, LL_RCC_PLLR_DIV_4);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);

  LL_Init1msTick(80000000);

  LL_SetSystemCoreClock(80000000);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
