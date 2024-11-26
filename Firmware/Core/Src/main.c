/* USER CODE BEGIN Header */

/*******************************************************************************
*
* TITLE:	Prosthesis v1 Firmware
* AUTHOR:	Greg Berkeley
* RELEASE:	??
*
* NOTES
* 1. See "Documents" directory for description of how v1 firmware is used.??
* 2. The below lines can be used to measure PB2 on oscilloscope:
*		- LL_GPIO_SetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_ResetOutputPin(OSCOPE_GPIO_Port, OSCOPE_Pin);
*		- LL_GPIO_TogglePin(OSCOPE_GPIO_Port, OSCOPE_Pin);
* 3. Test programs provided prior to main loop to independently test device
*    functionality.
* 4. Double question marks (??) are commented at locations throughout project
*    files where possible improvements may be made.
* 5. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*    A new magnetic encoder bias position must be found and defined whenever
*    the magnet is reassembled into the prosthesis device. A test program is
*    provided to find the bias.
*    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
* USER PRELIMINARIES
*******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "mcp25625.h"
#include "mpu925x_spi.h"
#include "prosthesis_v1.h"
#include "stm32l4xx_ll_tim.h"
#include <string.h>

#define LPTIM2_PERIOD 0x3F	// Timer frequency = timer clock frequency / (prescaler * (period + 1))


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


/*******************************************************************************
* USER DEFINITIONS
*******************************************************************************/

	AS5145B_t AnkleEncoder; // add pins??
	AnkleEncoder.DO_GPIOx = KNEE_ENCODER_DO_GPIO_Port;
	AnkleEncoder.CLK_GPIOx = KNEE_ENCODER_CLK_GPIO_Port;
	AnkleEncoder.CSn_GPIOx = KNEE_ENCODER_CSn_GPIO_Port;
	AnkleEncoder.DO_Pin = KNEE_ENCODER_DO_Pin;
	AnkleEncoder.CLK_Pin = KNEE_ENCODER_CLK_Pin;
	AnkleEncoder.CSn_Pin = KNEE_ENCODER_CSn_Pin;

	AS5145B_t KneeEncoder;
	KneeEncoder.DO_GPIOx = KNEE_ENCODER_DO_GPIO_Port;
	KneeEncoder.CLK_GPIOx = KNEE_ENCODER_CLK_GPIO_Port;
	KneeEncoder.CSn_GPIOx = KNEE_ENCODER_CSn_GPIO_Port;
	KneeEncoder.DO_Pin = KNEE_ENCODER_DO_Pin;
	KneeEncoder.CLK_Pin = KNEE_ENCODER_CLK_Pin;
	KneeEncoder.CSn_Pin = KNEE_ENCODER_CSn_Pin;

	EPOS4_t AnkleMotor;
	AnkleMotor.Requirements.isFirstStepRequired = 1;
	AnkleMotor.Requirements.isModeOfOperationRequired = 1;
	AnkleMotor.FirstStep.CAN_BitRate = rate500Kbps;
	AnkleMotor.FirstStep.MotorType = trapezoidalPmBlMotor;
	AnkleMotor.FirstStep.nominalCurrent = 6600;
	AnkleMotor.FirstStep.outputCurrentLimit = 29300;
	AnkleMotor.FirstStep.numberOfPolePairs = 21;
	AnkleMotor.FirstStep.thermalTimeConstantWinding = 400;
	AnkleMotor.FirstStep.torqueConstant = 95000;
	AnkleMotor.FirstStep.maxMotorSpeed = 2384;
	AnkleMotor.FirstStep.maxGearInputSpeed = 100000;
	AnkleMotor.FirstStep.sensorsConfiguration = 0x00100000; // ??
	AnkleMotor.FirstStep.controlStructure = 0x00030111; // ??
	AnkleMotor.FirstStep.commutationSensors = 0x00000030; // ??
	AnkleMotor.FirstStep.axisConfigMiscellaneous = 0x00000000; // ??
	AnkleMotor.FirstStep.currentControllerP_Gain = 643609;
	AnkleMotor.FirstStep.currentControllerI_Gain = 2791837;
	AnkleMotor.ModeOfOperation = cyclicSynchronousTorqueMode;

	EPOS4_t KneeMotor;
	KneeMotor.Requirements.isFirstStepRequired = 1;
	KneeMotor.Requirements.isModeOfOperationRequired = 1;
	KneeMotor.FirstStep.CAN_BitRate = rate500Kbps;
	KneeMotor.FirstStep.MotorType = trapezoidalPmBlMotor;
	KneeMotor.FirstStep.nominalCurrent = 6600;
	KneeMotor.FirstStep.outputCurrentLimit = 29300;
	KneeMotor.FirstStep.numberOfPolePairs = 21;
	KneeMotor.FirstStep.thermalTimeConstantWinding = 400;
	KneeMotor.FirstStep.torqueConstant = 95000;
	KneeMotor.FirstStep.maxMotorSpeed = 2384;
	KneeMotor.FirstStep.maxGearInputSpeed = 100000;
	KneeMotor.FirstStep.sensorsConfiguration = 0x00100000; // ??
	KneeMotor.FirstStep.controlStructure = 0x00030111; // ??
	KneeMotor.FirstStep.commutationSensors = 0x00000030; // ??
	KneeMotor.FirstStep.axisConfigMiscellaneous = 0x00000000; // ??
	KneeMotor.FirstStep.currentControllerP_Gain = 643609;
	KneeMotor.FirstStep.currentControllerI_Gain = 2791837;
	KneeMotor.ModeOfOperation = cyclicSynchronousTorqueMode;

  	MCP25625_t CAN_Controller;
  	memset(&CAN_Controller, 0, sizeof(CAN_Controller));
  	CAN_Controller.SPIx = SPI2;
  	CAN_Controller.CS_Port = SPI2_CS_GPIO_Port;
  	CAN_Controller.csPin = SPI2_CS_Pin;
  	CAN_Controller.CANCTRL_Reg.Bits.CLKPRE = clockoutDiv1; // ??
  	CAN_Controller.CANCTRL_Reg.Bits.CLKEN = clockoutDisabled;
  	CAN_Controller.CANCTRL_Reg.Bits.OSM = oneShotModeEnabled;
  	CAN_Controller.CANCTRL_Reg.Bits.ABAT = abortAllTransmissions;
  	CAN_Controller.CANCTRL_Reg.Bits.REQOP = normalOperationMode;
  	CAN_Controller.CNF1_Reg.Bits.BRP = 1;
  	CAN_Controller.CNF1_Reg.Bits.SJW = length1xT_Q;
  	CAN_Controller.CNF2_Reg.Bits.PRSEG = 4;
  	CAN_Controller.CNF2_Reg.Bits.PHSEG1 = 1;
  	CAN_Controller.CNF2_Reg.Bits.SAM = busSampledOnceAtSamplePoint;
  	CAN_Controller.CNF2_Reg.Bits.BLTMODE = ps2LengthDeterminedByCNF3;
  	CAN_Controller.CNF3_Reg.Bits.PHSEG2 = 1;
  	CAN_Controller.CNF3_Reg.Bits.WAKFIL = wakeUpFilterIsDisabled;
  	CAN_Controller.CNF3_Reg.Bits.SOF = clockoutPinIsEnabledForClockOutFunction;

  	MPU925x_t IMU;
  	IMU.SPI_Handle = SPI1;
  	IMU.CS_GPIOx = IMU_CS_GPIO_Port;
  	IMU.csPin = IMU_CS_Pin;

	Prosthesis_t Prosthesis;
	Prosthesis.Joint = ankle; // remove this??
	Prosthesis.Side = left;
	Prosthesis.kneeMotorId = 1; // add to epos4_t?? do all this in ProsInit() instead??
	Prosthesis.ankleMotorId = 2;


/*******************************************************************************
* USER INITIALIZATIONS
*******************************************************************************/

	LL_SYSTICK_EnableIT();

	LL_LPTIM_Enable(LPTIM2);
	LL_LPTIM_EnableIT_ARRM(LPTIM2);
	LL_LPTIM_SetAutoReload(LPTIM2, LPTIM2_PERIOD);
	LL_LPTIM_StartCounter(LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);

	LL_TIM_EnableCounter(TIM6);

	LL_SPI_Enable(SPI1);
	LL_SPI_Enable(SPI2);
	LL_ADC_Enable(ADC1);
	LL_ADC_Enable(ADC2);

	LL_mDelay(10);	// Allow startup delays for devices

	if(MPU925x_Init(0, &IMU))
		Error_Handler();
	MPU925x_SetAccelSensitivity(0, MPU925x_AccelSensitivity_8g); // could this be better??
	MPU925x_SetGyroSensitivity(0, MPU925x_GyroSensitivity_1000dps);

	if(MCP25625_Init(&CAN_Controller))
		Error_Handler();

	if(Prosthesis.Joint == ankle || Prosthesis.Joint == combined) // check this??
	{
		AS5145B_Init(AnkleEncoderIndex, &AnkleEncoder);
		EPOS4_Init(Prosthesis.ankleMotorId, &AnkleMotor);
	}
	if((Prosthesis.Joint == knee) || (Prosthesis.Joint == combined))
	{
		AS5145B_Init(KneeEncoderIndex, &KneeEncoder);
		EPOS4_Init(Prosthesis.kneeMotorId, &KneeMotor);
	}

	InitProsthesisControl(&Prosthesis);



/*******************************************************************************
* USER TEST PROGRAMS
*******************************************************************************/

	RequireTestProgram(readOnly);


/*******************************************************************************
* USER MAIN LOOP
*******************************************************************************/

  while(1)
  {
	  if(isProsthesisControlRequired)
	  {
		  RunProsthesisControl();
		  isProsthesisControlRequired = 0;
	  }

	  // use this for can reads??

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
