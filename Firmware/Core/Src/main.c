/* USER CODE BEGIN Header */

/*******************************************************************************
 *
 * TITLE  : Prosthesis Knee Control
 * AUTHOR : Greg Berkeley
 * RELEASE: XX/XX/XXXX
 *
 * NOTES:
 * 1. The below lines can be used to measure PB11 on oscilloscope:
 *     - LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
 *     - LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
 *     - LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_11);
 *
 * ABSTRACT:
 * The below code XXX.
 *
 ******************************************************************************/


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lptim.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/

#include "as5145b.h"
#include "EPOS4.h"
#include "mcp25625.h"


/******************************************************************************/

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*******************************************************************************
 * PRIVATE DEFINES
 ******************************************************************************/

#define EPOS4_Mode		0x0A	// Constant torque mode
#define LPTIM2_Period	0x3F	// Period for LPTIM2


/******************************************************************************/

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

/*******************************************************************************
 * USER DEFINITIONS
 ******************************************************************************/

	// Declare variables
	uint16_t CAN_ID = 0x601;	// CAN ID for EPOS4


/******************************************************************************/

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
  /* USER CODE BEGIN 2 */

/******************************************************************************
* USER INITIALIZATIONS
******************************************************************************/

	// Start LPTIM2 interrupt
	LL_LPTIM_Enable(LPTIM2);												// Initially enable timer
	LL_LPTIM_EnableIT_ARRM(LPTIM2);											// Enable timer interrupt mode
	LL_LPTIM_SetAutoReload( LPTIM2, LPTIM2_Period );						// Set auto reload register with period
	LL_LPTIM_StartCounter( LPTIM2, LL_LPTIM_OPERATING_MODE_CONTINUOUS);		// Start counter

	// Enable peripherals
	LL_SPI_Enable(SPI2);

	// Configure devices
	CAN_configure();									// Configure MCP25625
	EPOS4_enable(CAN_ID);								// Enable EPOS4
	EPOS4_set_operation_mode( CAN_ID, EPOS4_Mode );		// Set EPOS4 to torque mode
	EPOS4_enable(CAN_ID);								// Why is this enabled again?? (it is needed at the moment)

	// Remove spikes from beginning (can we do better??)
	for ( int jj = 1; jj < 1000; ++jj );

	// Command motor to 0.1 N*m
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
	EPOS4_CST_apply_torque( CAN_ID, 100 );
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);


/******************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
