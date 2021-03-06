/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FBL_DEBUG_MSG_EN	
#define	FBL_UART_RX_LEN					(200U)

/* Define length response for each command */
#define FBL_GET_VERSION_ANSWER_LENGTH					(1U)
#define FBL_GET_CID_ANSWER_LENGTH							(2U)
#define FBL_GET_RDP_ANSWER_LENGTH							(1U)
#define FBL_GO_TO_ADDRESS_ANSWER_LENGTH		    (1U)
#define FBL_ERASE_ANSWER_LENGTH		    				(1U)
#define FBL_WRITE_ANSWER_LENGTH		    				(1U)
#define FBL_RW_PROTECT_LENGTH									(2U)

#define FBL_GENERIC_NO_ERROR									(0xCC)
#define FBL_GENERIC_ERROR											(0xAA)

#define FBL_ADDRESS_INVALID										(0x0C)
#define FBL_ADDRESS_VALID											(0x0A)

#define FBL_INVALID_SECTOR_NUMBER							(0x0D)
#define FBL_INVALID_NUMBER_SECTORS						(0x0B)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

HCD_HandleTypeDef hhcd_USB_OTG_FS;



/* USER CODE BEGIN PV */


#define FBL_UART_CHANNEL_DEBUG_INTERFACE				(&huart3)

uint8_t auc8UartRxBuffer[FBL_UART_RX_LEN];

uint8_t aucSupportedCommands[] = {
															 FBL_FLASH_BINARY_FILE,
                               FBL_GET_VER ,
                               FBL_GET_HELP,
                               FBL_GET_CID,
                               FBL_GET_RDP_STATUS,
                               FBL_GO_TO_ADDR,
                               FBL_FLASH_ERASE,
                               FBL_MEM_WRITE,
															 FBL_EN_RW_PROTECT,
														   FBL_MEM_READ,
                               FBL_READ_SECTOR_P_STATUS,
															 FBL_OTP_READ,
															 FBL_DIS_R_W_PROTECT} ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void FBL_vPrintMsg(char *format,...);
uint16_t FBL_Get_Mcu_ID(void);
uint8_t FBL_Get_RDP(void);
uint16_t FBL_Verify_Address(uint32_t ulAddress);
uint8_t FBL_ucExecute_MemoryWrite(uint8_t *pucBuffer, uint32_t ulAddress, uint32_t ulLen);
uint8_t FBL_ucExecute_MemoryRead(uint32_t ulAddress, uint8_t ucLen);
uint8_t FBL_Modify_RW_Protection(uint8_t u8Sectors, uint8_t u8ProtectionMode, uint8_t uc8EN);
uint16_t FBL_vRw_ProtectionStatus(void);															 
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
	
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_HCD_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	char somedata[] = "Hello from the bootloader";
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		FBL_vPrintMsg("FBL\n");
		uint32_t ul32CurrentTickValue = HAL_GetTick();
		while ( HAL_GetTick() <= (ul32CurrentTickValue + 500));
		//
		

		FBL_vUartReadData();
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*****************************************COMMENT FUNCTION TEMPLATE*******************************************************/

/*
*
* \brief 
*	\param 
* \return	
*
*/

/*************************************************************************************************************************/

/*
*
* \brief  Prints a message over the USART3 debug loader interface.
*	\param  puc8Format: message format
* \param	... : arguments 
* \return	-
*
*/

void FBL_vPrintMsg(char *puc8Format,...)
{
	
	char ucStr[80];
	
	va_list tArgs;
	va_start(tArgs, puc8Format);
	vsprintf(ucStr, puc8Format, tArgs);
	HAL_UART_Transmit(FBL_UART_CHANNEL_DEBUG_INTERFACE, (uint8_t *) ucStr,strlen(ucStr),HAL_MAX_DELAY);
	va_end(tArgs);
	
}


/*
*
* \brief 	Jumps to user application. User application is stored in sector 2, start address is 0x8008000
*	\param 	void
* \return	void. Prints a message on the debug interface.
*
*/

void FBL_vJumpToUserApplication(void)
{
	/* Declare void pointer function for user application reset handler */
	void (*pvAppResetHandler)(void);
	
	//FBL_vPrintMsg ("FBL_DEBUG_MSG: FBL_vJumpToUserApplication");
	
	/* Store user application MSP from flash sector 2) */
	
	uint32_t ulMainStackPointer = * (volatile uint32_t *) FLASH_SECTOR2_BASE_ADDRESS;
	
	/* Set user application MSP */
	__set_MSP(ulMainStackPointer);
	
	/* Store user application Reset Handler address from flash sector 2) */
	uint32_t ulResetHandlerAddr = * (volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);
	
	
	pvAppResetHandler = (void *) ulResetHandlerAddr;
	
	/* Jump to Reset Handler  of user application */
	
	pvAppResetHandler();
		
}


/*
*
* \brief Sends acknowlenge messages on UART debug interface.  
*	\param ucLengthResponse: length to follow
* \return	-
*
*/

/*****************************************START DECLARE COMMON FUNCTIONS *******************************************************/

void FBL_vSendAck(uint8_t ucLengthResponse)
{
	uint8_t aucAck[2];
	aucAck[0] = FBL_ACK_VALUE;
	aucAck[1] = ucLengthResponse;
	HAL_UART_Transmit(FBL_UART_CHANNEL_DEBUG_INTERFACE, aucAck , 2, HAL_MAX_DELAY);
}

/*
*
* \brief Sends not acknowlenge messages on UART debug interface.
*	\param -
* \return	-
*
*/

void FBL_vSendNack(void)
{
	uint8_t ucNack = FBL_NACK_VALUE;
	HAL_UART_Transmit(FBL_UART_CHANNEL_DEBUG_INTERFACE, &ucNack , 1, HAL_MAX_DELAY);
	
}
/*
*
* \brief Verifies the CRC computed by the host application.
*	\param uint8_t *pucData: pointer to received data buffer
*	\param uint32_t ulLength: length of the received data buffer
*	\param uint32_t ulCRC: received CRC value to be compared with
* \return	FBL_CRC_SUCCESS or FBL_CRC_FAIL error codes
*
*/

uint8_t FBL_ucVerifyCRC(uint8_t *pucData, uint32_t ulLength,uint32_t ulCRCHost)
{
	
	  uint8_t  ucReturnValue = FBL_CRC_FAIL;
/* Choose to use HAL CRC library */
#if (FBL_CRC_VERIFY_VERSION == FBL_CRC_HAL)
	
	uint32_t ulCRCValue = 0xFF;
	uint32_t ulData;
	for(uint32_t ulI = 0; ulI < ulLength; ulI++)
	{
		uint32_t ulData = pucData[ulI];
		ulCRCValue = HAL_CRC_Accumulate(&hcrc,&ulData,1);
	}
	
	__HAL_CRC_DR_RESET(&hcrc);
	
	if (ulCRCValue == ulCRCHost)
	{
		ucReturnValue = FBL_CRC_SUCCESS;
		
	}

	return ucReturnValue;
#else
	
	uint32_t ulCRCValue = 0xFFFFFFFF;

	
	/* Outer loop for byte progessing */
	for(uint8_t ucI = 0; ucI < ulLength; ucI++)
	{
			ulCRCValue = ulCRCValue ^ pucData[ucI];
			/* Inner loop for bit processing */
			for (uint8_t ucJ = 0; ucJ < 32; ucJ++)
			{
				if( ulCRCValue & 0x80000000)
				{
					ulCRCValue = (ulCRCValue << 1) ^ FBL_CRC_POLYNOME;
				}
				else
				{
					ulCRCValue = (ulCRCValue << 1);
				}
			}
	}
	
	/* Check if the computed value is the same as the received value */
	if (ulCRCValue == ulCRCHost)
	{
		ucReturnValue = FBL_CRC_SUCCESS;		
	}
#endif
		return ucReturnValue;
}



/*
*
* \brief Transmits data on UART debug interface
*	\param uint8_t *pucBuffer: transmit buffer
*	\param uint32_t ulLength: transmit buffer length
* \return	-
*
*/
void FBL_vUartWriteData(uint8_t *pucBuffer, uint32_t ulLength)
{
	HAL_UART_Transmit(FBL_UART_CHANNEL_DEBUG_INTERFACE,pucBuffer,ulLength,HAL_MAX_DELAY);
}

/*****************************************STOP DECLARE COMMON FUNCTIONS *******************************************************/



/*****************************************START DECLARE GETVERS FUNCTIONS *******************************************************/

/*
*
* \brief  Returns the flash bootloader current version
*	\param  -
* \return	FBL_VERSION
*
*/
uint8_t FBL_ucGetBootloaderVersion(void)
{
	return FBL_VERSION;
}


/*
*
* \brief  Calls FBL_ucGetBootloaderVersion and sends on UART debug interface the bootloader version.
*	\param  uint8_t *puc8RxBuffer: received command buffer
* \return	-
*
*/
void FBL_vGetVersion_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucVersion;
	uint32_t ulLength = puc8RxBuffer[0] + 1;
	uint32_t ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		FBL_vSendAck(FBL_GET_VERSION_ANSWER_LENGTH);
		ucVersion = FBL_ucGetBootloaderVersion();
		FBL_vUartWriteData(&ucVersion,1);
  }
	else
	{
		FBL_vSendNack();
	}
	
}
/*****************************************STOP DECLARE GETVERS FUNCTIONS *******************************************************/

/*****************************************START DECLARE GETHELP FUNCTIONS *******************************************************/

/*
*
* \brief	Handle for get help command. Transmits on UART debug interface the bootloader supported commands.
*	\param	uint8_t *puc8RxBuffer: received command buffer 
* \return	-
*
*/
void FBL_vGetHelp_Cmd(uint8_t *puc8RxBuffer)
{
	 uint32_t ulLength = puc8RxBuffer[0] + 1;
	 uint32_t ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4)); 
	 if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	 {
		FBL_vSendAck(sizeof(aucSupportedCommands));
	 	FBL_vUartWriteData(aucSupportedCommands,sizeof(aucSupportedCommands));
	 }
	 else
	 {
		FBL_vSendNack();
	 }
}



/*****************************************STOP DECLARE GETVERS FUNCTIONS *******************************************************/


/*****************************************START DECLARE GETCID FUNCTIONS *******************************************************/

/*
*
* \brief  Gets chip identification number
*	\param  -
* \return	-
*
*/
uint16_t FBL_Get_Mcu_ID(void)
{
	
	uint16_t usCID = (uint16_t) (DBGMCU->IDCODE) & 0x0FFF;
	
	return usCID;
}

/*
*
* \brief  Handler for get chip identification number.
*	\param  uint8_t *puc8RxBuffer: received command buffer 
* \return	-
*
*/
void FBL_vGetCID_Cmd(uint8_t *puc8RxBuffer)
{
	 uint16_t usCID = 0;
	 uint32_t ulLength = puc8RxBuffer[0] + 1;
	 uint32_t ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4)); 
	 if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	 {
		 
		usCID = FBL_Get_Mcu_ID();
		FBL_vSendAck(FBL_GET_CID_ANSWER_LENGTH);
	 	FBL_vUartWriteData((uint8_t *) &usCID, 2);
		 
	 }
	 else
	 {
		FBL_vSendNack();
	 }
	
}




/*****************************************STOP DECLARE GETCID FUNCTIONS *******************************************************/

/*****************************************START DECLARE GETRDP FUNCTIONS *******************************************************/

/*
*
* \brief  Reads the option bytes from 0x1FFFC000.
*	\param  -
* \return	uint8. Returns the bits 15:8 of RDP.
*
*/
uint8_t FBL_Get_RDP(void)
{
	
	volatile uint32_t *pul_Address = (uint32_t *) 0x1FFFC000;
	
  return (uint8_t) ((*pul_Address) >> 8);
}

/*
*
* \brief	Handler function for get read protection option byte command.	
*	\param 	uint8_t *puc8RxBuffer: received command buffer 
* \return	-
*
*/
void FBL_vGetRDP_Cmd(uint8_t *puc8RxBuffer)
{
	 uint8_t ucRDP = 0;
	 uint32_t ulLength = puc8RxBuffer[0] + 1;
	 uint32_t ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4)); 
	 if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	 {
		 
		ucRDP = FBL_Get_RDP();
		FBL_vSendAck(FBL_GET_RDP_ANSWER_LENGTH);
	 	FBL_vUartWriteData((uint8_t *)&ucRDP, 1);
		 
	 }
	 else
	 {
		FBL_vSendNack();
	 }
	
}


/*****************************************STOP DECLARE GETRDP FUNCTIONS *******************************************************/

/*****************************************START DECLARE GOTOADDRESS FUNCTIONS *******************************************************/

/*
*
* \brief	Verifies if the address is in the valid range addresses.
*	\param	uint32_t ulAddress: address to be checked
* \return	uint16. FBL_ADDRESS_VALID or FBL_ADDRESS_INVALID
*
*/
uint16_t FBL_Verify_Address(uint32_t ulAddress)
{
	if (((ulAddress  >=FLASH_BASE) && (ulAddress  <=FLASH_END))  ||
			((ulAddress  >=SRAM1_BASE) && (ulAddress  <=SRAM1_BASE + FBL_SRAM1_SIZE)) ||
			((ulAddress  >=SRAM2_BASE) && (ulAddress  <=SRAM2_BASE + FBL_SRAM2_SIZE)) ||
			((ulAddress  >=BKPSRAM_BASE) && (ulAddress  <=BKPSRAM_BASE + FBL_BKPSRAM_SIZE)))
	{
		
			return FBL_ADDRESS_VALID;
		
	}
	else
	{
			return FBL_ADDRESS_INVALID;
				
	}
	
}
/*
*
* \brief  Handler function for go to address command.
*	\param	uint8_t *puc8RxBuffer: received command buffer 
* \return	-
*
*/
void FBL_vGoToAddress_Cmd(uint8_t *puc8RxBuffer)
{
	 uint8_t ucStatus;
	 uint32_t ulAddress;
	 uint32_t ulLength = puc8RxBuffer[0] + 1;
	 /* 
			Get CRC position from rx command. Position to the end of the command puc8RxBuffer + ulLength 
			and then subtract 4 because CRC represents the last 4 bytes in the command
	 */
	 uint32_t ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4)); 
	 if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	 {
		FBL_vSendAck(FBL_GO_TO_ADDRESS_ANSWER_LENGTH);
		/*
			Get address position from rx command
		*/
    ulAddress = *((uint32_t *)&puc8RxBuffer[2]); 
		/* 
		 Don't forget about the T bit to execute thumb instructions.
		 The program hands in HardFault_Handler when T bit is 0 because the cortex M doesn't support ARM instruction set.
		*/
		 if ((ulAddress & (uint32_t) 0x01) == (uint32_t) 0x00)
		 {
				ulAddress += 1;
		 }

		if(FBL_Verify_Address(ulAddress) == FBL_ADDRESS_VALID)
		{
			void (*FBL_Jump)(void) = (void *) ulAddress;
			ucStatus = FBL_ADDRESS_VALID;
			FBL_vUartWriteData(&ucStatus, 1);
			FBL_Jump();
		}
		else
		{
			ucStatus = FBL_ADDRESS_INVALID;
			FBL_vUartWriteData(&ucStatus, 1);
		}
		 
	 }
	 else
	 {
		 FBL_vSendNack();
	 }
}


/*****************************************STOP DECLARE GOTOADDRESS FUNCTIONS *******************************************************/

/*****************************************START DECLARE FLASHERASE FUNCTIONS *******************************************************/

/*
*
* \brief  Handler function for flash erase command.
*	\param	uint8_t *puc8RxBuffer: incoming command buffer.
* \return	-	
*
*/
void FBL_vFlashErase_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	uint32_t ulError;
	FLASH_EraseInitTypeDef tFlash1;
	/* 
		Get length and add 1 to include the total length computed by CRC 
	*/
	ulLength = puc8RxBuffer[0] + 1;
	/* 
		Get CRC position from rx command. Position to the end of the command puc8RxBuffer + ulLength 
		and then subtract 4 because CRC represents the last 4 bytes in the command
	 */
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4)); 
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	 {
		 FBL_vSendAck(FBL_ERASE_ANSWER_LENGTH);
		 
		 
		 if(puc8RxBuffer[2] > 8)
		 {
			 ucReturnValue = FBL_INVALID_SECTOR_NUMBER;
		 }
		 
		 if((puc8RxBuffer[2] == 0xFF) || (puc8RxBuffer[2] <8))
		 {
			 /* 
				When sector number is 0xFF the command performs a mass erase 
			 */
			 if(puc8RxBuffer[2] == 0xFF)
			 {
				 tFlash1.TypeErase = FLASH_TYPEERASE_MASSERASE;
				 // tFlash1.Sector = puc8RxBuffer[2] - Does not matter
				 // tFlash1.NbSectors = puc8RxBuffer[3] - Does not matter
			 }
			 else
			 {
				  /*
						Check if the reference sector and the number of sectors do not overflow the flash memory sectors 
				  */
				  if((8-puc8RxBuffer[2]) >= puc8RxBuffer[3])
					{
						/* 
							Perform sector type erase 
						*/
						tFlash1.TypeErase = FLASH_TYPEERASE_SECTORS;
						tFlash1.Sector = puc8RxBuffer[2];
						tFlash1.NbSectors = puc8RxBuffer[3];
					}	
					else
					{
						ucReturnValue = FBL_INVALID_NUMBER_SECTORS;
					}
			 }
			 tFlash1.Banks = FLASH_BANK_1;
			 tFlash1.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			 if ((ucReturnValue != FBL_INVALID_SECTOR_NUMBER) && (ucReturnValue != FBL_INVALID_NUMBER_SECTORS))
			 {
				 HAL_FLASH_Unlock();
				 ucReturnValue = (uint8_t) HAL_FLASHEx_Erase(&tFlash1, &ulError);
				 HAL_FLASH_Lock();
			 }
		 }
		 
		 FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
	 }
	else
	{
		FBL_vSendNack();
	}
}

/*****************************************STOP DECLARE FLASHERASE FUNCTIONS *******************************************************/

/*****************************************START DECLARE MEMORY WRITE FUNCTIONS *******************************************************/


/*
*
* \brief  Performs flash programming byte by byte.
*	\param	uint8_t *pucBuffer: input buffer that contains data to be write to memory.
*	\param  uint32_t ulAddress: address memory location where to write the incoming buffer.
*	\param  uint32_t ulLen: length of the incoming buffer.
* \return	FBL_GENERIC_NO_ERROR or HAL_FLASH_Program error
*
*/
uint8_t FBL_ucExecute_MemoryWrite(uint8_t *pucBuffer, uint32_t ulAddress, uint32_t ulLen)
{
	uint8_t ucReturnValue = FBL_GENERIC_NO_ERROR;
	HAL_FLASH_Unlock();

  for(uint32_t ulI = 0 ; ulI <ulLen ; ulI++)
    {
        /* 
				Flash programming
			  */
        ucReturnValue = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,ulAddress+ulI,pucBuffer[ulI] );
    }

  HAL_FLASH_Lock();
	
	return ucReturnValue;
	
}



/*
*
* \brief	Handler function for memory write command. 
*	\param	uint8_t *puc8RxBuffer: incoming command buffer. 
* \return	-
*
*/
void FBL_vMemoryWrite_Cmd(uint8_t *puc8RxBuffer)
{
	
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	uint32_t ulMemoryAddress;
	uint32_t ulPayloadLength;
	
	/* 
		Get length and add 1 to include the total length computed by CRC 
	*/
	ulLength = puc8RxBuffer[0] + 1;
	/* 
		Get CRC position from rx command. Position to the end of the command puc8RxBuffer + ulLength 
		and then subtract 4 because CRC represents the last 4 bytes in the command
	*/
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	/* 
		Get the base memory address
	*/
	ulMemoryAddress = *((uint32_t *)(&puc8RxBuffer[2]));
	/* 
		Get the payload length
	*/
	ulPayloadLength = puc8RxBuffer[6];
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		FBL_vSendAck(FBL_WRITE_ANSWER_LENGTH);
		/*
			Check the validity of the base memory address
		*/
		if(FBL_Verify_Address(ulMemoryAddress) == FBL_ADDRESS_VALID)
		{
		 
		 ucReturnValue = FBL_ucExecute_MemoryWrite(&puc8RxBuffer[7],ulMemoryAddress,ulPayloadLength);
		
		}
		else
		{
			
			ucReturnValue = FBL_ADDRESS_INVALID;
		}
		
		FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
	}
	else
	{
		FBL_vSendNack();
	}
	
}
/*****************************************STOP DECLARE MEMORY WRITE FUNCTIONS *******************************************************/



/*****************************************START DECLARE MEMORY EN/DIS READ/WRITE PROTECT FUNCTIONS *******************************************************/



/*
*
* \brief  Handler function for modify read/write protection command. 
*	\param  uint8_t u8Sectors: each bit represents a flash memory sector
*	\param  uint8_t u8ProtectionMode: 0 or 1. Decides the protection mode, the value is written in SPRMOD register 
*	\param  uint8_t uc8EN: keeps a flag value in order to enable or disable R/W protection
* \return	FBL_GENERIC_NO_ERROR or FBL_GENERIC_ERROR
*
*/
uint8_t FBL_Modify_RW_Protection(uint8_t u8Sectors, uint8_t u8ProtectionMode, uint8_t uc8EN)
{
	volatile uint32_t *pulOPTCR = (uint32_t*) 0x40023C14;
	uint8_t ucReturnValue = FBL_GENERIC_NO_ERROR;
	if(uc8EN)
	{
		/* 
				Enable R/W protection
		*/
		if (u8ProtectionMode == 0x00)
		{
			
				HAL_FLASH_OB_Unlock();

				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
				 /*
					Set SPRMOD to 0.
			  */
				*pulOPTCR &= ~(1 << 31); // default value is  0x0FFF AAED
			  *pulOPTCR &= ~ (u8Sectors << 16);
			  /* 
					Trigger option start.
			  */
				*pulOPTCR |= ( 1 << 1);
				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
				HAL_FLASH_OB_Lock();
			
		}
		else if (u8ProtectionMode == 0x01)
		{
				HAL_FLASH_OB_Unlock();

				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			  /*
					Set SPRMOD to 1.
			  */
				*pulOPTCR = *pulOPTCR | (1 << 31);
			
			  *pulOPTCR &= ~(0xff << 16);
				*pulOPTCR |= (u8Sectors << 16);
			
				/* 
					Trigger option start.
			  */
				*pulOPTCR |= ( 1 << 1);
				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
				HAL_FLASH_OB_Lock();
			
		}
		else
		{
			ucReturnValue = FBL_GENERIC_ERROR;
		}
	}
	else
	{
		/* 
			Disable R/W protection
		*/
		HAL_FLASH_OB_Unlock();

		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		
		*pulOPTCR = *pulOPTCR & ~(1<<31); // default value is  0x0FFF AAED
		*pulOPTCR = *pulOPTCR | FLASH_OPTCR_nWRP_Msk;
		/* 
			Trigger option start.
		*/
		*pulOPTCR = *pulOPTCR | FLASH_OPTCR_OPTSTRT_Msk;
		
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
		HAL_FLASH_OB_Lock();
	}
	
	return ucReturnValue;
}


/*
*
* \brief  Handler function for enable write and read protection command.
*	\param 	uint8_t *puc8RxBuffer: incoming command buffer. 
* \return	-
*
*/

void FBL_vEnable_RW_Protect_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	#define ENABLE_RW (1U)
	
	
	ulLength = puc8RxBuffer[0] + 1;
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		 FBL_vSendAck(FBL_RW_PROTECT_LENGTH);
		
		 ucReturnValue = FBL_Modify_RW_Protection(puc8RxBuffer[2],puc8RxBuffer[3],ENABLE_RW);
		
		 FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);

		
	}
	else
	{
		FBL_vSendNack();	
	}
	
}



/*
*
* \brief  Handler function for read protection status command.
*	\param  uint8_t *pucBuffer: input buffer that contains data to be write to memory.
* \return	-
*
*/

void FBL_vDisable_RW_Protect_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	#define DISABLE_RW (0U)
	
	
	ulLength = puc8RxBuffer[0] + 1;
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		 FBL_vSendAck(FBL_RW_PROTECT_LENGTH);
		
		 ucReturnValue = FBL_Modify_RW_Protection(puc8RxBuffer[2],puc8RxBuffer[3],DISABLE_RW);
		
		 FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);

	}
	else
	{
		FBL_vSendNack();	
	}
	
}


/*****************************************STOP DECLARE MEMORY EN/DIS READ/WRITE PROTECT FUNCTIONS *******************************************************/

/*****************************************START DECLARE MEMORY READ PROTECTION STATUS FUNCTIONS *******************************************************/

/*
*
* \brief Read protection status option byte
*	\param -
* \return	uint16, the value of WRP bits
*
*/
uint16_t FBL_vRw_ProtectionStatus(void)
{
	
	
	FLASH_OBProgramInitTypeDef OBInit;

	HAL_FLASH_OB_Unlock();

	HAL_FLASHEx_OBGetConfig(&OBInit);

	HAL_FLASH_Lock();

	return (uint16_t)OBInit.WRPSector;

}



/*
*
* \brief Handler function for read protection status command.
*	\param uint8_t *puc8RxBuffer: incoming command buffer. 
* \return	-
*
*/
void FBL_vRead_Sector_ProtectionStatus_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	volatile uint32_t *pulOPTCR = (uint32_t*) 0x40023C14;

	
	ulLength = puc8RxBuffer[0] + 1;
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		 FBL_vSendAck(FBL_RW_PROTECT_LENGTH);
		
		 ucReturnValue = FBL_vRw_ProtectionStatus();
		
		 FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
		 ucReturnValue = (uint8_t) (*pulOPTCR >> 31) ;
		 FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
	}
	else
	{
		FBL_vSendNack();	
	}
	
}


/*****************************************STOP DECLARE MEMORY READ PROTECTION STATUS WRITE FUNCTIONS *******************************************************/


/*****************************************START DECLARE MEMORY READ OTP FUNCTIONS *******************************************************/
/*
*
* \brief Handler function for read OTP register command.
*	\param uint8_t *puc8RxBuffer: incoming command buffer. 
* \return	-
*
*/

void FBL_vRead_OTP_Cmd(uint8_t *puc8RxBuffer)
{
	uint8_t ucFlag = 0;
	uint8_t ucReturnValue = FBL_GENERIC_NO_ERROR;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	uint8_t ucOTPBank;
	volatile uint32_t *pulOPT_Base = (uint32_t*) 0x1FFF7800;
  uint32_t ulOTP_DataSetValue;
	uint8_t  aucOTP_DataSetBytes[4];
	uint8_t  ucSizeOfDataSet = sizeof(aucOTP_DataSetBytes);
	ulLength = puc8RxBuffer[0] + 1;
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	ucOTPBank = *((uint32_t *)(puc8RxBuffer + 2));
	
	if(ucOTPBank == 0x00)
	{
		pulOPT_Base = (uint32_t*) 0x1FFF7800;
	}
	else if(ucOTPBank == 0x01)
	{
	  pulOPT_Base = (uint32_t*) 0x1FFF7900;
	}
	else
	{
		ucFlag = 1;	
	}
	
	if((FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS))
	{
		
		if (ucFlag == 0)
		{
		 FBL_vSendAck(0xFF);
		 //HAL_FLASH_Unlock();
		
		 for(uint8_t usI =1; usI <= 0x40; usI++)
		 {
			
				ulOTP_DataSetValue = (uint32_t) (*pulOPT_Base);
				pulOPT_Base = pulOPT_Base + 1;
				for (uint8_t ucJ = 0; ucJ <= 3; ucJ++)
			  {
					aucOTP_DataSetBytes[ucJ] = (uint8_t) (ulOTP_DataSetValue >> (ucJ * 8)) & 0x000000FF;
			  }
				
				FBL_vUartWriteData(aucOTP_DataSetBytes,ucSizeOfDataSet);

		 }
		
		// HAL_FLASH_Lock();
	 }
	 else
	 {
			FBL_vSendAck(0x01);
		  ucReturnValue = FBL_GENERIC_ERROR;
		  FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
	 }
	}
	else
	{
		FBL_vSendNack();	
	}
	
	
}

/*****************************************STOP DECLARE MEMORY READ OTP FUNCTIONS *******************************************************/

/*****************************************STOP DECLARE MEMORY READ FUNCTIONS *******************************************************/

/*
*
* \brief Performs a flash memory read.
*	\param uint32_t ulAddress: first address to be read
*	\param uint8_t ulLen: bytes to be read
* \return	Status of read operation.
*
*/
uint8_t FBL_ucExecute_MemoryRead(uint32_t ulAddress, uint8_t ucLen)
{
	uint8_t ucReturnValue = FBL_GENERIC_NO_ERROR;
	uint8_t ucDataLength;
	volatile uint32_t *pulAddress = (uint32_t*) ulAddress;
	ucDataLength = ucLen * 4;
	uint8_t aucData[ucDataLength];
	memset( aucData, 0, ucDataLength*sizeof(uint8_t) );
	//HAL_FLASH_Unlock();

  for(uint32_t ulI = 0 ; ulI < ucLen; ulI++)
  {
		  /* Read 4 bytes each for iteration */
			/* Byte 0 */
      *(aucData + 4* ulI + 0) = (uint8_t) ((*pulAddress) & 0x000000FF);
		  /* Byte 1 */
		  *(aucData + 4* ulI + 1)= (uint8_t) ((*pulAddress >> 8) & 0x000000FF);
			/* Byte 2 */
		  *(aucData + 4* ulI + 2) = (uint8_t) ((*pulAddress >> 16) & 0x000000FF);
		  /* Byte 3 */
		  *(aucData + 4* ulI + 3) = (uint8_t) ((*pulAddress >> 24) & 0x000000FF);
		
			pulAddress++;
        
  }
	FBL_vUartWriteData(aucData,ucLen * 4);
  //HAL_FLASH_Lock();
	
	return ucReturnValue;
}

/*
*
* \brief Handler function for read flash memory command.
*	\param uint8_t *puc8RxBuffer: incoming command buffer. 
* \return	-
*
*/void FBL_vRead_MemoryCmd(uint8_t *puc8RxBuffer)
{
		
	uint8_t ucReturnValue;
	uint32_t ulLength;
	uint32_t ulCRCHost;
	uint32_t ulMemoryAddress;
	uint8_t ucLengthToRead;
	
	/* 
		Get length and add 1 to include the total length computed by CRC 
	*/
	ulLength = puc8RxBuffer[0] + 1;
	/* 
		Get CRC position from rx command. Position to the end of the command puc8RxBuffer + ulLength 
		and then subtract 4 because CRC represents the last 4 bytes in the command
	*/
	ulCRCHost = *((uint32_t *)(puc8RxBuffer + ulLength - 4));
	/* 
		Get the base memory address
	*/
	ulMemoryAddress = *((uint32_t *)(&puc8RxBuffer[2]));
	/* 
		Get the length to be read
	*/
	ucLengthToRead = puc8RxBuffer[6];
	if(FBL_ucVerifyCRC(&puc8RxBuffer[0], ulLength-4,ulCRCHost) == FBL_CRC_SUCCESS)
	{
		FBL_vSendAck(ucLengthToRead * 4);
		/*
			Check the validity of the base memory address
		*/
		if(FBL_Verify_Address(ulMemoryAddress) == FBL_ADDRESS_VALID)
		{
		 
		 ucReturnValue = FBL_ucExecute_MemoryRead(ulMemoryAddress,ucLengthToRead);
		
		}
		else
		{
			ucReturnValue = FBL_ADDRESS_INVALID;
		}
		
		FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
	}
	else
	{
		FBL_vSendNack();
	}
	
	/* not supported yet */
	
}
/*****************************************STOP DECLARE MEMORY READ FUNCTIONS *******************************************************/

/*
*
* \brief Main handler function of the bootloader.
*	\param -
* \return	-
*
*/
void FBL_vUartReadData(void)
{
	uint8_t ucReturnValue = FBL_NONEXISTENT_COMMAND;
	while(1)
	{
		/* Print debug message */
		//FBL_vPrintMsg("FBL_DEBUG_MSG: Enter in command mode. \n");
		/* Prepare buffer to receive data */
		memset(auc8UartRxBuffer,0,FBL_UART_RX_LEN);
		/* Wait for first byte. Fist byte represents the length of the command */
		HAL_UART_Receive(FBL_UART_CHANNEL_DEBUG_INTERFACE,auc8UartRxBuffer,1,FBL_UART_DELAY);
		/* Wait for the following bytes - command, parameters (or not) and CRC value */ 
		HAL_UART_Receive(FBL_UART_CHANNEL_DEBUG_INTERFACE,&auc8UartRxBuffer[1],*auc8UartRxBuffer,FBL_UART_DELAY);
		switch (auc8UartRxBuffer[1])
		{
				case FBL_GET_VER:
				FBL_vGetVersion_Cmd(auc8UartRxBuffer);
						break;
				case FBL_GET_HELP:
			  FBL_vGetHelp_Cmd(auc8UartRxBuffer);
						break;
				case FBL_GET_CID:
				FBL_vGetCID_Cmd(auc8UartRxBuffer);
						break;
				case FBL_GET_RDP_STATUS:
			  FBL_vGetRDP_Cmd(auc8UartRxBuffer);
						break;
				case FBL_GO_TO_ADDR:
				FBL_vGoToAddress_Cmd(auc8UartRxBuffer);
						break;
				case FBL_FLASH_ERASE:
			  FBL_vFlashErase_Cmd(auc8UartRxBuffer);
						break;
				case FBL_MEM_WRITE:
			  FBL_vMemoryWrite_Cmd(auc8UartRxBuffer);
						break;
				case FBL_EN_RW_PROTECT:
			  FBL_vEnable_RW_Protect_Cmd(auc8UartRxBuffer);
						break;
				case FBL_MEM_READ:
				FBL_vRead_MemoryCmd(auc8UartRxBuffer);
						break;
				case FBL_READ_SECTOR_P_STATUS:
				FBL_vRead_Sector_ProtectionStatus_Cmd(auc8UartRxBuffer);
						break;
				case FBL_OTP_READ:
				FBL_vRead_OTP_Cmd(auc8UartRxBuffer);
						break;
				case FBL_DIS_R_W_PROTECT:
				FBL_vDisable_RW_Protect_Cmd(auc8UartRxBuffer);
						break;
				case FBL_FLASH_BINARY_FILE:
				FBL_vMemoryWrite_Cmd(auc8UartRxBuffer);
						break;
				 default:
				//FBL_vPrintMsg("FBL_DEBUG_MSG:Invalid command code received from host \n");
				//FBL_vUartWriteData((uint8_t *)&ucReturnValue, 1);
						break;
		}
	}
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
