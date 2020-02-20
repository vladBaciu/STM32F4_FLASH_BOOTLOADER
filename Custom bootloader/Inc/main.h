/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
	* @autor					: Vlad-Eusebiu Baciu
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define FBL_CRC_CUSTOM 							(0U)	
#define FBL_CRC_HAL									(1U)
#define FBL_CRC_POLYNOME				    (0x04C11DB7)

#define FBL_CRC_VERIFY_VERSION			FBL_CRC_CUSTOM

#define FBL_SRAM1_SIZE (1024 * 112)
#define FBL_SRAM2_SIZE (1024 * 16)
#define FBL_FLASH_SIZE (1024 * 1000)
#define FBL_BKPSRAM_SIZE (1024 * 4 )

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void FBL_vJumpToUserApplication(void);

void FBL_vUartReadData(void);
void FBL_vGetVersion_Cmd(uint8_t *puc8RxBuffer);
void FBL_vGetHelp_Cmd(uint8_t *puc8RxBuffer);
void FBL_vGetCID_Cmd(uint8_t *puc8RxBuffer);
void FBL_vGetRDP_Cmd(uint8_t *puc8RxBuffer);
void FBL_vGoToAddress_Cmd(uint8_t *puc8RxBuffer);
void FBL_vFlashErase_Cmd(uint8_t *puc8RxBuffer);
void FBL_vMemoryWrite_Cmd(uint8_t *puc8RxBuffer);
void FBL_vEnable_RW_Protect_Cmd(uint8_t *puc8RxBuffer);
void FBL_vDisable_RW_Protect_Cmd(uint8_t *puc8RxBuffer);
void FBL_vRead_Sector_ProtectionStatus_Cmd(uint8_t *puc8RxBuffer);
void FBL_vRead_MemoryCmd(uint8_t *puc8RxBuffer);
void FBL_vRead_OTP_Cmd(uint8_t *puc8RxBuffer);

void FBL_vSendAck(uint8_t ucLengthResponse);
void FBL_vSendNack(void);


uint8_t FBL_ucVerifyCRC(uint8_t *pucData, uint32_t ulLength,uint32_t ulCRCHost);
uint8_t FBL_ucGetBootloaderVersion(void);
void FBL_vUartWriteData(uint8_t *pucBuffer, uint32_t ulLength);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_I2C_SPI_Pin GPIO_PIN_3
#define CS_I2C_SPI_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_9
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define OLD_APPLICATION		(1U)
#define QT_APPLICATION		(2U)
#define COMMAND_SET		QT_APPLICATION	


#if (COMMAND_SET == OLD_APPLICATION)
/* This command is used to read the bootloader version from the MCU */
#define FBL_GET_VER						(0x51)

/* This command is used to know what are the commands supported by the bootloader */
#define FBL_GET_HELP					(0x52)

/*This command is used to read the MCU chip identification number */
#define FBL_GET_CID						(0x53)

/*This command is used to read the FLASH Read Protection level */
#define FBL_GET_RDP_STATUS		(0x54)

/*This command is used to jump bootloader to specified address */
#define FBL_GO_TO_ADDR			  (0x55)

/*This command is used to mass erase or sector erase of the user flash */
#define FBL_FLASH_ERASE       (0x56)

/*This command is used to write data in to different memories of the MCU */
#define FBL_MEM_WRITE					(0x57)

/*This command is used to enable or disable read/write protect on different sectors of the user flash */
#define FBL_EN_RW_PROTECT			(0x58)

/*This command is used to read data from different memories of the microcontroller */
#define FBL_MEM_READ					(0x59)

/*This command is used to read all the sector protection status */
#define FBL_READ_SECTOR_P_STATUS	(0x5A)


/*This command is used to read the OTP contents */
#define FBL_OTP_READ							(0x5B)


/*This command is used disable all sector read/write protection */
#define FBL_DIS_R_W_PROTECT				(0x5C)

#elif (COMMAND_SET == QT_APPLICATION)

/* This command is used to read the bootloader version from the MCU */
#define FBL_GET_VER						(0x14)

/* This command is used to know what are the commands supported by the bootloader */
#define FBL_GET_HELP					(0x22)

/*This command is used to read the MCU chip identification number */
#define FBL_GET_CID						(0x44)

/*This command is used to read the FLASH Read Protection level */
#define FBL_GET_RDP_STATUS		(0x25)

/*This command is used to jump bootloader to specified address */
#define FBL_GO_TO_ADDR			  (0x19)

/*This command is used to mass erase or sector erase of the user flash */
#define FBL_FLASH_ERASE       (0x84)

/*This command is used to write data in to different memories of the MCU */
#define FBL_MEM_WRITE					(0x32)

/*This command is used to enable or disable read/write protect on different sectors of the user flash */
#define FBL_EN_RW_PROTECT			(0x3C)

/*This command is used to read data from different memories of the microcontroller */
#define FBL_MEM_READ					(0x2B)

/*This command is used to read all the sector protection status */
#define FBL_READ_SECTOR_P_STATUS	(0x78)

/*This command is used to read the OTP contents */
#define FBL_OTP_READ							(0x1F)

/*This command is used disable all sector read/write protection */
#define FBL_DIS_R_W_PROTECT				(0x6C)

#else

#error "Unavailable command set"

#endif

#define FLASH_SECTOR2_BASE_ADDRESS (0x08008000U)
#define FBL_ACK_VALUE 						 (0xA5)
#define FBL_NACK_VALUE 						 (0x7F)
#define FBL_CRC_FAIL							 (0xC0)
#define FBL_CRC_SUCCESS						 (0xB0)
#define FBL_NONEXISTENT_COMMAND		 (0xD0)

#define FBL_VERSION								 (0xAA)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
