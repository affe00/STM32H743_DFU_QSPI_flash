/*
 * usbd_dfu_qspi_flash.c
 *
 * QSPI flash interface for DFU media access.
 *
 *  Created on: Jan 21, 2020
 *      Author: affe00
------------------------------------------------------------------------------
 BSD 3-Clause License

Copyright (c) 2020, affe00
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------
 */
#include "usbd_dfu_qspi_flash.h"
#include "w25qxx.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern QSPI_HandleTypeDef hqspi;

// for debug output:
extern UART_HandleTypeDef huart1;
extern char sOutput[160];
// for debug output


static uint16_t QSPI_If_Init_FS(void);
static uint16_t QSPI_If_Erase_FS(uint32_t Add);
static uint16_t QSPI_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint8_t *QSPI_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len);
static uint16_t QSPI_If_DeInit_FS(void);
static uint16_t QSPI_If_GetStatus_FS(uint32_t Add, uint8_t Cmd, uint8_t *buffer);


/**
  * @}
  */

#if defined ( __ICCARM__ ) /* IAR Compiler */
  #pragma data_alignment=4
#endif
__ALIGN_BEGIN USBD_DFU_MediaTypeDef USBD_DFU_QSPI_FLASH_fops_FS __ALIGN_END =
{
   (uint8_t*)QSPI_FLASH_DESC_STR,
   QSPI_If_Init_FS,
   QSPI_If_DeInit_FS,
   QSPI_If_Erase_FS,
   QSPI_If_Write_FS,
   QSPI_If_Read_FS,
   QSPI_If_GetStatus_FS
};

/**
  * @brief  Memory initialization routine.
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t QSPI_If_Init_FS(void){
	uint16_t uRetVal;

	//snprintf(sOutput,159,"\r\n[QSPI_If_Init_FS]: Entry.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);


	//snprintf(sOutput,159,"\r\n[QSPI_If_Init_FS]: before W25Qxx_Init().\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);

	uRetVal=W25Qxx_Init();

	snprintf(sOutput,159,"[QSPI_If_Init_FS]: uRetVal=%d.\r\n",uRetVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);

	if (uRetVal==0)
		return (USBD_OK);
	else
		return (USBD_FAIL);
}

/**
  * @brief  De-Initializes Memory
  * @retval USBD_OK if operation is successful, MAL_FAIL else
  */
uint16_t QSPI_If_DeInit_FS(void)
{
  return (USBD_OK);
}

/**
  * @brief  Erase block(64KB).
  * @param  Add: Address of sector to be erased.
  * @retval 0 if operation is successful, MAL_FAIL else.
  */
uint16_t QSPI_If_Erase_FS(uint32_t Add){

	uint16_t uRetVal;

	snprintf(sOutput,159,"\r\n[QSPI_If_Erase_FS]: Add=%08lX.\r\n",Add);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);

	if (Add>=QSPI_FLASH_BASE && Add<=QSPI_FLASH_END){
		uRetVal=W25Qxx_Erase_Block((Add-QSPI_FLASH_BASE)>>16);

		snprintf(sOutput,159,"\r\n[QSPI_If_Erase_FS]: uRetVal=%ud.\r\n",uRetVal);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		//HAL_Delay(2);
	}
	else
		return(USBD_FAIL);
	if (uRetVal==0)
		return (USBD_OK);
	else
		return(USBD_FAIL);
}

/**
  * @brief  Memory write routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be written (in bytes).
  * @retval USBD_OK if operation is successful, MAL_FAIL else.
  */
uint16_t QSPI_If_Write_FS(uint8_t *src, uint8_t *dest, uint32_t Len)
{
	uint16_t uRetVal;

	snprintf(sOutput,159,"\r\n[QSPI_If_Write_FS]: src=%08lX, dest=%08lX, Len=%ld.\r\n",(uint32_t)src, (uint32_t)dest, Len);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);

	if (dest>=(uint8_t*)QSPI_FLASH_BASE && (dest+Len)<=(uint8_t*)QSPI_FLASH_END){
		uRetVal=W25Qxx_Write_NoCheck(src, ((uint32_t)dest-QSPI_FLASH_BASE), Len);

		snprintf(sOutput,159,"\r\n[QSPI_If_Write_FS]: uRetVal=%ud.\r\n",uRetVal);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		//HAL_Delay(2);
	}
	else
		return(USBD_FAIL);
	if (uRetVal==0)
		return (USBD_OK);
	else
		return(USBD_FAIL);
}

/**
  * @brief  Memory read routine.
  * @param  src: Pointer to the source buffer. Address to be written to.
  * @param  dest: Pointer to the destination buffer.
  * @param  Len: Number of data to be read (in bytes).
  * @retval Pointer to the physical address where data should be read.
  */
uint8_t *QSPI_If_Read_FS(uint8_t *src, uint8_t *dest, uint32_t Len){
	uint16_t uRetVal;

	//snprintf(sOutput,159,"\r\n[QSPI_If_Read_FS]: src=%08lX, dest=%08lX, Len=%ld.\r\n",(uint32_t)src, (uint32_t)dest, Len);
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	//HAL_Delay(2);


	if (src>=(uint8_t*)QSPI_FLASH_BASE && (src+Len)<=(uint8_t*)QSPI_FLASH_END){
		uRetVal=W25Qxx_Read(dest, ((uint32_t)src-QSPI_FLASH_BASE), Len);

		//snprintf(sOutput,159,"\r\n[QSPI_If_Read_FS]: uRetVal=%ud.\r\n",uRetVal);
		//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		//HAL_Delay(2);
	}
	UNUSED(uRetVal);
	return (uint8_t *) (dest);
	/* no way to indicate fail from return value */
}

/**
  * @brief  Get status routine
  * @param  Add: Address to be read from
  * @param  Cmd: Number of data to be read (in bytes)
  * @param  buffer: used for returning the time necessary for a program or an erase operation
  * @retval USBD_OK if operation is successful
  */
uint16_t QSPI_If_GetStatus_FS(uint32_t Add, uint8_t Cmd, uint8_t *buffer)
{
  switch (Cmd)
  {
    case DFU_MEDIA_PROGRAM:
	buffer[1] = (uint8_t) QSPI_FLASH_PROGRAM_TIME;
	buffer[2] = (uint8_t) (QSPI_FLASH_PROGRAM_TIME << 8);
	buffer[3] = 0;
    break;

    case DFU_MEDIA_ERASE:
    default:
	buffer[1] = (uint8_t) QSPI_FLASH_ERASE_TIME;
	buffer[2] = (uint8_t) (QSPI_FLASH_ERASE_TIME << 8);
	buffer[3] = 0;
    break;
  }
  return (USBD_OK);
}

