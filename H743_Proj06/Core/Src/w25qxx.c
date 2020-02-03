/*
 * w25qxx.c
 *
 * Media Access Layer for Winbond W25Qxx series flash memories.
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

#include "w25qxx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_qspi.h"
#include <stdio.h>
#include <string.h>

extern QSPI_HandleTypeDef hqspi;

extern UART_HandleTypeDef huart1;
extern char sOutput[160];

static uint16_t  W25Qxx_TYPE		= W25Q64;	//W25Q64: 64M bits, 4KB page, 64KB block, 128 blocks
static uint8_t W25Qxx_ADDRESS_BITS	= 22;		//8MB: A[22:0]
static uint8_t W25Qxx_QPI_MODE		= 0;		//0:SPI, 1:QSPI, QPI
static uint8_t W25Qxx_DummyCycles	= 8;		//default value


uint16_t W25Qxx_DummyCyclesCfg(void){

	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef Status;
	uint8_t uBuf=0;

	//snprintf(sOutput,159,"[W25Qxx_DummyCyclesCfg]: Entry.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	W25Qxx_Write_Enable();

	/* Set Parameters */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}
	//sCommand.Address		   = 0;
	//sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData            = 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	uBuf= 0x02<<4;   //0x00<<4: 2 clocks, 0x01<<4: 4 clocks, 0x02<<4: 6 clocks, 0x03<<4: 8 clocks
					 //applicable to FAST_READ_CMD 0x0B, Fast Read Quad I/O 0xEB, Burst Read w/Wrap 0x0C
	W25Qxx_DummyCycles = ((uBuf>>4) + 1)<<1;
	sCommand.Instruction       = SET_READ_PARAM_CMD;
    Status=HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	if (Status != HAL_OK){
		return (0xFF00 |(uint16_t)Status);
	}
	Status=HAL_QSPI_Transmit(&hqspi, &uBuf, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	if (Status!= HAL_OK){
		return (0xFB00|(uint16_t)Status);
	}
	return (0);
}

uint16_t W25Qxx_Write_Enable(void) {
	QSPI_CommandTypeDef     sCommand;
	QSPI_AutoPollingTypeDef sConfig;

	HAL_StatusTypeDef Status;
	//uint8_t uBuf;
	
	//snprintf(sOutput,159,"[W25Qxx_Write_Enable]: Entry.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	

	/* Enable write operations */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode	= QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	else {
		sCommand.InstructionMode	= QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	sCommand.AlternateByteMode	= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles		= 0;
	sCommand.DdrMode			= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode			= QSPI_SIOO_INST_EVERY_CMD;

	sCommand.Instruction		= WRITE_ENABLE_CMD;
	Status = HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	//snprintf(sOutput,159,"[W25Qxx_Write_Enable]: WRITE_ENABLE_CMD:%04X.\r\n",Status);
    //HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	if ( Status!= HAL_OK) {
		return(0xFE00 | (uint16_t)Status);
	}

	/* Configure automatic polling mode to wait for write enabling */
	sConfig.Match           = SR_WEL;
	sConfig.Mask            = SR_WEL;
	sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval        = 0x0F;
	sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode	= QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode	= QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_1_LINE;
	}
	sCommand.Instruction    = READ_STATUS_REG_1_CMD;

	Status=HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	//snprintf(sOutput,159,"[W25Qxx_Write_Enable]: READ_STATUS_REG_1_CMD:%04X.\r\n",Status);
    //HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	if ( Status != HAL_OK){
		return(0xFD00 | (uint16_t)Status);
	}
	//snprintf(sOutput,159,"[W25Qxx_Write_Enable]: return 0.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	return(0);
}

uint16_t W25Qxx_Write_Disable(void) {
	QSPI_CommandTypeDef     sCommand;
	//QSPI_AutoPollingTypeDef sConfig;
	HAL_StatusTypeDef Status;
	uint8_t uBuf;
	uint16_t i;

	/* read SR1 and check before act */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}
	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	hqspi.Instance->DLR=1-1;                           //配置数据长度
	sCommand.Instruction       = READ_STATUS_REG_1_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return(0xFFFF);
	}
	HAL_QSPI_Receive(&hqspi,&uBuf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	if ( (uBuf & SR_WEL) == 0)
		return (0);

	/* Disable write operations */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode	= QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	else {
		sCommand.InstructionMode	= QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode		= QSPI_ADDRESS_NONE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	sCommand.Instruction       = WRITE_DISABLE_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return(0xFFFE);
	}

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}
	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData            = 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	hqspi.Instance->DLR=1-1;  // bytes to transfer -1
	sCommand.Instruction       = READ_STATUS_REG_1_CMD;
	uBuf=SR_WEL;
	while((uBuf&SR_WEL) !=0){
		Status=HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if (Status!= HAL_OK)
			return(0xFF00 | (uint16_t)Status);
	    Status=HAL_QSPI_Receive(&hqspi,&uBuf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if ( Status != HAL_OK)
			return(0xFE00|(uint16_t)Status);
		snprintf(sOutput,159,".");
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		for(i=0;i<0xFFFF;i++) uBuf&=0xFF;
	}

	return(0);
}

uint16_t W25Qxx_ReadID(void){
	QSPI_CommandTypeDef     sCommand;
	//QSPI_AutoPollingTypeDef sConfig;
	uint8_t uBuf[2];
	uint16_t uID;

	//snprintf(sOutput,159,"[W25Qxx_ReadID]: Entry.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}
	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	sCommand.NbData            = 2;
	hqspi.Instance->DLR=2-1;                           //配置数据长度
	sCommand.Instruction       = MFT_DEV_ID_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return(0xFFFF);
	}
	if (HAL_QSPI_Receive(&hqspi,uBuf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		return(0xFFFE);
	}
	else {
		uID=(uBuf[0]<<8)|uBuf[1];
		return uID;
	}
}

uint16_t W25Qxx_Init(void){
	QSPI_CommandTypeDef sCommand;
	uint16_t uRetVal;
	uint8_t uBuf;

	snprintf(sOutput,159,"\r\n[W25Qxx_Init]: Entry.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	uRetVal = W25Qxx_QPI_Enable();	// remember to do W25Qxx_DummyCyclesCfg() after W25Qxx_QPI_Enable()
							// consider putting W25Qxx_DummyCyclesCfg() inside W25Qxx_QPI_Enable()

	snprintf(sOutput,159,"[W25Qxx_Init]: W25Qxx_QPI_Enable()=%04x.\r\n",uRetVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	uRetVal = W25Qxx_DummyCyclesCfg();
	snprintf(sOutput,159,"[W25Qxx_Init]: W25Qxx_DummyCyclesCfg()=%04x.\r\n",uRetVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	uRetVal = W25Qxx_Write_Enable();
	snprintf(sOutput,159,"[W25Qxx_Init]: W25Qxx_Write_Enable()=%04x.\r\n",uRetVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	W25Qxx_TYPE = W25Qxx_ReadID();
	snprintf(sOutput,159,"[W25Qxx_Init]: W25Qxx_ReadID()=%04x.\r\n",W25Qxx_TYPE);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	
	if (W25Qxx_TYPE == W25Q64)
		W25Qxx_ADDRESS_BITS=22;
	else if (W25Qxx_TYPE == W25Q80)
		W25Qxx_ADDRESS_BITS=19;
	else if (W25Qxx_TYPE == W25Q16)
		W25Qxx_ADDRESS_BITS=20;
	else if (W25Qxx_TYPE == W25Q32)
		W25Qxx_ADDRESS_BITS=21;
	else if (W25Qxx_TYPE == W25Q128)
		W25Qxx_ADDRESS_BITS=23;
	else if (W25Qxx_TYPE == W25Q256){
		W25Qxx_ADDRESS_BITS=24;
		/* Enable 4 byte address mode */
		if(W25Qxx_QPI_MODE){
			sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
			sCommand.AddressMode       = QSPI_ADDRESS_NONE;
			sCommand.DataMode          = QSPI_DATA_4_LINES;
		}
		else {
			sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
			sCommand.AddressMode       = QSPI_ADDRESS_NONE;
			sCommand.DataMode          = QSPI_DATA_1_LINE;
		}
		//sCommand.Address		   = 0;
		//sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
		sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
		sCommand.DummyCycles       = 0;
		sCommand.NbData            = 1;
		sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
		sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
		sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

		sCommand.Instruction       = READ_STATUS_REG_3_CMD;
		if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
			return (0xFFFF);
		}
		if (HAL_QSPI_Receive(&hqspi, (uint8_t *)(&uBuf), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
			return (0xFFFE);
		}
		if((uBuf & 0x01) == 0) {
			sCommand.DataMode          = QSPI_DATA_NONE;
			sCommand.Instruction       = ENABLE_4BYTE_ADDR_CMD;
			if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
				return (0xFFFB);
			}
		}
	} else
		return (0xFFFA);  // W25Qxx_TYPE not recognized

	snprintf(sOutput,159,"[W25Qxx_Init]: return 0.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	return (0);
}

uint16_t W25Qxx_QPI_Enable(void){
	QSPI_CommandTypeDef sCommand;
	HAL_StatusTypeDef Status;
	uint8_t uBuf[2];
	uint16_t uRetVal;

	//snprintf(sOutput,159,"[W25Qxx_QPI_Enable]: Entry.\r\n");
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	/* read SR2 and check for QE bit */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.DataMode          = QSPI_DATA_1_LINE;

	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData            = 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	hqspi.Instance->DLR=1-1;  // bytes to transfer -1
	sCommand.Instruction       = READ_STATUS_REG_2_CMD;
	Status=HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
	if (Status!= HAL_OK)
		return(0xFF00 | (uint16_t)Status);
    Status=HAL_QSPI_Receive(&hqspi,&uBuf[1],HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    snprintf(sOutput,159,"[W25Qxx_QPI_Enable]: HAL_QSPI_Receive()=%04X. REG_2=%02X\r\n",Status, uBuf[1]);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	

	hqspi.Instance->DLR=1-1;  // bytes to transfer -1
	sCommand.Instruction       = READ_STATUS_REG_1_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		return(0xFFFF);
    Status=HAL_QSPI_Receive(&hqspi,&uBuf[0],HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
    snprintf(sOutput,159,"[W25Qxx_QPI_Enable]: HAL_QSPI_Receive()=%04X. REG_1=%02X\r\n",Status, uBuf[0]);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	if ( (uBuf[1] & (SR_QE>>8)) != (SR_QE>>8)){
		uRetVal=W25Qxx_Write_Enable();

		snprintf(sOutput,159,"[W25Qxx_QPI_Enable]: W25Qxx_Write_Enable()=%04X\r\n",uRetVal);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		
		uBuf[1]|=(SR_QE>>8);

		hqspi.Instance->DLR=2-1;  // bytes to transfer -1
		sCommand.Instruction       = WRITE_STATUS_REG_1_CMD;
		Status=HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if (Status != HAL_OK)
			return(0xFE00 |(uint16_t)Status);
		Status=HAL_QSPI_Transmit(&hqspi, &uBuf[0], HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if ( Status != HAL_OK){
			return (0xFB00|(uint16_t)Status);
		}
	}

	/* ENABLE_QPI_CMD */
	sCommand.Instruction       = ENABLE_QPI_CMD;
	sCommand.DataMode          = QSPI_DATA_NONE;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		return(0xFFFA);
	else{
		W25Qxx_QPI_MODE=1;
		return(0);
	}
}

uint16_t W25Qxx_QPI_Disable(void){
	QSPI_CommandTypeDef sCommand;
	//uint8_t uBuf[2];

	/* read SR2 and check for QE bit */
	sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressMode       = QSPI_ADDRESS_NONE;
	sCommand.DataMode          = QSPI_DATA_NONE;

	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData            = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.Instruction       = ENABLE_QPI_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
		return(0xFFFA);
	else{
		W25Qxx_QPI_MODE=0;
		return(0);
	}
}

uint16_t W25Qxx_Read(uint8_t* pBuffer,uint32_t Address,uint16_t Length) {
	QSPI_CommandTypeDef     sCommand;

	snprintf(sOutput,159,"[W25Qxx_Read]: Entry. pBuffer=%08lX, Address=%08lX, Length=%d\r\n", (uint32_t)pBuffer, Address, Length);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
	

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}

	sCommand.Address		   = Address;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = W25Qxx_DummyCycles;
	sCommand.NbData			   = (uint32_t)Length + 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	if(W25Qxx_TYPE == W25Q256)
		sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;

	sCommand.Instruction       = FAST_READ_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		return(0xFFFF);
	}
    if (HAL_QSPI_Receive(&hqspi,pBuffer,HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		return(0xFFFE);
	}
	else
		return(0);
}

uint16_t W25Qxx_Write_Page(uint8_t* pBuffer,uint32_t Address,uint32_t Length) {
	QSPI_CommandTypeDef     sCommand;
	uint32_t LengthValid;
	uint16_t uRetVal;

	UNUSED(uRetVal);
	snprintf(sOutput,159,"[W25Qxx_Write_Page]: Entry. pBuffer=%08lX, Address=%08lX, Length=%ld\r\n", (uint32_t)pBuffer, Address, Length);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	W25Qxx_Write_Enable();
	/* Boundary check */
	if ( ( ((Address + Length) & 0xFFFFFF00) - (Address & 0xFFFFFF00) ) == 0)
		LengthValid = Length-1;
	else
		LengthValid = (Address & 0xFFFFFF00) + 0xFF - Address;

	//snprintf(sOutput,159,"[W25Qxx_Write_Page]: LengthValid=%ld\r\n", LengthValid);
	//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}

	sCommand.Address		   = Address;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData			   = LengthValid + 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	if(W25Qxx_TYPE == W25Q256)
		sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;

	hqspi.Instance->DLR=LengthValid;
	sCommand.Instruction       = PAGE_PROGRAM_CMD;
	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		snprintf(sOutput,159,"[W25Qxx_Write_Page]: return %d\r\n", 0xFFFF);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

		return(0xFFFF);
	}
	if (HAL_QSPI_Transmit(&hqspi,pBuffer,HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		snprintf(sOutput,159,"[W25Qxx_Write_Page]: return %d\r\n", 0xFFFE);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		return(0xFFFE);
	}
	else {
		uRetVal=W25Qxx_Wait_Busy();
		//snprintf(sOutput,159,"[W25Qxx_Write_Page]: W25Qxx_Wait_Busy()=%d\r\n", (uint16_t)uRetVal);
		//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

		//snprintf(sOutput,159,"[W25Qxx_Write_Page]: return %d\r\n", (uint16_t)LengthValid);
		//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

		return((uint16_t)LengthValid);
	}
}

uint16_t W25Qxx_Write_NoCheck(uint8_t* pBuffer,uint32_t Address,uint32_t Length) {
	uint32_t LengthWritten;

	snprintf(sOutput,159,"[W25Qxx_Write_NoCheck]: Entry. pBuffer=%08lX, Address=%08lX, Length=%ld\r\n", (uint32_t)pBuffer, Address, Length);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	while (Length>0){
		LengthWritten=W25Qxx_Write_Page(pBuffer,Address,Length);
		if (LengthWritten>0xFF)
			return(0xFFFF);
		pBuffer+=LengthWritten+1;
		Address+=LengthWritten+1;
		Length-=LengthWritten+1;
	}
	return(0);
}

// erase 64k block
uint16_t W25Qxx_Erase_Block(uint32_t BlockIndex) {
	QSPI_CommandTypeDef     sCommand;
	uint16_t uRetVal;
	uint32_t Address=BlockIndex<<16;
	W25Qxx_Write_Enable();

	/* Erasing Sequence : from QSPI */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode	= QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode		= QSPI_ADDRESS_4_LINES;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	else {
		sCommand.InstructionMode	= QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode		= QSPI_ADDRESS_1_LINE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	sCommand.Instruction		= BLOCK_ERASE_CMD;
	sCommand.Address			= Address;
	sCommand.AddressSize		= QSPI_ADDRESS_24_BITS;

	sCommand.AlternateByteMode	= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles		= 0;
	sCommand.DdrMode			= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode			= QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		return(0xFFFF);
	}

	/* Configure automatic polling mode to wait for end of erase */
	//QSPI_AutoPollingMemReady(&hqspi);
	uRetVal=W25Qxx_Wait_Busy();
	snprintf(sOutput,159,"[W25Qxx_Erase_Block]: W25Qxx_Wait_Busy()=%d\r\n", (uint16_t)uRetVal);
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	return (0);
}

// erase a 4k sector
uint16_t W25Qxx_Erase_Sector(uint32_t SectorIndex) {
	QSPI_CommandTypeDef     sCommand;
	uint32_t Address=SectorIndex<<12;
	W25Qxx_Write_Enable();

	/* Erasing Sequence : from QSPI */
	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode	= QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode		= QSPI_ADDRESS_4_LINES;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	else {
		sCommand.InstructionMode	= QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode		= QSPI_ADDRESS_1_LINE;
		sCommand.DataMode			= QSPI_DATA_NONE;
	}
	sCommand.Instruction		= SECTOR_ERASE_CMD;
	sCommand.Address			= Address;
	sCommand.AddressSize		= QSPI_ADDRESS_24_BITS;

	sCommand.AlternateByteMode	= QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles		= 0;
	sCommand.DdrMode			= QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode			= QSPI_SIOO_INST_EVERY_CMD;

	if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK){
		return(0xFFFF);
	}

	/* Configure automatic polling mode to wait for end of erase */
	//QSPI_AutoPollingMemReady(&hqspi);
	W25Qxx_Wait_Busy();
	return(0);
}

// wait for busy bit to clear
uint16_t W25Qxx_Wait_Busy(void) {
	QSPI_CommandTypeDef     sCommand;
	//QSPI_AutoPollingTypeDef sConfig;

	HAL_StatusTypeDef Status;
	uint8_t uBuf;
	uint16_t i;

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_NONE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}
	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_8_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = 0;
	sCommand.NbData            = 1;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

	hqspi.Instance->DLR=1-1;  // bytes to transfer -1
	sCommand.Instruction       = READ_STATUS_REG_1_CMD;
	uBuf=SR_BUSY;
	while((uBuf&SR_BUSY) !=0){
		Status=HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if (Status!= HAL_OK)
			return(0xFF00 | (uint16_t)Status);
	    Status=HAL_QSPI_Receive(&hqspi,&uBuf,HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
		if ( Status != HAL_OK)
			return(0xFE00|(uint16_t)Status);
		snprintf(sOutput,159,".");
		//HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		for(i=0;i<0xFFFF;i++) uBuf&=0xFF;
	}
	return(0);
}



uint16_t W25Qxx_MemMap(void) {
	QSPI_CommandTypeDef     sCommand;
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;
	uint16_t uRetVal;
	HAL_StatusTypeDef Status;

	snprintf(sOutput,159,"[W25Qxx_MemMap]: Entry.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);

	uRetVal=W25Qxx_Init();

	if(uRetVal!=0){
		snprintf(sOutput,159,"[W25Qxx_MemMap]: W25Qxx_Init() failed! uRetVal=%04X.\r\n",uRetVal);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		return(0xFF00|(uint32_t)uRetVal);
	}

	/* Configure the memory mapped mode */
	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	s_mem_mapped_cfg.TimeOutPeriod     = 0;

	if(W25Qxx_QPI_MODE){
		sCommand.InstructionMode   = QSPI_INSTRUCTION_4_LINES;
		sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
		sCommand.DataMode          = QSPI_DATA_4_LINES;
	}
	else {
		sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
		sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
		sCommand.DataMode          = QSPI_DATA_1_LINE;
	}

	sCommand.Address		   = 0;
	sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DummyCycles       = W25Qxx_DummyCycles;
	sCommand.NbData			   = 0;
	sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	if(W25Qxx_TYPE == W25Q256)
		sCommand.AddressSize       = QSPI_ADDRESS_32_BITS;

	sCommand.Instruction       = FAST_READ_CMD;

	Status=HAL_QSPI_MemoryMapped(&hqspi, &sCommand,  &s_mem_mapped_cfg);
	if (Status != HAL_OK) {
		snprintf(sOutput,159,"[W25Qxx_MemMap]: HAL_QSPI_MemoryMapped() failed! Status=%08lX.\r\n",(uint32_t)Status);
		HAL_UART_Transmit(&huart1, (uint8_t*)sOutput,strlen(sOutput), 0xFFFF);
		
		return(0xFF00|(uint32_t)Status);
	}
	else
		return(0);
}
