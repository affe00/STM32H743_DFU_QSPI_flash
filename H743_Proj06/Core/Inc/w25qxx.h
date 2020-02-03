/*
 * w25qxx.h
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

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

#include <stdint.h>

/* list of supported chips */
#define W25Q80 	0xEF13
#define W25Q16 	0xEF14
#define W25Q32 	0xEF15
#define W25Q64 	0xEF16
#define W25Q128	0xEF17
#define W25Q256 0xEF18

// extern u16 W25QXX_TYPE;

/* Winbond W25Q64 */
/* address bit of flash, 8MB: A[22:0] */
#define QSPI_FLASH_SIZE						22
#define DUMMY_CLOCK_CYCLES_READ_QUAD		8

#define ENABLE_QPI_CMD						((uint8_t)0x38)
#define DISABLE_QPI_CMD						((uint8_t)0xFF)
#define WRITE_ENABLE_CMD					((uint8_t)0x06)
#define WRITE_DISABLE_CMD					((uint8_t)0x04)
#define VOLAT_STATUS_REG_WE_CMD				((uint8_t)0x50)

#define SET_READ_PARAM_CMD					((uint8_t)0xC0)
#define READ_STATUS_REG_1_CMD				((uint8_t)0x05)
#define READ_STATUS_REG_2_CMD				((uint8_t)0x35)
#define READ_STATUS_REG_3_CMD				((uint8_t)0x15)
#define WRITE_STATUS_REG_1_CMD				((uint8_t)0x01)
//#define WRITE_STATUS_REG_2_CMD				((uint8_t)0x31)
//#define WRITE_STATUS_REG_3_CMD				((uint8_t)0x11)

#define FAST_READ_CMD						((uint8_t)0x0B)
#define PAGE_PROGRAM_CMD					((uint8_t)0x02)
// 64K erase:
#define BLOCK_ERASE_CMD						((uint8_t)0xD8)
// 4K erase:
#define SECTOR_ERASE_CMD					((uint8_t)0x20)

#define ENABLE_4BYTE_ADDR_CMD 			((uint8_t)0xB7)
#define EXIT_4BYTE_ADDR_CMD   			((uint8_t)0xE9)
#define MFT_DEV_ID_CMD					((uint8_t)0x90)

#define W25X_ReadData			0x03
#define W25X_FastReadDual		0x3B
#define W25X_ChipErase			0xC7
#define W25X_PowerDown			0xB9
#define W25X_ReleasePowerDown	0xAB
#define W25X_DeviceID			0xAB
#define W25X_JedecDeviceID		0x9F

// Status Register bits
#define SR_BUSY					((uint8_t)1<<0)
#define SR_WEL					((uint8_t)1<<1)
#define SR_BP0					((uint8_t)1<<2)
#define SR_BP1					((uint8_t)1<<3)
#define SR_BP2					((uint8_t)1<<4)
#define SR_TB					((uint8_t)1<<5)
#define SR_SEC					((uint8_t)1<<6)
#define SR_SRP0					((uint8_t)1<<7)
#define SR_SRP1					((uint8_t)1<<8)
#define SR_QE					((uint8_t)1<<9)
#define SR_R					((uint8_t)1<<10)
#define SR_LB1					((uint8_t)1<<11)
#define SR_LB2					((uint8_t)1<<12)
#define SR_LB3					((uint8_t)1<<13)
#define SR_CMP					((uint8_t)1<<14)
#define SR_SUS					((uint8_t)1<<15)

uint16_t W25Qxx_DummyCyclesCfg(void);
uint16_t W25Qxx_Write_Enable(void);
uint16_t W25Qxx_Write_Disable(void);
uint16_t W25Qxx_ReadID(void);
uint16_t W25Qxx_Init(void);
uint16_t W25Qxx_QPI_Enable(void);
uint16_t W25Qxx_QPI_Disable(void);
uint16_t W25Qxx_Read(uint8_t* pBuffer,uint32_t Address,uint16_t Length);
uint16_t W25Qxx_Write_Page(uint8_t* pBuffer,uint32_t Address,uint32_t Length);
uint16_t W25Qxx_Write_NoCheck(uint8_t* pBuffer,uint32_t Address,uint32_t Length);
uint16_t W25Qxx_Erase_Block(uint32_t BlockIndex);
uint16_t W25Qxx_Erase_Sector(uint32_t SectorIndex);
uint16_t W25Qxx_Wait_Busy(void);
uint16_t W25Qxx_MemMap(void);

#endif /* INC_W25QXX_H_ */
