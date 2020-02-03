# STM32H743_DFU_QSPI_flash
This is a host program with DFU programming interface on STM32H743 dev. board. The application program can be written onto QSPI flash on the dev. board, or spare memory location in the onchip flash memory.

This program works on a STM32H743VIT6 dev. board with Winbond W25Q64 QSPI flash. It provides DFU host functions for work with ST's DfuSe Demo program. 

Reset with K2 pressed(pin C5=0) will direct program into DFU mode. Reset without K2 pressed will allow program to find code for XIP on flash memory.

By:

```c
USBD_DFU_RegisterMedia(&hUsbDeviceFS, &USBD_DFU_QSPI_FLASH_fops_FS)
```
in H743_Proj06\USB_DEVICE\App\usb_device.c, it will program onto W25Q64, which can be mapped at 0x90000000 for XIP.

By:
```c
USBD_DFU_RegisterMedia(&hUsbDeviceFS, &USBD_DFU_fops_FS)
```
in H743_Proj06\USB_DEVICE\App\usb_device.c, it will program onto on-chip flash starting at 0X08020000.

It is possible to add a Media Access Layer so that both memory area will be shown in DfuSe Demo, and it will be possible to choose which memory area is to be programmed. However this is not implemented yet.

### The application program
The program to be flashed onto the designated memory area need to be linked with awareness of the target address.
The location of interrupt vector table as specifed in SystemInit() in system_stm32h7xx.c also need to be specified.

### The STM32CubeMX generated template
This project does not contain the standard files that comes with STM32Cube_FW_H7_V1.5.0.

You need to generate your template source code from the provided .ioc file, and then add back the programs from here.

Also need to mention is that in V1.6.0 of STM32CubeH7 firmware, everything QSPI was renamed into OSPI, therefore the HAL defs and calls will need to be updated.

### The Initiative
For STM32H750 which comes with 128KB flash memory, there is a more urgent need for XIP on QSPI flash functionality. STM32H743 comes with 2MB flash memory. So size is not the main concern.

The initiative comes from such worries that frequent erase and program to the on-chip flash memory may exhaust its life. Somewhere in an ST document mentioned 10k cycles. It can be achieved by programming 30 times everyday for about a year. But 10k is not the minimum life time. The document basically says the listed erase and programing timings are tested after 10k cycles.

With XIP on QSPI functionality it makes the programmer feel at ease. Even if the first block of the QSPI flash is worn out, it is highly possible the rest of it is still usable and this host program can be changed to boot from e.g. 0x90020000.

### Board markings: 
mcudev.taobao.com DevEBox STM32F7XX_M Ver:V2.0 SN:1907

The dev. board can be bought at: [taobao](https://item.taobao.com/item.htm?&id=601083694791) or: [AliExpress](https://www.aliexpress.com/item/4000235276780.html)

### Cross References:

In `STM32Cube_FW_H7_V1.5.0\Projects\STM32H743I-EVAL\Applications\ExtMem_CodeExecution\ExtMem_Boot`, sample code is provided for booting from QSPI flash or NOR flash.

In `STM32Cube_FW_H7_V1.5.0\Projects\STM32H743I-EVAL\Applications\USB_Device\DFU_Standalone`, sample code is provided for DFU functions.

[STM32Cube USB Device Library dual DFU](https://community.st.com/s/question/0D50X00009Xka1tSAB/stm32cube-usb-device-library-dual-dfu) on ST Community.

[How to export Alternate memory regions for USB DFU?](https://community.st.com/s/question/0D50X00009XkeS5SAJ/how-to-export-alternate-memory-regions-for-usb-dfu) on ST Community.
