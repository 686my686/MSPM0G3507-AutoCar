#ifndef _BSP_FLASH_W25Q64_H_
#define _BSP_FLASH_W25Q64_H_

/**
本程序用于w25q64,其他内存flash通用
有些地方稍做修改即可
w25q64有8MB内存
一页256个字节
4096字节也就是（4KB）为一个扇区
16个扇区为一块（65536字节）
一共128块（8388608字节）
This program is used for w25q64, and other memory flash is common
Some areas can be slightly modified
W25q64 has 8MB of memory
256 bytes per page
4096 bytes, which means (4KB) is a sector
16 sectors in one block (65536 bytes)
A total of 128 blocks (8388608 bytes)
**/

#include "AllHeader.h"

#define W25Q80 	0XEF13
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16 //ID


extern u16 SPI_FLASH_TYPE;		//Define the flash chip model we use 定义我们使用的flash芯片型号
extern u8 SPI_FLASH_BUF[4096];


//SPI_CS
#define CS_LOW  {DL_GPIO_clearPins(SPI_PORT,SPI_CS_PIN);}
#define CS_HIGH {DL_GPIO_setPins(SPI_PORT,SPI_CS_PIN);}//DL_GPIO_setPins DL_GPIO_clearPins



//W25X64读写
#define FLASH_ID 0XEF16

//指令表
#define W25X_WriteEnable		0x06  //Enable Write Enable 开启写使能
#define W25X_WriteDisable		0x04  //Turn off write enable 关闭写使能
#define W25X_ReadStatusReg		0x05  //Read Status Register 读状态寄存器
#define W25X_WriteStatusReg		0x01
#define W25X_ReadData			0x03  //READ 读命令
#define W25X_FastReadData		0x0B
#define W25X_FastReadDual		0x3B
#define W25X_PageProgram		0x02  //WRITE 写命令
#define W25X_BlockErase			0xD8  //Block erase 64kB 块擦除64kB
#define W25X_SectorErase		0x20  //Sector erase 4kB 扇区擦除4kB
#define W25X_ChipErase			0xC7  //Erase All 全部擦除
#define W25X_PowerDown			0xB9  //Power saving mode 省电模式
#define W25X_ReleasePowerDown	0xAB  //wake up mode 唤醒模式
#define W25X_DeviceID			0xAB
#define W25X_ManufactDeviceID	0x90  //Read vendor 读取厂商id
#define W25X_JedecDeviceID		0x9F


void W25Q64_Init(void);
uint8_t SPI_Read_Write_Byte(uint8_t BYTE);
u16  Spi_Flash_ReadID(void);  	    								//Read FLASH ID
u16 Spi_Flash_ReadID_two(void);
u8	 Spi_Flash_Read_SR(void);        								//Read Status Register
void Spi_Flash_Write_SR(u8 sr);  									//Write Status Register
void Spi_Flash_Write_Enable(void);  								//write enable
void Spi_Flash_Write_Disable(void);									//Write protected
void Spi_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);    //read flash
void Spi_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite); //write flash
void Spi_Flash_Erase_Chip(void);    	  							//Chip Erase 整片擦除
void Spi_Flash_Erase_Sector(u32 Dst_Addr);							//Sector Erase 扇区擦除
void Spi_Flash_BlockErase_Sector(u32 Dst_Addr);						//Block Erase 块擦除
void Spi_Flash_Wait_Busy(void);           							//Waiting for idle 等待空闲
void Spi_Flash_Power_Down(void);           							//Entering power down mode 进入掉电模式
void Spi_Flash_WAKEUP(void);			  							//Wakeup 唤醒

void Spi_Flash_Exit4ByteMode(void);
uint8_t SPI_Flash_ReadSR(uint8_t reg_num);
#endif
