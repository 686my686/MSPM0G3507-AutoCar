#include "bsp_flash_W25Q64.h"

#define HAL_Delay(x) delay_ms(x)

u16 SPI_FLASH_TYPE = W25Q64; // 默认就是25Q64 The default is 25Q64

// flash init
void W25Q64_Init(void)
{

	delay_ms(300);
    

	SPI_FLASH_TYPE = Spi_Flash_ReadID();
    
    
    
    while(SPI_FLASH_TYPE!=W25Q64 && SPI_FLASH_TYPE!=W25Q32 )
    {
        SPI_FLASH_TYPE = Spi_Flash_ReadID();
        delay_ms(300);
    }
}


//退出4字节模式，用3字节
void Spi_Flash_Exit4ByteMode(void)
{
    CS_LOW;
    SPI_Read_Write_Byte(0xE9); // 退出4字节地址模式命令
    CS_HIGH;
    delay_us(10);
}


uint8_t SPI_Flash_ReadSR(uint8_t reg_num)
{
    uint8_t cmd = 0x05;
    if(reg_num == 2) cmd = 0x35;
    if(reg_num == 3) cmd = 0x15; // W25Q64专用状态寄存器3
    
    uint8_t sr;
    CS_LOW;
    SPI_Read_Write_Byte(cmd);
    sr = SPI_Read_Write_Byte(0xFF);
    CS_HIGH;
    return sr;
}


/**********************
spi协议发送1个字节函数
uint8_t BYTE ：你需要发送的字节数据
SPI protocol sends 1 byte function
Uint8_ BYTE: Byte data you need to send
*********************/
uint8_t SPI_Read_Write_Byte(uint8_t BYTE)
{
	uint8_t SNED_BYTE = BYTE;
	uint8_t READ_BYTE = 0;
    
     //等待SPI总线空闲
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    
    //发送数据
    DL_SPI_transmitData8(SPI_W25Q64_INST,SNED_BYTE);
    //等待SPI总线空闲
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    //接收数据
    READ_BYTE = DL_SPI_receiveData8(SPI_W25Q64_INST);
    //等待SPI总线空闲
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    
	return READ_BYTE;
}

// Read SPI_ Status register of FLASH  读取SPI_FLASH的状态寄存器
// BIT7  6   5   4   3   2   1   0
// SPR   RV  TB BP2 BP1 BP0 WEL BUSY
// SPR:  default 0, status register protection bit, used in conjunction with WP 默认0,状态寄存器保护位,配合WP使用
// TB,BP2,BP1,BP0:FLASH region write protection settings  FLASH区域写保护设置
// WEL:Write enable lock  写使能锁定
// BUSY:busy flag bit (1, busy; 0, idle) 忙标记位(1,忙;0,空闲)
// Default:0x00
u8 Spi_Flash_Read_SR(void)
{
	u8 byte = 0;
	CS_LOW;									 // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_ReadStatusReg); // Send Read Status Register Command 发送读取状态寄存器命令
	byte = SPI_Read_Write_Byte(0Xff);		 // Read a Byte 读取一个字节
	CS_HIGH;								 // Cancel Film Selection 取消片选
	return byte;
}
// Write SPI_ FLASH status register 写SPI_FLASH状态寄存器
// Only SPR, TB, BP2, BP1, BP0 (bit 7,5,4,3,2) can be written!!! 只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void Spi_Flash_Write_SR(u8 sr)
{
	CS_LOW;									  // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_WriteStatusReg); // Send Write Status Register Command 发送写取状态寄存器命令
	SPI_Read_Write_Byte(sr);				  // Write a byte 写入一个字节
	CS_HIGH;								  // Cancel Film Selection 取消片选
}
// SPI_ FLASH write enable SPI_FLASH写使能
// Set WEL to position 将WEL置位
void Spi_Flash_Write_Enable(void)
{
	CS_LOW;								   // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_WriteEnable); // Send Write Enable 发送写使能
	CS_HIGH;							   // Cancel Film Selection 取消片选
}
// SPI_ FLASH write inhibit SPI_FLASH写禁止
// Reset WEL to zero 将WEL清零
void Spi_Flash_Write_Disable(void)
{
	CS_LOW;									// Reset WEL to zero 使能器件
	SPI_Read_Write_Byte(W25X_WriteDisable); // Send write inhibit command s发送写禁止指令
	CS_HIGH;								// Cancel Film Selection 取消片选
}

// 函数功能：读取ID
// 传入参数:无
// 返回参数：ID号(hex)
// Function function: Read ID
// Incoming parameter: None
// Return parameter: ID number (hex)
u16 Spi_Flash_ReadID(void)
{
	u16 Temp = 0;
	CS_LOW;
	SPI_Read_Write_Byte(0x90); // Send Read ID Command 发送读取ID命令
	SPI_Read_Write_Byte(0x00);
	SPI_Read_Write_Byte(0x00);
	SPI_Read_Write_Byte(0x00);
	Temp |= SPI_Read_Write_Byte(0xFF) << 8;
	Temp |= SPI_Read_Write_Byte(0xFF);
	CS_HIGH;
	return Temp;
}



u16 Spi_Flash_ReadID_two(void) {
    u16 Temp = 0;
    CS_LOW;
    SPI_Read_Write_Byte(0x9F);  // JEDEC ID 标准命令
    Temp = SPI_Read_Write_Byte(0xFF) << 8;  // 制造商ID (应0xEF)
    Temp |= SPI_Read_Write_Byte(0xFF);      // 设备ID高字节 (应0x40)
    // uint8_t capacity = SPI_Read_Write_Byte(0xFF); // 容量字节(应0x17)
    CS_HIGH;
    return Temp;
}


// 读取SPI FLASH
// 在指定地址开始读取指定长度的数据
// pBuffer:数据存储区
// ReadAddr:开始读取的地址(24bit)
// NumByteToRead:要读取的字节数(最大65535)
// Read SPI FLASH
// Starting from the specified address to read data of the specified length
// PBuffer: data storage area
// ReadAddr: Address to start reading (24bit)
// NumByteToRead: Number of bytes to read (maximum 65535)
void Spi_Flash_Read(u8 *pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
	u16 i;
	CS_LOW;										 // Enabling device
	SPI_Read_Write_Byte(W25X_ReadData);			 // Send read command
	SPI_Read_Write_Byte((u8)((ReadAddr) >> 16)); // Send a 24bit address
	SPI_Read_Write_Byte((u8)((ReadAddr) >> 8));
	SPI_Read_Write_Byte((u8)ReadAddr);
	for (i = 0; i < NumByteToRead; i++)
	{
		pBuffer[i] = SPI_Read_Write_Byte(0XFF); // Cyclic reading
	}
	CS_HIGH; // Cancel Film Selection
}

// SPI在一页(0~65535)内写入少于256个字节的数据
// 在指定地址开始写入最大256字节的数据
// pBuffer:数据存储区
// WriteAddr:开始写入的地址(24bit)
// NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!
// SPI writes less than 256 bytes of data on a page (0-65535)
// Write up to 256 bytes of data starting at the specified address
// PBuffer: data storage area
// WriteAddr: Address to start writing (24bit)
// NumByteToWrite: The number of bytes to write (maximum 256), which should not exceed the remaining bytes on the page!!!
void Spi_Flash_Write_Page(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 i;
	Spi_Flash_Write_Enable();					  // SET WEL
	CS_LOW;										  // Enabling device
	SPI_Read_Write_Byte(W25X_PageProgram);		  // Send page write command
	SPI_Read_Write_Byte((u8)((WriteAddr) >> 16)); // Send a 24bit address
	SPI_Read_Write_Byte((u8)((WriteAddr) >> 8));
	SPI_Read_Write_Byte((u8)WriteAddr);
	for (i = 0; i < NumByteToWrite; i++)
		SPI_Read_Write_Byte(pBuffer[i]); // Number of cyclic writes
	CS_HIGH;							 // Cancel Film Selection
	Spi_Flash_Wait_Busy();				 // Waiting for write to end
}

/********************************************************
// 无检验写SPI FLASH
// 必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
// 具有自动换页功能
// 在指定地址开始写入指定长度的数据,但是要确保地址不越界!
// pBuffer:数据存储区
// WriteAddr:开始写入的地址(24bit)
// NumByteToWrite:要写入的字节数(最大65535)
// CHECK OK
//Write SPI FLASH without verification
//It is necessary to ensure that all data within the address range written is 0XFF, otherwise data written at non 0XFF locations will fail!
//With automatic page wrapping function
//Start writing data of the specified length at the specified address, but ensure that the address does not exceed the limit!
//PBuffer: data storage area
//WriteAddr: Address to start writing (24bit)
//NumByteToWrite: Number of bytes to write (maximum 65535)
//CHECK OK
**********************************************************/
void Spi_Flash_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 pageremain;
	pageremain = 256 - WriteAddr % 256; // Number of bytes remaining on a single page
	if (NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite; // No more than 256 bytes 不大于256个字节
	while (1)
	{
		Spi_Flash_Write_Page(pBuffer, WriteAddr, pageremain);
		// Writing completed 写入结束了
		if (NumByteToWrite == pageremain)
			break;
		else // NumByteToWrite>pageremain
		{
			pBuffer += pageremain;
			WriteAddr += pageremain;

			NumByteToWrite -= pageremain; // Subtract the number of bytes already written 减去已经写入了的字节数
			if (NumByteToWrite > 256)
				pageremain = 256; // Can write 256 bytes at a time 一次可以写入256个字节
			else
				pageremain = NumByteToWrite; // Not enough 256 bytes left 不够256个字节了
		}
	};
}
// Write SPI FLASH 写SPI FLASH
// Write data of the specified length starting from the specified address 在指定地址开始写入指定长度的数据
// This function has an erase operation! 该函数带擦除操作!
// pBuffer: data storage area 数据存储区
// WriteAddr:Address to start writing (24bit) 开始写入的地址(24bit)
// NumByteToWrite:Number of bytes to write (maximum 65535) 要写入的字节数(最大65535)
u8 SPI_FLASH_BUF[4096];
void Spi_Flash_Write(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u32 secpos;
	u16 secoff;
	u16 secremain;
	u16 i;

	secpos = WriteAddr / 4096; // sector address
	secoff = WriteAddr % 4096; // Offset within sectors
	secremain = 4096 - secoff; // Size of remaining space in the sector

	// No more than 4096 bytes 不大于4096个字节
	if (NumByteToWrite <= secremain)
		secremain = NumByteToWrite;
	while (1)
	{
		// Read the content of the entire sector 读出整个扇区的内容
		Spi_Flash_Read(SPI_FLASH_BUF, secpos * 4096, 4096);
		for (i = 0; i < secremain; i++) // Verification data 校验数据
		{
			// Need to erase 需要擦除
			if (SPI_FLASH_BUF[secoff + i] != 0XFF)
				break;
		}
		if (i < secremain) // Need to erase 需要擦除
		{
			Spi_Flash_Erase_Sector(secpos); // Erase this sector 擦除这个扇区
			for (i = 0; i < secremain; i++) // Copy 复制
			{
				SPI_FLASH_BUF[i + secoff] = pBuffer[i];
			}
			// Write entire sector 写入整个扇区
			Spi_Flash_Write_NoCheck(SPI_FLASH_BUF, secpos * 4096, 4096);
		}
		else // Write what has been erased and directly write to the remaining interval of the sector 写已经擦除了的,直接写入扇区剩余区间.
			Spi_Flash_Write_NoCheck(pBuffer, WriteAddr, secremain);
		if (NumByteToWrite == secremain)
			break; // Writing completed 写入结束了
		else	   // Write not ended 写入未结束
		{
			secpos++;	// Increase sector address by 1 扇区地址增1
			secoff = 0; // Offset position is 0 偏移位置为0

			pBuffer += secremain;		 // Pointer offset 指针偏移
			WriteAddr += secremain;		 // Write address offset 写地址偏移
			NumByteToWrite -= secremain; // Bytes decreasing 字节数递减
			if (NumByteToWrite > 4096)
				secremain = 4096; // Unable to finish writing the next sector 下一个扇区还是写不完
			else
				secremain = NumByteToWrite; // The next sector can be completed 下一个扇区可以写完了
		}
	};
}

// 擦除整个芯片
// 整片擦除时间:W25X64:40s
// Erase the entire chip
// Whole chip erase time:W25X64:40s
void Spi_Flash_Erase_Chip(void)
{
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;								 // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_ChipErase); // Send chip erase command 发送片擦除命令
	CS_HIGH;							 // Cancel Film Selection 取消片选
	Spi_Flash_Wait_Busy();				 // Waiting for chip erase to end 等待芯片擦除结束
}
// 擦除一个扇区
// Dst_Addr:扇区地址 0~511(0-2047) for w25x16
// 擦除一个山区的最少时间:150ms
// Erase a sector
// Dst_ Addr: Sector addresses 0-511 (0-2047) for w25x16
// Minimum time to erase a mountainous area: 150ms
void Spi_Flash_Erase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 4096;
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;										 // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_SectorErase);		 // Send sector erase command 发送扇区擦除指令
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 16)); // Send a 24bit address 发送24bit地址
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 8));
	SPI_Read_Write_Byte((u8)Dst_Addr);
	CS_HIGH;			   // Cancel Film Selection 取消片选
	Spi_Flash_Wait_Busy(); // Waiting for erase to complete 等待擦除完成
}

// 擦除一个块
// Dst_Addr:块地址 0~128
// 擦除一个山区的最少时间:150ms
// Erase a block
// Dst_ Addr: Block addresses 0~128
// Minimum time to erase a mountainous area: 150ms
void Spi_Flash_BlockErase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 65536;
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;										 // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_BlockErase);		 // Send block erase instruction 发送块擦除指令
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 16)); // Send a 24bit address 发送24bit地址
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 8));
	SPI_Read_Write_Byte((u8)Dst_Addr);
	CS_HIGH;			   // Cancel Film Selection 取消片选
	Spi_Flash_Wait_Busy(); // Waiting for erase to complete 等待擦除完成
}

// Waiting for idle 等待空闲
void Spi_Flash_Wait_Busy(void)
{
	while ((Spi_Flash_Read_SR() & 0x01) == 0x01)
		; // Waiting for BUSY bit to clear 等待BUSY位清空
}
// Entering power down mode 进入掉电模式
void Spi_Flash_Power_Down(void)
{
	CS_LOW;								 // Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_PowerDown); // Send power down command 发送掉电命令
	CS_HIGH;							 // Cancel Film Selection 取消片选
	HAL_Delay(1);
}
// WAKEUP 唤醒
void Spi_Flash_WAKEUP(void)
{
	CS_LOW;										// Enabling device 使能器件
	SPI_Read_Write_Byte(W25X_ReleasePowerDown); // send W25X_PowerDown command 0xAB
	CS_HIGH;									// Cancel Film Selection 取消片选
	HAL_Delay(1);
}
