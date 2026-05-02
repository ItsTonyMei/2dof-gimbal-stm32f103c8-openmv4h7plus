#include "stmflash.h"
#include "delay.h"

#define FLASH_SAVE_ADDR  0X0800E000  // 内部FLASH保存地址(必须为偶数地址，且要大于应用程序所占用FLASH的大小+0X08000000)

// 解锁STM32的FLASH
void STMFLASH_Unlock(void)
{
  FLASH->KEYR = FLASH_KEY1;  // 写入解锁序列
  FLASH->KEYR = FLASH_KEY2;
}

// flash上锁
void STMFLASH_Lock(void)
{
  FLASH->CR |= 1<<7;  // 上锁
}

// 读取FLASH状态
u8 STMFLASH_GetStatus(void)
{
	u32 res;
	res = FLASH->SR;
	if(res & (1<<0))    return 1;      // 忙
	else if(res & (1<<2))  return 2;  // 操作超时
	else if(res & (1<<4))  return 3;  // 写保护错误
	return 0;                            // 操作成功
}

// 等待操作完成
// time: 等待超时时间
// 返回值: 状态
u8 STMFLASH_WaitDone(u16 time)
{
	u8 res;
	do
	{
		res = STMFLASH_GetStatus();
		if(res != 1) break;  // 若忙,退出等待
		delay_us(1);
		time--;
	} while(time);
	if(time == 0) res = 0xff;  // 超时
	return res;
}

// 擦除指定页
// paddr: 页地址
// 返回值: 执行结果
u8 STMFLASH_ErasePage(u32 paddr)
{
	u8 res = 0;
	res = STMFLASH_WaitDone(0X5FFF);  // 等待上次操作完成，>20ms
	if(res == 0)
	{
		FLASH->CR |= 1<<1;      // 页擦除置位
		FLASH->AR = paddr;       // 设置页地址
		FLASH->CR |= 1<<6;       // 开始擦除
		res = STMFLASH_WaitDone(0X5FFF);  // 等待擦除完成，>20ms
		if(res != 1)  // 擦除失败
		{
			FLASH->CR &= ~(1<<1);  // 清除页擦除标志
		}
	}
	return res;
}

// 向FLASH指定地址写入半字(16位数据)
// faddr: 指定地址(此地址必须为2的倍数!!)
// dat: 要写入的数据
// 返回值: 写入结果
u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
{
	u8 res;
	res = STMFLASH_WaitDone(0XFF);
	if(res == 0)  // OK
	{
		FLASH->CR |= 1<<0;        // 半字编程使能
		*(vu16*)faddr = dat;      // 写入数据
		res = STMFLASH_WaitDone(0XFF);  // 等待写入完成
		if(res != 1)  // 写入失败
		{
			FLASH->CR &= ~(1<<0);  // 清除PG位
		}
	}
	return res;
}

// 读取指定地址的半字(16位数据)
// faddr: 指定地址
// 返回值: 对应数据
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr;
}

#if STM32_FLASH_WREN  // 允许FLASH读写
// 写入半字数据(不检查)
// WriteAddr: 起始地址
// pBuffer: 数据指针
// NumToWrite: 半字数目
void STMFLASH_Write_NoCheck(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
	u16 i;
	for(i=0; i<NumToWrite; i++)
	{
		STMFLASH_WriteHalfWord(WriteAddr, pBuffer[i]);
		WriteAddr += 2;  // 地址增加2
	}
}

// 从指定地址开始写入指定长度的数据(自动擦除)
// WriteAddr: 起始地址(此地址必须为2的倍数!!)
// pBuffer: 数据指针
// NumToWrite: 半字数目(需要写入的16位数据的个数)
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024  // 字节
#else
#define STM_SECTOR_SIZE 2048
#endif
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];  // 最多2K字节

void STMFLASH_Write(u32 WriteAddr, u16 *pBuffer, u16 NumToWrite)
{
	u32 secpos;       // 扇区地址
	u16 secoff;       // 扇区内的偏移地址(以半字为单位)
	u16 secremain;     // 扇区剩余空间(以半字为单位)
	u16 i;
	u32 offaddr;      // 去掉0X08000000后的地址

	if(WriteAddr < STM32_FLASH_BASE || (WriteAddr >= (STM32_FLASH_BASE + 1024 * STM32_FLASH_SIZE))) return;  // 地址非法
	STMFLASH_Unlock();           // 解锁
	offaddr = WriteAddr - STM32_FLASH_BASE;        // 实际偏移地址
	secpos = offaddr / STM_SECTOR_SIZE;            // 扇区地址 0~127 for STM32F103RBT6
	secoff = (offaddr % STM_SECTOR_SIZE) / 2;     // 扇区内的偏移(以半字为单位)
	secremain = STM_SECTOR_SIZE/2 - secoff;        // 扇区剩余空间
	if(NumToWrite <= secremain) secremain = NumToWrite;  // 不超过扇区剩余空间

	while(1)
	{
		STMFLASH_Read(secpos*STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE/2);  // 读取整个扇区
		for(i=0; i<secremain; i++)  // 校验数据
		{
			if(STMFLASH_BUF[secoff+i] != 0XFFFF) break;  // 需要擦除
		}
		if(i < secremain)  // 需要擦除
		{
			STMFLASH_ErasePage(secpos*STM_SECTOR_SIZE + STM32_FLASH_BASE);  // 擦除扇区
			for(i=0; i<secremain; i++)  // 写入
			{
				STMFLASH_BUF[i+secoff] = pBuffer[i];
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE + STM32_FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE/2);  // 写入扇区缓存
		} else {
			STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain);  // 写入剩余数据
		}
		if(NumToWrite == secremain) break;  // 写入结束
		else  // 写入未完成
		{
			secpos++;              // 扇区地址增1
			secoff = 0;           // 偏移位置为0
			pBuffer += secremain;  // 指针偏移
			WriteAddr += secremain*2;  // 写入地址偏移(16位数据地址,需要*2)
			NumToWrite -= secremain;    // 写入字节数
			if(NumToWrite > (STM_SECTOR_SIZE/2)) secremain = STM_SECTOR_SIZE/2;  // 下一个扇区还是写不完
			else secremain = NumToWrite;  // 下一个扇区可以写完
		}
	}
	STMFLASH_Lock();  // 上锁
}
#endif

// 从指定地址开始读取指定长度的数据
// ReadAddr: 起始地址
// pBuffer: 数据指针
// NumToWrite: 半字数目
void STMFLASH_Read(u32 ReadAddr, u16 *pBuffer, u16 NumToRead)
{
	u16 i;
	for(i=0; i<NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr);  // 读取2个字节
		ReadAddr += 2;  // 偏移2个字节
	}
}

//////////////////////////////////////////测试函数///////////////////////////////////////////
// WriteAddr: 起始地址
// WriteData: 要写入的数据
void Test_Write(u32 WriteAddr, u16 WriteData)
{
	STMFLASH_Write(WriteAddr, &WriteData, 1);  // 写入一个数据
}

/**************************************************************************
 * 函数功能: 从Flash读取指定参数
 * 入口参数: 无
 * 返 回 值: 无
 **************************************************************************/
void Flash_Read(void)
{
	STMFLASH_Read(FLASH_SAVE_ADDR, (u16*)PID_Parameter, 10);
	if(PID_Parameter[0]==65535 && PID_Parameter[1]==65535 && PID_Parameter[2]==65535 && PID_Parameter[3]==65535)
	{
		Balance_Kp = 350;
		Balance_Kd = 0;
		Velocity_Kp = 70;
		Velocity_Ki = 0.7;
	}
	else
	{
		Balance_Kp = (float)PID_Parameter[0]/100;
		Balance_Kd = (float)PID_Parameter[1]/100;
		Velocity_Kp = (float)PID_Parameter[2]/100;
		Velocity_Ki = (float)PID_Parameter[3]/100;
	}
}

/**************************************************************************
 * 函数功能: 向Flash写入指定参数
 * 入口参数: 无
 * 返 回 值: 无
 **************************************************************************/
void Flash_Write(void)
{
	Flash_Parameter[0] = Balance_Kp*100;  // 扩大100倍存储，保持参数精度
	Flash_Parameter[1] = Balance_Kd*100;
	Flash_Parameter[2] = Velocity_Kp*100;
	Flash_Parameter[3] = Velocity_Ki*100;
	STMFLASH_Write(FLASH_SAVE_ADDR, (u16*)Flash_Parameter, 10);
}
