/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
#include "sdio.h"		/* Example: MMC/SDC contorl */

#include "stdio.h"

#define SECTOR_SIZE		512

/* Definitions of physical drive number for each drive */
#define MMC		0	/* Example: Map MMC/SD card to drive number 0 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;

	switch (pdrv) {

	case MMC :
		
		stat = RES_OK;
		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;

	switch (pdrv) {
		
	case MMC :
		if(SD_Init() == SD_OK){
			stat = RES_OK;
		}
		else{
			stat = STA_NOINIT;
		}

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}

//#include <string.h>
//static uint32_t dma_buffer[SECTOR_SIZE/sizeof(uint32_t)];
extern SD_Error SD_ReadDisk(uint8_t *readbuff, uint32_t sector, uint32_t count);
extern SD_Error SD_WriteDisk(const uint8_t *writebuff, uint32_t sector, uint32_t count);
/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;

	switch (pdrv) {

	case MMC :
		{
			SD_Error Status = SD_OK;
			Status = SD_ReadDisk(buff, sector, count);
			if(Status != SD_OK)
				return RES_ERROR;
			
			res = RES_OK;
			return res;
		}
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;

	switch (pdrv) {
	case MMC :
		{
			SD_Error Status = SD_OK;
			Status = SD_WriteDisk(buff, sector, count);
			if(Status != SD_OK)
				return RES_ERROR;
			
			res = RES_OK;
			return res;
		}
	}

	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
extern SD_CardInfo SDCardInfo;
#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;

	switch (pdrv) 
	{
		case MMC :
		{
			switch(cmd)
			{
				case CTRL_SYNC:
					if(SD_GetStatus() == SD_TRANSFER_OK)
					{
						res = RES_OK;
					}
					else
					{
						res = RES_ERROR;
					}
					break;

				case GET_SECTOR_COUNT:     //读卡容量
				{
					if (SD_GetCardInfo(&SDCardInfo)==SD_OK)//读sd卡信息
					{
						if (SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD){
							 *(DWORD*)buff = (SDCardInfo.SD_csd.DeviceSize + 1)  * 1024;
						}
						else{
							*(DWORD*)buff = SDCardInfo.CardCapacity/SDCardInfo.CardBlockSize;
						}
						res = RES_OK;
					}
					else
					{
						res = RES_ERROR ;
					}
					break;
				}
				
				case GET_SECTOR_SIZE:
					*(WORD*)buff = SECTOR_SIZE;
					res = RES_OK;
					break;
				
				case GET_BLOCK_SIZE:
				{
					if (SD_GetCardInfo(&SDCardInfo)==SD_OK)//读sd卡信息
					{
						*(WORD*)buff = SDCardInfo.CardBlockSize;
						res = RES_OK;
					}
					else
					{
						res = RES_ERROR ;
					}	
					
					break;				
				}

				case CTRL_ERASE_SECTOR:
					res = RES_ERROR;
					break;
				default:
					res = RES_PARERR;
			}
			
			return res;
		}
	}

	return RES_PARERR;
}
#endif

/*
*********************************************************************************************************
*	函 数 名: get_fattime
*	功能说明: 获得系统时间，用于改写文件的创建和修改时间。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
DWORD get_fattime (void)
{
	/* 如果有全局时钟，可按下面的格式进行时钟转换. 这个例子是2013-01-01 00:00:00 */

	return	  ((DWORD)(2014 - 1980) << 25)	/* Year = 2014 */
			| ((DWORD)11 << 21)				/* Month = 11 */
			| ((DWORD)5 << 16)				/* Day_m = 5*/
			| ((DWORD)0 << 11)				/* Hour = 0 */
			| ((DWORD)0 << 5)				/* Min = 0 */
			| ((DWORD)0 >> 1);				/* Sec = 0 */
}
