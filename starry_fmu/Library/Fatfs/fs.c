/*
对 FATFS 文件系统进行了 POSIX 标准封装

使用了动态内存管理

封装中 open 函数的打开方式可能需要改进

by AnKong          2015/01/09
*/

#include <stdlib.h>
#include "fs.h"

int open(const char *path, int oflags)
{
	FRESULT res=FR_INT_ERR;
	int fd;
	
	FIL *fp;
	fp = (FIL *)rt_malloc(sizeof (FIL));
	
	res = f_open(fp, path, oflags);
	if(res == FR_OK)
	{
		fd = (uint32_t)fp;
		
		return fd;
	}
	else
	{
		rt_free(fp);
		
		return -1;
	}
}

int close(int fd)
{
	FRESULT res=FR_INT_ERR;
	FIL *fp;
	fp = (FIL *)fd;

	res = f_close(fp);
	
	rt_free(fp);
	
	
	if(FR_OK == res)
		return 0;
	else
		return -1;
}

int32_t lseek(int fd, int32_t offset, int whence)
{
	FRESULT res=FR_INT_ERR;
	FIL *fp;
	fp = (FIL *)fd;
	
	if(whence == SEEK_SET)
	{
		res = f_lseek(fp, offset);
	}
	else if(whence == SEEK_CUR)
	{
		offset += f_tell(fp);
		res = f_lseek(fp, offset);
	}
	else if(whence == SEEK_END)
	{
		offset += f_size(fp);
		res = f_lseek(fp, offset);
	}
	
	if(res != FR_OK)
		return -1;
	else
		return offset;
}

int32_t write(int fd, const void *buf, uint32_t nbytes)
{
	FRESULT res=FR_INT_ERR;
	uint32_t bw;
	
	FIL *fp;
	fp = (FIL *)fd;	
	
	res = f_write(fp, buf, nbytes, &bw);
	
	if(res == FR_OK)
		return bw;
	else
		return -1;
}

int32_t read(int fd, void *buf, uint32_t nbytes)
{
	FRESULT res=FR_INT_ERR;
	uint32_t br;
	
	FIL *fp;
	fp = (FIL *)fd;	

	res = f_read(fp, buf, nbytes, &br);
	
	if(res == FR_OK)
		return br;
	else
		return -1;
}

int fsync(int fd)
{
	FRESULT res=FR_INT_ERR;
	FIL *fp;
	fp = (FIL *)fd;	

	res = f_sync(fp);
	
	if(res == FR_OK)
		return 0;
	else
		return -1;
}

int unlink(const char *pathname)
{
	if(FR_OK != f_unlink(pathname))
		return -1;
	else
		return 0;
}
