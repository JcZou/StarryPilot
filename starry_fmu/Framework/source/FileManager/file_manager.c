/**
  ******************************************************************************
  * @file    file_manager.c 
  * @author  J Zou
  * @version V1.0
  * @date    29-Nov-2017
  * @brief   File Manager
  ******************************************************************************
*/  

#include <string.h>
#include "file_manager.h"
#include "console.h"
#include "shell.h"
#include "yxml.h"

#define MAX_CAT_MALLOC_SIZE		128

static char* TAG  = "File Manager";

static FATFS _fs;
static TCHAR Buffer[MAXPATH];
static uint8_t _fmInit = 0;
 
FRESULT f_deldir(TCHAR *path)  
{  
    FRESULT res;  
    DIR   dir;   
    FILINFO fno; 
#if _USE_LFN  
    TCHAR lname[_MAX_LFN + 2] = {0};  
#endif  
      
#if _USE_LFN  
    fno.lfsize = _MAX_LFN;  
    fno.lfname = lname;     
#endif    
    res = f_opendir(&dir, path); 
      
    while((res == FR_OK) && (FR_OK == f_readdir(&dir, &fno)))  
    {   
        if(0 == strlen(fno.fname))          break;       
        if(0 == strcmp(fno.fname, "."))     continue;   
        if(0 == strcmp(fno.fname, ".."))    continue;    
 
#if _USE_LFN  
        sprintf((char*)file, "%s/%s", path, (*fno.lfname) ? fno.lfname : fno.fname);  
#else    
		strcat(path, "/");
		strcat(path, fno.fname);
#endif  
        if (fno.fattrib & AM_DIR)  
        { 
            res = f_deldir(path);  
        }  
        else  
        {
            res = f_unlink(path);  
        }  
		
		for(int i = strlen(path)-1 ; i >= 0 ; i--){
			if(path[i] == '/'){
				path[i] = '\0';	
				break;
			}
			path[i] = 0;
		}
    }  
	
    if(res == FR_OK){
		res = f_unlink(path); 
	} 
	
	f_closedir(&dir);
      
    return res;  
} 

//===================================================================================

int fm_init(const TCHAR* path)
{
	FRESULT f_res = f_mount(&_fs, "0:", 0);  
	if(f_res == FR_OK){
		_fmInit = 1;
		//Console.print("File Manager Init Success\n");
		return 0;
	}else{
		_fmInit = 0;
		//Console.print("File Manager Init Fail\n");
		return 1;
	}
}

uint8_t fm_init_complete(void)
{
	return _fmInit;
}

TCHAR* fm_get_cwd(void)
{
	FRESULT res;
	
	res = f_getcwd(Buffer, MAXPATH);
	
	if(res == FR_OK){
		return Buffer;
	}else{
		//Console.e(TAG, "fm get current working directory err\n");
		return NULL;
	}
}

int fm_cmd_cd(const TCHAR* path)
{
	FRESULT res = f_chdir(path);
	
	if(res == FR_OK)
		return 0;
	else
		return 1;
}

void fm_cmd_ls(int mode)
{
	DIR dir;
	FILINFO fno;
	
	if(mode == 0){	// normal
		if(f_opendir(&dir,".") == FR_OK) {
			while(f_readdir(&dir, &fno) == FR_OK) {
				if(!fno.fname[0]) break;
				if(fno.fattrib & AM_DIR ) {
					Console.print("%s/  ", fno.fname);
				}else{
					Console.print("%s  ", fno.fname);
				}
			}
			Console.print("\n");
		}
	}
	
	if(mode == 1){	// -l
		Console.print("%-20s %-6s %s\n", "filename", "size", "attribute");
		if(f_opendir(&dir,".") == FR_OK) {
			while(f_readdir(&dir, &fno) == FR_OK) {
				if(!fno.fname[0]) break;
				if(fno.fattrib & AM_DIR ) {
					Console.print("%-20s %-6lu 0x%x\n", strcat(fno.fname,"/"), fno.fsize, fno.fattrib);
				}else{
					Console.print("%-20s %-6lu 0x%x\n", fno.fname, fno.fsize, fno.fattrib);
				}
			}
		}
	}
}

int fm_mkdir(const TCHAR* DirName)
{
	FRESULT res = f_mkdir(DirName);
	
	if(res == FR_OK)
		return 0;
	else
		return 1;
}

int fm_mkfs(void)
{
	FRESULT res = f_mkfs("0:", 0, 0);
	
	if(res == FR_OK){
		FRESULT res = f_mount(&_fs, "0:", 0); 
		
		if(res == FR_OK){
			/* make default folders */
			fm_mkdir("sys");
			fm_mkdir("user");
			return 0;
		}else
			return 1;
	}else{
		return 1;
	}
}

int fm_getfree(void)
{
	FRESULT res;
	FATFS *fs;
	DWORD fre_clust, fre_sect, tot_sect;

	/* Get volume information and free clusters of drive 1 */
	res = f_getfree("0:", &fre_clust, &fs);
	if (res) return 1;

	/* Get total sectors and free sectors */
	tot_sect = (fs->n_fatent - 2) * fs->csize;
	fre_sect = fre_clust * fs->csize;

	/* Print the free space (assuming 512 bytes/sector) */
	Console.print("%10lu KiB total drive space.\n%10lu KiB available.\n",
		   tot_sect/2, fre_sect/2);
	
	return 0;
}

int fm_cat(const TCHAR* FileName)
{
	FIL fp;
	
	FRESULT res = f_open(&fp, FileName, FA_OPEN_EXISTING | FA_READ);
	
	if(res != FR_OK){
		Console.print("file open failed:%d\n", res);
		return 1;
	}
	
	char* f_buff;
	UINT br;
	uint32_t malloc_size = fp.fsize > MAX_CAT_MALLOC_SIZE ? MAX_CAT_MALLOC_SIZE : fp.fsize;
	f_buff = (char*)rt_malloc(malloc_size);
	if(f_buff == NULL){
		Console.print("malloc fail\n");
		goto error;
	}
	if(malloc_size < MAX_CAT_MALLOC_SIZE){
		res = f_read(&fp, f_buff, fp.fsize, &br);
	
		if(br != fp.fsize){
			Console.print("read fail err, size:%d br:%d\n", fp.fsize, br);
			goto error;
		}
		Console.write(f_buff, br);
		Console.print("\n");
	}else{
		while(!f_eof(&fp)){
			res = f_read(&fp, f_buff, malloc_size, &br);
			Console.write(f_buff, br);
		}
		Console.print("\n");
	}
	
	rt_free(f_buff);
	f_close(&fp);
	return 0;
	
error:
	rt_free(f_buff);
	f_close(&fp);
	return 1;
}

int fm_mv(const TCHAR* OldName, const TCHAR* NewName)
{
	FRESULT res = f_rename(OldName, NewName);
	
	if(res != FR_OK){
		Console.print("mv fail:%d\n", res);
	}
	
	return (res==FR_OK) ? 0 : 1;
}

int handle_fm_shell_cmd(int argc, char** argv)
{
	if(strcmp(argv[0], "ls") == 0){
		int mode = 0;
		for(int i = 1 ; i < argc ; i++){
			if(strcmp(argv[i], "-l") == 0){
				mode = 1;
				break;
			}
		}
		fm_cmd_ls(mode);
		return 0;
	}else if(strcmp(argv[0], "cd") == 0){
		if(argc > 1){
			int res = fm_cmd_cd(argv[1]);
			if(res){
				Console.print("can not find directory:%s\n", argv[1]);
				return 1;
			}
		}
		return 0;
	}else if(strcmp(argv[0], "mkdir") == 0){
		if(argc == 2){
			int res = fm_mkdir(argv[1]);
			if(res){
				Console.print("mkdir %s fail\n", argv[1]);
				return 1;
			}
			return 0;
		}else{
			return 1;
		}
	}else if(strcmp(argv[0], "mv") == 0){
		if(argc == 3){
			return fm_mv(argv[1], argv[2]);
		}else{
			return 1;
		}
	}else if(strcmp(argv[0], "rm") == 0){
		if(argc > 1){
			FILINFO finfo;
			FRESULT fres = f_stat(argv[1], &finfo);
			if(finfo.fattrib & AM_DIR){
				int res = f_deldir(argv[1]);
				if(res){
					Console.print("rm %s fail %d\n", argv[1], res);
					return 1;
				}
			}else{
				fres = f_unlink(argv[1]);
				if(fres){
					Console.print("rm %s fail %d\n", argv[1], fres);
					return 1;
				};
			}
			return 0;
		}else{
			return 1;
		}
	}else if(strcmp(argv[0], "mkfs") == 0){
		Console.print("Are you sure to make file system? That will erase all data storage. (Y/N)\n");
		char ch = shell_wait_ch();
		if(ch == 'Y' || ch == 'y'){
			Console.print("mkfs...\n");
			fm_mkfs();
			Console.print("mkfs finish\n");
		}
		return 0;
	}else if(strcmp(argv[0], "getfree") == 0){
		return fm_getfree();
	}else if(strcmp(argv[0], "cat") == 0){
		if(argc > 1){
			fm_cat(argv[1]);
			return 0;
		}else{
			return 1;
		}
	}else{
		Console.print("unknow command\n");
		return 1;
	}
}

