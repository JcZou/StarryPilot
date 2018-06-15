/**
  ******************************************************************************
  * @file    file_manager.h 
  * @author  J Zou
  * @version V1.0
  * @date    29-Nov-2017
  * @brief   File Manager
  ******************************************************************************
*/  

#include "global.h"
#include "ff.h"

#define MAXPATH			256		

int fm_init(const TCHAR* path);
uint8_t fm_init_complete(void);
TCHAR* fm_get_cwd(void);
