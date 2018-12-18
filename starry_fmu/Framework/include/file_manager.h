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

typedef enum
{
  fm_init_none = 0,
  fm_init_ok,
  fm_init_err
}file_manager_status;

file_manager_status filemanager_init(const TCHAR* path, uint8_t opt);
file_manager_status filemanager_status(void);
TCHAR* filemanager_get_cwd(void);
