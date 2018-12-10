/*****************************************************************************
Copyright (c) 2018, StarryPilot Development Team. All rights reserved.

Author: Jiachi Zou

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of StarryPilot nor the names of its
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
*******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ftp_manager.h"
#include "console.h"

#define MAX_DIR_PATH_LEN        50

static const char   kDirentFile = 'F';  ///< Identifies File returned from List command
static const char   kDirentDir = 'D';   ///< Identifies Directory returned from List command
static const char   kDirentSkip = 'S';  ///< Identifies Skipped entry from List command

static uint8_t _errno = FR_OK;
static FIL _fp;

Session_Info _ftp_session_info;

uint8_t ftp_list(uint8_t *payload)
{
    FTP_Msg_Payload *ftp_msg_t = (FTP_Msg_Payload *)payload;
    char dir_buffer[MAX_DIR_PATH_LEN+1] = {0};
    char direntType;
    DIR dir;
    FILINFO fno;
    uint8_t offset = 0;

    if(ftp_msg_t->size > MAX_DIR_PATH_LEN){
        Console.print("path of root dir is too long:%d\n", ftp_msg_t->size);
        return kErrEOF;
    }

    strncpy(dir_buffer, ftp_msg_t->data, ftp_msg_t->size);

    if(f_opendir(&dir, dir_buffer) != FR_OK){
        return kErrEOF;
    }

    uint32_t req_offset = ftp_msg_t->offset;
    while(req_offset--){
        f_readdir(&dir, &fno);
    }

    while(1) 
    {
        if(f_readdir(&dir, &fno) != FR_OK || !fno.fname[0]){
            // end of dir
            break;
        }

        uint32_t str_len;
        if(fno.fname[0] == '.'){
            // hiden file or dir
            direntType = kDirentSkip;

            str_len = 2;
        }else{
            if(fno.fattrib & AM_DIR ) {
                direntType = kDirentDir;

                str_len = strlen(fno.fname)+2;
            }else{
                direntType = kDirentFile;

                sprintf(dir_buffer, "%s\t%d", fno.fname, fno.fsize);
                str_len = strlen(dir_buffer)+2;
            }
        }

        if(offset+str_len > MAX_FTP_DATA_LEN){
            break;
        }

        ftp_msg_t->data[offset++] = direntType;
        if(direntType == kDirentSkip){
            ftp_msg_t->data[offset] = '\0';
        }else if(direntType == kDirentFile){
            sprintf(&ftp_msg_t->data[offset], "%s\t%d", fno.fname, fno.fsize);
        }else{
            sprintf(&ftp_msg_t->data[offset], "%s", fno.fname);
        }
    
        offset += strlen(&ftp_msg_t->data[offset])+1;   // 1byte for direntType and 1byte for '\0'
    }
    ftp_msg_t->size = offset;

    f_closedir(&dir);

    return kErrNone;
}

uint8_t ftp_open(uint8_t *payload, uint8_t oflag)
{
    FTP_Msg_Payload *ftp_msg_t = (FTP_Msg_Payload *)payload;
    char file_name[MAX_DIR_PATH_LEN+1];
    FILINFO fno;
    FRESULT res;

    strncpy(file_name, ftp_msg_t->data, ftp_msg_t->size);
    file_name[ftp_msg_t->size] = '\0';

    res = f_stat(file_name, &fno);
    if(res != FR_OK && (oflag & FA_READ)){
        // fail only if read operation
        _errno = res;
        return kErrFailErrno;
    }

    res = f_open(&_fp, file_name, oflag);
    if(res != FR_OK){
        _errno = res;
        return kErrFailErrno;
    }

    ftp_msg_t->session = 0;
    ftp_msg_t->size = sizeof(uint32_t);
    memcpy(ftp_msg_t->data, &fno.fsize, ftp_msg_t->size);

    return kErrNone;
}

uint8_t ftp_read(uint8_t *payload, uint8_t target_system)
{
    /* since we are in a seperate thread, just read data here */
    FTP_Msg_Payload *ftp_msg_t = (FTP_Msg_Payload *)payload;
    FRESULT fres;

    fres = f_lseek(&_fp, ftp_msg_t->offset);
    if(fres != FR_OK){
        _errno = fres;
        return kErrFailErrno;
    }

    UINT br;
    fres = f_read(&_fp, ftp_msg_t->data, MAX_FTP_DATA_LEN, &br);
    if(fres != FR_OK){
        _errno = fres;
        return kErrFailErrno;
    }

    ftp_msg_t->size = br;
    if(f_eof(&_fp) && br == 0){
        return kErrEOF;
    }

    return kErrNone;
}

uint8_t ftp_close(uint8_t *payload)
{
    FTP_Msg_Payload *ftp_msg_t = (FTP_Msg_Payload *)payload;

    FRESULT fres = f_close(&_fp);
    if(fres != FR_OK){
        _errno = fres;
        return kErrFailErrno;
    }

    ftp_msg_t->size = 0;
    return kErrNone;
}

void ftp_msg_parse(uint8_t *payload, uint8_t target_system)
{
    FTP_Msg_Payload *ftp_msg_t = (FTP_Msg_Payload *)payload;
    uint8_t err_code;

    // Console.print("seq:%d sess:%d op:%d size:%d req_op:%d burst_comp:%d pad:%d offset:%d\n", ftp_msg_t->seq_number,
    //     ftp_msg_t->session, ftp_msg_t->opcode, ftp_msg_t->size, ftp_msg_t->req_opcode, ftp_msg_t->burst_complete, 
    //     ftp_msg_t->padding, ftp_msg_t->offset);
    
    switch(ftp_msg_t->opcode)
    {
        case kCmdTerminateSession:
        case kCmdResetSessions:
        {
            err_code = ftp_close(payload);
        }break;
        case kCmdListDirectory:
        {
            err_code = ftp_list(payload);
        }break;
        case kCmdOpenFileRO:
        {
            err_code = ftp_open(payload, FA_READ);
        }break;
        case kCmdReadFile:
        case kCmdBurstReadFile:
        {
            err_code = ftp_read(payload, target_system);
        }break;
        default:
        {
            Console.print("ftp unknow opcode:%d\n", ftp_msg_t->opcode);
            return ;
        }
    }

    ftp_msg_t->seq_number++;
    if(err_code == kErrNone){
        ftp_msg_t->req_opcode = ftp_msg_t->opcode;
        ftp_msg_t->opcode = kRspAck;
    }else{
        ftp_msg_t->req_opcode = ftp_msg_t->opcode;
        ftp_msg_t->opcode = kRspNak;
        ftp_msg_t->size = 1;

		ftp_msg_t->data[0] = err_code;

        if (err_code == kErrFailErrno) {
			ftp_msg_t->size = 2;
			ftp_msg_t->data[1] = _errno;
		}
    }
}
