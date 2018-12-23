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

#ifndef __FTP_MANAGER_H__
#define __FTP_MANAGER_H__

#include "global.h"
#include "ff.h"

#define MAX_FTP_DATA_LEN        239

__PACKED__(
typedef struct {
	uint16_t	seq_number;	///< sequence number for message
	uint8_t		session;	///< Session id for read and write commands
	uint8_t		opcode;		///< Command opcode
	uint8_t		size;		///< Size of data
	uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
	uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
	uint8_t		padding;        ///< 32 bit aligment padding
	uint32_t	offset;		///< Offsets for List and Read commands
	uint8_t		data[];		///< command data, varies by Opcode
})FTP_Msg_Payload;

enum Opcode {
	kCmdNone = 0,		    ///< ignored, always acked
	kCmdTerminateSession,	///< Terminates open Read session
	kCmdResetSessions,      ///< Terminates all open Read sessions
	kCmdListDirectory,	    ///< List files in <path> from <offset>
	kCmdOpenFileRO,		    ///< Opens file at <path> for reading, returns <session>
	kCmdReadFile,		    ///< Reads <size> bytes from <offset> in <session>
	kCmdCreateFile,		    ///< Creates file at <path> for writing, returns <session>
	kCmdWriteFile,		    ///< Writes <size> bytes to <offset> in <session>
	kCmdRemoveFile,		    ///< Remove file at <path>
	kCmdCreateDirectory,    ///< Creates directory at <path>
	kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
	kCmdOpenFileWO,		    ///< Opens file at <path> for writing, returns <session>
	kCmdTruncateFile,	    ///< Truncate file at <path> to <offset> length
	kCmdRename,		        ///< Rename <path1> to <path2>
	kCmdCalcFileCRC32,	    ///< Calculate CRC32 for file at <path>
	kCmdBurstReadFile,	    ///< Burst download session file

	kRspAck = 128,		    ///< Ack response
	kRspNak			        ///< Nak response
};

enum ErrorCode {
	kErrNone,
	kErrFail,			///< Unknown failure
	kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
	kErrInvalidDataSize,		///< PayloadHeader.size is invalid
	kErrInvalidSession,		///< Session is not currently open
	kErrNoSessionsAvailable,	///< All available Sessions in use
	kErrEOF,			///< Offset past end of file for List and Read commands
	kErrUnknownCommand,		///< Unknown command opcode
	kErrFailFileExists,		///< File exists already
	kErrFailFileProtected		///< File is write protected
};

void ftp_msg_parse(uint8_t* payload, uint8_t target_system);

#endif