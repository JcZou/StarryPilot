
#include <stdio.h>
#include <stdint.h>
#include "ff.h"

////FS
#define SEEK_SET    0  /* From the start of the file */
#define SEEK_CUR    1  /* From the current file offset */
#define SEEK_END    2  /* From the end of the file */

/* open flag settings for open() (and related APIs) */
#define O_RDONLY    FA_READ        /* Open for read access (only) */
#define O_RDOK      O_RDONLY        /* Read access is permitted (non-standard) */
#define O_WRONLY    FA_WRITE        /* Open for write access (only) */
#define O_WROK      O_WRONLY        /* Write access is permitted (non-standard) */
#define O_RDWR      (O_RDOK|O_WROK) /* Open for both read & write access */
#define O_CREAT     FA_OPEN_ALWAYS        /* Create file/sem/mq object */
#define O_EXCL      FA_OPEN_EXISTING        /* Name must not exist when opened  */
#define O_BINARY    0        /* Open the file in binary (untranslated) mode. */




int open(const char *path, int oflags);

int close(int fd);

int32_t lseek(int fd, int32_t offset, int whence);

int32_t write(int fd, const void *buf, uint32_t nbytes);

int32_t read(int fd, void *buf, uint32_t nbytes);

int fsync(int fd);

int unlink(const char *pathname);

