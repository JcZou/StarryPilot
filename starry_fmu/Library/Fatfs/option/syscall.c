/*------------------------------------------------------------------------*/
/* Sample code of OS dependent controls for FatFs                         */
/* (C)ChaN, 2012                                                          */
/*------------------------------------------------------------------------*/

#include <rtthread.h>
#include <stdlib.h>		/* ANSI memory controls */
#include <stdio.h>
#include "ff.h"
#include "global.h"


//static rt_mutex_t fatfs_mutex;

#if _FS_REENTRANT
/*------------------------------------------------------------------------*/
/* Create a Synchronization Object
--------------------------------------------------------------------------*/
/* This function is called by f_mount() function to create a new
/  synchronization object, such as semaphore and mutex. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_cre_syncobj (	/* 1:Function succeeded, 0:Could not create due to any error */
	BYTE vol,			/* Corresponding logical drive being processed */
	_SYNC_t* sobj		/* Pointer to return the created sync object */
)
{
	static int mutex_id = 0;
	int ret;
	char name[10];
	sprintf(name, "fatfs%d", mutex_id);

	*sobj = rt_mutex_create (name, RT_IPC_FLAG_PRIO);
	mutex_id++;
	ret = (*sobj!=RT_NULL);

	return ret;
}



/*------------------------------------------------------------------------*/
/* Delete a Synchronization Object                                        */
/*------------------------------------------------------------------------*/
/* This function is called in f_mount() function to delete a synchronization
/  object that created with ff_cre_syncobj() function. When a 0 is
/  returned, the f_mount() function fails with FR_INT_ERR.
*/

int ff_del_syncobj (	/* 1:Function succeeded, 0:Could not delete due to any error */
	_SYNC_t sobj		/* Sync object tied to the logical drive to be deleted */
)
{
	int ret;

	rt_err_t err = rt_mutex_delete(sobj);
	ret = (err == RT_EOK);

	return ret;
}



/*------------------------------------------------------------------------*/
/* Request Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on entering file functions to lock the volume.
/  When a FALSE is returned, the file function fails with FR_TIMEOUT.
*/

int ff_req_grant (	/* TRUE:Got a grant to access the volume, FALSE:Could not get a grant */
	_SYNC_t sobj	/* Sync object to wait */
)
{
	int ret;

	rt_err_t err = rt_mutex_take(sobj, _FS_TIMEOUT);
	
	ret = (err == RT_EOK);

	return ret;
}



/*------------------------------------------------------------------------*/
/* Release Grant to Access the Volume                                     */
/*------------------------------------------------------------------------*/
/* This function is called on leaving file functions to unlock the volume.
*/

void ff_rel_grant (
	_SYNC_t sobj	/* Sync object to be signaled */
)
{
	rt_mutex_release(sobj);
}

#endif




#if _USE_LFN == 3	/* LFN with a working buffer on the heap */
/*------------------------------------------------------------------------*/
/* Allocate a memory block                                                */
/*------------------------------------------------------------------------*/
/* If a NULL is returned, the file function fails with FR_NOT_ENOUGH_CORE.
*/

void* ff_memalloc (	/* Returns pointer to the allocated memory block */
	UINT msize		/* Number of bytes to allocate */
)
{
	return rt_malloc(msize);
}


/*------------------------------------------------------------------------*/
/* Free a memory block                                                    */
/*------------------------------------------------------------------------*/

void ff_memfree (
	void* mblock	/* Pointer to the memory block to free */
)
{
	rt_free(mblock);
}

#endif
