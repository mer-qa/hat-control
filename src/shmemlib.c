/*
 * This file is part of HAT tester
 *
 * Copyright (C) 2010 Optofidelity and/or its subsidiary(-ies).
 *
 * Contact: Marko Junttila <marko.junttila@optofidelity.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 2.1 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */


/* ------------------------------------------------------------------------- */
/* INCLUDE FILES */
#include <stdio.h>
#include <sys/ipc.h> 
#include <sys/shm.h>
#include <sys/stat.h>
#include "shmemlib.h"
#include <errno.h>
#include <string.h>

/* ------------------------------------------------------------------------- */
/* EXTERNAL FUNCTION PROTOTYPES */
/* None */
	
/* ------------------------------------------------------------------------- */
/* GLOBAL VARIABLES */
/* None */
	
/* ------------------------------------------------------------------------- */
/* CONSTANTS */
/* None */
	
	
/* ------------------------------------------------------------------------- */
/* MACROS */
/* None */
/* ------------------------------------------------------------------------- */
#define APP_NAME "shmemlib"
#define PRINTOUT(...) printf(APP_NAME ": " __VA_ARGS__)
#define PRINTERR(...) fprintf(stderr, APP_NAME ": " __VA_ARGS__)

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */
void init_buf(struct rbuf *rbuf)
{
    rbuf->start = 0;
    rbuf->end = 0;
}

int get_buf_size(struct rbuf *buf)
{
    if (buf->end >= buf->start) {
        return (buf->end - buf->start);
    }
    else {
        return BUF_SIZE - (buf->start - buf->end);
    }
}

int add_to_buf(struct rbuf *buf, unsigned int value)
{
    if (get_buf_size(buf) >= BUF_SIZE-1) {
        return -1;
    }
    buf->buf[buf->end++] = value;
    if (buf->end >= BUF_SIZE) {
        buf->end = 0;
    }
    return 0;
}

int get_from_buf(struct rbuf *buf, unsigned int *value)
{
    if (get_buf_size(buf) == 0) {
        return -1;
    }
    *value = buf->buf[buf->start++];
    if (buf->start >= BUF_SIZE) {
        buf->start = 0;
    }
    return 0;
}


int allocSharedMem(struct shmem **shmem, int *segment_id)
{
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  errno = 0;
  /* Allocate a shared memory segment.  */ 
  *segment_id = shmget (SH_MEM_ID, shared_segment_size, IPC_CREAT /*| IPC_EXCL*/ | S_IRUSR | S_IWUSR); 
  if (segment_id < 0) {
      PRINTERR("Error to allocate shared mem (err:%d)\n",errno);
      return -1;
  }
  /* Attach the shared memory segment.  */ 
  *shmem = (struct shmem*) shmat (*segment_id, 0, 0); 
  //printf ("shared memory attached at address %p\n", *shmem); 
  /* Determine the segment's size. */ 
  shmctl (*segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  //printf ("segment size: %d\n", segment_size); 
  memset(*shmem,0,sizeof(struct shmem));

  return 0; 
}

int deAllocAndFreeSharedMem(struct shmem *shmem, int seg_id)
{
 /* Deallocate the shared memory segment.  */ 
  shmdt (shmem);
  return shmctl (seg_id, IPC_RMID, 0); 
}

int getShareMem(struct shmem **shmem, int *segment_id)
{
    *segment_id = shmget (SH_MEM_ID, SH_MEM_SEG_SIZE, 
                     IPC_EXCL | S_IRUSR | S_IWUSR); 

    if (*segment_id < 0) {
        PRINTERR("Error to get shared memory ID\n");
        return -1;
    }
    /* Attach the shared memory segment.  */
    if ( (*shmem = (struct shmem*) shmat (*segment_id, 0, 0)) < 0) {
        PRINTERR("Error to allocate shared memory\n");
        return -1; 
    }
    //printf("shared memory attached at address %p\n", shmem); 
    return 0;
}

int deAllocSharedMem(struct shmem *shmem)
{
 /* Deallocate the shared memory segment.  */ 
  return shmdt (shmem);
}

/* ------------------------------------------------------------------------- */
/* End of file */
