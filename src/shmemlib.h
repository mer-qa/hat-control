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
 * This library is distributed in the hope that it will be useful, but
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
#ifndef SHMEMLIBH
#define SHMEMLIBH

#include <sys/types.h>

// Shared memory properties
#define SH_MEM_ID   0xaa
#define SH_MEM_SEG_SIZE 0x6400
#define SHMEMNAME "HATSHMEM"

// Data buffer size for each channel in shared memory
#define BUF_SIZE 1000

// Sensor types
#define NONE            0
#define VOLTAGE         1
#define CURRENT         2
#define LIGHT           3
#define ACCELERATION    4
#define TEMPERATURE     5

// Digital IO lines
#define AN0_IO          8
#define AN1_IO          9
#define AN2_IO          10
#define AN3_IO          11 
#define USB_DATA_1      12
#define USB_DATA_2      13
#define USB_PWR_1       14
#define USB_PWR_2       15
#define DC_POWER_SHDN1  16
#define DC_POWER_SHDN2  17

struct rbuf {
    unsigned int buf[BUF_SIZE];
    unsigned int start;
    unsigned int end;
    int overfull;
    int io_state;
};

struct ioctrl {
    unsigned int iostate;
    unsigned int changeIO;
};

struct sensor {
    unsigned int type;
    unsigned int offset_mv;
    unsigned int mult_mv;
};
struct shmem {
    struct rbuf ch_data[4];
    pid_t hat_drv_pid;
    struct ioctrl ioctrl;
    struct sensor sensor[4];
};

int getShareMem(struct shmem **shmem, int *segment_id);
int deAllocSharedMem(struct shmem *shmem);
int deAllocAndFreeSharedMem(struct shmem *shmem, int seg_id);
int allocSharedMem(struct shmem **shmem, int *segment_id);

void init_buf(struct rbuf *rbuf);
int get_buf_size(struct rbuf *buf);
int add_to_buf(struct rbuf *buf, unsigned int value);
int get_from_buf(struct rbuf *buf, unsigned int *value);

#endif

/* ------------------------------------------------------------------------- */
/* End of file */
