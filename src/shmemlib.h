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
#include <stdio.h>

// Shared memory properties
#define SH_MEM_ID   0xaa
#define SH_MEM_SEG_SIZE 0x6400
#define SHMEMNAME "HATSHMEM"
#define COMMSEM   "HATSHCOMM"


// Data buffer size for each channel in shared memory
#define BUF_SIZE 1000

struct rbuf {
    unsigned int buf[BUF_SIZE];
    unsigned int start;
    unsigned int end;
    int overfull;
};

struct ioctrl {
    unsigned int iostate;
    unsigned int sensorDigOutput[4];
};

struct sensor {
    unsigned int type;
    unsigned int offset_mv;
    unsigned int mult_mv;
};

struct streamConfig {
    unsigned int sensors;
    struct sensor sensor[4];
    unsigned int samplerate;
    char pathFilename[255];
    unsigned long samples;
    pid_t hat_ctrl_pid;
};

struct streamSetup {
    unsigned int samplerate;
    unsigned int numberOfChannels;
    unsigned int samplesPerPacket;
    unsigned int readSizeMultiplier;
    unsigned int readSamples;
    unsigned long samples;
    FILE *fhandle;
};

struct shmem {
    struct rbuf ch_data[4];
    pid_t hat_drv_pid;
    pid_t hat_ctrl_pid;
    int retValToCtrl;
    struct ioctrl ioctrl;
    unsigned int cmd;
    struct streamConfig streamConfig;
    unsigned long serialNumber;
};

int getShareMem(struct shmem **shmem, int *segment_id);
int deAllocSharedMem(struct shmem *shmem);
int deAllocAndFreeSharedMem(struct shmem *shmem, int seg_id);
int allocSharedMem(struct shmem **shmem, int *segment_id);

void initBuf(struct rbuf *rbuf);
int getBufSize(struct rbuf *buf);
int addToBuf(struct rbuf *buf, int value);
int getFromBuf(struct rbuf *buf, int *value);

#endif

/* ------------------------------------------------------------------------- */
/* End of file */
