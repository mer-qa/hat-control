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
#include <semaphore.h>

#define MAX_LEN_NAME 30
// Shared memory properties
// #define SH_MEM_ID   0xaa
int SH_MEM_ID;
#define SH_MEM_SEG_SIZE 0x6400
// #define SHMEMNAME "HATSHMEM"
char SHMEMNAME[MAX_LEN_NAME];
// #define COMMSEM   "HATSHCOMM"
char COMMSEM[MAX_LEN_NAME];

// Max sensor inputs
#define MAX_SENSOR_INPUTS    4

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
    unsigned int digdir;
    unsigned int sensorDigOutput[MAX_SENSOR_INPUTS];
};

// Data store for data handling.
// This is used if function key is specified in sensor configuration

struct data_function {
    unsigned int function;
    unsigned int size;
    double *fdata;
    unsigned int stored;
};

#define MULTIPLE_FIRST      1
#define OFFSET_FIRST    2

struct sensor {
    unsigned int type;
    int offset_mv;
    int mult_mv[2];
    int offset_mult_order;
    char str[2][10];
    int samplerate;
    char pathFilename[255];
    struct data_function data_func;
};

struct streamConfig {
    unsigned int sensors;
    unsigned int ch_table[MAX_SENSOR_INPUTS];
    struct sensor sensor[MAX_SENSOR_INPUTS];
    unsigned int samplerate;
    char pathFilename[255];
    unsigned long samples;
    pid_t hat_ctrl_pid;
    long digout;
    long digdir;
    unsigned long time_between_samples;
};

struct streamSetup {
    unsigned int samplerate;
    unsigned int numberOfChannels;
    unsigned int samplesPerPacket;
    unsigned int readSizeMultiplier;
    unsigned int readSamples;
    unsigned long samples;
    unsigned int start_ch;
    FILE *fhandle;
};

struct shmem {
    struct rbuf ch_data[MAX_SENSOR_INPUTS];
    pid_t hat_drv_pid;
    pid_t hat_ctrl_pid;
    int retValToCtrl;
    struct ioctrl ioctrl;
    unsigned int cmd;
    struct streamConfig streamConfig;
    unsigned long serialNumber;
};

struct hatCtrl {
    struct shmem *shmem;
    sem_t *semshmem;
    sem_t *semcomm;
    double time;
};

int getShareMem(struct shmem **shmem, int *segment_id);
int deAllocSharedMem(struct shmem *shmem);
int deAllocAndFreeSharedMem(struct shmem *shmem, int seg_id);
int allocSharedMem(struct shmem **shmem, int *segment_id);

void initBuf(struct rbuf *rbuf);
int getBufSize(struct rbuf *buf);
int addToBuf(struct rbuf *buf, int value);
int getFromBuf(struct rbuf *buf, int *value);

void getShName(char *SerialNumber, char *semaphoreName);
void getShComm(char *SerialNumber, char *semaphoreName);
// Names are HATSHMEM_[SN] and HATSHCOMM_[SN]
void getShMemID(char *SerialNumber);
// Get weak "CheckSum" from SN. It is used as shared mem ID
int hash(char *str, unsigned int *hashValue);
// calculate weak "CheckSum"
#endif

/* ------------------------------------------------------------------------- */
/* End of file */
