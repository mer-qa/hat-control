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
#include "hat_drv.h"
#include "u3.h"
#include <unistd.h>
#include <string.h>
#include "shmemlib.h"
#include <signal.h>
#include <semaphore.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <errno.h>

/* ------------------------------------------------------------------------- */
/* MACROS */
#define APP_NAME "hat_drv"
#define PRINTOUT(...) printf(APP_NAME ": " __VA_ARGS__)
#define PRINTERR(...) fprintf(stderr, APP_NAME ": " __VA_ARGS__)

/* ------------------------------------------------------------------------- */
/* LOCAL FUNCTION PROTOTYPES */
/* ------------------------------------------------------------------------- */
int configIO(HANDLE hDevice);
int streamConfigure(HANDLE hDevice, struct streamSetup *streamSetup);
int streamStart(HANDLE hDevice);
int streamRead(HANDLE hDevice,  struct streamSetup *streamSetup, sem_t *sem, struct shmem *shmem, 
               u3CalibrationInfo *caliInfo,  unsigned int *data, unsigned long *size);
int streamStop(HANDLE hDevice);
int readSensorConfig(struct shmem *shmem);
int updateIO(HANDLE hdevice, sem_t *sem, struct shmem *shmem);
long maskDO(HANDLE Handle, long wmask, long smask);
int streamDataAndStore(HANDLE hDevice, struct streamSetup *streamSetup, struct shmem *shmem, 
                       u3CalibrationInfo *caliInfo, sem_t *sem, unsigned long *samples);
int writeToFile(struct shmem *shmem, struct streamSetup *streamSetup, int value, int ch);
int handleStreamConfig(struct streamSetup *streamSetup, struct streamConfig *streamConfig);
void sendRespToCtrl(struct shmem *shmem, int ret);

/* ------------------------------------------------------------------------- */
/* GLOBAL VARIABLES */
int ch_table[8] = {0};
int autoRecoveryOn = 1;
int packetCounter = 0;

sem_t *semshmem;
sem_t *semcomm;

volatile sig_atomic_t exit_flag = 0, usr1_flag = 0, alrm_flag = 0;

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */
void sig_int(int sig) {
 exit_flag = 1;
}

void sig_usr1(int sig) {
 usr1_flag = 1;
}

void sig_alrm(int sig) {
 alrm_flag = 1;
}

int main(int argc, char **argv)
{
    HANDLE hDevice;
    u3CalibrationInfo caliInfo;
    struct shmem *shmem = NULL; 
    int ret = 0, sh_seq_id = 0, dataStreaming = 0;
    struct streamSetup streamSetup;

    (void) signal(SIGINT, sig_int);
    (void) signal(SIGUSR1, sig_usr1);
    (void) signal(SIGALRM, sig_alrm);

    memset(&streamSetup,0,sizeof(struct streamSetup));

    sigset_t blockMask, sigsuMask, oldMask;

    // Prepare signal mask for sigsuspend
    sigprocmask(SIG_BLOCK, NULL, &oldMask); // Save old mask 
    sigprocmask(SIG_BLOCK, NULL, &blockMask); // Initialize new mask
    sigprocmask(SIG_BLOCK, NULL, &sigsuMask); // Initialize new mask

    // Block SIGUSR1 and SIGINT
    sigaddset(&blockMask, SIGUSR1);
    sigaddset(&blockMask, SIGINT);
    sigaddset(&blockMask, SIGALRM);
    sigprocmask(SIG_SETMASK, &blockMask, NULL);

    // remove SIGUSR1 from mask to be used by sigsuspend 
    sigdelset(&sigsuMask, SIGUSR1); // form mask to unblock SIGUSR1 and SIGINT 
    sigdelset(&sigsuMask, SIGINT); // form mask to unblock SIGUSR1 and SIGINT 
    sigdelset(&sigsuMask, SIGALRM); // form mask to unblock SIGUSR1 and SIGINT 
    
    struct itimerval val;
    val.it_interval.tv_sec = 1;
    val.it_interval.tv_usec = 0;
    val.it_value = val.it_interval;
    setitimer(ITIMER_REAL, &val, NULL);

    // Create semaphores for handling shared memory
    sem_unlink(SHMEMNAME);
    semshmem = sem_open(SHMEMNAME, O_CREAT | O_EXCL, 0644, 1);

    if(semshmem == SEM_FAILED) {
        PRINTERR("Unable to create semaphore (%d)\n",errno);
        goto done;
    }

    sem_unlink(COMMSEM);
    semcomm = sem_open(COMMSEM, O_CREAT | O_EXCL, 0644, 1);

    if(semcomm == SEM_FAILED) {
        PRINTERR("Unable to create semaphore (%d)\n",errno);
        goto done;
    }

    allocSharedMem(&shmem, &sh_seq_id);
    if (shmem == NULL) {
        PRINTERR("Error to allocate shared mem\n");
    }
    shmem->hat_drv_pid = getpid();

    //Opening first found U3 over USB
    if ( (hDevice = openUSBConnection(-1)) == NULL)
        goto done;

    if (getCalibrationInfo(hDevice, &caliInfo) < 0)
        goto close;
    
    if (configIO(hDevice) != 0)
        goto close;

    PRINTOUT("HAT driver started succesfully\n");
    PRINTOUT("Driver uses device SN:%lX\n",caliInfo.serialNumber);
    shmem->serialNumber = caliInfo.serialNumber;

    // Set all switches off state 
    shmem->cmd = CHANGE_IO;
    updateIO(hDevice, semshmem, shmem);
    
    //Stopping any previous streams
    streamStop(hDevice);

    unsigned int cmd;
    unsigned long samples;
    struct timespec timespec;
    
    timespec.tv_sec = 1;
    timespec.tv_nsec = 0;

    while (!exit_flag) {
        // Block signals during flag check
        sigprocmask(SIG_BLOCK, &blockMask, NULL);
        while (usr1_flag == 0 && exit_flag == 0 && dataStreaming == 0 && alrm_flag == 0) {
            //printf("sleep\n");
            sigsuspend(&sigsuMask);
            //printf("wake\n");
        }
        // Unblock signals after flag check
        sigprocmask(SIG_UNBLOCK, &blockMask, NULL);
        if (alrm_flag) {
            alrm_flag = 0;
            if (!dataStreaming && !usr1_flag) {
                if (updateIO(hDevice, semshmem, shmem) != 0) {
                    exit_flag = 1;
                }
            }
        }

        if (usr1_flag) {
            usr1_flag = 0;
            sem_wait(semshmem);
            cmd = shmem->cmd;
            sem_post(semshmem);
        }
        else {
            cmd = NONE;
        }

        if (cmd == CHANGE_IO) {
            sendRespToCtrl(shmem, updateIO(hDevice, semshmem, shmem));
        }
        else if (cmd == START_STREAMING) {
            //Stopping any previous streams
            streamStop(hDevice);
            errno = 0;
            if (strcmp(shmem->streamConfig.pathFilename, "")) {
                streamSetup.fhandle = fopen(shmem->streamConfig.pathFilename,"w");
                if (streamSetup.fhandle == NULL) {
                    PRINTERR("stream file error (%d)\n",errno);
                    //sendErrorToClienterr = -1;
                }
            }
 
            if (handleStreamConfig(&streamSetup, &shmem->streamConfig)) {
                PRINTERR("error in stream config\n");
            }
            else {
                samples = 0;
                if (streamConfigure(hDevice, &streamSetup) != 0) {
                    ret = -1;
                    goto close;
                }
    
                if (streamStart(hDevice) != 0)
                    goto close;
                
                // Iinit buffer
                initBuf(&shmem->ch_data[0]);
                initBuf(&shmem->ch_data[1]);
                initBuf(&shmem->ch_data[2]);
                initBuf(&shmem->ch_data[3]);
    
                dataStreaming = 1;
            }
            sendRespToCtrl(shmem,0);
        }
        else if (cmd == STOP_STREAMING) {
            if (streamSetup.fhandle != NULL) {
                fclose(streamSetup.fhandle);
                streamSetup.fhandle = NULL;
            }
            dataStreaming = 0;
            ret = streamStop(hDevice);
            // Send return 
            sendRespToCtrl(shmem, ret);
        }

        if (dataStreaming) {
            streamDataAndStore(hDevice, &streamSetup, shmem, &caliInfo, semshmem, &samples);
            if (streamSetup.samples != 0 && (samples >= streamSetup.samples )) {
                if (streamSetup.fhandle != NULL) {
                    fclose(streamSetup.fhandle);
                    streamSetup.fhandle = NULL;
                }
                dataStreaming = 0;
                streamStop(hDevice);
            }
        }
    }

close:
    shmem->retValToCtrl = ret;
    if (shmem->streamConfig.hat_ctrl_pid != 0)
        kill(shmem->streamConfig.hat_ctrl_pid, SIGINT); 
    if (shmem->hat_ctrl_pid != 0) 
        kill(shmem->hat_ctrl_pid, SIGINT); 
    
    closeUSBConnection(hDevice);

done:

    deAllocAndFreeSharedMem(shmem, sh_seq_id);
    sem_unlink(SHMEMNAME);
    sem_unlink(COMMSEM);
    // restore the original mask
    sigprocmask(SIG_SETMASK, &oldMask, NULL);
    return 0;
}

void sendRespToCtrl(struct shmem *shmem, int ret)
{
    shmem->retValToCtrl = ret;
    kill(shmem->hat_ctrl_pid, SIGUSR1);
}


int handleStreamConfig(struct streamSetup *streamSetup, struct streamConfig *streamConfig)
{
    int t = 0;

    // Count channels
    streamSetup->numberOfChannels = 0;
    for (t = 0; t < 4; t++) {
        if (streamConfig->sensor[t].type != NONE) {
            ch_table[streamSetup->numberOfChannels++] = ach[t];
        }
    }
    // Samplerate
    // TODO: Check that samplerate is valid
    streamSetup->samplerate = streamConfig->samplerate;
    streamSetup->samples = streamConfig->samples * streamSetup->numberOfChannels;

    streamSetup->readSizeMultiplier = 1;

    if (streamSetup->samplerate <= 20) {
        streamSetup->samplesPerPacket = streamSetup->numberOfChannels;
        streamSetup->readSamples = streamSetup->numberOfChannels;
    }
    else if (streamSetup->samplerate > 20 && streamSetup->samplerate <= 150) {
        streamSetup->samplesPerPacket = streamSetup->numberOfChannels*2;
        streamSetup->readSamples = streamSetup->numberOfChannels*2;
    }
    else if (streamSetup->samplerate > 150 && streamSetup->samplerate <= 500) {
        streamSetup->samplesPerPacket = streamSetup->numberOfChannels*4;
        streamSetup->readSamples = streamSetup->numberOfChannels*4;
    }
    else {   
        switch (streamSetup->numberOfChannels) {
            case 1:
                streamSetup->samplesPerPacket = 25;
                streamSetup->readSizeMultiplier = 5;
                streamSetup->readSamples = 25;
            break;
            case 2:
            case 3:
            case 4:
                // This is not good. High samplerates won't work with this parameters
                // Stream reading function needs fixing to use different values.
                streamSetup->samplesPerPacket = 24;
                streamSetup->readSizeMultiplier = 1;
                streamSetup->readSamples = 24;
            break;
            default:
                streamSetup->readSizeMultiplier = 5;
                streamSetup->samplesPerPacket = 25;
                streamSetup->readSamples = 25;
            break;
        }
    }

    if (streamSetup->numberOfChannels == 0) {
        return -1;
    }
    else {
        return 0;
    }
}   

int writeToFile(struct shmem *shmem, struct streamSetup *streamSetup, int value, int ch)
{
    FILE *fhandle = streamSetup->fhandle;
    if (value == 0xffff) {
       fprintf(fhandle,"Over!  ");
    }
    else {
        switch(shmem->streamConfig.sensor[ch].type) {
        case VOLTAGE:
            fprintf(fhandle,"%d mV ",value);
            break;
        case CURRENT:
            if (shmem->ioctrl.sensorDigOutput[ch] == 0) {
                fprintf(fhandle,"%4.1f mA ",(double)(value)*LO_GAIN_MULT);
            }
            else {
                fprintf(fhandle,"%4.2f mA ",(double)(value)*HI_GAIN_MULT);
            }
            break;
        }
    }
    if (ch == (streamSetup->numberOfChannels-1) ) {
        fprintf(fhandle, "\n");
    }
    return 0;
}

int streamDataAndStore(HANDLE hDevice, struct streamSetup *streamSetup, struct shmem *shmem, 
                       u3CalibrationInfo *caliInfo, sem_t *sem, unsigned long *samples)
{
    int i;
    double dvolt;
    unsigned int data[streamSetup->numberOfChannels * streamSetup->readSamples * streamSetup->readSizeMultiplier];
    unsigned long readSize = 0;

    if (!streamRead(hDevice, streamSetup, sem, shmem, caliInfo, data, &readSize)) {
        for (i = 0; i < readSize; i++) {
            if ( (((*samples)+i) >= streamSetup->samples) && (streamSetup->samples != 0) ) {
                break;
            }
            getAinVoltCalibrated_hw130(caliInfo, (i % streamSetup->numberOfChannels), 31, data[i], &(dvolt));
            dvolt = (dvolt*1000);
            if (dvolt > MAX_VALUE) {
                dvolt = 0xffff;
            }

            sem_wait(sem);
            if (addToBuf(&shmem->ch_data[i % streamSetup->numberOfChannels],(int)(nearbyint(dvolt))) < 0) {
                //printf("buf:full\n");
            }
            if (streamSetup->fhandle != NULL) {
                 writeToFile(shmem, streamSetup, (int)(dvolt), i % streamSetup->numberOfChannels);
            }
            sem_post(sem);
        }
        (*samples) += i;
    }
    else {
        return -1;
    }
    return i;
}

int updateIO(HANDLE hDevice, sem_t *sem, struct shmem *shmem)
{
    long error;
    // USB_DATA switches are inverted
    if((error = maskDO(hDevice, 0xffffff,shmem->ioctrl.iostate ^ ((1 << USB_DATA_1) | (1 << USB_DATA_2)))) != 0) {
        PRINTERR("Error setting IO: %d\n",(int)error);
    }
    else {
        shmem->ioctrl.sensorDigOutput[0] = (shmem->ioctrl.iostate & (1 << AN0_IO)) >> AN0_IO;
        shmem->ioctrl.sensorDigOutput[1] = (shmem->ioctrl.iostate & (1 << AN1_IO)) >> AN1_IO;
        shmem->ioctrl.sensorDigOutput[2] = (shmem->ioctrl.iostate & (1 << AN2_IO)) >> AN2_IO;
        shmem->ioctrl.sensorDigOutput[3] = (shmem->ioctrl.iostate & (1 << AN3_IO)) >> AN3_IO;
    }
    return error;
}

long maskDO(HANDLE Handle, long wmask, long smask)
{
    uint8 sendDataBuff[7];
    uint8 Errorcode, ErrorFrame;
  
    /* Setting up Feedback command to set digital Channel to output and to set the state */
    sendDataBuff[0] = 27;             //PortStateWrite
    sendDataBuff[1] = (wmask & 0xff);                   //IONumber 0-7
    sendDataBuff[2] = (wmask & (0xff << 8)) >> 8;       //IONumber 8-15
    sendDataBuff[3] = (wmask & (0xff << 16)) >> 16;     //IONumber 16-23

    sendDataBuff[4] = (smask & 0xff);                   //IONumber 0-7
    sendDataBuff[5] = (smask & (0xff << 8)) >> 8;       //IONumber 8-15
    sendDataBuff[6] = (smask & (0xff << 16)) >> 16;     //IONumber 16-23

    if(ehFeedback(Handle, sendDataBuff, 7, &Errorcode, &ErrorFrame, NULL, 0) < 0)
        return -1;
    if(Errorcode)
        return (long)Errorcode;

    return 0;
}
//Sends a ConfigIO low-level command that configures the FIOs, DAC, Timers and
//Counters for this example
int configIO(HANDLE hDevice)
{
    uint8 sendBuff[12], recBuff[12];
    uint16 checksumTotal;
    int sendChars, recChars;

    sendBuff[1] = (uint8)(0xF8);  //Command byte
    sendBuff[2] = (uint8)(0x03);  //Number of data words
    sendBuff[3] = (uint8)(0x0B);  //Extended command number

    sendBuff[6] = 13;  //Writemask : Setting writemask for timerCounterConfig (bit 0),
                     //            FIOAnalog (bit 2) and EIOAnalog (bit 3)

    sendBuff[7] = 0;  //Reserved
    sendBuff[8] = 64; //TimerCounterConfig: Disabling all timers and counters, set
                    //                    TimerCounterPinOffset to 4 (bits 4-7)
    sendBuff[9] = 0;  //DAC1Enable

    //FIOAnalog : setting all FIOs as analog inputs
    sendBuff[10] = (1 << AI0) | (1 << AI1) | (1 << AI2) | (1 << AI3); 
    sendBuff[11] = 0x00;   //EIOAnalog : setting all EIOs as analog inputs
    extendedChecksum(sendBuff, 12);

    //Sending command to U3
    if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 12)) < 12)
    {
        if(sendChars == 0)
            PRINTERR("ConfigIO error : write failed\n");
        else
            PRINTERR("ConfigIO error : did not write all of the buffer\n");
        return -1;
    }

    //Reading response from U3
    if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 12)) < 12)
    {
        if(recChars == 0)
            PRINTERR("ConfigIO error : read failed\n");
        else
            PRINTERR("ConfigIO error : did not read all of the buffer\n");
        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 12);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
    {
        PRINTERR("ConfigIO error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        PRINTERR("ConfigIO error : read buffer has bad checksum16(LBS)\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        PRINTERR("ConfigIO error : read buffer has bad checksum8\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x03) || recBuff[3] != (uint8)(0x0B) )
    {
        PRINTERR("ConfigIO error : read buffer has wrong command bytes\n");
        return -1;
    }

    if( recBuff[6] != 0)
    {
        PRINTERR("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
        return -1;
    }

    if( recBuff[8] != 64)
    {
        PRINTERR("ConfigIO error : TimerCounterConfig did not get set correctly\n");
        return -1;
    }

    if( recBuff[10] != 255 && recBuff[10] != (uint8)((1 << AI0) | (1 << AI1) | (1 << AI2) | (1 << AI3)))
    {
        PRINTERR("ConfigIO error : FIOAnalog did not set get correctly\n");
        return -1;
    }

    if( recBuff[11] != 0)
    {
        PRINTERR("ConfigIO error : EIOAnalog did not set get correctly (%d)\n", recBuff[11]);
        return -1;
    }

    return 0;
}

//Sends a StreamConfig low-level command to configure the stream.
int streamConfigure(HANDLE hDevice, struct streamSetup *streamSetup)
{
    int sendBuffSize;
    sendBuffSize = 12+streamSetup->numberOfChannels*2;
    uint8 sendBuff[sendBuffSize], recBuff[8];
    int sendChars, recChars;
    uint16 checksumTotal;
    uint16 scanInterval;
    int i;
   
    sendBuff[1] = (uint8)(0xF8);    //command byte
    sendBuff[2] = 3 + streamSetup->numberOfChannels;  //number of data words = NumChannels + 3
    sendBuff[3] = (uint8)(0x11);    //extended command number
    sendBuff[6] = streamSetup->numberOfChannels;      //NumChannels
    sendBuff[7] = streamSetup->samplesPerPacket; //SamplesPerPacket
    sendBuff[8] = 0;  //Reserved

    if (streamSetup->samplerate < 1000) {
        sendBuff[9] = 0x0c; // clock divided by 256
        scanInterval = 48000000/256/(streamSetup->samplerate);
    }
    else {
        sendBuff[9] = 0x0a;  //ScanConfig:
                          // Bit 7: Reserved
                          // Bit 6: Reserved
                          // Bit 3: Internal stream clock frequency = b0: 4 MHz
                          // Bit 2: Divide Clock by 256 = b0
                          // Bits 0-1: Resolution = b01: 11.9-bit effective
        scanInterval = 48000000/streamSetup->samplerate;
    }
    sendBuff[10] = (uint8)(scanInterval&(0x00FF));  //scan interval (low byte)
    sendBuff[11] = (uint8)(scanInterval >> 8);       //scan interval (high byte)

    for(i = 0; i < streamSetup->numberOfChannels; i++)
    {
        sendBuff[12 + i*2] = ch_table[i];  //PChannel = i
        sendBuff[13 + i*2] = 31; //NChannel = 31: Single Ended
    }

    extendedChecksum(sendBuff, sendBuffSize);

    //Sending command to U3
    sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, sendBuffSize);
    if(sendChars < sendBuffSize)
    {
        if(sendChars == 0)
            PRINTERR("Error : write failed (StreamConfig).\n");
        else
            PRINTERR("Error : did not write all of the buffer (StreamConfig).\n");
        return -1;
    }

    for(i = 0; i < 8; i++)
        recBuff[i] = 0;

    //Reading response from U3
    recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 8);
    if(recChars < 8)
    {
        if(recChars == 0)
            PRINTERR("Error : read failed (StreamConfig).\n");
        else
            PRINTERR("Error : did not read all of the buffer, %d (StreamConfig).\n", recChars);

        for(i=0; i<8; i++)
            PRINTERR("%d ", recBuff[i]);

        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 8);
    if( (uint8)((checksumTotal / 256) & 0xff) != recBuff[5])
    {
        PRINTERR("Error : read buffer has bad checksum16(MSB) (StreamConfig).\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        PRINTERR("Error : read buffer has bad checksum16(LBS) (StreamConfig).\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        PRINTERR("Error : read buffer has bad checksum8 (StreamConfig).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x01) || recBuff[3] != (uint8)(0x11) || recBuff[7] != (uint8)(0x00))
    {
        PRINTERR("Error : read buffer has wrong command bytes (StreamConfig).\n");
        return -1;
    }

    if(recBuff[6] != 0)
    {
        PRINTERR("Errorcode # %d from StreamConfig read.\n", (unsigned int)recBuff[6]);
        return -1;
    }

    return 0;
}

//Sends a StreamStart low-level command to start streaming.
int streamStart(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xA8);  //CheckSum8
    sendBuff[1] = (uint8)(0xA8);  //command byte

    //Sending command to U3
    sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 2);
    if(sendChars < 2)
    {
        if(sendChars == 0)
            PRINTERR("Error : write failed.\n");
        else
            PRINTERR("Error : did not write all of the buffer.\n");
        return -1;
    }

    //Reading response from U3
    recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 4);
    if(recChars < 4)
    {
        if(recChars == 0)
            PRINTERR("Error : read failed.\n");
        else
            PRINTERR("Error : did not read all of the buffer.\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xA9) || recBuff[3] != (uint8)(0x00) )
    {
        PRINTERR("Error : read buffer has wrong command bytes \n");
        return -1;
    }

    if(recBuff[2] != 0)
    {
        PRINTERR("Errorcode # %d from StreamStart read.\n", (unsigned int)recBuff[2]);
        return -1;
    }

    return 0;
}

int checkStreamResponse(uint8 *recBuff, int recBuffSize, struct streamSetup *streamSetup)
{
    uint16 checksumTotal;
       
    checksumTotal = extendedChecksum16(recBuff, recBuffSize);
    if( (uint8)((checksumTotal >> 8) & 0xff) != recBuff[5])
    {
        PRINTERR("Error : read buffer has bad checksum16(MSB) (StreamData).\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        PRINTERR("Error : read buffer has bad checksum16(LBS) (StreamData).\n");
        return -1;
    }

    checksumTotal = extendedChecksum8(recBuff);
    if( checksumTotal != recBuff[0])
    {
        PRINTERR("Error : read buffer has bad checksum8 (StreamData).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF9) || recBuff[2] != 4 + streamSetup->samplesPerPacket || recBuff[3] != (uint8)(0xC0) )
    {
        PRINTERR("Error : read buffer has wrong command bytes (StreamData).\n");
        return -1;
    }

    if(recBuff[11] == 59)
    {
        if(!autoRecoveryOn)
        {
            PRINTERR("\nU3 data buffer overflow detected in packet %d.\nNow using auto-recovery and reading buffered samples.\n", packetCounter);
            autoRecoveryOn = 1;
        }
    }
    else if(recBuff[11] == 60)
    {
        PRINTERR("Auto-recovery report in packet %d: %d scans were dropped.\nAuto-recovery is now off.\n", packetCounter, recBuff[6] + (recBuff[7] << 8));
        autoRecoveryOn = 0;
    }
    else if(recBuff[11] != 0)
    {
        PRINTERR("Errorcode # %d from StreamData read.\n", (unsigned int)recBuff[11]);
        return -1;
    }

    if(packetCounter != (int)recBuff[10])
    {
        PRINTERR("PacketCounter (%d) does not match with with current packet count (%d)(StreamData).\n", recBuff[10], packetCounter);
        return -1;
    }

    if(packetCounter >= 255)
        packetCounter = 0;
    else
        packetCounter++;


    // Check ok.
    return 0;
}


//Reads the StreamData low-level function response in a loop.
//All voltages from the stream are stored in the voltages 2D array.
int streamRead(HANDLE hDevice, struct streamSetup *streamSetup, sem_t *sem, struct shmem *shmem, 
               u3CalibrationInfo *caliInfo, unsigned int *data, unsigned long *size )
{
    long recBuffSize, readn,i, scanNumber = 0;
    int recChars = 0, backLog;
    int k, currChannel = 0,p;
    //uint16 voltageBytes;
    long startTime, endTime;
    int responseSize;           //The number of bytes in a StreamData response (differs with SamplesPerPacket)

    responseSize = (14 + streamSetup->samplesPerPacket*2) * streamSetup->readSizeMultiplier;        
    uint8 recBuff[responseSize];
    recBuffSize = 14 + streamSetup->samplesPerPacket*2;

    startTime = getTickCount();
    readn = streamSetup->readSamples / streamSetup->samplesPerPacket;
    for (i = 0; i < readn; i++) 
    {
        //Reading stream response from U3
        recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP3_IN, recBuff, responseSize);
        if(recChars < responseSize)
        {
            PRINTERR("Error : read failed, expected %d bytes but received %d (StreamData).\n",responseSize, recChars);
            return -1;
        }
       
        //printf("%d, %d, %d\n",backLog, recChars, responseSize);
        for (p = 0; p < streamSetup->readSizeMultiplier; p++) {
            checkStreamResponse(&recBuff[p*recBuffSize], recBuffSize, streamSetup);
            backLog = (int)recBuff[12 + streamSetup->samplesPerPacket*2 + (p*recBuffSize)];
            if (backLog > 100) {
                PRINTERR("Buffer > 100\n");
            }
            for(k = 12; k < (12 + streamSetup->samplesPerPacket*2); k += 2)
            {   
                data[currChannel + scanNumber*streamSetup->numberOfChannels] = (uint16)recBuff[k+(p*recBuffSize)] + ((uint16)recBuff[k+1+(p*recBuffSize)] << 8);
                (*size)++;
                currChannel++;
                if(currChannel >= streamSetup->numberOfChannels)
                {
                    currChannel = 0;
                    scanNumber++;
                }
            }
        }
    }

    endTime = getTickCount();
    //printf("\nRate of samples: %.0lf samples per second\n", (scanNumber*NumChannels)/((endTime - startTime)/1000.0));
    //printf("Rate of scans: %.0lf scans per second\n\n", scanNumber/((endTime - startTime)/1000.0));

    return 0;
}

//Sends a StreamStop low-level command to stop streaming.
int streamStop(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    // Reset packet counter
    autoRecoveryOn = 1;
    packetCounter = 0;     

    sendBuff[0] = (uint8)(0xB0);  //CheckSum8
    sendBuff[1] = (uint8)(0xB0);  //command byte

    //Sending command to U3
    sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, 2);
    if(sendChars < 2)
    {
        if(sendChars == 0)
            PRINTERR("Error : write failed (StreamStop).\n");
        else
            PRINTERR("Error : did not write all of the buffer (StreamStop).\n");
        return -1;
    }

    //Reading response from U3
    recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, 4);
    if(recChars < 4)
    {
        if(recChars == 0)
            PRINTERR("Error : read failed (StreamStop).\n");
        else
            PRINTERR("Error : did not read all of the buffer (StreamStop).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xB1) || recBuff[3] != (uint8)(0x00) )
    {
        PRINTERR("Error : read buffer has wrong command bytes (StreamStop).\n");
        return -1;
    }

    if(recBuff[2] != 0)
    {
        //printf("Errorcode # %d from StreamStop read.\n", (unsigned int)recBuff[2]);
        return 0;
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* End of file */
