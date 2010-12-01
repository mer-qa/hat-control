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

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES */
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
int streamConfig(HANDLE hDevice);
int streamStart(HANDLE hDevice);
int streamRead(HANDLE hDevice, sem_t *sem, struct shmem *shmem, 
               u3CalibrationInfo *caliInfo,  unsigned int *data, long data_size);
int streamStop(HANDLE hDevice);
int readSensorConfig(struct shmem *shmem);
int streamData(HANDLE hDevice, struct shmem *shmem, u3CalibrationInfo *caliInfo, 
               sem_t *sem);
int updateIO(HANDLE hdevice, sem_t *sem, struct shmem *shmem);
long maskDO(HANDLE Handle, long wmask, long smask);

/* ------------------------------------------------------------------------- */
/* GLOBAL VARIABLES */
uint8 NumChannels = 0;          // Number of analog input channels channels
uint8 SamplesPerPacket = 25;    // Needs to be 25 to read multiple StreamData 
                                // responses in one large packet, otherwise can 
                                // be any value between 1-25 for 1 StreamData 
                                // response per packet.
uint16 sampleRate = 5000;
int ch_table[8] = {0};
long samplesRead = 200;
volatile sig_atomic_t exit_flag = 0, usr1_flag = 0;

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */
void sig_int(int sig) {
 exit_flag = 1;
}

void sig_usr1(int sig) {
 usr1_flag = 1;
}


int main(int argc, char **argv)
{
    HANDLE hDevice;
    u3CalibrationInfo caliInfo;
    long i;
    struct shmem *shmem = NULL; 
    int sh_seq_id = 0;
    sem_t *sem;
    (void) signal(SIGINT, sig_int);
    (void) signal(SIGUSR1, sig_usr1);

    sigset_t newMask, oldMask;

    // Prepare signal mask for sigsuspend
    sigprocmask(SIG_BLOCK, NULL, &oldMask); // Save old mask 
    sigprocmask(SIG_BLOCK, NULL, &newMask); // Initialize new mask
    // Block SIGUSR1 and SIGINT
    sigaddset(&newMask, SIGUSR1 | SIGINT);
    sigprocmask(SIG_SETMASK, &newMask, NULL);

    // remove SIGUSR1 from mask to be used by sigsuspend 
    sigdelset(&newMask, SIGUSR1 | SIGINT); // form mask to unblock SIGUSR1 and SIGINT 
    
    // Create semaphore for handling shared memory
    sem_unlink(SHMEMNAME);
    sem = sem_open(SHMEMNAME, O_CREAT | O_EXCL, 0644, 1);

    if(sem == SEM_FAILED)
    {
        PRINTERR("Unable to create semaphore (%d)\n",errno);
        goto done;
    }

    allocSharedMem(&shmem, &sh_seq_id);
    if (shmem == NULL) {
        PRINTERR("Error to allocate shared mem\n");
    }
    shmem->hat_drv_pid = getpid();

    if (argc > 1) {
        for (i = 1; i < argc; i++) {
            if (!strcmp(argv[i],"-ai0")) {
                //strcat(chan, "Dev1/ai0,");
                ch_table[NumChannels++] = 0;
            }
            else if (!strcmp(argv[i],"-ai1")) {
                //strcat(chan, "Dev1/ai1,");
                ch_table[NumChannels++] = 1;
            }
            else if (!strcmp(argv[i],"-ai2")) {
                //strcat(chan, "Dev1/ai2,");
                ch_table[NumChannels++] = 2;
            }
            else if (!strcmp(argv[i],"-ai3")) {
                //strcat(chan, "Dev1/ai3,");
                ch_table[NumChannels++] = 3;
            }
            else if (!strcmp(argv[i],"-sr") && (argc >= i)) {
                char *endptr;
                long val;
                val = strtoul(argv[i+1] ,&endptr, 10);
                if (val <= 0) {
                    sampleRate = 1;
                    PRINTERR("Error in samplerate value. Using default 1Hz\n");
                }
                else {
                    //printf("Using samplerate: %dHz\n",(unsigned int)val);
                    sampleRate = (uint16)val;
                }
            }
            else if (!strcmp(argv[i],"-scount") && (argc >= i)) {
                char *endptr;
                long val;
                val = strtoul(argv[i+1] ,&endptr, 10);
                if (val <= 0) {
                    samplesRead = 1000;
                    PRINTERR("Error in sample count value. Using 1000\n");
                }
                else {
                    PRINTERR("samples read: %d\n",(unsigned int)val);
                    samplesRead = (long)val;
                }
            }
        }
        if (NumChannels == 0 || NumChannels > 4) {
            PRINTERR("Error: number of channels.\n");
            goto done;
        }
    }
    else {
        readSensorConfig(shmem);
    }

    switch (NumChannels) {
        case 1:
            SamplesPerPacket = 25;
        break;
        case 2:
        case 3:
        case 4:
            SamplesPerPacket = 25;
        break;
        default:
            SamplesPerPacket = 25;
        break;
    }

    int rem;
    rem = samplesRead % SamplesPerPacket;
    if (rem) {
        samplesRead -= samplesRead % SamplesPerPacket;
    }

    if (samplesRead < 25){
        PRINTERR("Error: atleast 25 samples needs to be read\n");
        goto done;
    }
        
    //Opening first found U3 over USB
    if ( (hDevice = openUSBConnection(-1)) == NULL)
        goto done;

    if (getCalibrationInfo(hDevice, &caliInfo) < 0)
        goto close;

    if (configIO(hDevice) != 0)
        goto close;

    PRINTOUT("HAT driver started succesfully\n");
    // Set all switches off state 
    shmem->ioctrl.changeIO = 1;
    updateIO(hDevice, sem, shmem);

    //printf("samplerate: %d, channels: %d, samplesread: %d, SamplePer: %d\n",sampleRate, NumChannels, samplesRead, (int)SamplesPerPacket);
    
    //Stopping any previous streams
    streamStop(hDevice);

    if (NumChannels > 0 ) {
        streamData(hDevice,shmem, &caliInfo, sem);
    }
    else {
        while (!exit_flag) {
            while (usr1_flag == 0 && exit_flag == 0) {
                sigsuspend(&newMask);
            }
            // Block signals during flag check
            sigprocmask(SIG_BLOCK, &newMask, NULL);
            if (usr1_flag) {
                usr1_flag = 0;
                updateIO(hDevice, sem, shmem);
            }
            // Unblock signals after flag check
            sigprocmask(SIG_UNBLOCK, &newMask, NULL);
        }
    }

close:
    closeUSBConnection(hDevice);

done:

    deAllocAndFreeSharedMem(shmem, sh_seq_id);
    sem_unlink(SHMEMNAME);
    // restore the original mask
    sigprocmask(SIG_SETMASK, &oldMask, NULL);
    return 0;
}


int streamData(HANDLE hDevice, struct shmem *shmem, u3CalibrationInfo *caliInfo, sem_t *sem)
{
    int i;
    double dvolt;
    unsigned int *data = NULL;

    data = malloc((NumChannels * samplesRead * 5)*sizeof(data));
    //printf("mem: %d\n",NumChannels * samplesRead*sizeof(data));
    if (data == NULL) {
        PRINTERR("Error: memory allocation\n");
        goto error;
    }
    //Stopping any previous streams
    streamStop(hDevice);

    if(streamConfig(hDevice) != 0)
        goto error;


    if(streamStart(hDevice) != 0)
        goto error;
    
    // Iinit buffer
    init_buf(&shmem->ch_data[0]);
    init_buf(&shmem->ch_data[1]);
    init_buf(&shmem->ch_data[2]);
    init_buf(&shmem->ch_data[3]);
    
    while(!exit_flag && !streamRead(hDevice, sem, shmem, caliInfo, data, samplesRead)) {
        for (i = 0; i < samplesRead; i++) {
            getAinVoltCalibrated_hw130(caliInfo, (i % NumChannels), 30, data[i], &(dvolt));
            dvolt = (dvolt*1000)+2455;
            if (dvolt > 3450) {
                dvolt = 0xffff;
            }
            sem_wait(sem);
            if (add_to_buf(&shmem->ch_data[i % NumChannels],(int)(dvolt)) < 0) {
                //printf("buf:err\n");
            }
            sem_post(sem);

            // Chcek IO changes
            updateIO(hDevice, sem, shmem);
        }
    }
    streamStop(hDevice);

    if (data != NULL) {
        free(data);
    }
    return 0;
error:
    return -1;
}


int readSensorConfig(struct shmem *shmem)
{
    int i;
    for (i = 0; i < 4; i++) {
        if (shmem->sensor[i].type != NONE) {
            NumChannels++;
        }
    }
    return 0;
}

int updateIO(HANDLE hDevice, sem_t *sem, struct shmem *shmem)
{
    long error;

    sem_wait(sem);
    if (shmem->ioctrl.changeIO) {
        // USB_DATA switches are inverted
        shmem->ioctrl.iostate ^= ( (1 << USB_DATA_1) | (1 << USB_DATA_2) );
         
        if((error = maskDO(hDevice, 0xffffff,shmem->ioctrl.iostate)) != 0) {
            PRINTERR("Error setting IO: %d\n",(int)error);
        }
        PRINTOUT("IO set %X\n",shmem->ioctrl.iostate);
        shmem->ch_data[0].io_state = (shmem->ioctrl.iostate & (1 << AN0_IO)) >> 8;
        shmem->ch_data[1].io_state = (shmem->ioctrl.iostate & (1 << AN1_IO)) >> 9;
        shmem->ch_data[2].io_state = (shmem->ioctrl.iostate & (1 << AN2_IO)) >> 10;
        shmem->ch_data[3].io_state = (shmem->ioctrl.iostate & (1 << AN3_IO)) >> 11;

        shmem->ioctrl.changeIO = 0;
    }
    sem_post(sem);
    return 0;
}

long maskDO(HANDLE Handle, long wmask, long smask)
{
    uint8 sendDataBuff[7];
    uint8 Errorcode, ErrorFrame;
    //uint8 curFIOAnalog, curEIOAnalog, curTCConfig;
    //long error;

    /* Setting up Feedback command to set digital Channel to output and to set the state */
    sendDataBuff[0] = 27;             //PortStateWrite
    sendDataBuff[1] = (wmask & 0xff);               //IONumber 0-7
    sendDataBuff[2] = (wmask & (0xff << 8)) >> 8;     //IONumber 8-15
    sendDataBuff[3] = (wmask & (0xff << 16)) >> 16;   //IONumber 16-23

    sendDataBuff[4] = (smask & 0xff);               //IONumber 0-7
    sendDataBuff[5] = (smask & (0xff << 8)) >> 8;     //IONumber 8-15
    sendDataBuff[6] = (smask & (0xff << 16)) >> 16;   //IONumber 16-23
    //printf("%X,%X\n",wmask,smask);

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

    sendBuff[10] = 0x0f;   //FIOAnalog : setting all FIOs as analog inputs
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

    if( recBuff[10] != 255 && recBuff[10] != (uint8)(0x0f))
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
int streamConfig(HANDLE hDevice)
{
    int sendBuffSize;
    sendBuffSize = 12+NumChannels*2;
    uint8 sendBuff[sendBuffSize], recBuff[8];
    int sendChars, recChars;
    uint16 checksumTotal;
    uint16 scanInterval;
    int i;

    sendBuff[1] = (uint8)(0xF8);    //command byte
    sendBuff[2] = 3 + NumChannels;  //number of data words = NumChannels + 3
    sendBuff[3] = (uint8)(0x11);    //extended command number
    sendBuff[6] = NumChannels;      //NumChannels
    sendBuff[7] = SamplesPerPacket; //SamplesPerPacket
    sendBuff[8] = 0;  //Reserved

    if (sampleRate < 1000) {
        sendBuff[9] = 0x0c; // clock divided by 256
        scanInterval = 48000000/256/sampleRate;
    }
    else {
        sendBuff[9] = 0x0a;  //ScanConfig:
                          // Bit 7: Reserved
                          // Bit 6: Reserved
                          // Bit 3: Internal stream clock frequency = b0: 4 MHz
                          // Bit 2: Divide Clock by 256 = b0
                          // Bits 0-1: Resolution = b01: 11.9-bit effective
        scanInterval = 48000000/sampleRate;
    }

    sendBuff[10] = (uint8)(scanInterval&(0x00FF));  //scan interval (low byte)
    sendBuff[11] = (uint8)(scanInterval >> 8);       //scan interval (high byte)

    for(i = 0; i < NumChannels; i++)
    {
        printf("%d\n",NumChannels);
        sendBuff[12 + i*2] = ch_table[i];  //PChannel = i
        sendBuff[13 + i*2] = 30; //NChannel = 31: Single Ended
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

int checkStreamResponse(uint8 *recBuff, int recBuffSize)
{
    uint16 checksumTotal;
    static int autoRecoveryOn = 1;
    static int packetCounter = 0;

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

    if( recBuff[1] != (uint8)(0xF9) || recBuff[2] != 4 + SamplesPerPacket || recBuff[3] != (uint8)(0xC0) )
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
int streamRead(HANDLE hDevice, sem_t *sem, struct shmem *shmem, u3CalibrationInfo *caliInfo, unsigned int *data, long data_size)
{
    long recBuffSize, readn,i, scanNumber = 0;
    recBuffSize = 14 + SamplesPerPacket*2;
    int recChars = 0, backLog;
    int k, currChannel = 0,p;
    //uint16 voltageBytes;
    long startTime, endTime;

    int readSizeMultiplier = 5;  //Multiplier for the StreamData receive buffer size
    int responseSize = (14 + SamplesPerPacket*2) * readSizeMultiplier;        //The number of bytes in a StreamData response (differs with SamplesPerPacket)

    uint8 recBuff[responseSize];

    startTime = getTickCount();
    readn = data_size / SamplesPerPacket;
    for (i = 0; i < readn; i++) 
    {
        //Reading stream response from U3
        recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP3_IN, recBuff, responseSize);

        if(recChars < responseSize)
        {
            PRINTERR("Error : read failed, expected %d bytes but received %d (StreamData).\n",responseSize, recChars);
            return -1;
        }
        // Update IO pins
        updateIO(hDevice, sem, shmem);

        //printf("%d, %d, %d\n",backLog, recChars, responseSize);
        for (p = 0; p < readSizeMultiplier; p++) {
            checkStreamResponse(&recBuff[p*recBuffSize], recBuffSize);
            backLog = (int)recBuff[12 + SamplesPerPacket*2 + (p*recBuffSize)];
            if (backLog > 100) {
                PRINTERR("Buffer>100\n");
            }
            for(k = 12; k < (12 + SamplesPerPacket*2); k += 2)
            {
                data[currChannel + scanNumber*NumChannels] = (uint16)recBuff[k+(p*recBuffSize)] + ((uint16)recBuff[k+1+(p*recBuffSize)] << 8);
    
                currChannel++;
                if(currChannel >= NumChannels)
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
        return -1;
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/* End of file */
