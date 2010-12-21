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
#include "u3.h"
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <signal.h>
#include <sys/shm.h>
#include "shmemlib.h"
#include "hat_drv.h"

#define ANS_WAIT_MS 1000

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
#define APP_NAME "hat_ctrl"
#define PRINTOUT(...) printf(APP_NAME ": " __VA_ARGS__)
#define PRINTOUT2(...) printf(__VA_ARGS__)
#define PRINTERR(...) fprintf(stderr, APP_NAME ": " __VA_ARGS__)

/* LOCAL GLOBAL VARIABLES */

volatile sig_atomic_t prog_exit = 0, wait_ret = 0;

sem_t *semshmem;
sem_t *semcomm;

struct ch {
    int num;
    int mode;
};

struct io_cmd {
    char *cmd_str;
    unsigned int bit;
    unsigned int state;
};
                         
/* ------------------------------------------------------------------------- */
/* LOCAL CONSTANTS AND MACROS */
const char usage[] = {
               "Usage: hat_ctrl CONTROL...\n"
               "   or: hat_ctrl COMMAND\n\n"
               " Controls:\n"
               "  -usb1pwr=[on|off]        Switch usb 1 power line on or off\n"
               "  -usb2pwr=[on|off]        Switch usb 2 power line on or off\n"
               "  -usb1data=[on|off]       Switch usb 1 data lines on or off\n"
               "  -usb2data=[on|off]       Switch usb 2 data lines on or off\n"
               "  -pwr1=[on|off]           Switch power 1 output on or off\n"
               "  -pwr2=[on|off]           Switch power 2 output on or off\n"
               "  -sio1=[on|off]           Set sensor input 1 IO line\n"
               "  -sio2=[on|off]           Set sensor input 2 IO line\n"
               "  -sio3=[on|off]           Set sensor input 3 IO line\n"
               "  -sio4=[on|off]           Set sensor input 4 IO line\n"
               " Commands:\n"
               "  -ss                      Display switches status\n"
               "  --help                   Display this help\n"
               "  -alloff                  Set all switches off state\n"          
               "  -allon                   Set all switches on state\n"
               "  -stream:0                Stop data streaming\n"
               "  -stream [OPTION...]      Start streaming\n"
               "   Options for stream command:\n"
               "    -sr SAMLERATE            Streaming samplerate [3-10000]. Default 100Hz\n"
               "    -sc SAMPLES              Sample count [1-2^32]. Default 10\n"
               "    -s INPUT-TYPE            Sensor input and type [INPUT-TYPE]\n"
               "                             Types: 1 = None (voltage)\n"
               "                                    2 = Current\n"
               "                             Default 1-1\n"
               "    -f FILENAME              Store streamed data to file\n\n"
               "  -stream:SAMPLERATE:sINPUT-TYPE[:sINPUT-TYPE]...:[fFILENAME]:SAMPLES\n"
               "    Starts data streaming and configures used sensors.\n\n" 
               "  Example to stream from current sensor 1 at 100Hz and 1000 samples:\n\n"
               "      hat_ctrl -stream:100:s1-2:1000\n"
               "  or: \n"
               "      hat_ctrl -stream -sr 100 -s 1-2 -sc 1000\n\n"
               "  or same to file:\n\n"
               "      hat_ctrl -stream:100:s1-2:fmydata:1000\n"
               "  or: \n"
               "      hat_ctrl -stream -sr 100 -s 1-2 -sc 1000 -f mydata\n"};

const struct io_cmd io_command[] =  {{"-usb1pwr=on",     USB_PWR_1,       1},
                                     {"-usb1pwr=off",    USB_PWR_1,       0},
                                     {"-usb2pwr=on",     USB_PWR_2,       1},
                                     {"-usb2pwr=off",    USB_PWR_2,       0},
                                     {"-usb1data=on",    USB_DATA_1,      1},
                                     {"-usb1data=off",   USB_DATA_1,      0},
                                     {"-usb2data=on",    USB_DATA_2,      1},
                                     {"-usb2data=off",   USB_DATA_2,      0},
                                     {"-pwr1=on",        DC_POWER_SHDN1,  1},
                                     {"-pwr1=off",       DC_POWER_SHDN1,  0},
                                     {"-pwr2=on",        DC_POWER_SHDN2,  1},
                                     {"-pwr2=off",       DC_POWER_SHDN2,  0},
                                     {"-sio1=on",        AN0_IO,          1},
                                     {"-sio1=off",       AN0_IO,          0},
                                     {"-sio2=on",        AN1_IO,          1},
                                     {"-sio2=off",       AN1_IO,          0},
                                     {"-sio3=on",        AN2_IO,          1},
                                     {"-sio3=off",       AN2_IO,          0},
                                     {"-sio4=on",        AN3_IO,          1},
                                     {"-sio4=off",       AN3_IO,          0},
                                     {"\n",              0,               0}};

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */

void ex_program(int sig) {
 prog_exit = 1;
}

void ret_received(int sig) {
 wait_ret = 0;
}

long getTickCount()
{
    struct timeval tv;

    gettimeofday(&tv, NULL);

    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

int sendCmdToDrv(struct shmem *shmem, unsigned int cmd)
{
    long starttime;
    int ret;
    sem_wait(semcomm);
    shmem->hat_ctrl_pid = getpid();
    wait_ret = 1;
    shmem->cmd = cmd;
    kill(shmem->hat_drv_pid, SIGUSR1);
    starttime = getTickCount();
    while (wait_ret && (starttime + ANS_WAIT_MS) > getTickCount());
    if (wait_ret) {
        PRINTERR("Command replay time out error\n");
        ret = -1;
    }
    else {
        ret = shmem->retValToCtrl;
    }
    sem_post(semcomm);
    return ret;
}
int setDigitalIO(struct shmem *shmem, int num, int state)
{
    sem_wait(semshmem);
    if (state != 0) {
        shmem->ioctrl.iostate |= (1 << num);  
    }
    else {
        shmem->ioctrl.iostate &= ~(1 << num);
    }
    sem_post(semshmem);

    return sendCmdToDrv(shmem, CHANGE_IO);
}

int setDigitalIOMask(struct shmem *shmem, long io_mask)
{
    sem_wait(semshmem);
    shmem->ioctrl.iostate = io_mask;  
    sem_post(semshmem);
    return sendCmdToDrv(shmem, CHANGE_IO);;
}

int convert_and_print(struct streamConfig *streamConf, int value, int ch)
{
    if (value == 0xffff) {
        PRINTOUT2("Over!  ");
    }
    else {
        //printf("ch:%d  %d",ch,streamConf->sensor[ch].type);
        switch(streamConf->sensor[ch].type) {
        case VOLTAGE:
            PRINTOUT2("%d mV  ",value);
            break;
        case CURRENT:
            if (streamConf->sensor[ch].io_state == 0) {
                PRINTOUT2("%4.1f mA  ",(float)(value)*LO_GAIN_MULT);
            }
            else {
                PRINTOUT2("%4.2f mA  ",(double)(value)*HI_GAIN_MULT);
            }
            break;
        }
    }
    return 0;
}

void showSwitchStatus(struct shmem *shmem) 
{
    int k,i;
    for (i = 0; i < 20; i++) {
        k = 0;
        do {
            if (i == io_command[k].bit && ( ((shmem->ioctrl.iostate & (1 << i)) >> i) == io_command[k].state)) {
                 PRINTOUT2("%s\n",io_command[k].cmd_str);
            }
        } while ( strcmp(io_command[++k].cmd_str, "\n") );
    }
}


int readAndShowData(struct shmem *shmem)
{
    int sample_count = 0, i, k, x, ch, ret = 0;
    int data[5][1000] = {{0},{0}};

    while (1) {
        i = 0;
        for (ch = 0; ch < shmem->streamConfig.sensors; ch++) {
            sem_wait(semshmem);
            ret = getFromBuf(&shmem->ch_data[ch], &x);
            sem_post(semshmem);
            i = 0;
            while (ret == 0) {
                data[ch][i++] = x;
                sem_wait(semshmem);
                ret = getFromBuf(&shmem->ch_data[ch], &x);
                sem_post(semshmem);
            }
        }
         
        for (k = 0; k < i; k++) {
            sample_count++;
            for (ch = 0; ch < shmem->streamConfig.sensors; ch++) {
                convert_and_print(&shmem->streamConfig, data[ch][k], ch);
            }
            PRINTOUT2("\n");
        }
        sem_wait(semshmem);
        for (ch = 0; ch < shmem->streamConfig.sensors; ch++) {
            i += getBufSize(&shmem->ch_data[ch]);
        }
        sem_post(semshmem);
        if (prog_exit || (shmem->streamConfig.samples != 0 && sample_count >= shmem->streamConfig.samples)) {
            break;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/** Main
* @param
*/
int main(int argc, char **argv)
{
    struct shmem *shmem = NULL;
    int sh_seg_id = 0, i = 0, k = 0, ret = 0, param_error = -1, val = 0, val2 = 0, show_data = 0, stream = 0;
    char *endptr;
    unsigned long new_iostate = 0;
    struct streamConfig streamConfig;

    memset(&streamConfig,0,sizeof(struct streamConfig));

    streamConfig.samplerate = 100;
    streamConfig.samples = 10;

    (void) signal(SIGINT, ex_program);
    (void) signal(SIGUSR1, ret_received);

    if (argc == 2 && (!strcmp(argv[1], "-help") || !strcmp(argv[1], "--help"))) {
        PRINTOUT2("%s",usage);
        ret = 0;
        goto exit;
    }

    semshmem = sem_open(SHMEMNAME,0,0644,0);

    if(semshmem == SEM_FAILED) {
      PRINTERR("Error: HAT driver is not started.\n");
      ret = -1;
      goto exit;
    }

    semcomm = sem_open(COMMSEM,0,0644,0);

    if(semcomm == SEM_FAILED) {
      PRINTERR("Error: HAT driver is not started.\n");
      ret = -1;
      goto exit;
    }

    // Allocate shared memory
    if ( getShareMem(&shmem, &sh_seg_id) ) {
        ret = -1;
        goto exit;
    }
    // Check parameters
    if (argc > 1) {
        param_error = 0;
        new_iostate = shmem->ioctrl.iostate;

        for (i = 1; i < argc; i++) {
            if (!strcmp(argv[i],"-s") && argc > (i+1)) {
                val = strtoul(argv[++i] ,&endptr, 10);
                if (val > 4 || val < 1 ) {
                    param_error = 1;
                    break;
                }
                if ( *endptr == '\0') {
                    val2 = 1;
                }
                else {
                    val2 = strtoul(&endptr[1] ,&endptr, 10);
                    if ( val2 >= MAX_SENSOR || val2 < 1 ) {
                        param_error = 1;
                        break;
                    }
                }
                if (streamConfig.sensor[val-1].type) {
                    param_error = 1;
                    break;
                }
                streamConfig.sensor[val-1].type = val2;
            }
            else if (!strcmp(argv[i], "-sr") && argc > (i+1)) {
                val = strtoul(argv[++i] ,&endptr, 10);
                if ( *endptr != '\0' || (val < 3) || (val > 10000) ) {
                    param_error = 1;
                    break;
                }
                else {
                    streamConfig.samplerate = (unsigned int)val;
                }
            }
            else if (!strcmp(argv[i], "-sc") && argc > (i+1)) {
                val = strtoul(argv[++i] ,&endptr, 10);
                if ( *endptr != '\0' || (val < 1) ) {
                    param_error = 1;
                    break;
                }
                else {
                    streamConfig.samples = (unsigned int)val;
                }
            }
            else if (!strcmp(argv[i], "-f") && argc > (i+1)) {
                strcpy(streamConfig.pathFilename, argv[++i]);
                //streamConfig.pathFilename[strlen(argv[i])] = '\0';
            }
            else if (!strcmp(argv[i], "-stream:0") && (argc == 2)) {
                sendCmdToDrv(shmem,STOP_STREAMING);
                goto exit;
            }
            else if (!strcmp(argv[i], "-stream")) {
               stream = 1;
            }
            else if (!strncmp(argv[i], "-stream:",8)) {
                val = strtoul(argv[i]+8 ,&endptr, 10);
                if ( (val < 1) || (val > 10000) ) {
                    param_error = 1;
                    break;
                }
                else {
                    streamConfig.samplerate = (unsigned int)val;
                    show_data = 1;
                }
                if (*endptr == ':') {
                    do {
                        if (strncmp(endptr, ":s",2)) {
                            param_error = 1;
                            break;
                        }
        
                        val = strtoul(&endptr[2] ,&endptr, 10);
                        if ( (*endptr != '-') || val > 4 || val < 1 ) {
                            param_error = 1;
                            break;
                        }
                        val2 = strtoul(&endptr[1] ,&endptr, 10);
                        if ( val2 >= MAX_SENSOR || val2 < 1 ) {
                            param_error = 1;
                            break;
                        }
                        streamConfig.sensor[val-1].type = val2;
                        streamConfig.sensors++;    
                    } while (!strncmp(endptr,":s",2));
                    
                    strcpy(streamConfig.pathFilename,"");
    
                    if (!strncmp(endptr,":f",2)) {
                        endptr+=2;
                        strncpy(streamConfig.pathFilename, endptr, strcspn(endptr, ":"));
                        streamConfig.pathFilename[strcspn(endptr, ":")] = '\0';
                        PRINTOUT("Streaming to: %s\n",streamConfig.pathFilename);
                        endptr = &endptr[strlen(streamConfig.pathFilename)];
                        show_data = 0;
                    }
                    if (!strncmp(endptr,":",1)) {
                        val = strtoul(&endptr[1] ,&endptr, 10);
                        if (*endptr != 0 || val < 0) {
                            param_error = 1;
                            break;
                        }
                        streamConfig.samples = val;
                    }
                    else if (*endptr == 0) {
                        show_data = 1;
                    }
                    else {
                        param_error = 1;
                        break;
                    }
                    stream = 1;
                }
            }
            else if (!strcmp(argv[i], "-ss") && (argc == 2)) {
                showSwitchStatus(shmem);
                ret = 0;
                goto exit;
            }
            else if (!strcmp(argv[i], "-alloff") && (argc == 2)) {
                PRINTOUT2("Setting all ports off.\n");
                new_iostate &= ~PWR_USB_IOS;
            }
            else if (!strcmp(argv[i], "-allon") && (argc == 2)) {
                PRINTOUT2("Setting all ports on.\n");
                new_iostate |= PWR_USB_IOS;
            }
            else if (argc > i) {
                k = 0;
                do {
                    if (!strcmp(argv[i], io_command[k].cmd_str)) {
                        if (io_command[k].state) {
                            new_iostate |= (1 << io_command[k].bit);
                        }
                        else {
                            new_iostate &= ~(1 << io_command[k].bit);
                        }
                        break;
                    }
                } while (strcmp(io_command[++k].cmd_str, "\n"));

                if ( !strcmp(io_command[k].cmd_str, "\n") ) {
                    param_error = 1;
                    break;
                }
            }
        }
    }

    if (param_error)  {
        PRINTERR("Parameter error '%s'\n",argv[i]);
        PRINTOUT2("Try 'hat_ctrl -help' for more  information\n");
        ret = -1;
        goto exit;
    }

    // Set IO pins (=switches)
    ret = setDigitalIOMask(shmem, new_iostate);

    if (stream) {
        if (shmem->streamConfig.hat_ctrl_pid != 0) {
            PRINTERR("Error only one process can stream data. Stop previous streaming first.\n");
            ret = -1;
            goto exit;
        }
        streamConfig.sensors = 0;
        for (k = 0; k < 4; k++) {
            if (streamConfig.sensor[k].type) {
                streamConfig.sensors++;
            }
        }
        if (!streamConfig.sensors) {
            // Set default setting
            streamConfig.sensor[0].type = 1;
            streamConfig.sensors = 1;
        }
        sem_wait(semshmem);
        memcpy(&shmem->streamConfig,&streamConfig,sizeof(struct streamConfig));
        sem_post(semshmem);
        
        sendCmdToDrv(shmem, STOP_STREAMING);
        // If data is shown, store this process id to shared memmory             
        if (streamConfig.pathFilename[0] == 0) {
            sem_wait(semshmem);
            shmem->streamConfig.hat_ctrl_pid = getpid();
            sem_post(semshmem);
        }
        sendCmdToDrv(shmem, START_STREAMING);
    
        if ( streamConfig.pathFilename[0] == 0 ) {
            ret = readAndShowData(shmem);
            sem_wait(semshmem);
            shmem->streamConfig.hat_ctrl_pid = 0;
            sem_post(semshmem);
            sendCmdToDrv(shmem,STOP_STREAMING);
        }
    }
exit:
    sem_close(semshmem);
    sem_close(semcomm);
    deAllocSharedMem(shmem);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* End of file */


