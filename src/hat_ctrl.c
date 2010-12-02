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

sem_t *sem;

struct io_cmd {
    char *cmd_str;
    unsigned int bit;
    unsigned int state;
};
                         
volatile sig_atomic_t prog_exit = 0;

/* ------------------------------------------------------------------------- */
/* LOCAL CONSTANTS AND MACROS */
const char usage[] = {
               "Usage: hat_ctrl [CONTROL]...\n"
               " controls:\n"
               "  -usb1pwr=[on|off]        Switch usb 1 power line on or off\n"
               "  -usb2pwr=[on|off]        Switch usb 2 power line on or off\n"
               "  -usb1data=[on|off]       Switch usb 1 data lines on or off\n"
               "  -usb2data=[on|off]       Switch usb 2 data lines on or off\n"
               "  -pwr1=[on|off]           Switch power 1 output on or off\n"
               "  -pwr2=[on|off]           Switch power 2 output on or off\n"};

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
                                     {"-sio4=off",       AN3_IO,          0}};

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */

void ex_program(int sig) {
 prog_exit = 1;
}

int setDigitalIO(struct shmem *shmem, int num, int state)
{
    sem_wait(sem);
    if (state != 0) {
        shmem->ioctrl.iostate |= (1 << num);  
    }
    else {
        shmem->ioctrl.iostate &= ~(1 << num);
    }
    shmem->ioctrl.changeIO = 1;
    sem_post(sem);
    return 0;
}

int setDigitalIOMask(struct shmem *shmem, long io_mask)
{
    sem_wait(sem);
    shmem->ioctrl.iostate = io_mask;  
    shmem->ioctrl.changeIO = 1;
    kill(shmem->hat_drv_pid, SIGUSR1);
    sem_post(sem);
    return 0;
}

int convert_and_print(struct shmem *shmem, int mode, unsigned int value, int ch)
{
    if (value == 0xffff) {
        PRINTOUT2("Over!  ");
    }
    else {
        switch(mode) {
        case 0:
            PRINTOUT2("%d  ",value);
            break;
        case 1:
            if (shmem->ch_data[ch].io_state == 1) {
                PRINTOUT2("%4.1f  ",(double)(value)*0.392);
            }
            else {
                PRINTOUT2("%4.2f  ",(double)(value)*0.030);
            }
            break;
        case 2:
            if (shmem->ch_data[ch].io_state == 1) {
                PRINTOUT2("%1.2f  ",(double)(value)*0.392/1000);
            }
            else {
                PRINTOUT2("%1.2f  ",(double)(value)*0.030/1000);
            }
            break;
        }
    }
    return 0;
}

struct ch {
    int num;
    int mode;
};

/* ------------------------------------------------------------------------- */
/** Main
* @param
*/
int main(int argc, char **argv)
{
    struct shmem *shmem = NULL;
    int sh_seg_id = 0, i, k = 0, ch = 0, ret = 0, ch_count = 0;
    unsigned int data[5][1000] = {{0},{0}};
    char *endptr;
    long bit = -1,state = 0;
    unsigned int x, ok_param = 0;
    long io_mask = -1;
    struct ch chs[5];

    (void) signal(SIGINT, ex_program);

    if (argc > 1) {
        for (i = 1; i < argc; i++) {
            if (!strncmp(argv[i], "-d", 2)) {
                bit = strtoul(argv[i]+2 ,&endptr, 10);
                if (endptr[0] == ':') {
                    state = strtoul(endptr+1 ,&endptr, 10);
                    PRINTOUT2("%d:%d\n",(int)bit, (int)state);
                }                
            }
            else if (!strncmp(argv[i], "-i", 2)) {
                chs[ch_count].num = strtoul(argv[i]+2 ,&endptr, 10);
                ok_param = 1;
                if (!strcmp(endptr,":mA")) {
                    chs[ch_count].mode = 1;
                }
                else if (!strcmp(endptr,":A")) {
                    chs[ch_count].mode = 2;
                }
                else {
                    chs[ch_count].mode = 0;
                }
                ch_count++;
            }
            else {
                for (k = 0; k < sizeof(io_command)/sizeof(struct io_cmd); k++) {
                    if (!strcmp(argv[i], io_command[k].cmd_str)) {
                        ok_param = 1;
                        if (io_mask == -1) {
                            io_mask = 0;
                        }
                        if (io_command[k].state) {
                            io_mask |= (1 << io_command[k].bit);
                        }
                        else {
                            io_mask &= ~(1 << io_command[k].bit);
                        }
                        break;
                    }
                }
            }
        }
    }

    if (!ok_param) {
        PRINTOUT2("%s",usage);
        return -1;
    }

    sem = sem_open(SHMEMNAME,0,0644,0);

    if(sem == SEM_FAILED)
    {
      PRINTERR("Error: HAT driver is not started.\n");
      sem_close(sem);
      return -1;
    }

    getShareMem(&shmem, &sh_seg_id);

    if (bit >= 0) {
        setDigitalIO(shmem, bit, state);
    }
    else if (io_mask >= 0) {
        setDigitalIOMask(shmem, io_mask);
    }
    else {
        initBuf(&shmem->ch_data[0]);
        initBuf(&shmem->ch_data[1]);
        initBuf(&shmem->ch_data[2]);
        initBuf(&shmem->ch_data[3]);

        while (!prog_exit) {
            i = 0;
            for (ch = 0; ch < ch_count; ch++) {
                sem_wait(sem);
                ret = getFromBuf(&shmem->ch_data[chs[ch].num], &x);
                sem_post(sem);
                i = 0;
                while (ret == 0) {
                    data[ch][i++] = x;
                    sem_wait(sem);
                    ret = getFromBuf(&shmem->ch_data[chs[ch].num], &x);
                    sem_post(sem);
                }
            }
             
            for (k = 0; k < i; k++) {
                for (ch = 0; ch < ch_count; ch++) {
                    convert_and_print(shmem, chs[ch].mode, data[ch][k], ch);
                }
                PRINTOUT2("\n");
            }
        }
    }

    deAllocSharedMem(shmem);
    return 0;
}

/* ------------------------------------------------------------------------- */
/* End of file */


