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
#include <sys/stat.h>
#include <signal.h>
#include <sys/shm.h>
#include <stdint.h>
#include "shmemlib.h"
#include "hat_drv.h"
#include "hat_ctrl_lib.h"

/* ------------------------------------------------------------------------- */
/* EXTERNAL FUNCTION PROTOTYPES */
/* None */
	
/* ------------------------------------------------------------------------- */
/* GLOBAL VARIABLES */
/* None */
	
/* ------------------------------------------------------------------------- */
/* CONSTANTS */
#define APP_NAME "hat_ctrl"	
	
/* ------------------------------------------------------------------------- */
/* MACROS */
#define PRINTOUT(...) printf(APP_NAME ": " __VA_ARGS__)
#define PRINTOUT2(...) printf(__VA_ARGS__)
#define PRINTERR(...) fprintf(stderr, APP_NAME ": " __VA_ARGS__)

/* ------------------------------------------------------------------------- */
/* LOCAL GLOBAL VARIABLES */
volatile sig_atomic_t prog_exit = 0;

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
               "  -usb1chr=[on|off]        Switch usb 1 charger resistor (overrides usb data)\n"
               "  -usb2chr=[on|off]        Switch usb 2 charger resistor (overrides usb data)\n"
               "  -pwr1=[on|off]           Switch power 1 output on or off\n"
               "  -pwr2=[on|off]           Switch power 2 output on or off\n"
               "  -sio1=[on|off|hiz]       Set sensor 1 digital line\n"
               "  -sio2=[on|off|hiz]       Set sensor 2 digital line\n"
               "  -sio3=[on|off|hiz]       Set sensor 3 digital line\n"
               "  -sio4=[on|off|hiz]       Set sensor 4 digital line\n"
               "  -all=off                 Set all switches off state (Not sensors)\n"          
               "  -all=on                  Set all switches on state (Not sensors)\n"
               " Commands:\n"
               "  -c, --config_file FILE   Sensors configuration file\n"
               "  -ss                      Display switches status\n"
			   "  -sn SERIALNUMBER         Connect to specific HAT device\n"
			   "                           If not given, connecting to first HAT\n"
               "  --help                   Display this help\n"
               "  -stream:0                Stop data streaming\n"
               " Options for streaming:\n"
               "  -sr SAMLERATE            Streaming samplerate [3-10000]. Default 100Hz\n"
               "  -sc SAMPLES              Sample count [0-2^32]. 0=continuous. Default 10\n"
               "  -s INPUT-TYPE            Sensor input and type [INPUT-TYPE]\n"
               "                             Types: 1 = None (voltage)\n"
               "                                    2 = Current\n"
               "                                    3 = Temperature\n"
               "                                    4 = Audio\n"
               "                                    5 = Optical3\n"
               "                                    6 = Optical\n"
               "                                    7 = Acceleration\n"
               "                             Default 1-1\n"
               "  -f FILENAME              Store streamed data to file\n\n"
               "  -stream:SAMPLERATE:sINPUT-TYPE[:sINPUT-TYPE]...:[fFILENAME]:SAMPLES\n"
               "    Starts data streaming and configures used sensors.\n\n" 
               "  Example to stream from current sensor 1 at 100Hz and 1000 samples:\n\n"
               "      hat_ctrl -stream:100:s1-2:1000\n"
               "  or: \n"
               "      hat_ctrl -stream -sr 100 -s 1-2 -sc 1000\n\n"
               "  or same to file:\n\n"
               "      hat_ctrl -stream:100:s1-2:fmydata:1000\n"
               "  or: \n"
               "      hat_ctrl -sr 100 -s 1-2 -sc 1000 -f mydata\n"};

const struct io_cmd io_command[] =  {{"-usb1pwr=on",     USB_PWR_1,       ON},
                                     {"-usb1pwr=off",    USB_PWR_1,       OFF},
                                     {"-usb2pwr=on",     USB_PWR_2,       ON},
                                     {"-usb2pwr=off",    USB_PWR_2,       OFF},
                                     {"-usb1data=on",    USB_DATA_1,      ON},
                                     {"-usb1data=off",   USB_DATA_1,      OFF},
                                     {"-usb2data=on",    USB_DATA_2,      ON},
                                     {"-usb2data=off",   USB_DATA_2,      OFF},
                                     {"-usb1chr=on",     USB_CHR_1,       ON},
                                     {"-usb1chr=off",    USB_CHR_1,       OFF},
                                     {"-usb2chr=on",     USB_CHR_2,       ON},
                                     {"-usb2chr=off",    USB_CHR_2,       OFF},
                                     {"-pwr1=on",        DC_POWER_SHDN1,  ON},
                                     {"-pwr1=off",       DC_POWER_SHDN1,  OFF},
                                     {"-pwr2=on",        DC_POWER_SHDN2,  ON},
                                     {"-pwr2=off",       DC_POWER_SHDN2,  OFF},
                                     {"-sio1=on",        AN0_IO,          ON},
                                     {"-sio1=off",       AN0_IO,          OFF},
                                     {"-sio1=hiz",       AN0_IO,          HIZ},
                                     {"-sio2=on",        AN1_IO,          ON},
                                     {"-sio2=off",       AN1_IO,          OFF},
                                     {"-sio2=hiz",       AN1_IO,          HIZ},
                                     {"-sio3=on",        AN2_IO,          ON},
                                     {"-sio3=off",       AN2_IO,          OFF},
                                     {"-sio3=hiz",       AN2_IO,          HIZ},
                                     {"-sio4=on",        AN3_IO,          ON},
                                     {"-sio4=off",       AN3_IO,          OFF},
                                     {"-sio4=hiz",       AN3_IO,          HIZ},
                                     {"-all=on",         0,               ALLON},
                                     {"-all=off",        0,               ALLOFF},
                                     {"\n",              0,               0}};

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */

void ex_program(int sig) {
 prog_exit = 1;
}

void showSwitchStatus(struct hatCtrl *hatCtrl) 
{
    int k,i;
    for (i = 0; i < 20; i++) {
        k = 0;
        do {
            if (i == io_command[k].bit && ( ((HATgetSwitchStatus(hatCtrl) & (1 << i)) >> i) == io_command[k].state)) {
                 PRINTOUT2("%s\n",io_command[k].cmd_str);
            }
        } while ( strcmp(io_command[++k].cmd_str, "\n") );
    }
}

/*int readOpticalSensor(struct hatCtrl *hatCtrl, int count)
{
    int i;

    for (i = 0; i < 3; i++) {
        if (!HATstartDataStream(&hatCtrl,&new_streamConfig)) {

        }
    }
    // Start sensor(s) data streaming if needed.    
    if (!HATstartDataStream(&hatCtrl,&new_streamConfig)) {
        sample_count = HATsamplesToRead(&hatCtrl);
        if (sample_count == 0) {
            sample_count = INT32_MAX;
        }
        do {
            for (k = 0; k < HATsensors(&hatCtrl); k++) {
                // Read one 1 value from specified channel
                if ( (r = HATreadDataCh(&hatCtrl,k,data[k],1,data_str[k],&overfull)) > 0) {
                    sample_count -= r;
                }
                else {
                    PRINTERR("\nRead error!\n");
                    ret = -1;
                    break;
                }
            }

            for (k = 0; k < HATsensors(&hatCtrl); k++) {
                //printf("%5.1f %s  ",HATOpticalXYZtosRGB(data[0][0], data[1][0], data[2][0],k,255),data_str[k]);                
                PRINTOUT2("%5.2f %s  ",data[k][0],data_str[k]);                
            }

            // Use function for read data. Function is specified in sensor config file
            if (!HATsensorDataFunction(&hatCtrl, 0, &value, str, &error)) {
                PRINTOUT2("-> %s %.2f",str, value);
            }
            PRINTOUT2("\n");
            fflush(NULL);

            if (overfull) {
                PRINTERR("Buffer overrun!\n");
                //ret = -1;
                //break;
            }
            if (prog_exit) {
                break;
            }
        } while (sample_count > 0);

        if (!HATsensorDataFunction(&hatCtrl, 0, &value, str, &error)) {
            PRINTOUT2("-> %s %.2f",str, value);
        }
        PRINTOUT2("\n");

        // Stop streaming
        HATstopDataStream(&hatCtrl);
    }
}*/


/* ------------------------------------------------------------------------- */
/** Main
* @param
*/
int main(int argc, char **argv)
{
    struct hatCtrl hatCtrl;
    int error, i = 0, k = 0, ret = 0, param_error = 0, val = 0, val2 = 0, overfull,r,starttime;
    double value;
    char *endptr, str[MAX_STRING];
    struct streamConfig new_streamConfig;
    char *SerialNumber = NULL;
    long readSize;
    struct HATdata HATdata[4];
    unsigned long read_sets = 0;

    memset(&new_streamConfig,0,sizeof(struct streamConfig));
    memset(&hatCtrl,0,sizeof(struct hatCtrl));

    // Default parameters
    new_streamConfig.samplerate = 100;
    new_streamConfig.samples = 10;

    (void) signal(SIGINT, ex_program);
    (void) signal(SIGTERM, ex_program);

    if (argc == 2 && (!strcmp(argv[1], "-help") || !strcmp(argv[1], "--help"))) {
        PRINTOUT2("%s",usage);
        ret = 0;
        goto exit;
    }
    if (argc == 2 && !strcmp(argv[1],"--version")) {
        PRINTOUT2("%s\n",SW_VERSION);
        ret = 0;
        goto exit;
    }

    // Get SerialNumber
    if (argc > 1) {
        for (i = 1; i < argc; i++) {
        	if (!strcmp(argv[i],"-sn") && (argc > (i+1))) {
				SerialNumber = argv[++i];
				break;
        	}
        }
    }
   
    // Open HAT driver control
    if (HATopenDrvControl(SerialNumber, &hatCtrl)) {
        ret = -1;
        goto exit;
    }

    // Check parameters
    if (argc > 1) {
        for (i = 1; i < argc; i++) {
        	if (!strcmp(argv[i],"-sn") && (argc > (i+1))) {
        		i = i + 1;	// skip serialnumber
        		continue;
        	}
        	if (!strcmp(argv[i],"--config_file") || !strcmp(argv[i],"-c")) {
                if (argc > i+1) {
                    if ( (ret = HATreadSensorConfigFile(&hatCtrl,argv[i+1],&new_streamConfig)) != 0) {
                        goto exit;
                    }
                }
                else {
                    param_error = 1;
                }
                break;
            }
            else if (!strcmp(argv[i],"--serial")) {
                PRINTOUT2("%lX\n",hatCtrl.shmem->serialNumber);
                break;
            }
            else if (!strcmp(argv[i],"-s") && argc > (i+1)) {
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
                    if ( val2 > SENSOR_TYPES_MAX || val2 < 1 ) {
                        param_error = 1;
                        break;
                    }
                }
                if (new_streamConfig.sensor[val-1].type) {
                    param_error = 1;
                    break;
                }
                new_streamConfig.sensor[val-1].type = val2;
            }
            else if (!strcmp(argv[i], "-sr") && argc > (i+1)) {
                val = strtoul(argv[++i] ,&endptr, 10);
                if ( *endptr != '\0' || (val < 3) || (val > 10000) ) {
                    param_error = 1;
                    break;
                }
                else {
                    new_streamConfig.samplerate = (unsigned int)val;
                }
            }
            else if (!strcmp(argv[i], "-sc") && argc > (i+1)) {
                val = strtoul(argv[++i] ,&endptr, 10);
                if ( *endptr != '\0' || (val < 0) ) {
                    param_error = 1;
                    break;
                }
                else {
                    new_streamConfig.samples = (unsigned int)val;
                }
            }
            else if (!strcmp(argv[i], "-f") && argc > (i+1)) {
                strcpy(new_streamConfig.pathFilename, argv[++i]);
                //streamConfig.pathFilename[strlen(argv[i])] = '\0';
            }
            else if (!strcmp(argv[i], "-stream:0") && (argc >= 2)) {
                ret = HATstopDataStream(&hatCtrl);
                goto exit;
            }
            else if (!strncmp(argv[i], "-stream:",8)) {
                val = strtoul(argv[i]+8 ,&endptr, 10);
                if ( (val < 1) || (val > 10000) ) {
                    param_error = 1;
                    break;
                }
                else {
                    new_streamConfig.samplerate = (unsigned int)val;
                }
                if (*endptr == ':') {
                    do {
                        if (strncmp(endptr, ":s",2)) {
                            param_error = 1;
                            break;
                        }
        
                        val = strtoul(&endptr[2] ,&endptr, 10);
                        if ( (*endptr != '-') || val > SENSOR_TYPES_MAX || val < 1 ) {
                            param_error = 1;
                            break;
                        }
                        val2 = strtoul(&endptr[1] ,&endptr, 10);
                        if ( val2 > SENSOR_TYPES_MAX || val2 < 1 ) {
                            param_error = 1;
                            break;
                        }
                        new_streamConfig.sensor[val-1].type = val2;
                        new_streamConfig.sensors++;    
                    } while (!strncmp(endptr,":s",2));
                    
                    strcpy(new_streamConfig.pathFilename,"");
    
                    if (!strncmp(endptr,":f",2)) {
                        endptr+=2;
                        strncpy(new_streamConfig.pathFilename, endptr, strcspn(endptr, ":"));
                        new_streamConfig.pathFilename[strcspn(endptr, ":")] = '\0';
                        PRINTOUT("Streaming to: %s\n",new_streamConfig.pathFilename);
                        endptr = &endptr[strlen(new_streamConfig.pathFilename)];
                    }
                    if (!strncmp(endptr,":",1)) {
                        val = strtoul(&endptr[1] ,&endptr, 10);
                        if (*endptr != 0 || val < 0) {
                            param_error = 1;
                            break;
                        }
                        new_streamConfig.samples = val;
                    }
                    else if (*endptr != 0) {
                        param_error = 1;
                        break;
                    }
                }
            }
            else if (!strcmp(argv[i], "-ss") && (argc >= 2)) {
                showSwitchStatus(&hatCtrl);
                ret = 0;
                goto exit;
            }
            else if (argc > i) {
                k = 0;
                do {
                    if (!strcmp(argv[i], io_command[k].cmd_str)) {
                        ret = HATpresetSwitchState(&hatCtrl, io_command[k].bit, io_command[k].state);
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

    // Update digital IO
    HATsetOuts(&hatCtrl);

    if (!HATstartDataStream(&hatCtrl,&new_streamConfig)) {
        read_sets = HATsamplesToRead(&hatCtrl);
        readSize = 1;
   
        do {
            starttime = getTickCount();
            ret = 0;
            do {
                if ( (r = HATreadData(&hatCtrl,HATdata,readSize,&overfull, DATA_NO_WAIT)) > 0) {
                    read_sets -= r;
                }
                else if ( (starttime + DATA_TIME_OUT) < getTickCount()) {
                    PRINTERR("\nRead Time out\n");
                    ret = -1;
                    break;
                }
                else {
                    // Try again after some sleep.
                    usleep(hatCtrl.shmem->streamConfig.time_between_samples);
                }
            } while (r == 0 && ret == 0 && read_sets > 0);
            
            // Print read values 
            for (i = 0; i < r; i++) {
                for (k = 0; k < HATsensors(&hatCtrl); k++) {
                    if (HATdata[k].data[i] != NO_DATA) {
                        PRINTOUT2("%5.2f %s  ",HATdata[k].data[i],HATdata[k].str);                
                    }
                    else {
                        PRINTOUT2("    -    ");                
                    }
                }
                PRINTOUT2("\n");
                fflush(NULL);
            }

            if (overfull) {
                PRINTERR("Buffer overrun!\n");
                ret = -1;
                break;
            }
            if (prog_exit) {
                break;
            }
        } while (read_sets > 0);

        for (k = 0; k < HATsensors(&hatCtrl); k++) { 
            if (!HATsensorDataFunction(&hatCtrl, k, &value, str, &error)) {
                PRINTOUT2("-> %s %.2f",str, value);
            }
        }
        PRINTOUT2("\n");

        // Stop streaming
        HATstopDataStream(&hatCtrl);
    }

exit:
    // Close HAT driver control
    HATcloseDrvControl(&hatCtrl);
    return ret;
}

/* ------------------------------------------------------------------------- */
/* End of file */


