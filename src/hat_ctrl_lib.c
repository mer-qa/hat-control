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
#include <glib.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <semaphore.h>
#include <ctype.h>
#include <math.h>
#include "hat_ctrl_lib.h"
#include "hat_drv.h"
#include "u3.h"
#include "shmemlib.h"

/* ------------------------------------------------------------------------- */
/* CONSTANTS */
#define LIB_NAME "hat_ctrl_lib"

/* ------------------------------------------------------------------------- */
/* MACROS */
#define PRINTERR(...) fprintf(stderr, LIB_NAME ": " __VA_ARGS__)

/* ------------------------------------------------------------------------- */
/* LOCAL CONSTANTS AND MACROS */

// Driver communication parameters
#define ANS_TIME_OUT    1000    //ms

// HW max samplerate
#define MAX_SAMPLERATE  10000   //Hz


#define V2_SENSOR   // Use V2.0 temp sesnsor parameteres

// Temperature sensor parameters V1.0
#define CURR            0.99517629  //mA
#define R13             1000.0      //ohm
#define GAIN            6.1

// Temperature sensor parameters V2.0
#define CURR_2          0.0010800   //mA
#define R14_2           1000.0      //ohm
#define R13_2           1000.0      //ohm
#define R12_2           1000.0      //ohm
#define GAIN_2          24.1818

// Parameters for audio latency calculation
#define IGNORE_TIME     0.040   //s
#define TRIG_LEVEL      500     //mV
#define IGNORE_START    0.170   //ms

// Optical sensor constants. This is used in cisXYZ->sRGB conversion
const double xyz_to_rgb_matrix[3][3] = {{ 3.2406, -1.5372, -0.4986},
                                        {-0.9689,  1.8758,  0.0416},
                                        { 0.0557, -0.2040,  1.0570}};  

/* ------------------------------------------------------------------------- */
/* LOCAL GLOBAL VARIABLES */
volatile sig_atomic_t wait_ret = 0;

/* ------------------------------------------------------------------------- */
/* ==================== LOCAL FUNCTIONS ==================================== */
/* ------------------------------------------------------------------------- */
int sendCmdToDrv(struct hatCtrl *hatCtrl, unsigned int cmd);

void ret_received(int sig) {
 wait_ret = 0;
}

/* ------------------------------------------------------------------------- */
/** Send command to HAT driver process.
 * @param hatCtrl struct hatCtrl
 * @param cmd command to driver
 * @return 0 in success, -1 on failure
 */
int sendCmdToDrv(struct hatCtrl *hatCtrl, unsigned int cmd)
{
    long starttime;
    int ret;
    sem_wait(hatCtrl->semcomm);
    hatCtrl->shmem->hat_ctrl_pid = getpid();
    wait_ret = 1;
    hatCtrl->shmem->cmd = cmd;
    kill(hatCtrl->shmem->hat_drv_pid, SIGUSR1);
    starttime = getTickCount();
    while (wait_ret && (starttime + ANS_TIME_OUT) > getTickCount());
    if (wait_ret) {
        PRINTERR("Command replay time out error\n");
        ret = -1;
    }
    else {
        ret = hatCtrl->shmem->retValToCtrl;
    }
    sem_post(hatCtrl->semcomm);
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Linear aproximation for PT1000 temperature sensor.
 * @param x1 
 * @param x2 
 * @param y1 
 * @param y2 
 * @param x 
 * @return y
 */
float LinearApprox(float x1, float x2, float y1, float y2, float x)
{
    float y = y1 + (x - x1)*(y2 - y1)/(x2 - x1);
 
    return y;
}
 
/* ------------------------------------------------------------------------- */
/** Gives tempereature value
 * @param r resistance 
 * @return temperature
 */
float GetPt100Temperature(float r)
{  // TODO: Use table and binary search - this code is not efficent. Literals of type float should have F suffix: e.g. 80.31F
    if(r <    80.31   ) return  -50;
    if(r <    82.29   ) return  LinearApprox(      80.31   ,       82.29   ,       -50    ,       -45    ,       r);
    if(r <    84.27   ) return  LinearApprox(      82.29   ,       84.27   ,       -45    ,       -40    ,       r);
    if(r <    86.25   ) return  LinearApprox(      84.27   ,       86.25   ,       -40    ,       -35    ,       r);
    if(r <    88.22   ) return  LinearApprox(      86.25   ,       88.22   ,       -35    ,       -30    ,       r);
    if(r <    90.19   ) return  LinearApprox(      88.22   ,       90.19   ,       -30    ,       -25    ,       r);
    if(r <    92.16   ) return  LinearApprox(      90.19   ,       92.16   ,       -25    ,       -20    ,       r);
    if(r <    94.12   ) return  LinearApprox(      92.16   ,       94.12   ,       -20    ,       -15    ,       r);
    if(r <    96.09   ) return  LinearApprox(      94.12   ,       96.09   ,       -15    ,       -10    ,       r);
    if(r <    98.04   ) return  LinearApprox(      96.09   ,       98.04   ,       -10    ,       -5     ,       r);
    if(r <    100     ) return  LinearApprox(      98.04   ,       100     ,       -5     ,       0       ,       r);
    if(r <    101.95  ) return  LinearApprox(      100     ,       101.95  ,       0       ,       5       ,       r);
    if(r <    103.9   ) return  LinearApprox(      101.95  ,       103.9   ,       5       ,       10      ,       r);
    if(r <    105.85  ) return  LinearApprox(      103.9   ,       105.85  ,       10      ,       15      ,       r);
    if(r <    107.79  ) return  LinearApprox(      105.85  ,       107.79  ,       15      ,       20      ,       r);
    if(r <    109.73  ) return  LinearApprox(      107.79  ,       109.73  ,       20      ,       25      ,       r);
    if(r <    111.67  ) return  LinearApprox(      109.73  ,       111.67  ,       25      ,       30      ,       r);
    if(r <    113.61  ) return  LinearApprox(      111.67  ,       113.61  ,       30      ,       35      ,       r);
    if(r <    115.54  ) return  LinearApprox(      113.61  ,       115.54  ,       35      ,       40      ,       r);
    if(r <    117.47  ) return  LinearApprox(      115.54  ,       117.47  ,       40      ,       45      ,       r);
    if(r <    119.4   ) return  LinearApprox(      117.47  ,       119.4   ,       45      ,       50      ,       r);
    if(r <    121.32  ) return  LinearApprox(      119.4   ,       121.32  ,       50      ,       55      ,       r);
    if(r <    123.24  ) return  LinearApprox(      121.32  ,       123.24  ,       55      ,       60      ,       r);
    if(r <    125.16  ) return  LinearApprox(      123.24  ,       125.16  ,       60      ,       65      ,       r);
    if(r <    127.07  ) return  LinearApprox(      125.16  ,       127.07  ,       65      ,       70      ,       r);
    if(r <    128.98  ) return  LinearApprox(      127.07  ,       128.98  ,       70      ,       75      ,       r);
    if(r <    130.89  ) return  LinearApprox(      128.98  ,       130.89  ,       75      ,       80      ,       r);
    if(r <    132.8   ) return  LinearApprox(      130.89  ,       132.8   ,       80      ,       85      ,       r);
    if(r <    134.7   ) return  LinearApprox(      132.8   ,       134.7   ,       85      ,       90      ,       r);
    if(r <    136.6   ) return  LinearApprox(      134.7   ,       136.6   ,       90      ,       95      ,       r);
    if(r <    138.5   ) return  LinearApprox(      136.6   ,       138.5   ,       95      ,       100     ,       r);
    if(r <    140.39  ) return  LinearApprox(      138.5   ,       140.39  ,       100     ,       105     ,       r);
    if(r <    142.29  ) return  LinearApprox(      140.39  ,       142.29  ,       105     ,       110     ,       r);
    if(r <    157.31  ) return  LinearApprox(      142.29  ,       157.31  ,       110     ,       150     ,       r);
    if(r <    175.84  ) return  LinearApprox(      157.31  ,       175.84  ,       150     ,       200     ,       r);
    if(r <    195.84  ) return  LinearApprox(      175.84  ,       194.84  ,       200     ,       250     ,       r);
    if(r >=   195.84  ) return  250;
 
    return 0;
}

/* ------------------------------------------------------------------------- */
/** Converts voltage value to resistance value
 * @param volt voltage
 * @return resistance
 */
double voltageToResistance(double volt)
{
    volt = volt / 1000 / GAIN_2; 
    #ifdef V2_SENSOR
        return ((R12_2*volt+CURR_2*R13_2*R14_2+R13_2*volt+R14_2*volt)/(CURR_2*R12_2-volt));
    #else
        return (volt + GAIN*CURR/1000*R13)/(GAIN*CURR/1000);
    #endif
}

/* ------------------------------------------------------------------------- */
/** Converts resistance value to temperature value
 * @param res resistance
 * @return temperature
 */
double resistanceToTemp(double res)
{
    return GetPt100Temperature(res / 10);
}

/* ------------------------------------------------------------------------- */
/** Converts voltage value to temperature value
 * @param volt voltage 
 * @param state not used in this function 
 * @return temperature
 */
double voltageToTemp(double volt, int state)
{
    //printf("vtor:%f    ",voltageToResistance(volt));
    return resistanceToTemp(voltageToResistance(volt));
}

/* ------------------------------------------------------------------------- */
/** Converts raw voltage value according sensor type
 * @param shmem struct shmem 
 * @param value raw voltage value 
 * @param ch sensor number 
 * @param conv_value converted value 
 * @param value_str sensor specified string 
 * @return 0 success and -1 failure
 */
int convert(struct shmem *shmem, double value, int ch, double *conv_value, char *value_str)
{
    int sensor_type, dout;

    if (value < 0xffff) {
        sensor_type = shmem->streamConfig.sensor[ch].type;    
        if (sensor_params[sensor_type].convert == NULL) {
            *conv_value = (double)value;
        }
        else {
            *conv_value = sensor_params[sensor_type].convert( (double)value, shmem->ioctrl.sensorDigOutput[ch]);
        }

        if (shmem->ioctrl.sensorDigOutput[ch] >= 0 && shmem->ioctrl.sensorDigOutput[ch] <= 2) {
            dout = shmem->ioctrl.sensorDigOutput[ch];
            *conv_value += sensor_params[sensor_type].offset[dout];
            *conv_value *= sensor_params[sensor_type].multiplier[dout];
        }
        else {
            dout = 0;
            *conv_value += sensor_params[sensor_type].offset[dout];
            *conv_value *= sensor_params[sensor_type].multiplier[dout];
        }

        if (shmem->streamConfig.sensor[ch].offset_mult_order == OFFSET_FIRST) {
            *conv_value += shmem->streamConfig.sensor[ch].offset_mv;
        }

        *conv_value *= shmem->streamConfig.sensor[ch].mult_mv[dout];

        if (shmem->streamConfig.sensor[ch].offset_mult_order == MULTIPLE_FIRST) {
            *conv_value += shmem->streamConfig.sensor[ch].offset_mv;
        }
        if (value_str != NULL) 
            strcpy(value_str, shmem->streamConfig.sensor[ch].str[0]);
    }
    else {
        *conv_value = 0xffff;
        //if (value_str != NULL) 
            //strcpy(value_str, "OVER!");
    }

    return 0;
}

/* ------------------------------------------------------------------------- */
/** Gets audio latency from stored data
 * @param hatCtrl struct hatCtrl  
 * @param ch sensor number 
 * @param value calculated latency value 
 * @param str latency string 
 * @return 0 success and -1 failure
 */
int getLatency(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    int i,k;
    double avg = 0, max[2], ret = 0;
    struct sensor *sensor = &hatCtrl->shmem->streamConfig.sensor[ch];

    for (i = sensor->samplerate * IGNORE_START; i < sensor->data_func.stored; i++) {
        avg += (sensor->data_func.fdata[i] - avg)/(i+1); 
    }
    
    max[0] = -1;
    max[1] = -1;
    k = 0;
    for (i = sensor->samplerate * IGNORE_START; i < sensor->data_func.stored; i++) {
        if ((sensor->data_func.fdata[i] - avg) > TRIG_LEVEL) {
            max[k++] = i;
            i += sensor->samplerate * IGNORE_TIME;
            if (k == 2) {
                break;
            }
        }   
    }
    sensor->data_func.stored = 0;
    *value = (max[1] - max[0])/sensor->samplerate*1000;
    if (str != NULL) {
        strncpy(str,"Latency:",MAX_STRING);
    }

    if (max[0] < 0 || max[1] < 0) {
        ret = -1;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Calculates average value from data.
 * @param hatCtrl struct hatCtrl 
 * @param ch sensor number 
 * @param value calculated average value 
 * @param str string for average value 
 * @return 0 success and -1 failure
 */
int getAvg(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    int i;
    double avg = 0;
    struct sensor *sensor = &hatCtrl->shmem->streamConfig.sensor[ch];
    //printf("\nch:%d  st:%d\n",ch,sensor->data_func.stored);
    for (i = 0; i < sensor->data_func.stored; i++) {
        //printf("i:%d->%f\n",i,sensor->data_func.fdata[i]);
        avg += (sensor->data_func.fdata[i] - avg)/(i+1); 
    }
    *value = avg;
    sensor->data_func.stored = 0;
    //printf("avg: %f\n",avg);

    if (str != NULL) {
        strncpy(str,"Average:",MAX_STRING);
    }
    return 0;
}

int checkLimits(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    int i;
    double avg = 0;
    struct sensor *sensor = &hatCtrl->shmem->streamConfig.sensor[ch];
    //printf("\nch:%d  st:%d\n",ch,sensor->data_func.stored);
    for (i = 0; i < sensor->data_func.stored; i++) {
        //printf("i:%d->%f\n",i,sensor->data_func.fdata[i]);
        if (sensor->data_func.fdata[i] > param[0]) {
            *value = 1;
            if (str != NULL) {
                strncpy(str,"LIMIT",MAX_STRING);
            }  
        }
    }
    *value = avg;
    sensor->data_func.stored = 0;
    //printf("avg: %f\n",avg);

    if (str != NULL) {
        strncpy(str,"Average:",MAX_STRING);
    }
    return 0;
}

int getRms(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    int i;
    double rms = 0;
    struct sensor *sensor = &hatCtrl->shmem->streamConfig.sensor[ch];

    for (i = 0; i < sensor->data_func.stored; i++) {
        rms = sqrt( (i*rms*rms + sensor->data_func.fdata[i]*sensor->data_func.fdata[i]) / (i+1));
    }
    *value = rms;
    sensor->data_func.stored = 0;

    if (str != NULL) {
        strncpy(str,"RMS:",MAX_STRING);
    }
    return 0;
}




/* ------------------------------------------------------------------------- */
/** Calculates average value from data.
 * @param hatCtrl struct hatCtrl 
 * @param ch sensor number 
 * @param value calculated color value 
 * @param str string for color 
 * @return 0 success and -1 failure
 */
int getColor(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    double avg_cR, avg_cG, avg_cB;
    
    //printf("ch:%d\n",ch);
    if (hatCtrl->shmem->streamConfig.sensors >= 3) {
        getAvg(hatCtrl, 0, NULL, &avg_cR, NULL);
        getAvg(hatCtrl, 1, NULL, &avg_cG, NULL);
        getAvg(hatCtrl, 2, NULL, &avg_cB, NULL);
        //printf("(%.2f %.2f %.2f)",avg_cR,avg_cG,avg_cB);
        *value = HATOpticalDetectColor(hatCtrl, avg_cR,avg_cG,avg_cB, str);
        return 0;
    }
    else {
        *value = 0xffff;
    }
    return -1;
}

/* ------------------------------------------------------------------------- */
/** Calculates light "intensity", not real intensity.
 * @param hatCtrl struct hatCtrl 
 * @param ch sensor number 
 * @param value calculated average value 
 * @param str string for average value 
 * @return 0 success and -1 failure
 */
int getIntensity(struct hatCtrl *hatCtrl, unsigned int ch, double param[], double *value, char *str)
{
    double avg_cR, avg_cG, avg_cB;
    getAvg(hatCtrl, 0, NULL, &avg_cR, NULL);
    getAvg(hatCtrl, 1, NULL, &avg_cG, NULL);
    getAvg(hatCtrl, 2, NULL, &avg_cB, NULL);

    //printf("(%.2f %.2f %.2f)",avg_cR,avg_cG,avg_cB);

    if (hatCtrl->shmem->streamConfig.sensors == 3) {
        *value = avg_cR + avg_cG + avg_cB;
        return 0;
    }
    return -1;
}


/* ------------------------------------------------------------------------- */
/* ======================== FUNCTIONS ====================================== */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/** Get switches status from shared memory
 * @param hatCtrl struct hatCtrl  
 * @return switches status
 */
long HATgetSwitchStatus(struct hatCtrl *hatCtrl) 
{
    return hatCtrl->shmem->ioctrl.iostate;
}

/* ------------------------------------------------------------------------- */
/** Get amount of the sensors
 * @param hatCtrl struct hatCtrl  
 * @return amount sensors
 */
int HATsensors(struct hatCtrl *hatCtrl) 
{
    return hatCtrl->shmem->streamConfig.sensors;
}

/* ------------------------------------------------------------------------- */
/** Samples
 * @param hatCtrl struct hatCtrl  
 * @return samples
 */
int HATsamplesToRead(struct hatCtrl *hatCtrl) 
{
    return hatCtrl->shmem->streamConfig.MaxSamplesFromCh;
}


/* ------------------------------------------------------------------------- */
/** Read data from shared memory
 * @param hatCtrl struct hatCtrl 
 * @param ch read channel 
 * @param value read value 
 * @param overfull overfull indicator 
 * @param mode DATA_NO_WAIT or DATA_WAIT
 * @return 0 on success and negative on failure
 */
int readDataFromShBuf(struct hatCtrl *hatCtrl, int ch, int *value, int *overfull, int mode)
{
    int starttime = 0, ret;

    sem_wait(hatCtrl->semshmem);
    if (overfull) {
        *overfull = hatCtrl->shmem->ch_data[ch].overfull;
    }
    hatCtrl->shmem->ch_data[ch].overfull = 0;
    sem_post(hatCtrl->semshmem);

    if (mode == DATA_NO_WAIT) {
        sem_wait(hatCtrl->semshmem);
        ret = getFromBuf(&hatCtrl->shmem->ch_data[ch], value);
        sem_post(hatCtrl->semshmem);
    }
    else {
        starttime = getTickCount();
        do {
            sem_wait(hatCtrl->semshmem);
            ret = getFromBuf(&hatCtrl->shmem->ch_data[ch], value);
            sem_post(hatCtrl->semshmem);
            if ( (starttime + DATA_TIME_OUT) < getTickCount()) {
                ret = -2;
                break;
            }
            else if (ret == -1) {
                usleep(hatCtrl->shmem->streamConfig.time_between_samples);
            }
        } while ( ret < 0);
    }

    return ret;
}

/* ------------------------------------------------------------------------- */
/** Read data from shared memory
 * @param hatCtrl struct hatCtrl 
 * @param HATdata struct HATdata  
 * @param size read size 
 * @param str string of the data 
 * @param overfull overfull indicator 
 * @param mode DATA_NO_WAIT or DATA_WAIT
 * @return how many bytes read
 */
int HATreadData(struct hatCtrl *hatCtrl, struct HATdata *HATdata, unsigned int size, int *overfull, int mode)
{
    int sample_count = 0, x = 0, ret = 0, i = 0;
    int stored_count = 0, max_stored = 0; 
    static double vals[MAX_SENSOR_INPUTS] = {0};
    static long avg[MAX_SENSOR_INPUTS] = {0};
    static long avg_count[MAX_SENSOR_INPUTS] = {0};
    double con_val = 0;
    unsigned int cch,ch;

    int avg_values[4];

    memset(HATdata,0,sizeof(struct HATdata) * 4);

    do {
        stored_count = 0;
        sample_count = 0;
        do {
            // Check that all data buffer has atleast one data value.
            if (mode == DATA_NO_WAIT) {
                sem_wait(hatCtrl->semshmem);
                for (ch = 0; ch < HATsensors(hatCtrl); ch++) {
                    if (getBufSize(&hatCtrl->shmem->ch_data[ch]) == 0) {
                        sem_post(hatCtrl->semshmem);
                        return max_stored;
                    }
                }
                sem_post(hatCtrl->semshmem);
            }

            for (ch = 0; ch < HATsensors(hatCtrl); ch++) {
                HATdata[ch].data[i] = NO_DATA;
                avg_values[ch] = hatCtrl->shmem->streamConfig.samplerate / hatCtrl->shmem->streamConfig.sensor[hatCtrl->shmem->streamConfig.ch_table[ch]].samplerate;
    
                ret = readDataFromShBuf(hatCtrl, ch, &x, overfull, DATA_WAIT);
                if (ret == 0) {
                    sample_count++;
                    vals[ch] += (x - vals[ch]) / (avg_count[ch]+1);
                    if (++avg_count[ch] == avg_values[ch]) {
                        avg_count[ch] = 0;
                        avg[ch] = vals[ch];
                        vals[ch] = 0;
                        cch = hatCtrl->shmem->streamConfig.ch_table[ch];
                        convert(hatCtrl->shmem, (double)(avg[ch])/100, cch, &con_val, HATdata[ch].str);
                        HATdata[ch].data[i] = con_val;
                        HATdata[ch].size++;
                        if(HATdata[ch].size > max_stored) {
                            max_stored = HATdata[ch].size; 
                        }
                        stored_count++;
                        if (/*hatCtrl->shmem->streamConfig.sensor[cch].data_func.function > 0 && */ hatCtrl->shmem->streamConfig.sensor[cch].data_func.fdata != NULL) {
                            hatCtrl->shmem->streamConfig.sensor[cch].data_func.fdata[hatCtrl->shmem->streamConfig.sensor[cch].data_func.stored++] = con_val;
                        }
                    }
                }
                else {
                    break;       
                }
            }
        } while ( ret == 0 && !stored_count && (sample_count < hatCtrl->shmem->streamConfig.samples || hatCtrl->shmem->streamConfig.samples == 0) );
    } while (++i < size && ret == 0);
    
    return max_stored;
}

/* ------------------------------------------------------------------------- */
/** Preset sensors IO ports
 * @param hatCtrl struct hatCtrl 
 * @param ch read channel 
 * @param state read data 
 * @return 0 on success and -1 on failure
 */
int HATpresetSensorIO(struct hatCtrl *hatCtrl, int ch, int state)
{
    int ret = 0, bit = -1;
    struct streamConfig *streamConfig = &hatCtrl->shmem->streamConfig;

    switch (ch) {
    case 0:
        bit = AN0_IO;
        break;
    case 1:
        bit = AN1_IO;
        break;
    case 2:
        bit = AN2_IO;
        break;
    case 3:
        bit = AN3_IO;
        break;
    default:
        bit = -1;
    }

    if (bit < 0) {
        ret  = -1;
    }
    else {
        switch (state) {
            case 2:
                streamConfig->digdir |= (1 << bit);
                break;
            case 1:
                streamConfig->digdir &= ~(1 << bit);
                streamConfig->digout |= (1 << bit);
                break;
            case 0:
                streamConfig->digdir &= ~(1 << bit);
                streamConfig->digout &= ~(1 << bit);
                break;
            default:
                ret = -1;
        }
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Preset switches states.
 * @param hatCtrl struct hatCtrl 
 * @param sw_bit switch bit 
 * @param state switch state 
 * @return 0 on success and -1 on failure
 */
int HATpresetSwitchState(struct hatCtrl *hatCtrl, int bit, int state)
{
    int ret = 0;
    sem_wait(hatCtrl->semshmem);
    if (state == 0) { 
        hatCtrl->shmem->streamConfig.digout &= ~(1 << bit);
        hatCtrl->shmem->streamConfig.digdir &= ~(1 << bit);
    }
    else if (state == 1) {
        hatCtrl->shmem->streamConfig.digout |= (1 << bit);
        hatCtrl->shmem->streamConfig.digdir &= ~(1 << bit);
    }
    else if (state == 2) {
        hatCtrl->shmem->streamConfig.digout &= ~(1 << bit);
        hatCtrl->shmem->streamConfig.digdir |= (1 << bit);
    }
    else if (state == 3) {
        hatCtrl->shmem->streamConfig.digout |= PWR_USB_IOS;
    }
    else if (state == 4) {
        hatCtrl->shmem->streamConfig.digout &= ~PWR_USB_IOS;
    }
    else {
        ret = -1;
    }
    sem_post(hatCtrl->semshmem);
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Sets digital outs
 * @param hatCtrl struct hatCtrl 
 * @return 0 on success and -1 on failure
 */
int HATsetOuts(struct hatCtrl *hatCtrl)
{
    sem_wait(hatCtrl->semshmem);
    hatCtrl->shmem->ioctrl.iostate = hatCtrl->shmem->streamConfig.digout;
    hatCtrl->shmem->ioctrl.digdir = hatCtrl->shmem->streamConfig.digdir;
    sem_post(hatCtrl->semshmem);
    return sendCmdToDrv(hatCtrl, CHANGE_IO);;
}

/* ------------------------------------------------------------------------- */
/** Close driver control
 * @param hatCtrl struct hatCtrl 
 * @return 0 on success and -1 on failure
 */
int HATcloseDrvControl(struct hatCtrl *hatCtrl)
{
    int ch;
    for (ch = 0; ch < MAX_SENSOR; ch++) {
        if (hatCtrl->shmem != NULL && hatCtrl->shmem->streamConfig.sensor[ch].data_func.fdata != NULL) {
            free(hatCtrl->shmem->streamConfig.sensor[ch].data_func.fdata);
            memset(&hatCtrl->shmem->streamConfig.sensor[ch],0,sizeof(struct sensor));
        }
    }
    sem_close(hatCtrl->semshmem);
    sem_close(hatCtrl->semcomm);
    return deAllocSharedMem(hatCtrl->shmem);
}

/* ------------------------------------------------------------------------- */
/** Preset sensors IO ports
 * @param hatCtrl struct hatCtrl 
 * @param SerialNumber HAT device serial number 
 * @return 0 on success and -1 on failure
 */
int HATopenDrvControl(char *SerialNumber, struct hatCtrl *hatCtrl)
{
    int ret = 0, sh_seg_id = 0,i;

    if (SerialNumber != NULL) {
    	for( i = 0; SerialNumber[ i ]; i++)
    		SerialNumber[ i ] = toupper( SerialNumber[ i ] );
    }

    getShName(SerialNumber, SHMEMNAME);
    getShComm(SerialNumber, COMMSEM);
    getShMemID(SerialNumber);

    hatCtrl->semshmem = sem_open(SHMEMNAME,0,0644,0);

    if(hatCtrl->semshmem == SEM_FAILED) {
      PRINTERR("Error: HAT driver is not started.\n");
      ret = -1;
      goto exit;
    }

    hatCtrl->semcomm = sem_open(COMMSEM,0,0644,0);

    if(hatCtrl->semcomm == SEM_FAILED) {
      PRINTERR("Error: HAT driver is not started.\n");
      ret = -1;
      goto exit;
    }

    // Allocate shared memory
    if ( getShareMem(&hatCtrl->shmem, &sh_seg_id) ) {
        ret = -1;
        goto exit;
    }

    hatCtrl->shmem->streamConfig.digdir = hatCtrl->shmem->ioctrl.digdir;
    hatCtrl->shmem->streamConfig.digout = hatCtrl->shmem->ioctrl.iostate;

    (void) signal(SIGUSR1, ret_received);

exit:
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Stop data streaming
 * @param hatCtrl struct hatCtrl 
 * @return 0 on success and -1 on failure
 */
int HATstopDataStream(struct hatCtrl *hatCtrl)
{
    sem_wait(hatCtrl->semshmem);
    hatCtrl->shmem->streamConfig.hat_ctrl_pid = 0;
    sem_post(hatCtrl->semshmem);
    return sendCmdToDrv(hatCtrl,STOP_STREAMING);
}

/* ------------------------------------------------------------------------- */
/** Process stream configuration
 * @param streamConfig stream configuration 
 * @return 0 on success and -1 on failure
 */
int HATprocessStreamConfig(struct streamConfig *streamConfig)
{
    int k, ret = 0;

    streamConfig->sensors = 0;
    streamConfig->MaxSamplesFromCh = 0;
    if (streamConfig->samples == 0) {
        streamConfig->samples = INT_MAX;
    }

    for (k = 0; k < MAX_SENSOR; k++) {
        if (streamConfig->sensor[k].type) {
            streamConfig->ch_table[streamConfig->sensors++] = k;
            if (streamConfig->sensor[k].type == 5) {
                streamConfig->sensors +=2;
            }
            if (streamConfig->sensor[k].samplerate == 0) {
                streamConfig->sensor[k].samplerate = streamConfig->samplerate;
            }
            if ( ((streamConfig->samplerate % streamConfig->sensor[k].samplerate) != 0)) {
                PRINTERR("Sensor %d samplerate needs to be multiple with common samplerate\n",k+1);
                PRINTERR("and lower than common samplerate.\n");
                ret = -1;
                break;
            }
            if ( (streamConfig->samples/(streamConfig->samplerate / streamConfig->sensor[k].samplerate) > streamConfig->MaxSamplesFromCh)) {
                streamConfig->MaxSamplesFromCh = streamConfig->samples/(streamConfig->samplerate / streamConfig->sensor[k].samplerate);
            }
            //streamConfig->samplesToRead += streamConfig->samples/(streamConfig->samplerate / streamConfig->sensor[k].samplerate);
            if (streamConfig->sensor[k].data_func.function > 0 && streamConfig->samples != INT_MAX) {
                streamConfig->sensor[k].data_func.size = streamConfig->samples / (streamConfig->samplerate / streamConfig->sensor[k].samplerate) + 1;
                streamConfig->sensor[k].data_func.fdata = malloc(streamConfig->sensor[k].data_func.size * sizeof(double));
                if (streamConfig->sensor[k].data_func.fdata == NULL) {
                    PRINTERR("Memory allocation error\n");
                    ret = -1;
                    break;
                }
            }
            else {
                streamConfig->sensor[k].data_func.fdata = NULL;
                streamConfig->sensor[k].data_func.size = 0;
            }
            if (streamConfig->sensor[k].mult_mv[0] == 0) {
                streamConfig->sensor[k].mult_mv[0] = 1;
            }
            if (streamConfig->sensor[k].mult_mv[1] == 0) {
                streamConfig->sensor[k].mult_mv[1] = 1;
            }
            if (streamConfig->sensor[k].mult_mv[2] == 0) {
                streamConfig->sensor[k].mult_mv[2] = 1;
            }
        }
    }
    
    if (streamConfig->sensor[0].type == 5 /*optical3*/) {
        streamConfig->ch_table[1] = 1;
        streamConfig->ch_table[2] = 2;
        //streamConfig->samplesToRead *= 3; 
        memcpy(&streamConfig->sensor[1],&streamConfig->sensor[0], sizeof(struct sensor));
        memcpy(&streamConfig->sensor[2],&streamConfig->sensor[0], sizeof(struct sensor));
        if (strlen(streamConfig->sensor[0].str[0]) > 2) {
            strcpy(streamConfig->sensor[1].str[0],&streamConfig->sensor[0].str[0][1]);
            streamConfig->sensor[1].str[0][1] = 0;
            strcpy(streamConfig->sensor[2].str[0],&streamConfig->sensor[0].str[0][2]);
            streamConfig->sensor[0].str[0][1] = 0;
        }
        if (streamConfig->sensor[0].data_func.function > 0 && streamConfig->samples > 0) {
            for (k = 1; k < 3; k++) {
                streamConfig->sensor[k].data_func.function = 0;
                streamConfig->sensor[k].data_func.size = streamConfig->samples / (streamConfig->samplerate / streamConfig->sensor[k].samplerate);
                streamConfig->sensor[k].data_func.fdata = malloc(streamConfig->sensor[k].data_func.size * sizeof(double));
                if (streamConfig->sensor[k].data_func.fdata == NULL) {
                    PRINTERR("Memory allocation error\n");
                    ret = -1;
                    break;
                }
            }
        }
        else {
            streamConfig->sensor[k].data_func.fdata = NULL;
            streamConfig->sensor[k].data_func.size = 0;
        }
    }
    // This time is used to sleep when waiting data to buffer.
    streamConfig->time_between_samples = 1000000 / streamConfig->samplerate;

    if ( (streamConfig->samplerate * streamConfig->sensors) > MAX_SAMPLERATE) {
        PRINTERR("Samplerate too high. Reduce samplerate or used channels.\n");
        PRINTERR("Maximum samplerate is 10000Hz divided by channels in use.\n");
        ret = -1;
    }
    else if (streamConfig->MaxSamplesFromCh == 0 && streamConfig->sensors) {
        PRINTERR("The ratio of the common and sensor samplerate needs more samples.\n");
        PRINTERR("Samples=%ld is not enough.\n", streamConfig->samples);
        ret = -1;
    }
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Start data streaming
 * @param hatCtrl struct hatCtrl 
 * @param new_streamConfig stream configuration 
 * @return 0 on success and -1 on failure
 */
int HATstartDataStream(struct hatCtrl *hatCtrl, struct streamConfig *new_streamConfig)
{
    int ret = 0;
    
    if (new_streamConfig != NULL) {
        if ((ret = HATprocessStreamConfig(new_streamConfig))) {
            ret = -1;
            goto exit;
        }

        if (new_streamConfig->sensors) {
            if (hatCtrl->shmem->streamConfig.hat_ctrl_pid != 0) {
                PRINTERR("Error only one process can stream data. Stop previous streaming first.\n");
                ret = -1;
                goto exit;
            }
            sem_wait(hatCtrl->semshmem);
            // Copy stream config to shared memory
            memcpy(&hatCtrl->shmem->streamConfig, new_streamConfig, sizeof(struct streamConfig));
            // Store control SW pid to shared memory
            hatCtrl->shmem->streamConfig.hat_ctrl_pid = getpid();
            hatCtrl->time = 0;
            sem_post(hatCtrl->semshmem);
            
            // Stop previous streaming
            sendCmdToDrv(hatCtrl, STOP_STREAMING);
            ret = sendCmdToDrv(hatCtrl, START_STREAMING);
        }
        else {
            ret = -1;
        }
    }
    else {
        // Stop previous streaming
        sendCmdToDrv(hatCtrl, STOP_STREAMING);
        ret = sendCmdToDrv(hatCtrl, START_STREAMING);
    }
exit:
    return ret;
}

/* ------------------------------------------------------------------------- */
/** Convert cisXYZ value to sRGB value 
 * @param cX
 * @param cY 
 * @param cZ 
 * @param c channel 0 = R, 1 = G and 2 = B 
 * @return converted value
 */
double HATOpticalXYZtosRGB(double cX, double cY, double cZ, int c, int range)
{
    int i;
    double val = 0, cieXYZ[]={cX/range, cY/range, cZ/range};

    for (i = 0; i < 3; i++) {
        val += xyz_to_rgb_matrix[c][i]*cieXYZ[i];
    }

    // Gamma correction
    if (val <= 0.0031308) {
        val *=12.92;
    }
    else {
        val = (1 + 0.055)*pow(val,(1/2.4))-0.055;
    }

    return val*range;
}

/* ------------------------------------------------------------------------- */
/** Detect color from optical sensor cisXYZ values
 * @param hatCtrl struct hatCtrl 
 * @param cX
 * @param cY 
 * @param cZ 
 * @return 0 on success and -1 on failure
 */
int HATOpticalDetectColor(struct hatCtrl *hatCtrl, double cX, double cY, double cZ, char *str)
{
    int i = -1, k = 0;
    double avg = (cX + cY + cZ*1.9) / 3;
    int dif[3] = {3.1, 2.6, 2.1};
    double colors[] = {cX, cY, cZ*1.9};
    //printf("avg:%0.2f  ",avg);
    //printf("%0.2f  %0.2f  %0.2f  ",colors[0],colors[1],colors[2]);

    while(k < 3) {
        if ( (colors[k] - avg) > dif[k] ) {
            i = k;
        }
        k++;
    }
        
    if (i == -1) {
        if (avg > 20) {
            i = 3;
        }
        if (avg < 5) {
            i = 4;
        }
    }

    if (str != NULL) {
        switch (i) {
        case -1:
            strcpy(str,"?");
            break;
        case 0:
            strcpy(str,"RED");
            break;
        case 1:
            strcpy(str,"GREEN");
            break;
        case 2:
            strcpy(str,"BLUE");
            break;
        case 3:
            strcpy(str,"WHITE");
            break;
        case 4:
            strcpy(str,"BLACK");
            break;
        }
    }
    return i;
}


/* ------------------------------------------------------------------------- */
/** Find and execute function for measured data. 
 ** The used function is configured in sensor config file. 
 * @param hatCtrl struct hatCtrl 
 * @param ch read channel 
 * @param value color integer value
 * @param str color string
 * @return 0 on success and -1 on failure
 */
int HATsensorDataFunction(struct hatCtrl *hatCtrl, int ch, double *value, char *str, int *error)
{
    struct streamConfig *streamConfig = &hatCtrl->shmem->streamConfig;
    memset(str,0,MAX_STRING);
    if (streamConfig->sensor[ch].data_func.function > 0 && streamConfig->samples != INT_MAX) {
        *error = data_func_array[streamConfig->sensor[ch].data_func.function].data_function(hatCtrl, ch, streamConfig->sensor[ch].data_func.params, value, str);
        return 0;
    }
    return -1;
}


const char *main_keys[] = {"Common","Sensor1","Sensor2","Sensor3","Sensor4",NULL};
const char *common_keys[] = {"Samplerate","Samples","Datafile",NULL};

const char *sensor_keys[] = {"Type","String0","String1","Samplerate","Dout","Offset","Multiplier0","Multiplier1","Function",NULL};

const char *sensor_types[] = {"None", "Voltage", "Current", "Temperature", "Audio", "Optical3","Optical","Acceleration","Acceleration6g", NULL};

const char *data_functions[] = {"Audio_latency","Average","Optic_get_color","Limit","Rms", NULL};

/* ------------------------------------------------------------------------- */
/** Read sensor confugration file and store data to streamConfig struct
 * @param filename sensor configuration file name
 * @param streamConfig struct streamConfig
 * @return 0 on success and -1 on failure
 */
int HATreadSensorConfigFile(struct hatCtrl *hatCtrl, char *filename, struct streamConfig *streamConfig)
{   
    int i,k,j;
    char *val, *endptr;
    long v;
    double dval;

    printf("Reading config file: %s\n", filename);
    GKeyFile *key_file = NULL;
    GError *error = NULL;
    key_file = g_key_file_new();

    if (!g_key_file_load_from_file(key_file, filename, G_KEY_FILE_NONE, &error)) {
        PRINTERR("Error reading config file (%s)\n",error->message);
        goto error;
    }

    i = 0;
    // Read common keys
    while (common_keys[i]) {
        if ((val = g_key_file_get_value(key_file, main_keys[0], common_keys[i], &error))) {
            if (!strcmp(common_keys[i],"Samplerate")) {
                v = strtoul(val ,&endptr, 10);
                if (*endptr != '\0') {
                    PRINTERR("Config parsing error (%s)\n",common_keys[i]);
                    goto error;
                }
                streamConfig->samplerate = (unsigned int)v;
            }

            if (!strcmp(common_keys[i],"Samples")) {
                v = strtoul(val ,&endptr, 10);
                if (*endptr != '\0') {
                    PRINTERR("Config parsing error (%s)\n",common_keys[i]);
                    goto error;
                }
                streamConfig->samples = (unsigned int)v;                
            }
            if (!strcmp(common_keys[i],"Datafile")) {
                //strncpy(streamConfig->sensor[k-1].pathFilename, val, 254);
                //printf("file->%s\n",streamConfig->sensor[k-1].pathFilename);
                strncpy(streamConfig->pathFilename, val, 254);

            }

        }
        error = NULL;
        i++;
    }

    k = 1;
    // Read sensors keys
    while (main_keys[k]) {
        i = 0;
        while (sensor_keys[i]) {
            if ((val = g_key_file_get_value(key_file, main_keys[k], sensor_keys[i], &error))) {
                if (!strcmp(sensor_keys[i],"Type")) {
                    j = 0;
                    while (sensor_types[j] != NULL) {
                        if (val != NULL && !strcmp(val, sensor_types[j])) {
                            streamConfig->sensor[k-1].type = j;
                            if (streamConfig->sensor[k-1].samplerate == 0) {
                                streamConfig->sensor[k-1].samplerate = streamConfig->samplerate;
                            }
                            break;
                        }
                        j++;
                    }
                    if (sensor_types[j] == NULL) {
                        printf("Sensor type parsing error\n");
                        goto error;
                    }
                }
                if (!strcmp(sensor_keys[i],"Function")) {
                    j = 0;
                    while (data_functions[j] != NULL) {
                        char function[20];
                        sscanf(val,"%s %lf %lf",function,&streamConfig->sensor[k-1].data_func.params[0],&streamConfig->sensor[k-1].data_func.params[1]);
                        //printf("f:%s  %lf %lf\n",function,streamConfig->sensor[k-1].data_func.params[0],streamConfig->sensor[k-1].data_func.params[1]);
                        if (val != NULL && !strcmp(function, data_functions[j])) {
                            streamConfig->sensor[k-1].data_func.function = j+1;
                            //printf("hep:%d\n",j);
                            break;
                        }
                        j++;
                    }
                    if (data_functions[k-1] == NULL) {
                        printf("Function doesn't found\n");
                        goto error;
                    }
                }
                if (!strcmp(sensor_keys[i],"String0")) {
                    strncpy(streamConfig->sensor[k-1].str[0],val,10);
                }
                if (!strcmp(sensor_keys[i],"String1")) {
                    strncpy(streamConfig->sensor[k-1].str[1],val,10);
                }
                if (!strcmp(sensor_keys[i],"Samplerate")) {
                    v = strtoul(val ,&endptr, 10);
                    if (*endptr != '\0') {
                        PRINTERR("Config parsing error (%s)\n",sensor_keys[i]);
                    }
                    streamConfig->sensor[k-1].samplerate = v;
                }
                if (!strcmp(sensor_keys[i],"Offset")) {
                    dval = strtold(val ,&endptr);
                    if (*endptr != '\0' && *endptr != '*') {
                        PRINTERR("Config parsing error (%s) (%s)\n",sensor_keys[i], endptr);
                        goto error;
                    }
                    if (endptr[0] == '*' || streamConfig->sensor[k-1].offset_mult_order == 0) {
                        streamConfig->sensor[k-1].offset_mult_order = OFFSET_FIRST;
                    }
                    streamConfig->sensor[k-1].offset_mv = dval;
                }
                if (!strcmp(sensor_keys[i],"Multiplier0")) {
                    dval = strtold(val ,&endptr);
                    if (*endptr != '\0' && *endptr != '*') {
                        PRINTERR("Config parsing error (%s) (%s)\n",sensor_keys[i],endptr);
                        goto error;
                    }
                    if (endptr[0] == '*' || streamConfig->sensor[k-1].offset_mult_order == 0) {
                        streamConfig->sensor[k-1].offset_mult_order = MULTIPLE_FIRST;
                    }
                    streamConfig->sensor[k-1].mult_mv[0] = dval;
                }
                if (!strcmp(sensor_keys[i],"Multiplier1")) {
                    dval = strtold(val ,&endptr);
                    if (*endptr != '\0' && *endptr != '*') {
                        PRINTERR("Config parsing error (%s)\n",sensor_keys[i]);
                        goto error;
                    }
                    if (endptr[0] == '*' || streamConfig->sensor[k-1].offset_mult_order == 0) {
                        streamConfig->sensor[k-1].offset_mult_order = MULTIPLE_FIRST;
                    }
                    streamConfig->sensor[k-1].mult_mv[1] = dval;
                }
                if (!strcmp(sensor_keys[i],"Multiplier2")) {
                    dval = strtold(val ,&endptr);
                    if (*endptr != '\0' && *endptr != '*') {
                        PRINTERR("Config parsing error (%s)\n",sensor_keys[i]);
                        goto error;
                    }
                    if (endptr[0] == '*' || streamConfig->sensor[k-1].offset_mult_order == 0) {
                        streamConfig->sensor[k-1].offset_mult_order = MULTIPLE_FIRST;
                    }
                    streamConfig->sensor[k-1].mult_mv[2] = dval;
                }
                if (!strcmp(sensor_keys[i],"Dout")) {
                    v = strtoul(val ,&endptr, 10);
                    if ( (*endptr != '\0' && *endptr != ' ') || v > 2 || v < 0) {
                        printf("Config parsing error (%s)\n",sensor_keys[i]);
                        goto error;
                    }
                    HATpresetSensorIO(hatCtrl,k-1,v);
                }
            }
            error = NULL;
            i++;
        }
        k++;
    }

    g_key_file_free (key_file);                    
    return 0;                     
error:
    g_key_file_free (key_file);
    return -1;
}

/* ------------------------------------------------------------------------- */
/* End of file */

