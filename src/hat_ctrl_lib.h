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
#ifndef HATCTRLLIBH
#define HATCTRLLIBH

/* INCLUDE FILES */
#include "shmemlib.h"

// Current sensor multipliers
// R4 = 141k (47k x 3)
//#define CURRENT_SENSOR_M1    0.60024 // Proto sensor, R9 = 6k6 (2k2 x 3)
//#define CURRENT_SENSOR_M1    0.04058 // R8 = 287k

//#define CURRENT_SENSOR_M0    0.618087 // V2 board: 140k, 6k49, 274k
//#define CURRENT_SENSOR_M1    0.041962 // Trimmers at middle position (2.5k and 50ohm)

#define CURRENT_SENSOR_M0   0.622587 // 0.615087 V2 board: 140k, 6k49, 274k
#define CURRENT_SENSOR_M1   0.041690 // 0.041962 // Trimmers at middle position (2.5k and 50ohm)

#define MAX_STRING  50

// Digital io states
#define OFF     0
#define ON      1
#define HIZ     2
#define ALLON   3
#define ALLOFF  4

int HATcloseDrvControl(struct hatCtrl *hatCtrl);
int HATopenDrvControl(char *SerialNumber, struct hatCtrl *hatCtrl);
int HATstopDataStream(struct hatCtrl *hatCtrl);
int HATstartDataStream(struct hatCtrl *hatCtrl, struct streamConfig *new_streamConfig);
int HATsetOuts(struct hatCtrl *hatCtrl);
int HATsetSensorDigOut(struct hatCtrl *hatCtrl, int ch, int state);
int HATreadSensorConfigFile(char *filename, struct streamConfig *streamConfig);
int HATreadDataCh(struct hatCtrl *hatCtrl, int ch, double *data, long size, char *str, int *overfull);
long HATgetSwitchStatus(struct hatCtrl *hatCtrl);

int HATOpticalDetectColor(struct hatCtrl *hatCtrl, double cX, double cY, double cZ, char *str);
double HATOpticalXYZtosRGB(double cX, double cY, double cZ, int c, int range);
int HATsensorDataFunction(struct hatCtrl *hatCtrl, int ch, double *value, char *str, int *error);
int HATsensors(struct hatCtrl *hatCtrl);
int HATsamplesToRead(struct hatCtrl *hatCtrl);
int HATpresetSwitchState(struct hatCtrl *hatCtrl, int sw_bit, int state);



double voltageToTemp(double volt, int state);

struct senparams {
    double multiplier0;
    double multiplier1;
    double offset;
    double (*convert)(double,int);
};

static const struct senparams sensor_params[] = {{ 1,                   1,                  0,       NULL            }, // None
                                                 { 1,                   1,                  0,       NULL            }, // Voltage
                                                 { CURRENT_SENSOR_M0,   CURRENT_SENSOR_M1,  0,       NULL            }, // Current
                                                 { 1,                   1,                  0,       &voltageToTemp  }, // Temp
                                                 { 1,                   1,                 -1238,    NULL            }, // Audio
                                                 {-0.10625,            -0.10625,           -2404,    NULL            }, // Optical3
                                                 { 1,                   1,                  0,       NULL            }};

int convert(struct shmem *shmem, int value, int ch, double *conv_value, char *value_str);


int getLatency(struct hatCtrl *hatCtrl, unsigned int ch, double *value, char *str);
int getAvg(struct hatCtrl *hatCtrl, unsigned int ch, double *value, char *str);
int getColor(struct hatCtrl *hatCtrl, unsigned int ch, double *value, char *str);

struct data_func {
    int (*data_function)(struct hatCtrl*, unsigned int ch, double*, char*);
};

static const struct data_func data_func_array[] = {{NULL,       },  // None
                                                   {&getLatency },  // Get latency function for audio sensor
                                                   {&getAvg     },  // Get average 
                                                   {&getColor   },  // Get optical sensor color 
                                                   {NULL,       }};

#endif
