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

/* This is modified Labjack's helper library */

#ifndef _U3_H
#define _U3_H

/* ------------------------------------------------------------------------- */
/* INCLUDES */
/* None */
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "labjackusb.h"


typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

//Structure for storing calibration constants
struct U3_CALIBRATION_INFORMATION {
    uint8 prodID;
    double hardwareVersion; //helps to determine which calibration calculations
                            //to use
    unsigned long serialNumber; //Labjack serial number
    int highVoltage;        //indicates if the device is U3-HV
    double ccConstants[20];
    /*
    Calibration constants order
    0 - LV AIN SE Slope
    1 - LV AIN SE Offset
    2 - LV AIN Diff Slope
    3 - LV AIN Diff Offset
    4 - DAC0 Slope
    5 - DAC0 Offset
    6 - DAC1 Slope
    7 - DAC1 Offset
    8 - Temp Slope
    9 - Vref @Cal
    10 - Vref*1.5 @Cal
    11 - Vreg @Cal
    12 - HV AIN0 Slope
    13 - HV AIN1 Slope
    14 - HV AIN2 Slope
    15 - HV AIN3 Slope
    16 - HV AIN0 Offset
    17 - HV AIN1 Offset
    18 - HV AIN2 Offset
    19 - HV AIN3 Offset
    */
};

typedef struct U3_CALIBRATION_INFORMATION u3CalibrationInfo;

//Structure for storing LJTDAC calibration constants
struct U3_TDAC_CALIBRATION_INFORMATION{
    uint8 prodID;
    double ccConstants[4];
    /*
    DAC Calibration constants order
    0 - SlopeA;
    1 - OffsetA;
    2 - SlopeB;
    3 - OffsetB;
    */
};

typedef struct U3_TDAC_CALIBRATION_INFORMATION u3TdacCalibrationInfo;


/* Functions */

void normalChecksum( uint8 *b,
                     int n);
//Adds checksum to a data packet for normal command format.
//b = data packet for normal command
//n = size of data packet

void extendedChecksum( uint8 *b,
                       int n);
//Adds checksum to a data packet for extended command format.
//b = data packet for extended command
//n = size of data packet

uint8 normalChecksum8( uint8 *b,
                       int n);
//Returns the Checksum8 for a normal command data packet.
//b = data packet for normal command
//n = size of data packet

uint16 extendedChecksum16( uint8 *b,
                           int n);
//Returns the Checksum16 for a extended command data packet.
//b = data packet for extended command
//n = size of data packet

uint8 extendedChecksum8( uint8 *b);
//Returns the Checksum8 for a extended command data packet.
//b = data packet for extended command

HANDLE openUSBConnection( int localID);
//Opens a U3 with the given localID.  Returns NULL on failure, or a HANDLE on
//success.

HANDLE openUSBConnectionDev(int devID);
//Opens a U3 with the given devID.  Returns NULL on failure, or a HANDLE on
//success.

// Resets device
int resetDev(HANDLE hDevice);

void closeUSBConnection( HANDLE hDevice);
//Closes a HANDLE to a U3 device.

long getTickCount();
//Returns the number of milliseconds that has elasped since the system was
//started.

long getCalibrationInfo( HANDLE hDevice,
                         u3CalibrationInfo *caliInfo);
//Gets calibration information from memory blocks 0-4 of a U3.  Returns the
//calibration information in a calibrationInfo structure.
//hDevice = handle to a U3 device
//caliInfo = structure where calibrarion information will be stored

double FPuint8ArrayToFPDouble( uint8 *buffer,
                               int startIndex);
//Converts a fixed point byte array (starting a startIndex) to a floating point
//double value.  This function is used primarily by getCalibrationInfo.

long isCalibrationInfoValid(u3CalibrationInfo *caliInfo);
//Performs a simple check to determine if the caliInfo struct was set up by
//getCalibrationInfo.  Returns 0 if caliInfo is not valid, or 1 if it is.
//caliInfo = structure where calibrarion information is stored

long isTdacCalibrationInfoValid(u3TdacCalibrationInfo *caliInfo);
//Performs a simple check to determine if the caliInfo struct was set up by
//getLJTDACCalibrationInfo.  Returns 0 if caliInfo is not valid, or 1 if it is.
//caliInfo = structure where LJTDAC calibration information is stored

long getAinVoltCalibrated( u3CalibrationInfo *caliInfo,
                           int dac1Enabled,
                           uint8 negChannel,
                           uint16 bytesVolt,
                           double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(calibrated) in Volts.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.  Function will also work
//for hardware version 1.30 U3-LV, but not U3-HV.
//caliInfo = structure where calibrarion information is stored
//dac1Enabled = If this is nonzero (True), then it is indicated that DAC1 is
//             enabled and analog voltage will be calculated with Vreg.  If
//             this is 0 (False), then it is indicated that DAC1 is disabled
//             and the analog voltage will be calculated with the AIN slopes
//             and offsets.
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getAinVoltCalibrated_hw130( u3CalibrationInfo *caliInfo,
                                 uint8 positiveChannel,
                                 uint8 negChannel,
                                 uint16 bytesVolt,
                                 double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(calibrated) in Volts.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//caliInfo = structure where calibrarion information is stored
//positiveChannel = the positive channel of the differential analog reading
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getDacBinVoltCalibrated( u3CalibrationInfo *caliInfo,
                              int dacNumber,
                              double analogVolt,
                              uint8 *bytesVolt);
//Translates a analog output voltage value (Volts) to a binary 8 bit value
//(calibrated) that can be sent to a U3.  Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20, 1.21 and 1.30, and does the
//same thing as the analogToCalibratedBinary8BitVoltage function.
//caliInfo = structure where calibrarion information is stored
//DACNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt = the converted binary 8 bit value

long getDacBinVoltCalibrated8Bit( u3CalibrationInfo *caliInfo,
                                  int dacNumber,
                                  double analogVolt,
                                  uint8 *bytesVolt8);
//Translates a analog output voltage value (Volts) to a binary 8 bit value
//(calibrated) that can be sent to a U3.  Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20, 1.21 and 1.30.
//caliInfo = structure where calibrarion information is stored
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt8 = the converted binary 8 bit value

long getDacBinVoltCalibrated16Bit( u3CalibrationInfo *caliInfo,
                                   int dacNumber,
                                   double analogVolt,
                                   uint16 *bytesVolt16);
//Translates a analog output voltage value (Volts) to a binary 16 bit value
//(calibrated) that can be sent to a U3. Call getCalibrationInfo first to set
//up caliInfo.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//caliInfo = structure where calibrarion information is stored
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt16 = the converted binary 16 bit value

long getTempKCalibrated( u3CalibrationInfo *caliInfo,
                         uint32 bytesTemp,
                         double *kelvinTemp);
//Translates the binary reading from the U3, to a temperature value
//(calibrated) in Kelvins.  Call getCalibrationInfo first to set up caliInfo.
//Returns -1 on error, 0 on success.
//caliInfo = structure where calibrarion information is stored
//bytesTemp = the 2 byte binary temperature that will be converted
//kelvinTemp = the converted Kelvin temperature


long getAinVoltUncalibrated( int dac1Enabled,
                             uint8 negChannel,
                             uint16 bytesVolt,
                             double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(uncalibrated) in Volts. Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.
//dac1Enabled = If this is nonzero (True), then it is indicated that DAC1 is
//              enabled and analog voltage will be calculated with Vreg.  If
//              this is 0 (False), then it is indicated that DAC1 is disabled
//              and the analog voltage will be calculated with the AIN slopes
//              and offsets.
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getAinVoltUncalibrated_hw130( int highVoltage,
                                   uint8 positiveChannel,
                                   uint8 negChannel,
                                   uint16 bytesVolt,
                                   double *analogVolt);
//Translates the binary AIN reading from the U3, to a voltage value
//(uncalibrated) in Volts. Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//highVoltage = Set to 1 to indicate that U3-HV calculations should be used
//              for the binary to voltage conversion, otherwise U3-LV voltage
//              calculations will be used.
//positiveChannel = the positive channel of the differential analog reading
//negChannel = the negative channel of the differential analog reading
//bytesVolt = the 2 byte voltage that will be converted
//analogVolt = the converted analog voltage

long getDacBinVoltUncalibrated( int dacNumber,
                                double analogVolt,
                                uint8 *bytesVolt);
//Translates a DAC voltage value (Volts) to a binary 8 bit value (uncalibrated)
//that can be sent to a U3.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21, and does the same
//thing as the analogToUncalibratedBinary8BitVoltage function.
//dacNumber - channel number of the DAC
//analogVolt = the analog voltage that will be converted
//bytesVolt = the converted binary 8 bit value

long getDacBinVoltUncalibrated8Bit( int dacNumber,
                                    double analogVolt,
                                    uint8 *bytesVolt8);
//Translates a DAC voltage value (Volts) to a binary 8 bit value (uncalibrated)
//that can be sent to a U3.  Returns -1 on error, 0 on success.
//This function is for U3 hardware versions 1.20 and 1.21.
//dacNumber - channel number of the DAC
//analogVoltage = the analog voltage that will be converted
//bytesVoltage = the converted binary 8 bit value

long getDacBinVoltUncalibrated16Bit( int dacNumber,
                                     double analogVolt,
                                     uint16 *bytesVolt16);
//Translates a DAC voltage value (Volts) to a binary 16 bit value
//(uncalibrated) that can be sent to a U3-LV/HV.  Returns -1 on error, 0 on
//success.
//This function is for U3 hardware versions 1.30 (U3-LV/HV).
//dacNumber - channel number of the DAC
//analogVoltage = the analog voltage that will be converted
//bytesVoltage = the converted binary 16 bit value




/* Easy Function Helpers */



long ehFeedback( HANDLE hDevice,
                 uint8 *inIOTypesDataBuff,
                 long inIOTypesDataSize,
                 uint8 *outErrorcode,
                 uint8 *outErrorFrame,
                 uint8 *outDataBuff,
                 long outDataSize);
//Used by the all of the easy functions.  This function takes the Feedback
//low-level command and response bytes (not including checksum and command
//bytes) as its parameter and performs a Feedback call with the U3.  Returns -1
//or errorcode (>1 value) on error, 0 on success.


/* Easy function constants */


/* Timer clocks for Hardware Version 1.20 or lower */

// 2 MHz
#define LJ_tc2MHZ 10

// 6 MHz
#define LJ_tc6MHZ 11

// 24 MHz
#define LJ_tc24MHZ 12

// 500/Divisor KHz
#define LJ_tc500KHZ_DIV 13

// 2/Divisor MHz
#define LJ_tc2MHZ_DIV 14

// 6/Divisor MHz
#define LJ_tc6MHZ_DIV 15

// 24/Divisor MHz
#define LJ_tc24MHZ_DIV 16


/* Timer clocks for Hardware Version 1.21 or higher */

// 4 MHz
#define LJ_tc4MHZ 20

// 12 MHz
#define LJ_tc12MHZ 21

// 48 MHz
#define LJ_tc48MHZ 22

// 1/Divisor MHz
#define LJ_tc1MHZ_DIV 23

// 4/Divisor MHz
#define LJ_tc4MHZ_DIV 24

// 12/Divisor MHz
#define LJ_tc12MHZ_DIV 25

// 48/Divisor MHz
#define LJ_tc48MHZ_DIV 26


/* Timer modes */

// 16 bit PWM
#define LJ_tmPWM16 0

// 8 bit PWM
#define LJ_tmPWM8 1

// 32-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES32 2

// 32-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES32 3

// duty cycle measurement
#define LJ_tmDUTYCYCLE 4

// firmware based rising edge counter
#define LJ_tmFIRMCOUNTER 5

// firmware counter with debounce
#define LJ_tmFIRMCOUNTERDEBOUNCE 6

// frequency output
#define LJ_tmFREQOUT 7

// Quadrature
#define LJ_tmQUAD 8

// stops another timer after n pulses
#define LJ_tmTIMERSTOP 9

// read lower 32-bits of system timer
#define LJ_tmSYSTIMERLOW 10

// read upper 32-bits of system timer
#define LJ_tmSYSTIMERHIGH 11

// 16-bit rising to rising edge measurement
#define LJ_tmRISINGEDGES16 12

// 16-bit falling to falling edge measurement
#define LJ_tmFALLINGEDGES16 13

#endif

/* ------------------------------------------------------------------------- */
/* End of file */
