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

/* This is modified Labjack's helper library */

#include "u3.h"
#include <stdlib.h>
#include <time.h>


u3CalibrationInfo U3_CALIBRATION_INFO_DEFAULT = {
    3,
    1.31,
    0,
    0,
    //Nominal Values
    {   0.000037231,
        0.0,
        0.000074463,
        -2.44,
        51.717,
        0.0,
        51.717,
        0.0,
        0.013021,
        2.44,
        3.66,
        3.3,
        0.000314,
        0.000314,
        0.000314,
        0.000314,
        -10.3,
        -10.3,
        -10.3,
        -10.3}
};


void normalChecksum(uint8 *b, int n)
{
    b[0] = normalChecksum8(b,n);
}


void extendedChecksum(uint8 *b, int n)
{
    uint16 a;

    a = extendedChecksum16(b,n);
    b[4] = (uint8)(a & 0xff);
    b[5] = (uint8)((a/256) & 0xff);
    b[0] = extendedChecksum8(b);
}


uint8 normalChecksum8(uint8 *b, int n)
{
    int i;
    uint16 a, bb;

    //Sums bytes 1 to n-1 unsigned to a 2 byte value. Sums quotient and
    //remainder of 256 division.  Again, sums quotient and remainder of
    //256 division.
    for(i = 1, a = 0; i < n; i++)
        a+=(uint16)b[i];

    bb = a/256;
    a = (a-256*bb)+bb;
    bb = a/256;

    return (uint8)((a-256*bb)+bb);
}


uint16 extendedChecksum16(uint8 *b, int n)
{
    int i, a = 0;

    //Sums bytes 6 to n-1 to a unsigned 2 byte value
    for(i = 6; i < n; i++)
        a += (uint16)b[i];

    return a;
}


uint8 extendedChecksum8(uint8 *b)
{
    int i, a, bb;

    //Sums bytes 1 to 5. Sums quotient and remainder of 256 division. Again, sums
    //quotient and remainder of 256 division.
    for(i = 1, a = 0; i < 6; i++)
        a+=(uint16)b[i];

    bb=a/256;
    a=(a-256*bb)+bb;
    bb=a/256;

    return (uint8)((a-256*bb)+bb);
}


HANDLE openUSBConnection(int localID)
{
    BYTE sendBuffer[26], recBuffer[38];
    uint16 checksumTotal = 0;
    HANDLE hDevice = 0;
    uint32 numDevices = 0;
    uint32 dev;
    int i;

    numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
    if(numDevices == 0)
    {
        printf("Open error: No U3 devices could be found\n");
        return NULL;
    }

    for(dev = 1;  dev <= numDevices; dev++)
    {
        hDevice = LJUSB_OpenDevice(dev, 0, U3_PRODUCT_ID);
        if(hDevice != NULL)
        {
            if(localID < 0)
            {
                return hDevice;
            }
            else
            {
                checksumTotal = 0;

                //setting up a CommConfig command
                sendBuffer[1] = (uint8)(0xF8);
                sendBuffer[2] = (uint8)(0x0A);
                sendBuffer[3] = (uint8)(0x08);

                for(i = 6; i < 26; i++)
                    sendBuffer[i] = (uint8)(0x00);

                extendedChecksum(sendBuffer, 26);

                if(LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 26) != 26)
                    goto locid_error;

                if(LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuffer, 38) != 38)
                    goto locid_error;

                checksumTotal = extendedChecksum16(recBuffer, 38);
                if( (uint8)((checksumTotal / 256) & 0xff) != recBuffer[5])
                    goto locid_error;

                if( (uint8)(checksumTotal & 0xff) != recBuffer[4])
                    goto locid_error;

                if( extendedChecksum8(recBuffer) != recBuffer[0])
                    goto locid_error;

                if( recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x10) ||
                    recBuffer[3] != (uint8)(0x08) )
                    goto locid_error;

                if( recBuffer[6] != 0)
                    goto locid_error;

                if( (int)recBuffer[21] == localID) {
                    return hDevice;
                } else {
                	printf("This U3 ID: %d\n", (int)recBuffer[21]);
                    LJUSB_CloseDevice(hDevice);
                }
            } //else localID >= 0 end
        } //if hDevice != NULL end
    } //for end

    printf("Open error: could not find a U3 with a local ID of %d\n", localID);
    return NULL;

locid_error:
    printf("Open error: problem when checking local ID\n");
    return NULL;
}


HANDLE openUSBConnectionDev(int devID)
{
    HANDLE hDevice = 0;
    uint32 numDevices = 0;

    numDevices = LJUSB_GetDevCount(U3_PRODUCT_ID);
    if(numDevices == 0)
    {
        printf("Open error: No U3 devices could be found\n");
        return NULL;
    }
    if (devID <= numDevices) {
        hDevice = LJUSB_OpenDevice(devID, 0, U3_PRODUCT_ID);
        if(hDevice != NULL) {
			return hDevice;
		} else {
            printf("Open error: could not find a U3 with a devID of %d\n", devID);
            return NULL;
		}
    } else {
    	//printf("Open error: Only %d U3 devices found\n", numDevices);
        return NULL;

    }
}


void closeUSBConnection(HANDLE hDevice)
{
    LJUSB_CloseDevice(hDevice);
}


long getTickCount()
{
    struct timespec tp;    
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (tp.tv_sec * 1000) + (tp.tv_nsec / 1000000);
}


long isCalibrationInfoValid(u3CalibrationInfo *caliInfo)
{
    if(caliInfo == NULL)
        goto invalid;
    if(caliInfo->prodID != 3)
        goto invalid;
    return 1;
invalid:
    printf("Error: Invalid calibration info.\n");
    return -1;
}


long isTdacCalibrationInfoValid(u3TdacCalibrationInfo *caliInfo)
{
    if(caliInfo == NULL)
        goto invalid;
    if(caliInfo->prodID != 3)
        goto invalid;
    return 1;
invalid:
    printf("Error: Invalid LJTDAC calibration info.\n");
    return -1;
}


int resetDev(HANDLE hDevice)
{
    //uint8 sendBuffer[8], recBuffer[40];
    uint8 cU3SendBuffer[26], cU3RecBuffer[38];
    int sentRec = 0;
    //int i = 0, offset = 0;

    /* sending ConfigU3 command to get hardware version and see if HV */
    cU3SendBuffer[1] = (uint8)(0x99);  //command byte
    cU3SendBuffer[2] = (uint8)(0x02);  //hard reset
    cU3SendBuffer[3] = (uint8)(0x00);  //

    normalChecksum(cU3SendBuffer, 4);

    sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, cU3SendBuffer, 4);
    if(sentRec < 4)
    {
        printf("Send error (%d)\n",sentRec);
        return -1;
    }

    sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, cU3RecBuffer, 4);
    if(sentRec < 4)
    {
        printf("Read error\n");
        return -1;
    }

    if(cU3RecBuffer[1] != (uint8)(0x99) || cU3RecBuffer[2] != (uint8)(0x00))
    {
        printf("Received data error");
        return -1;
    }
    else {
        printf("Reset ok\n");
    }
    return 0;
}

long getCalibrationInfo(HANDLE hDevice, u3CalibrationInfo *caliInfo)
{
    uint8 sendBuffer[8], recBuffer[40];
    uint8 cU3SendBuffer[26], cU3RecBuffer[38];
    int sentRec = 0;
    int i = 0, offset = 0;

    /* sending ConfigU3 command to get hardware version and see if HV */
    cU3SendBuffer[1] = (uint8)(0xF8);  //command byte
    cU3SendBuffer[2] = (uint8)(0x0A);  //number of data words
    cU3SendBuffer[3] = (uint8)(0x08);  //extended command number

    //setting WriteMask0 and all other bytes to 0 since we only want to read the response
    for(i = 6; i < 26; i++)
        cU3SendBuffer[i] = 0;

    extendedChecksum(cU3SendBuffer, 26);

    sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, cU3SendBuffer, 26);
    if(sentRec < 26)
    {
        if(sentRec == 0)
            goto writeError0;
        else
            goto writeError1;
    }

    sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, cU3RecBuffer, 38);
    if(sentRec < 38)
    {
        if(sentRec == 0)
            goto readError0;
        else
            goto readError1;
    }

    if(cU3RecBuffer[1] != (uint8)(0xF8) || cU3RecBuffer[2] != (uint8)(0x10) || cU3RecBuffer[3] != (uint8)(0x08))
        goto commandByteError;

    caliInfo->hardwareVersion = cU3RecBuffer[14] + cU3RecBuffer[13]/100.0;
    caliInfo->serialNumber = (cU3RecBuffer[18] << 24) | (cU3RecBuffer[17] << 16) | (cU3RecBuffer[16] << 8) | (cU3RecBuffer[15] << 0);
    caliInfo->highVoltage = (((cU3RecBuffer[37]&18) == 18)?1:0);

    for(i = 0; i < 5; i++)
    {
        /* reading block i from memory */
        sendBuffer[1] = (uint8)(0xF8);  //command byte
        sendBuffer[2] = (uint8)(0x01);  //number of data words
        sendBuffer[3] = (uint8)(0x2D);  //extended command number
        sendBuffer[6] = 0;
        sendBuffer[7] = (uint8)i;       //Blocknum = i
        extendedChecksum(sendBuffer, 8);

        sentRec = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuffer, 8);
        if(sentRec < 8)
        {
            if(sentRec == 0)
                goto writeError0;
            else
                goto writeError1;
        }

        sentRec = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuffer, 40);
        if(sentRec < 40)
        {
            if(sentRec == 0)
                goto readError0;
            else
                goto readError1;
        }

        if(recBuffer[1] != (uint8)(0xF8) || recBuffer[2] != (uint8)(0x11) || recBuffer[3] != (uint8)(0x2D))
            goto commandByteError;


        offset = i*4;

        //block data starts on byte 8 of the buffer
        caliInfo->ccConstants[offset] = FPuint8ArrayToFPDouble(recBuffer + 8, 0);
        caliInfo->ccConstants[offset + 1] = FPuint8ArrayToFPDouble(recBuffer + 8, 8);
        caliInfo->ccConstants[offset + 2] = FPuint8ArrayToFPDouble(recBuffer + 8, 16);
        caliInfo->ccConstants[offset + 3] = FPuint8ArrayToFPDouble(recBuffer + 8, 24);
    }

    caliInfo->prodID = 3;

    return 0;

writeError0:
    printf("Error : getCalibrationInfo write failed\n");
    return -1;
writeError1:
    printf("Error : getCalibrationInfo did not write all of the buffer (%d)\n",sentRec);
    return -1;
readError0:
    printf("Error : getCalibrationInfo read failed\n");
    return -1;
readError1:
    printf("Error : getCalibrationInfo did not read all of the buffer\n");
    return -1;
commandByteError:
    printf("Error : getCalibrationInfo received wrong command bytes for ReadMem\n");
    return -1;
}

double FPuint8ArrayToFPDouble(uint8 *buffer, int startIndex)
{
    uint32 resultDec = 0;
    uint32 resultWh = 0;
    int i;

    for(i = 0; i < 4; i++)
    {
        resultDec += (uint32)buffer[startIndex + i] * pow(2, i*8);
        resultWh += (uint32)buffer[startIndex + i + 4] * pow(2, i*8);
    }

    return ( (double)((int)resultWh) + (double)(resultDec)/4294967296.0 );
}


long getAinVoltCalibrated(u3CalibrationInfo *caliInfo, int dacEnabled, uint8 negChannel, uint16 bytesVolt, double *analogVolt)
{
    if(isCalibrationInfoValid(caliInfo) == -1)
        return -1;

    if(caliInfo->hardwareVersion >= 1.30)
    {
        if(caliInfo->highVoltage == 1)
        {
            printf("getAinVoltCalibrated error: cannot handle U3-HV device.  Please use getAinVoltCalibrated_hw130 function.\n");
            return -1;
        }
        else
            return getAinVoltCalibrated_hw130(caliInfo, 0, negChannel, bytesVolt, analogVolt);
    }

    if(negChannel <= 15 || negChannel == 30)
    {
        if(dacEnabled == 0)
            *analogVolt = caliInfo->ccConstants[2]*((double)bytesVolt) + caliInfo->ccConstants[3];
        else
            *analogVolt = (bytesVolt/65536.0)*caliInfo->ccConstants[11]*2.0 - caliInfo->ccConstants[11];
    }
    else if(negChannel == 31)
    {
        if(dacEnabled == 0)
            *analogVolt = caliInfo->ccConstants[0]*((double)bytesVolt) + caliInfo->ccConstants[1];
        else
            *analogVolt = (bytesVolt/65536.0)*caliInfo->ccConstants[11];
    }
    else
    {
        printf("getAinVoltCalibrated error: invalid negative channel.\n");
        return -1;
    }

    return 0;
}


long getAinVoltCalibrated_hw130(u3CalibrationInfo *caliInfo, uint8 positiveChannel, uint8 negChannel, uint16 bytesVolt, double *analogVolt)
{
    if(isCalibrationInfoValid(caliInfo) == -1)
        return -1;

    if(negChannel <= 15 || negChannel == 30)
    {
        if(caliInfo->highVoltage == 0 || (caliInfo->highVoltage == 1 && positiveChannel >= 4 && negChannel >= 4)) {
            *analogVolt = caliInfo->ccConstants[2]*((double)bytesVolt) + caliInfo->ccConstants[3]  + caliInfo->ccConstants[9];
        } 
        else if(caliInfo->hardwareVersion >= 1.30 && caliInfo->highVoltage == 1)
        {
            printf("getAinVoltCalibrated_hw130 error: invalid negative channel for U3-HV.\n");
            return -1;
        }
    }
    else if(negChannel == 31)
    {
        if(caliInfo->highVoltage == 1 && positiveChannel < 4)
            *analogVolt = caliInfo->ccConstants[12+positiveChannel]*((double)bytesVolt) + caliInfo->ccConstants[16+positiveChannel];
        else {
            if (bytesVolt == 0) {
                *analogVolt = 0; //caliInfo->ccConstants[0]*((double)bytesVolt);
            }
            else {
                *analogVolt = caliInfo->ccConstants[0]*((double)bytesVolt) + caliInfo->ccConstants[1];
            }
        }
    }
    else if(negChannel == 32)  //special range (binary value should be from a differential AIN reading with negative channel 30)
    {
        if(caliInfo->highVoltage == 1 && positiveChannel < 4)
        {
            *analogVolt = (caliInfo->ccConstants[2]*((double)bytesVolt) + caliInfo->ccConstants[3] + caliInfo->ccConstants[9]) * caliInfo->ccConstants[12 + positiveChannel] / caliInfo->ccConstants[0] +
                            caliInfo->ccConstants[16 + positiveChannel];
        }
        else
        {
            *analogVolt = caliInfo->ccConstants[2]*((double)bytesVolt) + caliInfo->ccConstants[3] + caliInfo->ccConstants[9];
        }
    }
    else
    {
        printf("getAinVoltCalibrated_hw130 error: invalid negative channel.\n");
        return -1;
    }

    return 0;
}


long getDacBinVoltCalibrated(u3CalibrationInfo *caliInfo, int dacNumber, double analogVolt, uint8 *bytesVolt)
{
    return getDacBinVoltCalibrated8Bit(caliInfo, dacNumber, analogVolt, bytesVolt);
}


long getDacBinVoltCalibrated8Bit(u3CalibrationInfo *caliInfo, int dacNumber, double analogVolt, uint8 *bytesVolt8)
{
    double tBytesVolt;

    if(isCalibrationInfoValid(caliInfo) == -1)
        return -1;

    if(dacNumber < 0 || dacNumber > 2)
    {
        printf("getDacBinVoltCalibrated8Bit error: invalid channelNumber.\n");
        return -1;
    }
    tBytesVolt = analogVolt*caliInfo->ccConstants[4 + dacNumber*2] +   caliInfo->ccConstants[5 + dacNumber*2];

    //Checking to make sure bytesVolt will be a value between 0 and 255.  Too
    //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
    //volts) will cause a value not between 0 and 255.
    if(tBytesVolt < 0)
        tBytesVolt = 0;
    else if(tBytesVolt > 255 && caliInfo->hardwareVersion < 1.30)
        tBytesVolt = 255;

    *bytesVolt8 = (uint8)tBytesVolt;

    return 0;
}


long getDacBinVoltCalibrated16Bit(u3CalibrationInfo *caliInfo, int dacNumber, double analogVolt, uint16 *bytesVolt16)
{
    double tBytesVolt;

    if(isCalibrationInfoValid(caliInfo) == -1)
        return -1;

    if(dacNumber < 0 || dacNumber > 2)
    {
        printf("getDacBinVoltCalibrated16Bit error: invalid channelNumber.\n");
        return -1;
    }

    if(caliInfo->hardwareVersion < 1.30)
        tBytesVolt = analogVolt*caliInfo->ccConstants[4 + dacNumber*2] +   caliInfo->ccConstants[5 + dacNumber*2];
    else
        tBytesVolt = analogVolt*caliInfo->ccConstants[4 + dacNumber*2]*256 +   caliInfo->ccConstants[5 + dacNumber*2]*256;
     
     //Checking to make sure bytesVolt will be a value between 0 and 255/65535.  Too
    //high of an analogVoltage (about 4.95 and above volts) or too low (below 0
    //volts) will cause a value not between 0 and 255/65535.
    if(tBytesVolt < 0)
        tBytesVolt = 0;
    if(tBytesVolt > 65535 && caliInfo->hardwareVersion >= 1.30)
        tBytesVolt = 65535;
    else if(tBytesVolt > 255 && caliInfo->hardwareVersion < 1.30)
        tBytesVolt = 255;

    *bytesVolt16 = (uint16)tBytesVolt;

    return 0;
}

long getAinVoltUncalibrated(int dacEnabled, uint8 negChannel, uint16 bytesVolt, double *analogVolt)
{
    U3_CALIBRATION_INFO_DEFAULT.hardwareVersion = 1.20;
    U3_CALIBRATION_INFO_DEFAULT.highVoltage = 0;
    return getAinVoltCalibrated(&U3_CALIBRATION_INFO_DEFAULT, dacEnabled, negChannel, bytesVolt, analogVolt);
}


long getAinVoltUncalibrated_hw130(int highVoltage, uint8 positiveChannel, uint8 negChannel, uint16 bytesVolt, double *analogVolt)
{
    U3_CALIBRATION_INFO_DEFAULT.hardwareVersion = 1.30;
    U3_CALIBRATION_INFO_DEFAULT.highVoltage = highVoltage;
    return getAinVoltCalibrated_hw130(&U3_CALIBRATION_INFO_DEFAULT, positiveChannel, negChannel, bytesVolt, analogVolt);
}


long getDacBinVoltUncalibrated(int dacNumber, double analogVolt, uint8 *bytesVolt)
{
    U3_CALIBRATION_INFO_DEFAULT.hardwareVersion = 1.20;
    U3_CALIBRATION_INFO_DEFAULT.highVoltage = 0;
    return getDacBinVoltCalibrated(&U3_CALIBRATION_INFO_DEFAULT, dacNumber, analogVolt, bytesVolt);
}


long getDacBinVoltUncalibrated8Bit(int dacNumber, double analogVolt, uint8 *bytesVolt8)
{
    U3_CALIBRATION_INFO_DEFAULT.hardwareVersion = 1.20;
    U3_CALIBRATION_INFO_DEFAULT.highVoltage = 0;
    return getDacBinVoltCalibrated8Bit(&U3_CALIBRATION_INFO_DEFAULT, dacNumber, analogVolt, bytesVolt8);
}


long getDacBinVoltUncalibrated16Bit(int dacNumber, double analogVolt, uint16 *bytesVolt16)
{
    U3_CALIBRATION_INFO_DEFAULT.hardwareVersion = 1.30;
    U3_CALIBRATION_INFO_DEFAULT.highVoltage = 0;
    return getDacBinVoltCalibrated16Bit(&U3_CALIBRATION_INFO_DEFAULT, dacNumber, analogVolt, bytesVolt16);
}

long ehFeedback(HANDLE hDevice, uint8 *inIOTypesDataBuff, long inIOTypesDataSize, uint8 *outErrorcode, uint8 *outErrorFrame, uint8 *outDataBuff, long outDataSize)
{
    uint8 *sendBuff, *recBuff;
    uint16 checksumTotal;
    int sendChars, recChars, i, sendDWSize, recDWSize, commandBytes, ret;

    ret = 0;
    commandBytes = 6;

    if(((sendDWSize = inIOTypesDataSize + 1)%2) != 0)
        sendDWSize++;
    if(((recDWSize = outDataSize + 3)%2) != 0)
        recDWSize++;

    sendBuff = malloc(sizeof(uint8)*(commandBytes + sendDWSize));
    recBuff = malloc(sizeof(uint8)*(commandBytes + recDWSize));
    if(sendBuff == NULL || recBuff == NULL)
    {
        ret = -1;
        goto cleanmem;
    }

    sendBuff[sendDWSize + commandBytes - 1] = 0;

    // Setting up Feedback command
    sendBuff[1] = (uint8)(0xF8);  //Command byte
    sendBuff[2] = sendDWSize/2;   //Number of data words (.5 word for echo, 1.5
                                //words for IOTypes)
    sendBuff[3] = (uint8)(0x00);  //Extended command number

    sendBuff[6] = 0;    //Echo

    for(i = 0; i < inIOTypesDataSize; i++)
        sendBuff[i+commandBytes+1] = inIOTypesDataBuff[i];

    extendedChecksum(sendBuff, (sendDWSize+commandBytes));

    //Sending command to U3
    if( (sendChars = LJUSB_BulkWrite(hDevice, U3_PIPE_EP1_OUT, sendBuff, (sendDWSize+commandBytes))) < sendDWSize+commandBytes)
    {
        if(sendChars == 0)
            printf("ehFeedback error : write failed\n");
        else
            printf("ehFeedback error : did not write all of the buffer\n");
        ret = -1;
        goto cleanmem;
    }

    //Reading response from U3
    if( (recChars = LJUSB_BulkRead(hDevice, U3_PIPE_EP2_IN, recBuff, (commandBytes+recDWSize))) < commandBytes+recDWSize)
    {
        if(recChars == -1)
        {
            printf("ehFeedback error : read failed\n");
            ret = -1;
            goto cleanmem;
        }
        else if(recChars < 8)
        {
            printf("ehFeedback error : response buffer is too small\n");
            ret = -1;
            goto cleanmem;
        }
        else
            printf("ehFeedback error : did not read all of the expected buffer (received %d, expected %d )\n", recChars, commandBytes+recDWSize);
    }

    checksumTotal = extendedChecksum16(recBuff, recChars);
    if( (uint8)((checksumTotal / 256 ) & 0xff) != recBuff[5])
    {
        printf("ehFeedback error : read buffer has bad checksum16(MSB)\n");
        ret = -1;
        goto cleanmem;
    }

    if( (uint8)(checksumTotal & 0xff) != recBuff[4])
    {
        printf("ehFeedback error : read buffer has bad checksum16(LBS)\n");
        ret = -1;
        goto cleanmem;
    }

    if( extendedChecksum8(recBuff) != recBuff[0])
    {
        printf("ehFeedback error : read buffer has bad checksum8\n");
        ret = -1;
        goto cleanmem;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[3] != (uint8)(0x00) )
    {
        printf("ehFeedback error : read buffer has wrong command bytes \n");
        ret = -1;
        goto cleanmem;
    }

    *outErrorcode = recBuff[6];
    *outErrorFrame = recBuff[7];

    for(i = 0; i+commandBytes+3 < recChars && i < outDataSize; i++)
        outDataBuff[i] = recBuff[i+commandBytes+3];

cleanmem:
    free(sendBuff);
    free(recBuff);
    sendBuff = NULL;
    recBuff = NULL;

    return ret;
}


/* ------------------------------------------------------------------------- */
/* End of file */
