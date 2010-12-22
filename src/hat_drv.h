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
#ifndef HATDRVH
#define HATDRVH

// Current sensor multipliers
// R4 = 141k (47k x 3)
#define LO_GAIN_MULT    0.60024 // R9 = 6k6 (2k2 x 3)
#define HI_GAIN_MULT    0.04058 // R8 = 287k

// Maximun input voltge
#define MAX_VALUE   2467

// Command for driver
#define NONE            0
#define CHANGE_IO       1
#define START_STREAMING 2
#define STOP_STREAMING  3

// Sensor types
#define NONE            0
#define VOLTAGE         1
#define CURRENT         2
#define MAX_SENSOR      3
//#define LIGHT           3
//#define ACCELERATION    4
//#define TEMPERATURE     5



// Analog inputs
#define AI0             0
#define AI1             2
#define AI2             4
#define AI3             6
const int ach[] = {AI0, AI1, AI2, AI3};

// Digital IO lines
#define AN0_IO          8
#define AN1_IO          9
#define AN2_IO          10
#define AN3_IO          11 
#define USB_DATA_1      12
#define USB_DATA_2      13
#define USB_PWR_1       14
#define USB_PWR_2       15
#define DC_POWER_SHDN1  16
#define DC_POWER_SHDN2  17

#define PWR_USB_IOS     ((1 << USB_DATA_1) | (1 << USB_DATA_2) | (1 << USB_PWR_1) | (1 << USB_PWR_2) | (1 << DC_POWER_SHDN1) | (1 << DC_POWER_SHDN2))
#define SENSOR_IOS      ((1 << AN0_IO) | (1 << AN1_IO) | (1 << AN2_IO) | (1 << AN3_IO))
#endif

