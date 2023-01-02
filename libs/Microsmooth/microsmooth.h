/*
Microsmooth, DSP library for Arduino
Copyright (C) 2013, Asheesh Ranjan, Pranav Jetley

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

/*Standard Library Dependencies*/
#include <stdint.h>
//#include <math.h>
#include <stdlib.h>

#ifndef MICROSMOOTH
#define MICROSMOOTH

/*
These parameters should be tuned depending on need. Each of these parameters affects
run time and signal smoothing obtained. See documentation for specific instructions 
on tuning these parameters.
*/

/*Simple Moving Average - Length of window */
#ifndef SMA_LENGTH
#define SMA_LENGTH 5
#endif

/*Exponential Moving Average - Alpha parameter */
#ifndef EMA_ALPHA
#define EMA_ALPHA 10 /*This is in percentage. Should be between 0-99*/
#endif

/*Savitzky Golay Filter -  */
#ifndef SGA_LENGTH
#define SGA_LENGTH 5 /* Window may 5, 7 or 9. 
             For window 5, only quadratic or cubic smoothing may be used */
#endif

#ifndef SGA_DEGREE
#define SGA_DEGREE 3 /* For quadratic or cubic smoothing, enter degree 3. 
            For quartic or quintic smoothing, enter degree 4.*/
#endif

#ifndef SGA_INDEX
#define SGA_INDEX (SGA_DEGREE - SGA_LENGTH + 2)
#endif

#define SGA_MAX_LENGTH 9 /* Do not change */

//simple moving average
class SMAFilter {
public:
    SMAFilter();
    int32_t calc(int32_t);
private:
    int32_t history[SMA_LENGTH];
};

//Exponential moving average
class EMAFilter {
public:
    EMAFilter(uint8_t val);
    EMAFilter();
    int32_t calc(int32_t);
private:
    int32_t average;
    uint8_t alpha;
};

//Savitzky Golay filter
class SGAFilter {
public:
    SGAFilter();
    int32_t calc(int32_t);
private:
    /* SGA Coefficients*/
    const int16_t sga_coefficients[5][SGA_MAX_LENGTH]={
        {0, 0, -3, 12, 17, 12, -3, 0, 0},
        {-21, 14, 39, 54, 59, 54, 39, 14, -21},
        {15, -55, 30, 135, 179, 135, 30, -55, 15},
        {0, -2, 3, 6, 7, 6, 3, -2, 0},
        {0, 5, -30, 75, 131, 75, -30, 5, 0},
    };
    uint16_t normalization_value;
    int32_t history[SGA_LENGTH];
};

#endif
