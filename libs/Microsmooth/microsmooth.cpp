/*
Microsmooth, DSP library for Arduino
Copyright (C) 2013, Asheesh Ranjan, Pranav Jetley

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#include "microsmooth.h"
#include <Arduino.h>


SMAFilter::SMAFilter()
{
    for (int i = 0; i < SMA_LENGTH; i++) history[i] = 0;
}

//just takes the last few readings and averages them. Simple and easy
int32_t SMAFilter::calc(int32_t newVal)
{
    int64_t sum=0;
    int32_t average=0;
    uint8_t i;

    for(i = 1; i < SMA_LENGTH; i++)
    {
        history[i-1] = history[i];
    }
    history[SMA_LENGTH-1]= newVal;
    
    for(i = 0; i < SMA_LENGTH; i++)
    {
        sum += history[i];
    }
    average = sum / SMA_LENGTH;

    return average;
}

EMAFilter::EMAFilter()
{
    average = 0;
    alpha = EMA_ALPHA;
}

EMAFilter::EMAFilter(uint8_t val)
{
    average = 0;
    alpha = val;
}

int32_t EMAFilter::calc(int32_t newVal)
{   
    average = (alpha * (int32_t)newVal + (100 - alpha) * (int32_t)average) / 100;
    return average;

}

SGAFilter::SGAFilter()
{
    for(int i = 0; i < SGA_MAX_LENGTH; i++) normalization_value += sga_coefficients[SGA_INDEX][i]; /*Pre-calculating the normalization value*/
}

int32_t SGAFilter::calc(int32_t newVal)
{
    int64_t sum=0;
    uint8_t SGA_MID = SGA_LENGTH/2;
    int8_t i;

    for(i = 1; i < SGA_LENGTH; i++)
    {
        history[i-1] = history[i];
    }
    history[SGA_LENGTH-1] = newVal;
    
    for(i = -SGA_MID; i <= (SGA_MID); i++)
    {  
        sum += history[i + SGA_MID] * sga_coefficients[SGA_INDEX][i + SGA_MID];
        SerialUSB.println();
        SerialUSB.print(history[i + SGA_MID]);
        SerialUSB.print(" * ");
        SerialUSB.println(sga_coefficients[SGA_INDEX][i + SGA_MID]);
    }
    
    SerialUSB.print("Some, son: ");
    SerialUSB.println((int32_t)sum);
    
    history[SGA_MID] = sum / normalization_value;
    return history[SGA_MID];
}

       