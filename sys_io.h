/*
 * sys_io.h
 *
 * Handles raw interaction with system I/O
 *
Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */


#ifndef SYS_IO_H_
#define SYS_IO_H_

#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include "eeprom_layout.h"
#include "PrefHandler.h"
#include "Logger.h"

#define CS1	26
#define CS2	28
#define CS3	30

#define ADE7913_WRITE	0
#define ADE7913_READ	4

#define ADE7913_AMP_READING	0
#define ADE7913_ADC1_READING	1 << 3
#define ADE7913_ADC2_READING	2 << 3
#define ADE7913_ADC_CRC		4 << 3
#define ADE7913_CTRL_CRC	5 << 3
#define ADE7913_CNT_SNAP	7 << 3
#define ADE7913_CONFIG		8 << 3
#define ADE7913_STATUS0		9 << 3
#define ADE7913_LOCK		0xA << 3
#define ADE7913_SYNC_SNAP	0xB << 3
#define ADE7913_COUNTER0	0xC << 3
#define ADE7913_COUNTER1	0xD << 3
#define ADE7913_EMI_CTRL	0xE << 3
#define ADE7913_STATUS1		0xF << 3
#define ADE7913_TEMPOS		0x18 << 3

typedef struct {
    uint16_t offset;
    uint16_t gain;
} ADC_COMP;

void setup_sys_io();
void setup_ADC_params();
uint16_t getAnalog(uint8_t which); //get value of one of the 4 analog inputs
int32_t getCurrentReading();
int32_t getPackHighReading();
int32_t getPackLowReading();
uint16_t getDiffADC(uint8_t which);
uint16_t getRawADC(uint8_t which);
boolean getDigital(uint8_t which); //get value of one of the 4 digital inputs
void setOutput(uint8_t which, boolean active); //set output high or not
boolean getOutput(uint8_t which); //get current value of output state (high?)
void setupFastADC();
bool setupSPIADC();
int32_t getSPIADCReading(int CS, int sensor);
void sys_io_adc_poll();
void sys_early_setup();
void sys_boot_setup();

#endif
