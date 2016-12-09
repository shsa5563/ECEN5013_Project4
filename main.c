/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include "MKL25Z4.h"
#include "capsense.h"
#include "temp.h"
#include "ftoa.h"
#include "derivative.h"
#include "accelerometer.h"
#include "I2C.h"
#include <math.h>
#include <main.h>
#define CompileTime
uint8_t test_func_result;

void delay()
{
	for(int j = 0; j<10000;j++)
					{

					}
}

int main()
{
	void (* delayptr) (void);
	void (*anticlockwise)(void);
		void (*clockwise)(void);
		delayptr = delay;
		anticlockwise=LED_on;
		clockwise=Calc_temp;
	DataReady = 0;
		MCU_Init();
	  	Accelerometer_Init();
	  	Calibrate();
		UART0_init();

int counter = 0;
int flag = 0;

	int cap_sense_touched=0;
	//Initialize TSI Sensor With channel 10
	capsense_init(1 << 10);
#ifndef CompileTime
test_func_result = test_touch();
if(test_func_result ==1)
	print_stringl("\r\nTouch true");
test_func_result = test_not_touch();
print_stringl("\r\nTouch false");
#endif
	while (1)
	{
		(*delayptr)();
		//ADC0_CFG1 &= ~ADC0_CFG1;
		cap_sense_touched = touch_data(10);
		if (cap_sense_touched > 0x0f)
		{
			if(counter == 0)
			{
				flag = 1;
				counter++;
			}
			else
			{
				flag = 0;
				counter=0;
			}
		}
		(*delayptr)();

		if(flag == 1)
		{
		//if (DataReady)
			(*delayptr)();
			//{
					DataReady = 0; // Is a new set of data ready?
					//GPIOB_PTOR |= 1<<18;

					I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06

					Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
					Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
					Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value

					Xout_g = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
					Yout_g = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
					Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's

					if((Zout_g<0) && (Yout_g>0))
						{
					(*anticlockwise)();
						}
					if((Zout_g<0) && (Yout_g<0))
					{
						GPIOB_PDDR &=~GPIOB_PDDR;
										(*clockwise)();
					}


					Pitch = atan2 (-Xout_g, sqrt (Yout_g*Yout_g + Zout_g*Zout_g)) * 180 / PI;		// Equation 37 in the AN3461
					if (Zout_g > 0)																	// Equation 38 in the AN3461
						Roll = atan2 (Yout_g, sqrt (0.01*Xout_g*Xout_g + Zout_g*Zout_g)) * 180 / PI;
					else
						Roll = atan2 (Yout_g, - sqrt (0.01*Xout_g*Xout_g + Zout_g*Zout_g)) * 180 / PI;

		}
		else
		{
			GPIOB_PDDR &=~GPIOB_PDDR;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

