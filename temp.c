/*
 * temp.c
 *
 *  Created on: Dec 02, 2016
 *      Author: Sayan Barman
 */



#include <MKL25Z4.h>
#include "stdint.h"
#include "temp.h"
#include "ftoa.h"

void dma_setup16(uint16_t*arr1,uint16_t*arr2,uint32_t length)
{
	SIM->SCGC6|=0x2u; //Enabling clock to DMAMUX
	SIM->SCGC7|=0x100u;//Enable clock for DMA
	DMAMUX0_CHCFG0 = 0x00; // Disable DMA Mux channel first
	DMA_SAR0 = arr1; // Set Source Address
	DMA_DSR_BCR0 |= length; //Select number of bytes to be transfered
	DMA_DCR0 &= ~(DMA_DCR_SSIZE_MASK | DMA_DCR_DSIZE_MASK);// Clear Source size and Destination size fields.
	NVIC->ISER[0]=0x1; //select interrupt for DMA channel 0
	DMA_DCR0 = 0x806C0000u; //Selecting 16 bit transfer mode and source/destination as incrementing after each transfer
	DMA_DAR0 = arr2; //Setting Destination address
	DMAMUX0_CHCFG0 |= 0x39u; //Using 60th option of always enabled DMA channel for channel 0
	DMAMUX0_CHCFG0 |= 0x80u; //enable the channel
	DMA_DCR0 |= 0x00010000u; //setting the start bit

}

void DMA0_IRQHandler(void)
{
	DMA_DSR_BCR0 |= 0x1000000u; //setting interrupt off
		if(DMA_DSR_BCR0 & 0x70000000)  //monitoring any error due to overlap of address
			print_stringl("DMA TRANSFER FAILURE\n\r"); //logging error message
	}

uint16_t inline adc_temp(void)
 {
	uint16_t snd=0,cab=0;
SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;

ADC0_CFG1 |= (ADC_CFG1_MODE(2)  |  ADC_CFG1_ADICLK(1)|	ADC_CFG1_ADIV(2)| ADC_CFG1_ADLSMP_MASK) ;

ADC0_SC3 |= ADC_SC3_AVGE_MASK |	ADC_SC3_AVGS(3)  |	ADC_SC3_CAL_MASK;

	while(ADC0_SC3 & ADC_SC3_CAL_MASK){}

		cab += ADC0_CLPS + ADC0_CLP4 + ADC0_CLP3 +ADC0_CLP2 + ADC0_CLP1 + ADC0_CLP0;
		cab /= 2;
		cab |= 0x8000;
		ADC0_PG = cab;
		cab = 0;
		cab += ADC0_CLMS + ADC0_CLM4 + ADC0_CLM3 +ADC0_CLM2 + ADC0_CLM1 + ADC0_CLM0;
		cab /= 2;
		cab |= 0x8000;
		ADC0_MG = cab;

		ADC0_CFG1 = 0;

		ADC0_CFG1 |= (ADC_CFG1_MODE(2)  |  ADC_CFG1_ADICLK(0)|	ADC_CFG1_ADIV(1)| ADC_CFG1_ADLSMP_MASK | ADC_CFG1_ADLPC_MASK) ;

ADC0_SC2 |= ADC_SC2_DMAEN_MASK;
ADC0_SC3 = 0;
ADC0_SC1A = 0x1A;
while(ADC0_SC2 & ADC_SC2_ADACT_MASK){}
while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)){}
dma_setup16(&ADC0_RA,&snd,2); // return ADC hex result
	snd= ADC0_RA;
	return snd ;
 }


void UART0_init(void)
	{
		SIM->SCGC4 |= 0x400;
		SIM->SOPT2 |= 0x4000000;

		UART0->C2 = 0;
		UART0->BDH = 0x00;
		UART0->BDL = 0x17;
		UART0->C4 = 0x0F;
		UART0->C1 = 0x00;
		UART0->C2 = 0x08;

		SIM->SCGC5 = 0x0200;
		PORTA->PCR[2]= 0x0200;
	}


float inline Conv_temp (float b)
{
	float t = (b-0.7012)/0.001646;
	return t;
	}



void Calc_temp()
{
	//while(1)
	//{
		uint16_t raw=0, len=0;
		float Vtemp=0,Temp=0,Temp1=0,Temp2=0,Tempf=0;
		char transfer[20];
		char *b=transfer;
	raw = adc_temp();
	Vtemp = raw * 0.0029296875 ;
	if (Vtemp >= 0.7012)
	{
		Temp1 = Conv_temp(Vtemp);
		Temp = 25 - Temp1 ;
		Tempf = ((Temp - 32)/1.8) +10;
	}
	else
	{
		Temp2 = Conv_temp(Vtemp);
		Temp = 25 - Temp2 ;
		Tempf = ((Temp - 32)/1.8) + 10;
	}
	print_stringl("\r\nTemperature is :");
	ftoa(Tempf, transfer, 2);
		while(*b!='\0')
		 {
			 len++;
			 b++;
		 }
		print_string(transfer, len);
	//}
}

