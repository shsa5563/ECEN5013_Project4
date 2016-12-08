///*
// * capsense.c
// *
// *  Created on: 08-Dec-2016
// *      Author: Satyanarayana
// */
//
//
//
//
//
//#include <stdio.h>
//#include "MKL25Z4.h"
//#include "capsense.h"
////#include "ftoa.h"
//
//#define CHANNEL 16
//static volatile uint16_t rcounts[CHANNEL];
//static volatile uint16_t bcounts[CHANNEL];
//static uint32_t enable_mask;
//static uint32_t sense_data;
//
//void Initialize_LED()
//{
//    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK;
//	SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM0_MASK;
//    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(1);
//
//    TPM2_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
//    TPM2_BASE_PTR->MOD = 1875;
//    TPM0_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
//    TPM0_BASE_PTR->MOD = 1875;
//
//    SIM_BASE_PTR->SCGC5 |=  0x400;
//    SIM_BASE_PTR->SCGC5 |= 0x1000;
//    PORTB_BASE_PTR->PCR[18] = PORTB_BASE_PTR->PCR[19] = PORT_PCR_MUX(3);
//    PORTD_BASE_PTR->PCR[1]=PORT_PCR_MUX(4);
//
//      TPM2_BASE_PTR->CONTROLS[0].CnSC =  0x20 | 0x4;
//      TPM2_BASE_PTR->CONTROLS[1].CnSC =  0x20 | 0x4; ;
//      TPM0_BASE_PTR->CONTROLS[1].CnSC =  0x20 | 0x4; ;
//}
//
//
//int touch(int chnl)
//{
//    return rcounts[chnl] - bcounts[chnl];
//}
//
//
//inline static void scan_strt(int chnl)
//{
//    TSI0_DATA = TSI_DATA_TSICH(chnl) | TSI_DATA_SWTS_MASK;
//}
//
//
//inline static uint16_t scan_data(void)
//{
//    TSI0_GENCS |= TSI_GENCS_EOSF_MASK;
//    return TSI0_DATA & TSI_DATA_TSICNT_MASK;
//}
//
//void TSI0_IRQHandler() __attribute__((interrupt("IRQ")));
//void TSI0_IRQHandler(void)
//{
//
//    uint32_t chnl = (TSI0_DATA & TSI_DATA_TSICH_MASK) >> TSI_DATA_TSICH_SHIFT;
//    rcounts[chnl] = scan_data();
//
//
//    for(chnl=15; chnl>=0; chnl--) {
//        //chnl = (chnl + 1) % CHANNEL;
//        if ((1 << chnl) & enable_mask) {
//            scan_strt(chnl);
//            return;
//        }
//    }
//}
//
//
//void tsi_init(uint32_t chnl_mask)
//{
//
//
//    SIM_SCGC5 |= SIM_SCGC5_TSI_MASK;
//    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
//    TSI0_GENCS |= (TSI_GENCS_ESOR_MASK
//                   | TSI_GENCS_MODE(0)
//                   | TSI_GENCS_REFCHRG(4)
//                   | TSI_GENCS_DVOLT(0)
//                   | TSI_GENCS_EXTCHRG(7)
//                   | TSI_GENCS_PS(4)
//                   | TSI_GENCS_NSCN(11)
//                   | TSI_GENCS_TSIIEN_MASK
//                   | TSI_GENCS_STPE_MASK
//                   );
//
//    TSI0_GENCS |= TSI_GENCS_TSIEN_MASK;
//
//    TSI0_TSHD = 0x12f; //((0x01<<TSI_TSHD_THRESH_SHIFT) | 0x0f);
//
//
//   PORTB_PCR16 = PORT_PCR_MUX(0);
//    PORTB_PCR17 = PORT_PCR_MUX(0);
//
//
////    int i, first_chnl = 0;
////    enable_mask = chnl_mask;
////    for(i=15; i>=0; i--) {
////        if((1 << i) & enable_mask) {
////            scan_strt(i);
////            while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK))
////                ;
////
////            bcounts[i] = scan_data();
////            first_chnl = i;
////        }
////    }
//
//
//
//    NVIC_EnableIRQ(TSI0_IRQn);
//    scan_strt(first_chnl);
//}
//
//
//void tsi_data()
//{
//	uint8_t finalval[15];
//	sense_data= TSI0_DATA;
//	uint16_t len=0;
//	char transfer[20];
//	char *b=transfer;
//	int abc;
//	for( abc=15; abc>=0; abc--)
//	{
//		finalval[abc]= touch(abc);
//	}
//	uint8_t jo;
//	jo = 12;
//	//	print_stringl ("\n\rCapacitive touch: ");
////	ftoa(sense_data, transfer, 2);
////				while(*b!='\0')
////				 {
////					 len++;
////					 b++;
////				 }
////				print_string(transfer, len);
//
//}
//
//
//
//
//


/* Freescale Semiconductor Inc.
 * (c) Copyright 2004-2005 Freescale Semiconductor, Inc.
 * (c) Copyright 2001-2004 Motorola, Inc.
 *
 * mbed Microcontroller Library
 * (c) Copyright 2009-2012 ARM Limited.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "MKL25Z4.h"

#include "capsense.h"

#define NO_TOUCH                 0
#define SLIDER_LENGTH           40 //LENGTH in mm
#define TOTAL_ELECTRODE          2

#define TSI0a        0
#define TSI1         1
#define TSI2         2
#define TSI3         3
#define TSI4         4
#define TSI5         5
#define TSI6         6
#define TSI7         7
#define TSI8         8
#define TSI9         9
#define TSI10        10
#define TSI11        11
#define TSI12        12
#define TSI13        13
#define TSI14        14
#define TSI15        15

/*Chose the correct TSI channel for the electrode number*/
#define ELECTRODE0   TSI9
#define ELECTRODE1   TSI10
#define ELECTRODE2   TSI0a
#define ELECTRODE3   TSI1
#define ELECTRODE4   TSI2
#define ELECTRODE5   TSI3
#define ELECTRODE6   TSI4
#define ELECTRODE7   TSI5
#define ELECTRODE8   TSI6
#define ELECTRODE9   TSI7
#define ELECTRODE10  TSI8
#define ELECTRODE11  TSI11
#define ELECTRODE12  TSI12
#define ELECTRODE13  TSI13
#define ELECTRODE14  TSI14
#define ELECTRODE15  TSI15

#define THRESHOLD0   100
#define THRESHOLD1   100
#define THRESHOLD2   100
#define THRESHOLD3   100
#define THRESHOLD4   100
#define THRESHOLD5   100
#define THRESHOLD6   100
#define THRESHOLD7   100
#define THRESHOLD8   100
#define THRESHOLD9   100
#define THRESHOLD10   100
#define THRESHOLD11   100
#define THRESHOLD12   100
#define THRESHOLD13   100
#define THRESHOLD14   100
#define THRESHOLD15   100

static uint8_t total_electrode = TOTAL_ELECTRODE;
static uint8_t elec_array[16]={ELECTRODE0,ELECTRODE1,ELECTRODE2,ELECTRODE3,ELECTRODE4,ELECTRODE5,
                               ELECTRODE6,ELECTRODE7,ELECTRODE8,ELECTRODE9,ELECTRODE10,ELECTRODE11,
                               ELECTRODE12,ELECTRODE13,ELECTRODE14,ELECTRODE15};
static uint16_t gu16TSICount[16];
static uint16_t gu16Baseline[16];
static uint16_t gu16Threshold[16]={THRESHOLD0,THRESHOLD1,THRESHOLD2,THRESHOLD3,THRESHOLD4,THRESHOLD5,
                                   THRESHOLD6,THRESHOLD7,THRESHOLD8,THRESHOLD9,THRESHOLD10,THRESHOLD11,
                                   THRESHOLD12,THRESHOLD13,THRESHOLD14,THRESHOLD15};
static uint16_t gu16Delta[16];
static uint8_t ongoing_elec;
static uint8_t end_flag = 1;

static uint8_t SliderPercentegePosition[2] = {NO_TOUCH,NO_TOUCH};
static uint8_t SliderDistancePosition[2] = {NO_TOUCH,NO_TOUCH};
static uint32_t AbsolutePercentegePosition = NO_TOUCH;
static uint32_t AbsoluteDistancePosition = NO_TOUCH;

static void tsi_irq();
void tsi_init1();
uint8_t readDistance();
void tsi_init1()
{
    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
    SIM->SCGC5 |= SIM_SCGC5_TSI_MASK;

    TSI0->GENCS |= (TSI_GENCS_ESOR_MASK
                   | TSI_GENCS_MODE(0)
                   | TSI_GENCS_REFCHRG(4)
                   | TSI_GENCS_DVOLT(0)
                   | TSI_GENCS_EXTCHRG(7)
                   | TSI_GENCS_PS(4)
                   | TSI_GENCS_NSCN(11)
                   | TSI_GENCS_TSIIEN_MASK
                   | TSI_GENCS_STPE_MASK
                   );

    TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;

    NVIC_SetVector(TSI0_IRQn, (uint32_t)&tsi_irq);
    NVIC_EnableIRQ(TSI0_IRQn);

    selfCalibration();
}

void selfCalibration(void)
{
    unsigned char cnt;
    unsigned char trigger_backup;

    TSI0->GENCS |= TSI_GENCS_EOSF_MASK;      // Clear End of Scan Flag
    TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK;    // Disable TSI module

    if(TSI0->GENCS & TSI_GENCS_STM_MASK)     // Back-up TSI Trigger mode from Application
        trigger_backup = 1;
    else
        trigger_backup = 0;

    TSI0->GENCS &= ~TSI_GENCS_STM_MASK;      // Use SW trigger
    TSI0->GENCS &= ~TSI_GENCS_TSIIEN_MASK;    // Enable TSI interrupts

    TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;     // Enable TSI module

    for(cnt=0; cnt < total_electrode; cnt++)  // Get Counts when Electrode not pressed
    {
        TSI0->DATA = ((elec_array[cnt] << TSI_DATA_TSICH_SHIFT) );
        TSI0->DATA |= TSI_DATA_SWTS_MASK;
        while(!(TSI0->GENCS & TSI_GENCS_EOSF_MASK));
        TSI0->GENCS |= TSI_GENCS_EOSF_MASK;
        gu16Baseline[cnt] = (TSI0->DATA & TSI_DATA_TSICNT_MASK);
    }

    TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK;    // Disable TSI module
    TSI0->GENCS |= TSI_GENCS_TSIIEN_MASK;     // Enale TSI interrupt
    if(trigger_backup)                      // Restore trigger mode
        TSI0->GENCS |= TSI_GENCS_STM_MASK;
    else
        TSI0->GENCS &= ~TSI_GENCS_STM_MASK;

    TSI0->GENCS |= TSI_GENCS_TSIEN_MASK;     // Enable TSI module

    TSI0->DATA = ((elec_array[0]<<TSI_DATA_TSICH_SHIFT) );
    TSI0->DATA |= TSI_DATA_SWTS_MASK;
}

void sliderRead(void ) {
    if(end_flag) {
        end_flag = 0;
        if((gu16Delta[0] > gu16Threshold[0])||(gu16Delta[1] > gu16Threshold[1])) {
            SliderPercentegePosition[0] = (gu16Delta[0]*100)/(gu16Delta[0]+gu16Delta[1]);
            SliderPercentegePosition[1] = (gu16Delta[1]*100)/(gu16Delta[0]+gu16Delta[1]);
            SliderDistancePosition[0] = (SliderPercentegePosition[0]* SLIDER_LENGTH)/100;
            SliderDistancePosition[1] = (SliderPercentegePosition[1]* SLIDER_LENGTH)/100;
            AbsolutePercentegePosition = ((100 - SliderPercentegePosition[0]) + SliderPercentegePosition[1])/2;
            AbsoluteDistancePosition = ((SLIDER_LENGTH - SliderDistancePosition[0]) + SliderDistancePosition[1])/2;
         } else {
            SliderPercentegePosition[0] = NO_TOUCH;
            SliderPercentegePosition[1] = NO_TOUCH;
            SliderDistancePosition[0] = NO_TOUCH;
            SliderDistancePosition[1] = NO_TOUCH;
            AbsolutePercentegePosition = NO_TOUCH;
            AbsoluteDistancePosition = NO_TOUCH;
         }
    }
}

float readPercentage() {
    sliderRead();
    return (float)AbsolutePercentegePosition/100.0;
}

uint8_t readDistance() {
    sliderRead();
    return AbsoluteDistancePosition;
}

static void changeElectrode(void)
{
    int16_t u16temp_delta;

    gu16TSICount[ongoing_elec] = (TSI0->DATA & TSI_DATA_TSICNT_MASK);          // Save Counts for current electrode
    u16temp_delta = gu16TSICount[ongoing_elec] - gu16Baseline[ongoing_elec];  // Obtains Counts Delta from callibration reference
    if(u16temp_delta < 0)
        gu16Delta[ongoing_elec] = 0;
    else
        gu16Delta[ongoing_elec] = u16temp_delta;

    //Change Electrode to Scan
    if(total_electrode > 1)
    {
        if((total_electrode-1) > ongoing_elec)
            ongoing_elec++;
        else
            ongoing_elec = 0;

        TSI0->DATA = ((elec_array[ongoing_elec]<<TSI_DATA_TSICH_SHIFT) );
        TSI0->DATA |= TSI_DATA_SWTS_MASK;
    }
}

void tsi_irq(void)
{
    end_flag = 1;
    TSI0->GENCS |= TSI_GENCS_EOSF_MASK; // Clear End of Scan Flag
    changeElectrode();
}
