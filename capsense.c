


#include <stdio.h>
#include "MKL25Z4.h"
#include "capsense.h"
#include "ftoa.h"

#define CHANNEL 16
static volatile uint16_t rcounts[CHANNEL];
static volatile uint16_t bcounts[CHANNEL];
static uint32_t enable_mask;
static uint32_t sense_data;

int touch(int chnl)
{
    return rcounts[chnl] - bcounts[chnl];
}


inline static void scan_strt(int chnl)
{
    TSI0_DATA = TSI_DATA_TSICH(chnl) | TSI_DATA_SWTS_MASK;
}


inline static uint16_t scan_data(void)
{
    TSI0_GENCS |= TSI_GENCS_EOSF_MASK;
    return TSI0_DATA & TSI_DATA_TSICNT_MASK;
}

void TSI0_IRQHandler() __attribute__((interrupt("IRQ")));
void TSI0_IRQHandler(void)
{

    uint32_t chnl = (TSI0_DATA & TSI_DATA_TSICH_MASK) >> TSI_DATA_TSICH_SHIFT;
    rcounts[chnl] = scan_data();


    for(;;) {
        chnl = (chnl + 1) % CHANNEL;
        if ((1 << chnl) & enable_mask) {
            scan_strt(chnl);
            return;
        }
    }
}


void tsi_init(uint32_t chnl_mask)
{


    SIM_SCGC5 |= SIM_SCGC5_TSI_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    TSI0_GENCS |= (TSI_GENCS_ESOR_MASK
                   | TSI_GENCS_MODE(0)
                   | TSI_GENCS_REFCHRG(4)
                   | TSI_GENCS_DVOLT(0)
                   | TSI_GENCS_EXTCHRG(7)
                   | TSI_GENCS_PS(4)
                   | TSI_GENCS_NSCN(11)
                   | TSI_GENCS_TSIIEN_MASK
                   | TSI_GENCS_STPE_MASK
                   );

    TSI0_GENCS |= TSI_GENCS_TSIEN_MASK;


   PORTB_PCR16 = PORT_PCR_MUX(0);
    PORTB_PCR17 = PORT_PCR_MUX(0);


    int i, first_chnl = 0;
    enable_mask = chnl_mask;
    for(i=15; i>=0; i--) {
        if((1 << i) & enable_mask) {
            scan_strt(i);
            while(!(TSI0_GENCS & TSI_GENCS_EOSF_MASK))
                ;

            bcounts[i] = scan_data();
            first_chnl = i;
        }
    }



    NVIC_EnableIRQ(TSI0_IRQn);
    scan_strt(first_chnl);
}

void delay()
{
	int i;
	for(i=0;i<65535;i++);
}


void tsi_data()
{
	sense_data= TSI0_DATA;
	uint16_t len=0;
	int i=0;
	char transfer[20];
	char *b=transfer;
	//while(1)
	//{
	print_stringl ("\n\rCapacitive touch: ");
	ftoa(sense_data, transfer, 2);
				while(*b!='\0')
				 {
					 len++;
					 b++;
				 }
				print_string(transfer, len);
				//for(i=0;i<100;i++)
				//delay();
	//}


}





