/*
 * temp.h
 *
 *  Created on: Dec 02, 2016
 *      Author: Sayan Barman
 */

#ifndef SOURCES_TEMP_H_
#define SOURCES_TEMP_H_

void dma_setup16(uint16_t*arr1,uint16_t*arr2,uint32_t length);

void reverse(char *str, int len);
void send_data(char character);
void print_string(char *ptr_str, uint32_t len);
void print_stringl(char *ptr);
int intToStr(int x, char str[], int d);
int power(int a, int n);
void ftoa(float n, char *res, int afterpoint);
uint16_t adc_temp(void);
void UART0_init(void);
void Calc_temp();
float Conv_temp (float b);


#endif /* SOURCES_TEMP_H_ */
