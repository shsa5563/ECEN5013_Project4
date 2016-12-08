/*
 * ftoa.c
 *
 *  Created on: 02 Dec , 2016
 *      Author: Sayan Barman
 */



#include <stdio.h>
#include "MKL25Z4.h"

void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
    	temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

void send_data(char character)
{
	UART0->D = (uint8_t)character;						//Move the character to the Uart data buffer
	while(!(UART0->S1 & 0x80))
	{}													//Wait for the transmit to take place
}

void print_string(char *ptr_str, uint32_t len)
{
	int i;
	for(i=0;i<len;i++)
		{send_data(*ptr_str++);
		}
	//send_data("\n");
	//send_data("\r");
}

void print_stringl(char *ptr)
{
 int len=0;
while(*ptr!='\0')
  {
  len++;
  ptr++;
  }
ptr=ptr-len;
print_string(ptr,len);
}


int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

int power(int a, int n)
{
  int i,b=1;
  for(i=0;i<n;i++)
   b=b*a;
  return b;

 }

void ftoa(float n, char *res, int afterpoint)
{
    int ipart = (int)n;									// Extract integer part
    float fpart = n - (float)ipart;						// Extract floating part
    int i = intToStr(ipart, res, 0);					// convert integer part to string


    if (afterpoint != 0)
    {
       res[i] = '.';  									// add dot


        fpart = fpart * power(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}



