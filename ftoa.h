/*
 * ftoa.h
 *
 *  Created on: Dec 6, 2016
 *      Author: shva9978
 */

#ifndef SOURCES_FTOA_H_
#define SOURCES_FTOA_H_

void reverse(char *str, int len);
void send_data(char character);
void print_string(char *ptr_str, uint32_t len);
void print_stringl(char *ptr);
int intToStr(int x, char str[], int d);
int power(int a, int n);
void ftoa(float n, char *res, int afterpoint);

#endif /* SOURCES_FTOA_H_ */
