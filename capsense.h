/*
 * capsense.h
 *
 * Created on: 02-Dec-2016
 *      Author: Satyanarayana
 */

#ifndef SOURCES_CAPSENSE_H_
#define SOURCES_CAPSENSE_H_
#include "stdint.h"
#include "MKL25Z4.h"
#define NCHANNELS 16
void capsense_init(uint32_t channel_mask);
int touch_data(int channel);
void scan_start(int channel);
uint16_t scan_data(void);



 uint16_t raw_counts[NCHANNELS];
uint16_t base_counts[NCHANNELS];
 uint32_t enable_mask;
#endif /* SOURCES_CAPSENSE_H_ */
