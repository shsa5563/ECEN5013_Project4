/*
 * capsense.h
 *
 *  Created on: 08-Dec-2016
 *      Author: Satyanarayana
 */

#ifndef SOURCES_CAPSENSE_H_
#define SOURCES_CAPSENSE_H_

int touch(int channel);
inline static void scan_strt(int channel);
inline static uint16_t scan_data(void);
void tsi_init(uint32_t channel_mask);
void TSI0_IRQHandler() __attribute__((interrupt("IRQ")));
void TSI0_IRQHandler(void);
void tsi_data();

void sliderRead(void);
    void selfCalibration(void);

    float readPercentage();

#endif /* SOURCES_CAPSENSE_H_ */
