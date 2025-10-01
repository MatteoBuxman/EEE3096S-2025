/*
 * waveform.h
 *
 *  Created on: Oct 1, 2025
 *      Author: matteobuxman
 */

#ifndef CORE_SRC_WAVEFORM_H_
#define CORE_SRC_WAVEFORM_H_

#include <stdint.h>

#define NS 256   // Number of samples per cycle

// Lookup tables (defined in waveforms.c)
extern const uint16_t Sine_LUT[NS];
extern const uint16_t Saw_LUT[NS];
extern const uint16_t Triangle_LUT[NS];


#endif /* CORE_SRC_WAVEFORM_H_ */
