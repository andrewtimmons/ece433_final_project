/*
 * instrument_tuner.h
 *
 *  Created on: Mar 31, 2023
 *      Author: Andrew Timmons
 */

#ifndef INC_INSTRUMENT_TUNER_H_
#define INC_INSTRUMENT_TUNER_H_

extern const int NUM_GTR_STR;
extern const float GTR_STND [];

float getNearestNote(const float tuning [], int num_strings, float freq);

#endif /* INC_INSTRUMENT_TUNER_H_ */
