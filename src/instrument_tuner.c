/*
 * instrument_tuner.c
 *
 *  Created on: Mar 31, 2023
 *      Author: Andrew Timmons
 */


#include "instrument_tuner.h"

const struct {
	float E4;
	float B3;
	float G3;
	float D3;
	float A2;
	float E2;
} GTR_STND = {329.63, 246.94, 196.00, 146.83, 110.00, 82.41};

/*
 *
 */
float getNearestNote(struct tuning, float freq) {

}
