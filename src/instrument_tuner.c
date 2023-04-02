/*
 * instrument_tuner.c
 *
 *  Created on: Mar 31, 2023
 *      Author: Andrew Timmons
 */


#include <stdlib.h>
#include "instrument_tuner.h"

/*
 * STRINGS:
 * 	E4: 329.63
 * 	B3: 246.94
 * 	G3: 196.00
 * 	D3: 146.83
 * 	A2: 110.00
 * 	E2: 82.41
 */
const int NUM_GTR_STR = 6;
const float GTR_STND [] = {329.63, 246.94, 196.00, 146.83, 110.00, 82.41};

/*
 *
 */
float getNearestNote(const float tuning [], int num_strings, float freq) {
	float min_diff;
	int i = 0;
	for (i=0; i<num_strings; i++){
		float diff = abs(freq - tuning[i]);
		if (i == 0) min_diff = diff;
		else{
			if (diff < min_diff) min_diff = diff;
			else break;
		}
	}

	return tuning[i];
}
