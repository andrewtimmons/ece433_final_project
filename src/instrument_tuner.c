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
 * Calculates the note in the input tuning that is nearest to the input
 * frequency.
 *
 * Keyword arguments:
 *  - tuning (const float []): array of frequencies (descending order),
 *  						   one for each string.
 *  - num_strings (int): number of strings in tuning.
 *  - freq (float): calculated frequency.
 *
 * Returns:
 *  - The nearest frequency in tuning to the input frequency.
 */
float getNearestNote(const float tuning [], int num_strings, float freq) {
	float min_diff;
	int i = 0;
	for (i=0; i<num_strings; i++){
		// get difference between the freq and the ith member of tuning
		float diff = abs(freq - tuning[i]);
		// if i = 0, this is the smallest difference so far
		if (i == 0) min_diff = diff;
		else{
			// if new diff is less than old min diff, set new min diff
			if (diff < min_diff) min_diff = diff;
			// if not, the previous freq is the nearest note
			else return tuning[i-1];
		}
	}
	// if the last freq in tuning is the min diff, return it
	return tuning[i - 1];
}
