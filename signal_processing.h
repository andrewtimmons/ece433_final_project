/*
 * signal_processing.h
 *
 *  Created on: Mar 29, 2023
 *      Author: Andrew Timmons
 */

#ifndef INC_SIGNAL_PROCESSING_H_
#define INC_SIGNAL_PROCESSING_H_

#include <stdio.h>
#include <math.h>
#include <complex.h>

static double PI = atan2(1, 1) * 4;
typedef double complex cplx;

/*
 *
 */
void _fft(cplx buf[], cplx out[], int n, int step) {
	if (step < n) {
		_fft(out, buf, n, step * 2);
		_fft(out + step, buf + step, n, step * 2);

		for (int i = 0; i < n; i += 2 * step) {
			cplx t = cexp(-I * PI * i / n) * out[i + step];
			buf[i / 2]     = out[i] + t;
			buf[(i + n)/2] = out[i] - t;
		}
	}
}

/*
 *
 */
void fft(cplx buf[], int n) {
	cplx out[n];
	for (int i = 0; i < n; i++) out[i] = buf[i];

	_fft(buf, out, n, 1);
}

/*
 * Harmonic Product Spectrum (HPS) is the chosen pitch detection algorithm.
 * HPS finds the fundamental frequency in the given FT magnitude
 */
int hps(cplx fft[], int num_harmonics) {
	// get length of fft array
	int ft_len = sizeof(fft)/sizeof(fft[0]);

	// scaler value for downscaling fft values
	float ft_scaler = 0.001;

	// convert fft to an array of fft magnitudes,
	// and scale each magnitude using ft_scaler
	int _hps[ft_len / 2]; //only need to consider first half of fft
	for (inti=0; i<ft_len/2; i++) {
		// get magnitude
		float mag = sqrtf(powf(crealf(fft[i]), 2) + powf(cimagf(fft[i]), 2));
		// downscale
		_hps[i] = mag * scale;
	}

	// multiply each index in _hps by corresponding index in downsampled
	// array; repeat for number of harmonics being considered
	for (int i=2; i<=num_harmonics; i++) {
		for (int j=0; j<ft_len/2; j++) {
			if (j<(ft_len/2 / i)) _hps[j] *= ft_scaler * _hps[j * i];

			else _hps[j] = 0;
		}
	}

	// get the index of the max value in _hps - this is the fundamental freq
	int max = 0;
	for (int i=0; i<ft_len/2; i++) {
		if (_hps[i] > _hps[max]) {
			max = i;
		}
	}
	return max;
}


#endif /* INC_SIGNAL_PROCESSING_H_ */
