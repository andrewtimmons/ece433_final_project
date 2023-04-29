/*
 * Author: Andrew Timmons
 * Description: Signal processing source file; contains all signal
 * 				processing related algorithms.
 */
#include "signal_processing.h"


/*
 * Recursive helper function for fft.
 */
void _fft(cplx buf[], cplx out[], int n, int step)
{
	if (step < n) {
		_fft(out, buf, n, step * 2);
		_fft(out + step, buf + step, n, step * 2);

		for (int i = 0; i < n; i += 2 * step) {
			cplx t = cexp(-I * M_PI * i / n) * out[i + step];
			buf[i / 2]     = out[i] + t;
			buf[(i + n)/2] = out[i] - t;
		}
	}
}

/*
 * Fast Fourier Transform function. Takes the sample buffer
 * and performs the FT in-place. This algorithm was found here:
 * https://rosettacode.org/wiki/Fast_Fourier_transform
 *
 * Keyword arguments:
 *  - buf (float complex []): array of complex sample points.
 *  - n (int): length of buf array.
 */
void fft(cplx buf[], int n)
{
	cplx out[n];
	for (int i = 0; i < n; i++) out[i] = buf[i];

	_fft(buf, out, n, 1);
}


/*
 * Harmonic Product Spectrum (HPS) is the chosen pitch detection algorithm.
 * HPS finds the fundamental frequency in the given FT magnitude
 *
 * Keyword arguments:
 *  - fft (const float []): array of frequencies (descending order),
 *  						   one for each string.
 *  - n (int): length of fft array.
 *  - num_harmonics (float): number of harmonics to consider in calculation.
 */
int hps(cplx fft[], int n, int num_harmonics) {
	// scaler value for downscaling fft values
	float ft_scaler = 0.001;

	// convert fft to an array of fft magnitudes,
	// and scale each magnitude using ft_scaler
	float _hps[n / 2]; //only need to consider first half of fft
	for (int i=1; i<n/2; i++) {
		// get magnitude
		float mag = sqrtf(powf(crealf(fft[i]), 2) + powf(cimagf(fft[i]), 2));
		// downscale
		_hps[i] = mag * ft_scaler;
	}

	// remove DC contribution
	_hps[0] = 0;

	// multiply each index in _hps by corresponding index in downsampled
	// array; repeat for number of harmonics being considered
	for (int i=2; i<=num_harmonics; i++) {
		for (int j=0; j<n/2; j++) {
			if (j<(n/2 / i)) _hps[j] *= ft_scaler * _hps[j * i];
			else _hps[j] = 0;
		}
	}

	// get the index of the max value in _hps - this is the fundamental freq
	int max = 0;
	for (int i=0; i<n/2; i++) {
		if (_hps[i] > _hps[max]) max = i;
	}
	return max;
}
