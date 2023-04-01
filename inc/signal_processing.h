/*
 * signal_processing.h
 *
 *  Created on: Mar 29, 2023
 *      Author: Andrew Timmons
 */

#ifndef INC_SIGNAL_PROCESSING_H_
#define INC_SIGNAL_PROCESSING_H_

#include <complex.h>
#include <math.h>

typedef double complex cplx;

void _fft(cplx buf[], cplx out[], int n, int step);
void fft(cplx buf[], int n);
int hps(cplx fft[], int ft_len, int num_harmonics);


#endif /* INC_SIGNAL_PROCESSING_H_ */
