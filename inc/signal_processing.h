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
#include <stdlib.h>

typedef float complex cplx;

//cplx* fft(cplx sample[], int n);
void fft(cplx buf[], int n);
void _fft(cplx buf[], cplx out[], int n, int step);
int hps(cplx fft[], int ft_len, int num_harmonics);


#endif /* INC_SIGNAL_PROCESSING_H_ */
