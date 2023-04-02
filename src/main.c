#include "stm32l552xx.h"
#include "instrument_tuner.h"
#include "signal_processing.h"
#include "main.h"

////////////////////
// Global Variables
////////////////////

volatile cplx sample [NUM_SAMPLES];
volatile uint16_t sample_itr;


////////////////////
// Main Method
////////////////////

int main() {
	// set clocks
	setClks();

	// initialize timers
	TIM2init();

	// initialize ADC1
	ADC1init();

	// initialize GPIOE for LED Bar
	GPIOEinit();

	for (int i=0; i<6; i++) {
		setLEDBar(GTR_STND, NUM_GTR_STR, GTR_STND[i]);
		delayMs(1000);
	}

//	while(1) {
//		// sample audio signal
//		getSample(sample);
//
//		// get fundamental frequency
//		fft(sample, NUM_SAMPLES);
//		int max_idx = hps(sample, NUM_SAMPLES, 2);
//		int fund_freq = max_idx * SAMPLE_FREQ / NUM_SAMPLES;
//
//		// get nearest note to calculated freq
//		float note = getNearestNote(GTR_STND, fund_freq);
//
//		setLEDBar(GTR_STND, note);
//
//	}

	return 0;
}


////////////////////
// Interrupt handlers
////////////////////

/*
 * This interrupt triggers on rising clock edge from
 * TIM2. When triggered, the temperature sensor is read,
 * the ADC val is converted to temperature F and the
 * temperature is transmitted.
 */
void ADC1_2_IRQHandler(){
    // Disable interrupt to avoid nested calls
	NVIC_DisableIRQ(ADC1_2_IRQn);

    // EOC Flag: wait for conversion complete
    while(bitcheck(ADC1->ISR, 2) == 0){}

	// Read conversion result
	uint16_t adc_val = ADC1->DR;

	// add sample to sample array
	sample[sample_itr] = adc_val;

	// increment sample_itr
	sample_itr++;

	// Re-enable interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn);
}


////////////////////
// Helper functions
////////////////////

/*
 * Initializes the ADC normal and injected modes, as well
 * as the ADC interrupt.
 */
void ADC1init(void) {
	// Enable GPIOC for PC0 analog input
	GPIOCinit();

	// Enable ADC voltage regulator
	bitclear(ADC1->CR, 29); // ADC not in Deep-power down
    bitset(ADC1->CR, 28); // enable voltage regulator

    // Wait for the voltage regulator to stabilize
    delayMs(10);

    // External trigger enable and polarity - rising edge (01)
    bitclear(ADC1->CFGR, 11);
    bitset(ADC1->CFGR, 10);

    // External trigger connected to TIM2_CH2 (0011)
    bitclear(ADC1->CFGR, 9);
    bitclear(ADC1->CFGR, 8);
    bitset(ADC1->CFGR, 7);
    bitset(ADC1->CFGR, 6);

    // Set L=0
    bitclear(ADC1->SQR1, 3);
    bitclear(ADC1->SQR1, 2);
    bitclear(ADC1->SQR1, 1);
    bitclear(ADC1->SQR1, 0);

    // Set SQ1=IN0s
    ADC1->SQR1 = (1<<6)|(0);

    // Disable overrun mode
    bitset(ADC1->CFGR, 12);

    // Set injection discontinuous mode
    bitset(ADC1->CFGR, 20);

    // Setup NVIC interrupt
    NVIC_SetPriority(ADC1_2_IRQn, 0);
    NVIC_EnableIRQ(ADC1_2_IRQn);

    // Enable EOC interrupt
    bitset(ADC1->IER, 2);

    // Enable ADC1
    bitset(ADC1->CR, 0);

    // Wait until ADC is Ready (ADRDY)
    while(bitcheck(ADC1->ISR, 0)==0);
}

/*
 * Uses the SysTick timer to delay for a number
 * of ms specified by the ms kwarg.
 *
 * Keyword arguments:
 *  - ms (int): number of ms to delay.
 */
void delayMs(int ms) {
    // reload with number of clocks / ms
    SysTick->LOAD = 16000;

    // clear current value
    SysTick->VAL = 0;

    // enable timer
    SysTick->CTRL = 0x5;

    // delay 1 ms for the specified number of times
    for(int i=0; i<ms; i++) {
        while((SysTick->CTRL & 0x10000) == 0);
    }

    // disable timer
    SysTick->CTRL = 0;
}

void getSample(cplx buff[]) {
    // reset sample iterator
    sample_itr = 0;

    // set TIM2 count to 0
    TIM2->CNT = 0;

	// start ADC conversion
    bitset(ADC1->CR, 2);

    // enable TIM2
    TIM2->CR1 = 1;

    // stall while samples are collected
    while(sample_itr < NUM_SAMPLES);

    // disable TIM2
    TIM2->CR1 = 0;

    // stop ADC conversion
    bitclear(ADC1->CR, 2);
}

/*
 * Initialize GPIOC for the ADC and button.
 */
void GPIOCinit(void) {
	// Set AHB2ENR bit 2 for GPIOC
	bitset(RCC->AHB2ENR, 2);

	// Set PC0 to analog input (11) for ADC1
    bitset(GPIOC->MODER, 0);
    bitset(GPIOC->MODER, 1);
}

/*
 * Initialize GPIOF.
 */
void GPIOEinit(void) {
	//Set APB1ENR2 bit 4 for GPIOE
	bitset(RCC->AHB2ENR, 4);

	// Set GPIOF MODES 7-12 to digital output (01)
	GPIOE->MODER = 0x01554000;
}

/*
 * Sets clocks for initialization.
 */
void setClks(void) {
    // Set APB1ENR1 bit 28 for PWR
	bitset(RCC->APB1ENR1, 28);

	// Use HSI16 as SYSCLK
	bitset(RCC->CFGR, 0);

	// Enable hs16 clk
	bitset(RCC->CR, 8);

    // Set ADC clock
	bitset(RCC->AHB2ENR, 13);

	// Select system clock for ADC (11)
	bitset(RCC->CCIPR1, 29);
	bitset(RCC->CCIPR1, 28);
}

/*
 *
 */
void setLEDBar(const float tuning [], int num_strings, float note) {
	for (int i=0; i<num_strings; i++) {
		if (tuning[i] == note) {
			//set LED
			bitset(GPIOE->ODR, i+7);
			//clear all otherLEDs
			GPIOE->ODR &= 1<<(i+7);
			return;
		}
	}
	// clear LEDs if note not in tuning
	GPIOE->ODR &= 0;
}

/*
 *
 */
void TIM2init(void) {
	// Enable TIM2 clock
	bitset(RCC->APB1ENR1, 0);

	// Set 1us tick
	TIM2->PSC = 16 - 1;

	// Set timer for sample rate
	TIM2->ARR = (1 / SAMPLE_FREQ * 1000000) - 1;

	// Set output compare mode to toggle (0011)
	bitclear(TIM2->CCMR1, 24);
	bitclear(TIM2->CCMR1, 14);
	bitset(TIM2->CCMR1, 13);
	bitset(TIM2->CCMR1, 12);

	// Set capture/compare value
	TIM2->CCR1 = 1;

	// Enable CH2 compare mode
	bitset(TIM2->CCER, 4);

	// Clear counter
	TIM2->CNT = 0;
}
