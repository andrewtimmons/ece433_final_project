#include "stm32l552xx.h"
#include "instrument_tuner.h"
#include "signal_processing.h"
#include "main.h"
#include "stdio.h"
#include <math.h>

#include "fft_alt.h"

////////////////////
// Global Variables
////////////////////

cplx sample [NUM_SAMPLES];
uint16_t sample_itr;

void LPUART1init(void);
void LPUART1tx(char output_char);
void GPIOGinit(void);
void print(char msg[]);

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

	// initialize OPAMP1
	OPAMP1init();

	// initialize GPIOE for LED Bar
	GPIOEinit();

	// initialize board LEDs
	blueLEDinit();
	greenLEDinit();
	redLEDinit();

	LPUART1init();

	while(1) {
		// sample audio signal
		getSample();

		// get fundamental frequency
		fft(sample, NUM_SAMPLES);
		int max_idx = hps(sample, NUM_SAMPLES, 1);
		int fund_freq = max_idx * SAMPLE_FREQ / NUM_SAMPLES / 2;

		// get nearest note to calculated freq
		float note = getNearestNote(GTR_STND, NUM_GTR_STR, fund_freq);

		setLEDBar(GTR_STND, NUM_GTR_STR, note);

		setBoardLEDs(fund_freq, note);


		// this is for plotting fft and whatnot

//		float ft_mag [NUM_SAMPLES];
//		for (int i=0; i<NUM_SAMPLES; i++) {
//			ft_mag[i] = findMagnitude(sample[i]);
//		}
//		ft_mag[0] = 0;
//
//		char  txt [10];
//		for (int i=0; i<NUM_SAMPLES;i++) {
//			int smp = ft_mag[i];
//			sprintf(txt, "$%d;", smp);
//			print(txt);
//		}
	}

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
	uint16_t adc_val = ADC1->DR & 0xfff;

	// add sample to sample array
	sample[sample_itr] = adc_val;

	// increment sample_itr
	sample_itr++;

	// Re-enable interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

////////////////////
// TESTING FUNCS
////////////////////
/*
 * Initialize LPUART1.
 */
void LPUART1init(void){
	float BAUD_RATE = 115200;
	float CLOCK_FREQ = 16000000;
	GPIOGinit();

	// Set APB1ENR2 bit 0 for LPUART
	bitset(RCC->APB1ENR2, 0);

	// Select clock source for LPUART - (10) for hs16
	bitset(RCC->CCIPR1, 11);
	bitclear(RCC->CCIPR1, 10);

	// Set Baud rate
	uint32_t brr = CLOCK_FREQ / BAUD_RATE * 256;
	LPUART1->BRR = brr;

	// Set word length to 8
	bitclear(LPUART1->CR1, 28);
	bitclear(LPUART1->CR1, 12);

	// Disable parity
	bitclear(LPUART1->CR1, 10);

	// Set stop bits to 1
	bitclear(LPUART1->CR2, 13);
	bitclear(LPUART1->CR2, 12);

	// Enable LPUART, Tx
	LPUART1->CR1 = 0x9;
}

/*
 * Write the output char to the LPUART1 transmit
 * data register for transmission.
 *
 * Keyword arguments:
 *  - output_char (char): output character for transmission.
 */
void LPUART1tx(char output_char) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    	LPUART1->TDR = (output_char & 0xFF);
}

/*
 * Transmit the input string via the LPUART.
 *
 * Keyword arguments:
 *  - msg (char []): output string for transmission.
 */
void print(char msg[]){
    uint8_t idx = 0;
    while(msg[idx]!='\0') LPUART1tx(msg[idx++]);
}

/*
 * Initialize GPIOA.
 */
void GPIOAinit(void) {
	// Set APB1ENR2 bit 0 for GPIOA
	bitset(RCC->AHB2ENR, 0);

	// Set MODE0 to analog (11)
	bitset(GPIOA->MODER, 1);
	bitset(GPIOA->MODER, 0);
}

/*
 * Initialize GPIOG.
 */
void GPIOGinit(void) {
	// Set APB1ENR2 bit 6 for GPIOG
	bitset(RCC->AHB2ENR, 6);

	// Set power for GPIOG
	bitset(PWR->CR2, 9);

	// Set GPIOG MODE7 to AF (10)
	bitset(GPIOG->MODER, 15);
	bitclear(GPIOG->MODER, 14);

	// Set GPIOG AFSEL7 to AF8 (1000) for LPUART1_TX
	bitset(GPIOG->AFR[0], 31);
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	// Set GPIOG MODE8 to AF
	bitset(GPIOG->MODER, 17);
	bitclear(GPIOG->MODER, 16);

	// Set GPIOG AFSEL0 to AF8 for LPUART1_TX
	bitset(GPIOG->AFR[1], 3);
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);
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

//    // Set SQ1=IN0s
//    ADC1->SQR1 = (1<<6)|(0);

    // Set SQ1=IN8 for opamp
    ADC1->SQR1 = (8<<6)|(0);

    // Disable overrun mode
    bitset(ADC1->CFGR, 12);

    // Setup NVIC interrupt
    NVIC_SetPriority(ADC1_2_IRQn, 1);
    NVIC_EnableIRQ(ADC1_2_IRQn);

    // Enable EOC interrupt
    bitset(ADC1->IER, 2);

    // Enable ADC1
    bitset(ADC1->CR, 0);

    // Wait until ADC is Ready (ADRDY)
    while(bitcheck(ADC1->ISR, 0)==0);
}

/*
 *
 */
void blueLEDclear(void) {
	bitclear(GPIOB->ODR, 7);
}

/*
 *
 */
void blueLEDinit(void) {
	// set APB1ENR2 bit 1 for GPIOB
	bitset(RCC->AHB2ENR, 1);

	// set GPIOB MODE7 to digital output
	bitclear(GPIOB->MODER, 15);
	bitset(GPIOB->MODER, 14);
}

/*
 *
 */
void blueLEDset(void){
	bitset(GPIOB->ODR, 7);
}

/*
 *
 */
void blueLEDtoggle(void){
	bitflip(GPIOB->ODR, 7);
}

/*
 *
 */
void clearBoardLEDs(void){
	blueLEDclear();
	greenLEDclear();
	redLEDclear();
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

void getSample(void) {
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
 *
 */
void greenLEDclear(void) {
	bitclear(GPIOC->ODR, 7);
}

/*
 *
 */
void greenLEDinit(void) {
	// set APB1ENR2 bit 2 for GPIOC
	bitset(RCC->AHB2ENR, 2);

	// set GPIOC MODE7 to digital output
	bitclear(GPIOC->MODER, 15);
	bitset(GPIOC->MODER, 14);
}

/*
 *
 */
void greenLEDset(void) {
	bitset(GPIOC->ODR, 7);
}

/*
 *
 */
void greenLEDtoggle(void) {
	bitflip(GPIOC->ODR, 7);
}

/*
 *
 */
void OPAMP1init(void) {
	// init GPIOA pin 0
	GPIOAinit();

	// set clock
	bitset(RCC->APB1ENR1, 30);

	// internal PGA enable (10)
	bitset(OPAMP1->CSR, 3);
	bitclear(OPAMP1->CSR, 2);

	// set PGA gain to 2 (00)
	bitclear(OPAMP1->CSR, 5);
	bitclear(OPAMP1->CSR, 4);

	// select GPIO input (0)
	bitclear(OPAMP1->CSR, 10);

	// enable OPAMP
	bitset(OPAMP1->CSR, 0);
}


/*
 *
 */
void redLEDclear(void){
	bitclear(GPIOA->ODR, 9);
}

/*
 *
 */
void redLEDinit(void) {
	//Set APB1ENR2 bit 0 for GPIOA
	bitset(RCC->AHB2ENR, 0);

	// set GPIOA MODE9 to digital output
	bitclear(GPIOA->MODER, 19);
	bitset(GPIOA->MODER, 18);
}

/*
 *
 */
void redLEDset(void){
	bitset(GPIOA->ODR, 9);
}

/*
 * Sets clocks for initialization.
 */
void setClks(void) {
	// set voltage scaling range selection to 0 for 48M clock
	bitclear(PWR->CR1, 10);
	bitclear(PWR->CR1, 9);

	// delay
	delayMs(1000);

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
void setBoardLEDs(float fund_freq, float note) {
	// difference between target note and calculated freq
	int diff = fund_freq - note;

	// clear current board LEDs
	clearBoardLEDs();

	// if diff is within tolerance (hard coding 5 for now)
	if (abs(diff) < 5) redLEDset();

	// no LEDs if too out of range
	else if (abs(diff) > 300) return;

	else if (diff < 0) greenLEDset();

	else blueLEDset();
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
