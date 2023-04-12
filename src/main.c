/*
 * Author: Andrew Timmons
 * Title: Lab 2 - USART
 * Description: Controls LEDs using serial input from the PC, transmits my name one letter at a time whenever
 * 				the user button is pressed.
 */

/* Include statements */
#include "stm32l552xx.h"
#include <stdint.h>

/* Macros */
#define bitset(word,   idx)  ((word) |=  (1<<(idx)))
#define bitclear(word, idx)  ((word) &= ~(1<<(idx)))
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx)))
#define bitcheck(word, idx)  ((word) &   (1<<(idx)))

/* Constants */
char MY_NAME [] = "ANDREW TIMMONS ";
int CLOCK_FREQ = 16000000;
int BAUD_RATE = 57600;

/* Function headers */
void buttonInit();
void clearLEDs();
void delay_ms(uint32_t time);
void GPIOAinit();
void GPIOBinit();
void GPIOCinit();
void GPIOGinit();
void LPUART1init();
void LPUART1tx(char output_char);
uint8_t LPUART1rx();
uint8_t pollButtonInput();
char pollTypingInput();
void setClks();
void toggleBlue();
void toggleGreen();
void toggleLED(char input_char);
void toggleRed();


int main() {
	/* Enable clock */
	setClks();

	/* Configure GPIOs A, B, C, G */
	GPIOAinit();
	GPIOBinit();
	GPIOCinit();
	GPIOGinit();

	/* Configure user button */
	buttonInit();

	/* Configure LPUART1 */
	LPUART1init();

	/* Initialize loop variables */
	uint8_t res;
	uint8_t i = 0;

	/* Main loop */
	while(1) {
		/* Poll for LPUART rx read data */
		res = pollTypingInput();

		/* Set LEDs if rx data is received */
		if(res != 0) toggleLED((char)res);

		/* Poll user button input */
		res = pollButtonInput();

		/* Transmit char from MY_NAME if button is pressed */
		if(res != 0) {
			LPUART1tx(MY_NAME[i++]);
			/* Reset iterator if stop bit is received */
			if (MY_NAME[i] == '\0') i = 0;
		}
	}

	return 0;
}

/*
 * Initialize the user button
 */
void buttonInit() {
	/* Set GPIOC MODE13 to input mode (00) */
	bitclear(GPIOC->MODER, 27);
	bitclear(GPIOC->MODER, 26);
}

/*
 * Turn off all LEDs
 */
void clearLEDs() {
	/* Turn off all LEDs */
	bitclear(GPIOA->ODR, 9);
	bitclear(GPIOB->ODR, 7);
	bitclear(GPIOC->ODR, 7);
}

/*
 * Cause the microcontroller to delay for the input time (ms).
 *
 * Keyword arguments:
 * 	- ms (uin32_t): time in milliseconds for the system to delay.
 */
void delay_ms(uint32_t ms) {

	/* clock freq in ms */
	uint32_t ms_freq = CLOCK_FREQ / 1000;

	/* approx. number of clock cycles per for loop iteration */
	uint32_t loop_iters = 36;

	/* number of for loop iterations per ms */
	uint32_t iter_ms = ms_freq / loop_iters;

	/* number of iterations of for loop needed to achieve desired ms delay */
	uint32_t iterations = ms * iter_ms;

	/* loop iterates for approx ms seconds */
	for(uint32_t i=0; i<iterations; i++);
}

/*
 * Initialize GPIOA
 */
void GPIOAinit() {
	/* set GPIOA MODE9 to digital output (01) */
	bitset(GPIOA->MODER, 18);
	bitclear(GPIOA->MODER, 19);
}

/*
 * Initialize GPIOB
 */
void GPIOBinit() {
	/* set GPIOB MODE7 to digital output (01) */
	bitset(GPIOB->MODER, 14);
	bitclear(GPIOB->MODER, 15);
}

/*
 * Initialize GPIOC
 */
void GPIOCinit() {
	/* set GPIOC MODE7 to digital output (01) */
	bitset(GPIOC->MODER, 14);
	bitclear(GPIOC->MODER, 15);
}

/*
 * Initialize GPIOG
 */
void GPIOGinit() {
	/* Set power for GPIOG */
	bitset(PWR->CR2, 9);

	/* Set GPIOG MODE7 to AF (10) */
	bitset(GPIOG->MODER, 15);
	bitclear(GPIOG->MODER, 14);

	/* Set GPIOG AFSEL7 to AF8 (1000) for LPUART1_TX */
	bitset(GPIOG->AFR[0], 31);
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	/* set GPIOG MODE8 to AF */
	bitset(GPIOG->MODER, 17);
	bitclear(GPIOG->MODER, 16);

	/* Set GPIOG AFSEL0 to AF8 for LPUART1_TX  */
	bitset(GPIOG->AFR[1], 3);
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);
}

/*
 * Initialize LPUART1
 */
void LPUART1init() {
	// Set Baud rate
	uint32_t brr = (float) CLOCK_FREQ / (float) BAUD_RATE * 256;
	LPUART1->BRR = brr;

	// Set word length to 8
	bitclear(LPUART1->CR1, 28);
	bitclear(LPUART1->CR1, 12);

	// Disable parity
	bitclear(LPUART1->CR1, 10);

	// Set stop bits to 1
	bitclear(LPUART1->CR2, 13);
	bitclear(LPUART1->CR2, 12);

	// Enable LPUART, TXn RX
	LPUART1->CR1 = 0xD;
}

/*
 * Write the output char to the LPUART1 transmit
 * data register for transmission.
 *
 * Keyword arguments:
 *  - output_char (char): output character for transmission.
 */
void LPUART1tx(char output_char) {
	/* Write to LPUART TDR to transmit output_char */
	LPUART1->TDR = output_char;

	/* Delay to prevent multiple writes */
	delay_ms(250);
}

/*
 * Read the input data value on the LPUART1 receive
 * data register and reutnr it.
 *
 * Keyword arguments:
 *  - output_char (char): output character for transmission.
 *
 * Returns:
 *  - ret_val (uint8_t): 8-bit value read from LPUART1 RDR.
 */
uint8_t LPUART1rx() {
	/* Initialize received data and return value variables */
	uint8_t rx_data, ret_val = 0;
	/* Check ISR read data reg */
	while(bitcheck(LPUART1->ISR, 5) != 0) {
		/* Set return char to rx data until stop bit is encountered */
		rx_data = LPUART1->RDR;
		if(rx_data != '\0') ret_val = rx_data;
	}

	return ret_val;
}

/*
 * Check GPIOC input data register for input from the
 * user button (bit 13).
 *
 * Returns:
 *  - 1 if button was pushed, 0 otherwise.
 */
uint8_t pollButtonInput() {
	/* Initialize return character */
	uint8_t ret_val = 0;

	/* Check GPIOC IDR for button press */
	if(bitcheck(GPIOC->IDR, 13) != 0) ret_val = 1;

	return ret_val;
}

/*
 * Check LPUART1 status register to see whether the receive data
 * register has received input data. If data is received, read
 * and return the data has a char.
 *
 * Returns:
 *  - ret_char (char): character read from the RDR register, or 0
 *  				   if no data was read.
 */
char pollTypingInput() {
	/* Initialize return character */
	char ret_char = 0;

	/* Check ISR read data reg */
	if(bitcheck(LPUART1->ISR, 5) != 0) ret_char = (char)LPUART1rx();

	return ret_char;
}

/*
 * Enable and configure system clocks for all relevant IP blocks.
 */
void setClks() {
    //Set APB1ENR1 bit 28 for PWR
	bitset(RCC->APB1ENR1, 28);

	//Set APB1ENR2 bit 0 for GPIOA
	bitset(RCC->AHB2ENR, 0);

	//Set APB1ENR2 bit 1 for GPIOB
	bitset(RCC->AHB2ENR, 1);

	//Set APB1ENR2 bit 2 for GPIOC
	bitset(RCC->AHB2ENR, 2);

	//Set APB1ENR2 bit 6 for GPIOG
	bitset(RCC->AHB2ENR, 6);

	//Set APB1ENR2 bit 0 for LPUART
	bitset(RCC->APB1ENR2, 0);

	//Select clock source for LPUART (10) for hs16
	bitset(RCC->CCIPR1, 11);
	bitclear(RCC->CCIPR1, 10);

	// enable hs16 clk
	bitset(RCC->CR, 8);
}

/*
 * Toggle the blue LED on or off depending on its current state.
 */
void toggleBlue(void) {
	/* toggle ODR bit 7 */
	bitflip(GPIOB->ODR, 7);
}

/*
 * Toggle the green LED on or off depending on its current state.
 */
void toggleGreen(void) {
	/* toggle ODR bit 7 */
	bitflip(GPIOC->ODR, 7);
}

/*
 * Toggle LED based on the input character. Input character can be
 * 	- 'r' or 'R': toggle red LED,
 * 	- 'g' or 'G': toggle green LED,
 * 	- 'b' or 'B': toggle blue LED.
 * Any other input character will turn all LEDs off.
 *
 * Keyword arguments:
 *  - input_char (char): character specifying which LED to toggle.
 */
void toggleLED(char input_char) {
	switch(input_char) {
		/* Toggle green LED if input_char is 'g' or 'G' */
		case 'g':
		case 'G':
			toggleGreen();
			break;

		/* Toggle blue LED if input_char is 'b' or 'B' */
		case 'b':
		case 'B':
			toggleBlue();
			break;

		/* Toggle red LED if input_char is 'r' or 'R' */
		case 'r':
		case 'R':
			toggleRed();
			break;

		/* Turn off LEDs for any other input character */
		default:
			clearLEDs();
	}
}

/*
 * Toggle the red LED on or off depending on its current state.
 */
void toggleRed(void) {
	/* toggle ODR bit 9 */
	bitflip(GPIOA->ODR, 9);
}
