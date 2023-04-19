#include "stm32l552xx.h"
#include "instrument_tuner.h"
#include "signal_processing.h"
#include "main.h"
#include "stdio.h"
#include <math.h>


////////////////////
// Global Variables
////////////////////

cplx sample [NUM_SAMPLES];
uint16_t sample_itr;


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

	while(1) {
		// sample audio signal
		getSample();

		// get fundamental frequency
		fft(sample, NUM_SAMPLES);

		int max_idx = hps(sample, NUM_SAMPLES, 2);
		int fund_freq = max_idx * SAMPLE_FREQ / NUM_SAMPLES / 2;

		// get nearest note to calculated freq
		float note = getNearestNote(GTR_STND, NUM_GTR_STR, fund_freq);

		setLEDBar(GTR_STND, NUM_GTR_STR, note);

		setBoardLEDs(fund_freq, note);
	}

	return 0;
}

////////////////////
// Interrupt handlers
////////////////////

/*
 * This interrupt triggers on rising clock edge from
 * TIM2. When triggered, the audio sensor input is read
 * by the ADC and stores it in the next position in the
 * sample buffer.
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
 * Turns off the blue on-board LED.
 */
void blueLEDclear(void) {
	bitclear(GPIOB->ODR, 7);
}

/*
 * Initializes the blue on-board LED.
 */
void blueLEDinit(void) {
	// set APB1ENR2 bit 1 for GPIOB
	bitset(RCC->AHB2ENR, 1);

	// set GPIOB MODE7 to digital output
	bitclear(GPIOB->MODER, 15);
	bitset(GPIOB->MODER, 14);
}

/*
 * Turns on the blue on-board LED.
 */
void blueLEDset(void){
	bitset(GPIOB->ODR, 7);
}

///*
// *
// */
//void blueLEDtoggle(void){
//	bitflip(GPIOB->ODR, 7);
//}

/*
 * Turns off all on-board LEDs.
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

/*
 * Resets the sample buffer iterator, begins ADC
 * conversion, and enables TIM2. ADC1 interrupt is
 * triggered by TIM2, causing the microphone input
 * to be read and stored in the next sample buffer
 * position. This repeats until every position in
 * the sample buffer is filled.
 */
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
 * Initialize GPIOC for the ADC and button.
 */
void GPIOCinit(void) {
	// set AHB2ENR bit 2 for GPIOC
	bitset(RCC->AHB2ENR, 2);

	// set PC0 to analog input (11) for ADC1
    bitset(GPIOC->MODER, 0);
    bitset(GPIOC->MODER, 1);
}

/*
 * Initialize GPIOF.
 */
void GPIOEinit(void) {
	// set APB1ENR2 bit 4 for GPIOE
	bitset(RCC->AHB2ENR, 4);

	// set GPIOF MODES 7-12 to digital output (01)
	GPIOE->MODER = 0x01554000;
}

/*
 * Turns off the green on-board LED.
 */
void greenLEDclear(void) {
	bitclear(GPIOC->ODR, 7);
}

/*
 * Initializes the green on-board LED.
 */
void greenLEDinit(void) {
	// set APB1ENR2 bit 2 for GPIOC
	bitset(RCC->AHB2ENR, 2);

	// set GPIOC MODE7 to digital output
	bitclear(GPIOC->MODER, 15);
	bitset(GPIOC->MODER, 14);
}

/*
 * Turns on the green on-board LED.
 */
void greenLEDset(void) {
	bitset(GPIOC->ODR, 7);
}

///*
// *
// */
//void greenLEDtoggle(void) {
//	bitflip(GPIOC->ODR, 7);
//}

/*
 * Initializes the OPAMP with a PGA
 * gain of 2x.
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
 * Turns off the red on-board LED.
 */
void redLEDclear(void){
	bitclear(GPIOA->ODR, 9);
}

/*
 * Initializes the red on-board LED.
 */
void redLEDinit(void) {
	//Set APB1ENR2 bit 0 for GPIOA
	bitset(RCC->AHB2ENR, 0);

	// set GPIOA MODE9 to digital output
	bitclear(GPIOA->MODER, 19);
	bitset(GPIOA->MODER, 18);
}

/*
 * Turn on the red on-board LED.
 */
void redLEDset(void){
	bitset(GPIOA->ODR, 9);
}

/*
 * Sets clocks for initialization.
 */
void setClks(void) {
	// set sys clock to 110 MHz
	SystemClock_Config();

    // Set APB1ENR1 bit 28 for PWR
	bitset(RCC->APB1ENR1, 28);

//	// Use HSI16 as SYSCLK
//	bitset(RCC->CFGR, 0);

//	// Enable hs16 clk
//	bitset(RCC->CR, 8);

    // Set ADC clock
	bitset(RCC->AHB2ENR, 13);

	// Select system clock for ADC (11)
	bitset(RCC->CCIPR1, 29);
	bitset(RCC->CCIPR1, 28);
}

/*
 * Sets on-board LEDs based on difference between
 * the calculated fundamental frequency and the
 * target note.
 * 	- Red   = notes match within tolerance
 * 	- Green = note is below target
 * 	- Blue  = note is above target
 *
 * Keyword arguments:
 *  - fund_freq (float): calculated frequency.
 *  - note (float): target note frequency.
 */
void setBoardLEDs(float fund_freq, float note) {
	// difference between target note and calculated freq
	int diff = fund_freq - note;

	// clear current board LEDs
	clearBoardLEDs();

	// if diff is within tolerance (hard coding 5 for now)
	if (abs(diff) < FREQ_TOLERANCE) redLEDset();

	// no LEDs if too out of range
	else if (abs(diff) > 300) return;

	// green if too low
	else if (diff < 0) greenLEDset();

	// blue if too high
	else blueLEDset();
}

/*
 * Set the LED bar to indicate the string that is being tuned.
 *
 * Keyword arguments:
 *  - tuning (const float []): array of frequencies (descending order),
 *  						   one for each string.
 *  - num_strings (int): number of strings in tuning.
 *  - note (float): note in tuning that is being tuned.
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
 * Initializes TIM2 for controlling the sampling rate of ADC1.
 *
 */
void TIM2init(void) {
	// Enable TIM2 clock
	bitset(RCC->APB1ENR1, 0);

	// Set 1us tick
	TIM2->PSC = (CLK_FREQ / 1000000) - 1;

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

////////////////////
// Auto-generated Functions
////////////////////


/*
 * Auto-generated clock setting function for setting
 * the clock freq to max (110 MHz).
 */
void SystemClock_Config(void) {
  HAL_Init();

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void) {
	while(1);
}
