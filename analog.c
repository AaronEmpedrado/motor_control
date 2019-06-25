#include "analog.h"
#include "NU32.h"          // config bits, constants, funcs for startup and UART

void adc_setup(void){
  /* Set up the ADC */
  TRISBbits.TRISB0 = 1;           //Set B0 to be an input pin (AN0 reads Vout which is the sensed value)
  AD1PCFGbits.PCFG0 = 0;          //Set B0 to be an analog input (ADC pin now)
  AD1CON3bits.ADCS = 2;           //ADC clock period is Tad = 2*(ADCS+1)*Tpb (the minimum needed)
  AD1CON1bits.SSRC = 0b111;       //Set up automatic conversion
  AD1CON1bits.ASAM = 0;           //Set up manual sampling (default state, but let's make it explicit)
  AD1CON1bits.ADON = 1;           //Turn on A/D converter
}

//sampling function from 10.1 - "ADC_Read2.c"
unsigned int adc_sample_convert(int pin) { // sample & convert the value on the given 
                                           // adc pin the pin should be configured as an 
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = pin;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) { 
      ;                                   // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}

/*Calls our sampler reads times and returns an average (int math)*/
unsigned int adc_count_avg(int pin){
	unsigned int sum = 0;
	int i;
	for(i = 0; i < READINGS; i++){
		sum += adc_sample_convert(pin);
	}
	return (sum / READINGS);
}

/*Uses the Linear fit from 28.4.8.6 to output a current reading*/
int current_reading(int pin){		//output in mA
	return (int)((adc_count_avg(pin) * SLOPE) + INTERCEPT);
}

/*No Longer need => will use current reading now*/
// /*Returns the average voltage read [0,3.3V] with 3mV resolution*/
// float voltage_reading(int pin){
// 	return (float)(adc_count_avg(pin) * VOLTS_PER_COUNT);
// }







