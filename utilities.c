#include "utilities.h"
#include <stdio.h>
#include "NU32.h"          // config bits, constants, funcs for startup and UART

/* changes the mode to the new mode */
void set_mode(mode new_m) {
	MODE = new_m;
}

/* returns our enum telling us what the mode is */
mode get_mode(void){
	return MODE;
}

/*Configurations for ISRs in (28.4.9)*/
void isrTimer_setup(void){				//setting up T2 (for current controller)
  PR2 = 1999;             	        //runs at 0.2ms (5khz)
  T2CONbits.TCKPS = 0b011;        	//prescaler to 8 (011)
  TMR2 = 0;							//initialize count to 0
  // T2CONbits.TCS = 0; 				//PCBLK input, 80MHz (the default)
  T2CONbits.ON = 1;    	          	//turn on timer2    
  IPC2bits.T2IP = 5;              	// step 4: interrupt priority 5
  IPC2bits.T2IS = 0;              	// step 4: interrupt priority 0
  IFS0bits.T2IF = 0;              	// step 5: clear the timer2 interrupt flag
  IEC0bits.T2IE = 1;              	// step 6: enable timer2 interrupt
}

/*Configurations for Timer and Output compare for a PWM*/
/*Recall OC1 outputs PWM which is same pin as RD0*/
void PWM_setup(void){				//setting up T3 and OC1 (for motor controller)
	/*Setting up the Timer 3 for 20kHz*/
	PR3 = 999;              		//period = (PR3+1) * N * 12.5 ns = 50 us = 20khz
	T3CONbits.TCKPS = 0b010;     	//Timer3 prescaler to 4 (N)
	TMR3 = 0;                		//initial TMR2 count is 0
	// T3CONbits.TCS = 0; 				//PCBLK input, 80MHz (the default)
	T3CONbits.ON = 1;        		//turn on Timer3

	/*Setting up the Output Compare*/
	OC1CONbits.OCM = 0b110;  		//PWM mode without fault pin; other OC1CON bits are defaults
	OC1RS = 250;             		//duty cycle = OC1R/(PR3+1) = 25% [RS copied into R start of next period]
	OC1R = 250;              		//initialize before turning OC1 on; afterward it is read-only
	OC1CONbits.OC32 = 0;    		//use a 16 bit timer; either 2 or 3
	OC1CONbits.OCTSEL = 1;  		//use timer 3 for comparison
	OC1CONbits.ON = 1;       		//turn on OC1

}

/*Configure a digital output to control motor direction*/
void motor_direction_setup(void){
	/*Decided pin F3 will be configured as this digital output*/
	TRISFbits.TRISF3 = 0;			//Configure F3 as an output
	ODCFbits.ODCF3 = 0;				//Set as a buffered output (high or low)
}


/*Shit for position control => higher Priority because lower freq*/
void posisrTimer_setup(void){		//use T5
  PR5 = 24999;             	        //runs at 5ms (200Hz) //period = (PR3+1) * N * 12.5 ns = 5 ms = 200Hz
  T5CONbits.TCKPS = 0b100;        	//prescaler to 1:16 (100b)
  TMR5 = 0;							//initialize count to 0
  // T2CONbits.TCS = 0; 				//PCBLK input, 80MHz (the default)
  T5CONbits.ON = 1;    	          	//turn on timer5
  IPC5bits.T5IP = 6;              	// step 4: interrupt priority 6
  IPC5bits.T5IS = 0;              	// step 4: interrupt priority 0
  IFS0bits.T5IF = 0;              	// step 5: clear the timer5 interrupt flag
  IEC0bits.T5IE = 1;              	// step 6: enable timer5 interrupt	
}


