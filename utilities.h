#define DIRECTION_PIN LATFbits.LATF3		//Pin controlling the motor direction on the PIC

/* Enum for the 5 modes */
typedef enum{
	IDLE,
	PWM,
	ITEST,
	HOLD,
	TRACK
}mode;

/*Based off of phase/enable mode on the H bridge*/
typedef enum{
	REV,
	FWD
}motor_direction;

static mode MODE;

void set_mode(mode new_m);
mode get_mode(void);

/*Configuring timers for the ISRs (28.4.9)*/
void isrTimer_setup(void);
void PWM_setup(void);
void motor_direction_setup(void);

/*Shit for position control*/
void posisrTimer_setup(void);