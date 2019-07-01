/*Constants*/
#define SAMPLE_TIME 10                //10 core timer ticks = 250 ns (how long sampling process takes)
// #define VOLTS_PER_COUNT (3.3/1024)    //10 bit voltage sampling (3mV resolution)
#define READINGS 5					  //How many times to read our adc count to average
/*Constants from the Linear fit of Current vs. ADC counts*/
/*=> update these values if necessary (units of mA)*/
#define SLOPE 1.53						//updated 7/1
#define INTERCEPT -815

/*Fucntion Prototypes*/
void adc_setup(void);							//Configures our pins (using B0 for input)
unsigned int adc_sample_convert(int pin);		//samples and returns an adc count [0,1023]
unsigned int adc_count_avg(int pin);			//samples multiple times (READINGS times)
// float voltage_reading(int pin);
int current_reading(int pin);					//Uses ADC counts and linear fit to return current in mA
