#define ANGLE_RESET 32768
#define ENCODER_LINES 448		//plastic bar motor
#define RESOLUTION 4
#define CYCLE_COUNTS (ENCODER_LINES * RESOLUTION)	//360 degrees <=> 1792 counts

int encoder_counts(void);
void encoder_init(void);
void encoder_reset(void);
int encoder_angle(void);