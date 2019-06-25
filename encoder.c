#include "encoder.h"                   
#include <xc.h>

static int encoder_command(int read) { // send a command to the encoder chip
                                       // 0 = reset count to 32,768, 1 = return the count
  SPI4BUF = read;                      // send the command
  while (!SPI4STATbits.SPIRBF) { ; }   // wait for the response
  SPI4BUF;                             // garbage was transferred, ignore it
  SPI4BUF = 5;                         // write garbage, but the read will have the data
  while (!SPI4STATbits.SPIRBF) { ; }
  return SPI4BUF;
}

/* Returns the angle in terms of count (16 bits) */
int encoder_counts(void) {
  encoder_command(1);           //call once before displaying to get rid of lagg
  return encoder_command(1);
}

/* Resets the count to 32,768, halfway through the count range 0 to 65,535 */
void encoder_reset(void) {
    encoder_command(0);
}

/* Returns the angle in terms of degrees [-180,180]*/
int encoder_angle(void) {
	int raw_angle = (((encoder_counts() - ANGLE_RESET) % CYCLE_COUNTS) * 360 / CYCLE_COUNTS);
	if(raw_angle >= 0) {
		if(raw_angle <= 180) {
			return raw_angle;
		} else {  //should be (180, 360]
			return (raw_angle - 360);
		}
	} else { //negative angles
		if(raw_angle >= -180) {
			return raw_angle;
		} else {
			return (raw_angle + 360);
		}
	}
  //originally defined (360/CYCLE_COUNTS) but that gets truncated to 0
}

void encoder_init(void) {
  // SPI initialization for reading from the decoder chip
  SPI4CON = 0;              // stop and reset SPI4
  SPI4BUF;                  // read to clear the rx receive buffer
  SPI4BRG = 0x4;            // bit rate to 8 MHz, SPI4BRG = 80000000/(2*desired)-1
  SPI4STATbits.SPIROV = 0;  // clear the overflow
  SPI4CONbits.MSTEN = 1;    // master mode
  SPI4CONbits.MSSEN = 1;    // slave select enable
  SPI4CONbits.MODE16 = 1;   // 16 bit mode
  SPI4CONbits.MODE32 = 0; 
  SPI4CONbits.SMP = 1;      // sample at the end of the clock
  SPI4CONbits.ON = 1;       // turn SPI on
}
