#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include <stdio.h>

// include other header files here
#include "encoder.h"
#include "utilities.h"
#include "analog.h"

/*Constants*/
#define BUF_SIZE 200
#define PLOT_PTS 100                //for current loop ref array
#define TRAJ_PTS 10000               //the number of points we need to store from genref


static volatile int curr_PWM = 0;
static volatile int DIRECTION = 0;
/*Current control*/
static volatile float Kp_curr = 0.3, Ki_curr = 0.05;     //Control gains for current
static volatile float error_curr = 0, Eint_curr = 0;  //PI stuff for current
static volatile int REFarray[PLOT_PTS];    //reference values to plot
static volatile int CURarray[PLOT_PTS];    //measured values of the current
static volatile float u_curr = 0;          //the PI controller for current
static volatile float u_currNew = 0;          //the PI controller for current
static volatile int send = 0;           //if this flag is 1, we send to matlab          
/*Motor control*/
static volatile float Kp_pos = 0, Ki_pos = 0, Kd_pos = 0;     //Control gains for position
static volatile int deg_desired = 0;              //the angle that the user wants
static volatile int error_pos = 0, Eint_pos = 0;  //PID stuff for position control
static volatile int Edot_pos = 0, eprev = 0;                 //PID stuff for position control
static volatile float u_pos = 0;                  //the PID controller for current
static volatile int TREFarray[TRAJ_PTS];        //reference values to plot for trajectory
static volatile int TSAMParray[TRAJ_PTS];        //reference values to plot for trajectory
static volatile int trefsize = 0;               //used to know the size of sampes being received
static volatile int tcounter = 0;               //used to iterate thru the array
// static volatile float u_posNew = 0;               //the PID controller for current


/*Position controller ISR*/
void __ISR(_TIMER_5_VECTOR, IPL6SOFT) PController(void){   //_TIMER_2_VECTOR = 8
  char notice[BUF_SIZE];
  if(get_mode() == HOLD){
    /*Initialize relevant variables*/
    static int r, s;
    //read the encoder, ref is given by user
    r = deg_desired;
    s = encoder_angle();
    /*Compute the control*/
    //compare the actual angle to the desired angle
    error_pos = r - s;
    Eint_pos += error_pos;
    Edot_pos = error_pos - eprev;
    u_pos = Kp_pos*error_pos + Ki_pos*Eint_pos + Kd_pos*Edot_pos;         //compute the control signal
    /*send the control*/

    /*update the prev error*/
    eprev = error_pos;
    // sprintf(notice, "ref: %d\tsamp: %d\terr: %d\r\n",r,s,error_pos);
    // NU32_WriteUART3(notice);      

    //calculate a reference current using the pid controller gains (can compare via counts or degrees)
    //make sure to filter through what degree given between [-180,180]
  } else if(get_mode() == TRACK) {
    /*tracking for the last part of position control*/
    /*Initialize relevant variables*/
    static int rt, st;
    //read the encoder, ref is given by user
    rt = TREFarray[tcounter];                   //new to the track mode
    st = encoder_angle();

    /*store in an array for later plotting*/
    TSAMParray[tcounter] = st;

    /*Compute the control*/
    //compare the actual angle to the desired angle
    error_pos = rt - st;
    Eint_pos += error_pos;
    Edot_pos = error_pos - eprev;
    u_pos = Kp_pos*error_pos + Ki_pos*Eint_pos + Kd_pos*Edot_pos;         //compute the control signal
    /*send the control*/

    /*update the prev error*/
    eprev = error_pos;


    // sprintf(notice, "Mode is: %d\r\n", get_mode());
    // NU32_WriteUART3(notice);      
        
    //increment the counter
    tcounter++;
    //get out to avoid indexing past the array capacity
    if(tcounter == trefsize){
      deg_desired = TREFarray[trefsize-1];      //update degree for holding
      send = 1;
      set_mode(HOLD);
    }
  }


  // sprintf(notice, "Mode is: %d\r\n", get_mode());
  // NU32_WriteUART3(notice);      
  
  // NU32_LED1 = !NU32_LED1;  //Used to scope the freq
  IFS0bits.T5IF = 0;              // clear interrupt flag for Timer5
}


/*Current controller ISR*/
void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CController(void){   //_TIMER_2_VECTOR = 8
  char notice[BUF_SIZE];
  static int counter = 0;     //for the reference square wave (-200, 200)
  switch (get_mode()) {
    case IDLE:
    {
      /*Put the H Bridge in Brake mode*/
      //Enable should be low
      OC1RS = 0;
      //Phase doesn't matter
      break;
    }
    case PWM:
    {
      /*Set according to value given by client*/
      if(curr_PWM < 0){
        DIRECTION = REV;              //low phase is reverse (CW)
        DIRECTION_PIN = DIRECTION;        
        OC1RS = curr_PWM * -10;            //PR3+1 at 1000
      } else {
        DIRECTION = FWD;         //high phase if forward (CCW)
        DIRECTION_PIN = DIRECTION;        
        OC1RS = curr_PWM * 10;            //PR3+1 at 1000
      }
      break;
    }
    case ITEST:
    {
      static volatile int curr_amp = 200;
      /*Make the reference waveform => 2 cycles of 200/-200*/
      if(counter < PLOT_PTS){
        if(counter % 25 == 0){
          curr_amp *= -1;             //flip sign every 25
        }
        /*Actually store it in the ref array*/
        REFarray[counter] = curr_amp;
        /*Populate the measured current array*/
        CURarray[counter] = current_reading(0);         //Read current from B0 to arr
        /*End of the array*/
        if (counter == PLOT_PTS - 1){
          //Current loop test is over => Set mode to IDLE
          set_mode(IDLE);  
          counter = 0;
          send = 1;               //send to matlab
        }
      }
    
      // sprintf(notice, "%d ", CURarray[counter]);
      // NU32_WriteUART3(notice);      

      /*Just shorten names for ref and sample*/
      static int r, s;
      r = REFarray[counter];
      s = CURarray[counter];

      error_curr = r - s;  //calculate error
      Eint_curr += error_curr;               
      u_curr = Kp_curr*error_curr + Ki_curr*Eint_curr;     //the current controller
      u_currNew = u_curr;

      /*Cap duty cycle: [-100,100] => will update to abs() later*/
      if(u_currNew > 100.0){
        u_currNew = 100.0;
      } else if(u_currNew < -100.0) {
        u_currNew = -100.0;
      }

      /*Set direction bit*/
      if(u_currNew < 0.0) {
        DIRECTION = REV;
        DIRECTION_PIN = DIRECTION;
        u_currNew = abs(u_currNew);             //bring to positive
      } else{
        DIRECTION = FWD;
        DIRECTION_PIN = DIRECTION;
      }
      /*Update the duty cycle PWM*/
      OC1RS = (unsigned int)((u_currNew / 100.0) * (float)(PR3 + 1));


      // /*debug shit*/
      // sprintf(notice, "count: %d\tref: %d\tsample: %d\tdir: %d\tduty: %7.3f OC:%d\r\n",
      //                   counter,r,s,DIRECTION,u_currNew,OC1RS);
      // NU32_WriteUART3(notice);


      /*Increment the fucking counter*/
      counter++;
      // sprintf(notice, "%d\r\n", curr_amp);
      // NU32_WriteUART3(notice);      
      break;
    }
    case TRACK:{
      //falls thru to hold
    }
    /*Used in the position controller*/
    case HOLD:
    {

      /*Just shorten names for ref and sample*/
      static int rpos, spos;
      rpos = u_pos;                      //different from itest => use u_pos as the ref
      spos = current_reading(0);         //different from itest => current reading

      error_curr = rpos - spos;  //calculate error
      Eint_curr += error_curr;               
      u_curr = Kp_curr*error_curr + Ki_curr*Eint_curr;     //the current controller
      u_currNew = u_curr;

      /*Cap duty cycle: [-100,100] => will update to abs() later*/
      if(u_currNew > 100.0){
        u_currNew = 100.0;
      } else if(u_currNew < -100.0) {
        u_currNew = -100.0;
      }

      /*Set direction bit*/
      if(u_currNew < 0.0) {
        DIRECTION = REV;
        DIRECTION_PIN = DIRECTION;
        u_currNew = abs(u_currNew);             //bring to positive
      } else{
        DIRECTION = FWD;
        DIRECTION_PIN = DIRECTION;
      }
      /*Update the duty cycle PWM*/
      OC1RS = (unsigned int)((u_currNew / 100.0) * (float)(PR3 + 1));


      // /*debug shit*/
      // sprintf(notice, "count: %d\tref: %d\tsample: %d\tdir: %d\tduty: %7.3f OC:%d\r\n",
      //                   counter,r,s,DIRECTION,u_currNew,OC1RS);
      // NU32_WriteUART3(notice);


      // Increment the fucking counter
      // counter++;
      // sprintf(notice, "%d\r\n", curr_amp);
      // NU32_WriteUART3(notice);      


      break;
    }
  }
  /*Set the PWM duty cycle to 25%*/
  /*Invert the motor direction digital output (F3)*/
  // LATFbits.LATF3 = !LATFbits.LATF3;
  IFS0bits.T2IF = 0;              // clear interrupt flag for Timer2
}

/*******************************************************************************************************/

/* Main Function */
int main() 
{
  char buffer[BUF_SIZE];
  char message[BUF_SIZE];         /*Messages that MATLAB sends*/
  int pwmtemp = 0;                //temporary pwm value
  float kptemp = 0, kitemp = 0, kdtemp = 0;   //temporary local gains (either pos or current)
  int degtemp = 0;                //temporary degree value for position control
  int tsizetemp = 0, tsamptemp = 0;      //for trajectory

  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;       

  /*Setup the ADC*/
  adc_setup();  
  /*Setup T3 and Output Compare for 20kHz PWM*/
  PWM_setup();                    
  motor_direction_setup();          //sets G3 as a digital output


  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  encoder_init();
  isrTimer_setup();                 //T2 setup w 5/0 and 5kHz
  posisrTimer_setup();              //T5 setup w 6/0 and 200Hz
  __builtin_enable_interrupts();


  /*Set the MODE to idle*/
  set_mode(IDLE);

  while(1)
  {
    int i;
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    
    /*Switch case for what menu option to execute*/    
    switch (buffer[0]) {
      /* Displays the motor encoder count (angle) */
      case 'c':    
      {
        sprintf(buffer, "%d\r\n", encoder_counts());
        NU32_WriteUART3(buffer);    //send encoder count to client
        break;
      } 
      /* Resets the motor count to 1/2 range (32,768) */
      case 'e': 
      {
        encoder_reset();
        break;
      }
      /* Reads the encoder in degrees (4x resolution) */
      case 'd':                      
      {
        sprintf(buffer, "%d\r\n", encoder_angle());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here. 
        set_mode(IDLE);         /*Set to IDLE before exiting the menu*/
        break;
      }
      /* Get mode => prints what mode we are in */
      case 'r':
      {
        sprintf(buffer, "%d\r\n", get_mode());
        NU32_WriteUART3(buffer);
        break;
      }
      /* Read Current Sensor in ADC counts */
      case 'a':
      {
        //By current sensor we mean we read the voltage output from the current sensor (MAX9918)
        sprintf(buffer, "%d\r\n", adc_count_avg(0));    
        NU32_WriteUART3(buffer);
        break;
      }
      /* Read Current Sensor in milliAmps */
      case 'b':
      {
        //curent readings
        sprintf(buffer, "%d\r\n", current_reading(0));
        NU32_WriteUART3(buffer);
        break;
      }
      /* Set PWM (-100 to 100) */
      case 'f':
      {
        NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
        sscanf(message, "%d" , &pwmtemp);               //store the new PWM locally
        curr_PWM = pwmtemp;                        //Update globally 
        // /*Change Modes*/
        set_mode(PWM);
        break;
      }
      /*Unpower the motor => PIC goes into idle mode and stops spinning*/
      case 'p':
      {
        set_mode(IDLE);
        break;
      }
      /*Sets current gains*/
      case 'g':
      {
        NU32_ReadUART3(message, BUF_SIZE);          // wait for the first gain
        sscanf(message, "%f" , &kptemp);            //store the kp gain locally

        NU32_ReadUART3(message, BUF_SIZE);          // wait for the second gain
        sscanf(message, "%f" , &kitemp);            //store the ki gain locally

        Kp_curr = kptemp;
        Ki_curr = kitemp;
        break;
      }
      /*Gets current gains*/
      case 'h':
      {
        sprintf(buffer, "%f\r\n", Kp_curr);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", Ki_curr);
        NU32_WriteUART3(buffer);
        break;
      }
      /*Tests current control*/
      case 'k':
      {
				Eint_curr = 0;						//reset the integral error when we want to run a new test
        set_mode(ITEST);
        while(!send){ //wait for the data to be populated
          ;
        }

        /*Send the shits to matlab (current)*/    
        NU32_WriteUART3("100\n");               //YALREASKNOWAHTSIS
        for (i=0; i<PLOT_PTS; i++) {                   // send plot data to MATLAB
          sprintf(message, "%d %d\r\n", REFarray[i],CURarray[i]);
          NU32_WriteUART3(message);
        }      
        /*Clear flag*/
        send = 0;
        break;
      }
      /*Set position gains*/
      case 'i':
      {
        NU32_ReadUART3(message, BUF_SIZE);          // wait for the first gain
        sscanf(message, "%f" , &kptemp);            //store the kp gain locally

        NU32_ReadUART3(message, BUF_SIZE);          // wait for the second gain
        sscanf(message, "%f" , &kitemp);            //store the ki gain locally

        NU32_ReadUART3(message, BUF_SIZE);          // wait for the third gain
        sscanf(message, "%f" , &kdtemp);            //store the kd gain locally

        /*Store gains globally*/
        Kp_pos = kptemp;
        Ki_pos = kitemp;
        Kd_pos = kdtemp;
        break;
      }
      /*Get position gains*/
      case 'j':
      {
        sprintf(buffer, "%f\r\n", Kp_pos);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", Ki_pos);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", Kd_pos);
        NU32_WriteUART3(buffer);        
        break;
      }
      /*Go to angle (deg)*/
      case 'l':
      {
        NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
        sscanf(message, "%d" , &degtemp);        //store the desired degree value [-180,180] locally
        deg_desired = degtemp;                   //Update globally 
        // /*Change Modes*/
        set_mode(HOLD);
        break;
      }
      /*Load step trajectory*/
      case 'm':
      {
        //implement this
        NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
        sscanf(message, "%d" , &tsizetemp);        //store the number of samples we are expecting locally
        trefsize = tsizetemp;                   //Update globally 
        /*store each element */
        int t;
        for(t=0; t<tsizetemp; t++){
          /*store each element into the array*/
          NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
          sscanf(message, "%d" , &tsamptemp);        //store the current ref sample locally
          TREFarray[t] = tsamptemp;               //store it in the global array
        }
        tcounter = 0;     //reset the counter

        // sprintf(buffer, "%d\r\n", trefsize);
        // NU32_WriteUART3(buffer);
        break;
      }
      /*Load cubic trajectory*/
      case 'n':
      {
        //implement this
        //implement this
        NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
        sscanf(message, "%d" , &tsizetemp);        //store the number of samples we are expecting locally
        trefsize = tsizetemp;                   //Update globally 
        /*store each element */
        int t;
        for(t=0; t<tsizetemp; t++){
          /*store each element into the array*/
          NU32_ReadUART3(message, BUF_SIZE);       // wait for a message from MATLAB
          sscanf(message, "%d" , &tsamptemp);        //store the current ref sample locally
          TREFarray[t] = tsamptemp;               //store it in the global array
        }
        tcounter = 0;     //reset the counter
        break;
      }
      /*Execute trajectory*/
      case 'o':
      {
        //implement this
        encoder_reset();

        set_mode(TRACK);

        while(get_mode() == TRACK) {
          ;                                     //wait to get all data
        }
        int i;
        /*Send the shits to matlab (current)*/    
        sprintf(message, "%d\r\n", trefsize);
        NU32_WriteUART3(message);               //YALREASKNOWAHTSIS
        for (i=0; i<trefsize; i++) {                   // send plot data to MATLAB
          sprintf(message, "%d %d\r\n", TREFarray[i],TSAMParray[i]);
          NU32_WriteUART3(message);
        }      
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}



