//Programmer: Peter Han
//use PB4 for passenger (right) DIR connection (protoboard B-62)
//use PB5 for driver (left) DIR connection  (protoboard B-61)
//use PA1 for passenger (right) sensor connection (protoboard A-48)
//use PA0 for driver (left) sensor connection  (protoboard A-47)
//use PP1 for passenger (right) EN connection  (protoboard A-39)
//use PP3 for driver (left) EN connection (protoboard A-37)
//use PJ7 for lap counter interrupt (protoboard B-46)

#include <hidef.h>           /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <stdio.h>

// Function Prototypes
void init_PWM(void);
void init_Ports(void);
void init_PJ7(void);

//global variables
unsigned int count = 0;

//#define Driver 0x01
//#define Passenger 0x02       

//#define Driver_On 0x08
//#define Passenger_On 0x02

#define Driver_DIR 0x20
#define Passenger_DIR 0x10

const struct State {
  unsigned char PWMduty3;         //output data to PMWDTY for driver speed
  unsigned char PWMduty1;         //output data to PWMDTY for passenger speed 
  const struct State *Next[8];    //next state if input = 0,1,etc.
};

#define STRAIGHT &fsm[0]
#define SL_PASS &fsm[1]
#define SL_DRIVER &fsm[2]
#define STOP &fsm[3]
#define HRD_PASS &fsm[4]
#define HRD_DRIVER &fsm[5]
#define regular 0xAA            
#define slow 0x46
#define stop 0x00

typedef const struct State StateType;
//a
/*
	Inputs 		 : Result
	0x00 (B,B,B) : STRAIGHT (shouldnt be possible, but technically straight) 
	0x01 (B,B,W) : SL_DRIVER
	0x02 (B,W,B) : invalid state, keep current state
	0x03 (B,W,W) : HRD_DRIVER
	0x04 (W,B,B) : SL_DRIVER
	0x05 (W,B,W) : STRAIGHT
	0x06 (W,W,B) : HRD_PASS
	0x07 (W,W,W) : STOP
*/
StateType fsm[6] = { 
	{regular,regular,{STRAIGHT,SL_DRIVER,STRAIGHT,HRD_DRIVER,SL_DRIVER,STRAIGHT,HRD_PASS,STOP}},    //Straight
	{regular,slow,{S0,S1,S2,S3}},       //Slight passenger
	{slow,regular,{S0,S1,S2,S3}},       //Slight driver
	{stop,stop,{S0,S1,S2,S3}},          //Stop
	{regular,stop,{}},					//Hard passenger
	{stop,regular,{}}					//Hard driver
};

void main(void){
  
  StateType *Pt;                     //pointer to present state
  unsigned char sensor;             //sensor for the steering inputs
      
  Pt = S0;                          //initialize the present state
  
  init_PWM( );
  init_Ports( );
  init_PJ7( );
   
  EnableInterrupts;
  
  for(;;) 
  {
    if(count <= 2) { 
    
      sensor = PORTA & 0x03;              //bit 1 (PA1) is for passenger, bit 0 (PA0) for driver
      
      Pt = Pt->Next[sensor];              //move to next state

      PWMDTY3 = Pt->PWMduty3 ;            //set duty cycle for driver motor
      PWMDTY1 = Pt->PWMduty1 ;          //set duty cycle for passenger motor
      
          
    }else PWME = 0x00;
   
  } 
}


/******* functions ********/
void init_PWM()
{
  PWMCAE = 0x00;                // left aligned
  PWMPOL = 0x0A;                // initial HIGH output on ch. 1 and 3 
  PWMCLK = 0x00;                // Ch 1 source is clock A, ch 3 use clock B 
  PWMCLKAB = 0x00;              // Use clock A and B respectively
  PWMPRCLK = 0x55;              // Clk A/B pre-scale = 32 (PWM clock = 6.25MHz/32 = 195.3125 KHz =>period = 5.12uS)
  PWMPER1 = 0xC8;                // obtain a period width about 1.024ms to create 976.56Hz PWM
  PWMPER3 = 0xC8;                // obtain a period width about 1.024ms to create 976.56Hz PWM
  PWMDTY1 = 0xAA;               // 85% duty cycle
  PWMDTY3 = 0xAA;               // 85% duty cycle
  PWME = 0x0A ;                 // enable channel 1 and channel 3
}

void init_Ports( )
{ 
  DDRB |= 0x30;                 //let PB3 and PB5 be outputs
  PORTB |= Driver_DIR;          //set driver motor direction
  PORTB &= ~Passenger_DIR;      //set passenger motor direction
  DDRA = 0x00;                  //PORTA as inputs
}

void init_PJ7()
{
  DDRJ = 0x00;                  //PORT J7 interrupt input
  PPSJ = 0x00;                  //pull-up PortJ; define falling edge trigger
  PERJ = 0xFF;                  //enable pulls
  PIFJ = 0x80;                  //clear the PORT J7 interrupt flag before enabling
  PIEJ = 0x80;                  //PORT J7 = 1; enable interrupt
}

/************* Interrupt Service Routine ************/
#pragma CODE_SEG NON_BANKED
interrupt ( ( (0x10000 - 0xFFCE) / 2) - 1) void INTERRUPT_IRQISR(void)
{
  count++;
  PIFJ = 0x80;                  //clear interrupt flag for PORT J7
}

/************** End of Program File *****************/
