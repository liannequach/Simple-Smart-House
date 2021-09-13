// Documentation
// CECS346 Project 2 - A Simple Smart House
// Description: Design a simplified smart house use a stepper motor to simulate a garage door, 
// an onboard push button to simulate a garage door button, three onboard LEDs to indicate garage 
// door status, and an obstacle avoidance sensor to detect any object approaching/leaving the house.
// Student Name: Len Quach

// LaunchPad built-in hardware
// SW1 left switch is negative logic PF4 on the Launchpad
// red LED connected to PF1 on the Launchpad
// blue LED connected to PF2 on the Launchpad
// green LED connected to PF3 on the Launchpad
// sensor is connected to PE0 on the Launchpad
// PD3 connected to driver for stepper motor coil A/In1
// PD2 connected to driver for stepper motor coil A'/In2
// PD1 connected to driver for stepper motor coil B/In3
// PD0 connected to driver for stepper motor coil B'/In4

// 1. Pre-processor Directives Section
// Constant declarations to access port registers using 
// symbolic names instead of addresses
#define LIGHT                   (*((volatile unsigned long *)0x40025038)) // bits 3-1  
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))

#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))

#define SENSOR                  (*((volatile unsigned long *)0x400243FC)) // bit 0
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTE_RIS_R        (*((volatile unsigned long *)0x40024414))
#define GPIO_PORTE_PUR_R        (*((volatile unsigned long *)0x40024510))
#define GPIO_PORTE_IS_R         (*((volatile unsigned long *)0x40024404))
#define GPIO_PORTE_IBE_R        (*((volatile unsigned long *)0x40024408))
#define GPIO_PORTE_IEV_R        (*((volatile unsigned long *)0x4002440C))
#define GPIO_PORTE_ICR_R        (*((volatile unsigned long *)0x4002441C))
#define GPIO_PORTE_IM_R         (*((volatile unsigned long *)0x40024410))

#define STEPPER					         (*((volatile unsigned long *)0x400073FC)) //bits 3-0
#define GPIO_PORTD_DIR_R         (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_DEN_R         (*((volatile unsigned long *)0x4000751C))
#define GPIO_PORTD_AMSEL_R       (*((volatile unsigned long *)0x40007528))
#define GPIO_PORTD_AFSEL_R       (*((volatile unsigned long *)0x40007420))
#define GPIO_PORTD_PCTL_R        (*((volatile unsigned long *)0x4000752C))
#define GPIO_PORTD_DR8R_R        (*((volatile unsigned long *)0x40007040))

#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOD      0x00000008  // port D Clock Gating Control

#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register	
#define NVIC_PRI1_R             (*((volatile unsigned long *)0xE000E404))  // IRQ 5 to 7 Priority Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority

#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_ST_CTRL_COUNT      0x00010000  // Count flag
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_M        0x00FFFFFF  // Counter load value

#define GREEN 	0x08
#define RED 		0x02
#define BLUE 		0x04


// 2. Declarations Section

//   Function Prototypes
void PortF_Init(void);
void PortE_Init(void);
void Stepper(void);
void Stepper_Init(void);
void SysTick_Init(unsigned long period);
extern void EnableInterrupts(void);  // Enable interrupts

unsigned char s; // current state
unsigned press_sw = 0;
unsigned sensor_out = 0;
volatile unsigned long FallingEdges = 0;
volatile unsigned long BothEdges = 0;
volatile unsigned long Counts = 1;
volatile unsigned long open = 0;
volatile unsigned long close = 0;

struct State{
  unsigned int Out;     // Output
  unsigned int Next[2]; // CW/CCW
};
typedef const struct State StateType;

#define clockwise 0        // Next index
#define counterclockwise 1 // Next index
StateType fsm[4]={
	// index 0: state 0, state goes form 0 to 3, output 1100,
	// if next state index is 0: move clockwise, next state for clockwise movement is 1
	// CW state transition is: 0->1->2->3 then repeat
	// CCW state transtion is: 0->3->2->1 then repeat
  {12,{1,3}}, // state 0, PD3-0:1100
  { 6,{2,0}}, // state 1, PD3-0:0110
  { 3,{3,1}}, // state 2, PD3-0:0011
  { 9,{0,2}}  // state 3, PD3-0:1001
};


// Interrupt service routine
// Executed every 62.5ns*(period)
void SysTick_Handler(void){
	Stepper();    
	Counts = Counts + 1;
	if (Counts % 100 == 0) // LED flash every 0.25s at 100Hz
		 LIGHT ^= RED;       //toggle PF1 
	if (Counts == 1000)    // turn 180 degrees: 0.18 degree for each step 
		 Counts = 0;
}

void GPIOPortF_Handler(void){ //PF0
	 for(unsigned long time=0; time<727240;time++){}
	 GPIO_PORTF_ICR_R = 0x10;   // acknowledge flag4
   press_sw = 1;              //sw1 
	 FallingEdges = FallingEdges + 1; //falling edges interrupt
}

void GPIOPortE_Handler(void){ //PE0
	 for(unsigned long time=0; time<727240;time++){}
   GPIO_PORTE_ICR_R |= 0x01;  // acknowledge flag0
	 sensor_out = 1;            // sensor detects 
	 BothEdges = BothEdges + 1; //both edges interrupt
}


// 3. Subroutines Section
// MAIN: Mandatory for a C Program to be executable
int main(void){    
	
	PortE_Init();        // Call initialization of port E
  PortF_Init();        // Call initialization of port F    
  Stepper_Init();			 // Call initialization of port D
  EnableInterrupts();
	
	LIGHT = GREEN;
	
	while(1){
		
		  if (sensor_out == 1) { 				//sensor detects
				  sensor_out = 0;
					if ((SENSOR == 0x00) && (LIGHT == GREEN)) {	//obtacles moving into
							LIGHT = 0; 						//turn off LED
							open = 1;             //door is open
							SysTick_Init(40000); 	//waiting 0.25s -> 0.25s/62.5ns = 4,000,000
							while (Counts) {} 		//does nothing
							NVIC_ST_CTRL_R = 0x00;//turn off SysTic timer 
							LIGHT = BLUE;  				//LED is blue 
							Counts = 1;           //reset count
							open = 0;             //clear
						  sensor_out= 0;				//clear sensor 		
					}
	
					if((SENSOR == 0x01) && (LIGHT == BLUE)) {  //obtacles moving away
							LIGHT = 0; 
							close = 1;            //door is closed
							SysTick_Init(40000); 
							while (Counts) {} 		
							NVIC_ST_CTRL_R = 0x00;//turn off SysTic timer 
							LIGHT = GREEN;  			//LED is greeen 
							Counts = 1;
							close = 0;
							sensor_out = 0;	 
					}
		 }	
			
		 if ((LIGHT == GREEN) && press_sw) { //sw1 is pressed
					LIGHT = 0;
			 		open = 1;              //door is open
					SysTick_Init(40000); 
					while (Counts) {} 		 
					NVIC_ST_CTRL_R = 0x00; //turn off SysTic timer 
					LIGHT = BLUE;  				 //LED is blue  
					Counts = 1;
					open = 0;						   
				  press_sw = 0;
		 }
			
		 if ((LIGHT == BLUE) && press_sw) { //sw1 is pressed
					LIGHT = 0;
					close = 1;             //door is closed
					SysTick_Init(40000);
					while (Counts) {} 		  
					NVIC_ST_CTRL_R = 0x00; //turn off SysTic timer 
					LIGHT = GREEN;  			 //LED is green  
					Counts = 1;
					open = 0;
					press_sw = 0;
		 }	
	}	
}


// Subroutine to initialize port F pins for input and output
// Inputs: PF4 for SW1
// Outputs: PF3,PF2,PF1 to the LEDs
// Notes: These four pins are connected to hardware on the LaunchPad
void PortF_Init(void){
  SYSCTL_RCGC2_R |= 0x00000020;     	// activate F clock
	while ((SYSCTL_RCGC2_R&0x00000020)!=0x00000020){} // wait for the clock to be ready
		  
	GPIO_PORTF_CR_R |= 0x1E;         		// allow changes to PF4    
  GPIO_PORTF_AMSEL_R &= ~0x1E;        // disable analog function
  GPIO_PORTF_PCTL_R &= ~0x000FFFF0; 	// GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R &= ~0x10;          // PF4 input   
  GPIO_PORTF_DIR_R |= 0x0E;          	// PF3,PF2,PF1 output   
	GPIO_PORTF_AFSEL_R &= ~0x1E;        // no alternate function
  GPIO_PORTF_PUR_R |= 0x10;          	// enable pullup resistors on PF4       
  GPIO_PORTF_DEN_R |= 0x1E;          	// enable digital pins PF4-PF0  
	
	GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4 
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // (i) Clears the I bit
}


//Input: PE0 for SENSOR
void PortE_Init(void){
	SYSCTL_RCGC2_R |= 0x00000010; //Activate Port E clocks
	while ((SYSCTL_RCGC2_R & 0x00000010) != 0x00000010){}
	GPIO_PORTE_AMSEL_R &= ~0x01; // Disable analog function on PE0
  GPIO_PORTE_PCTL_R &= ~0x0000000F; // Enable regular GPIO   
  GPIO_PORTE_DIR_R &= ~0x01;   // Inputs on PE0
  GPIO_PORTE_AFSEL_R &= ~0x01; // Regular function on PE0
  GPIO_PORTE_DEN_R |= 0x01;    // Enable digital on PE0
		
  GPIO_PORTE_IS_R &= ~0x01;     // (d) PE0 is edge-sensitive 1111 1110
  GPIO_PORTE_IBE_R |= 0x01;    //     PE0 is both edges 0000 0001
  GPIO_PORTE_ICR_R |= 0x01;      // (e) clear flag3
  GPIO_PORTE_IM_R |= 0x01;      // (f) arm interrupt on PE0
  NVIC_PRI1_R = (NVIC_PRI1_R&0xFFFFFF1F)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x00000010;      // (h) enable interrupt 4 in NVIC    (portE: bit 4 -> 0001_0000 -> 0x10)
}


//Turn stepper motor CW/CWW 
void Stepper(void){
	if(open){ //door is open
		s = fsm[s].Next[clockwise]; // clock wise circular
		STEPPER = fsm[s].Out; // step motor
	}
	else if(close){ //door is closed
		s = fsm[s].Next[counterclockwise]; // counter clock wise circular
		STEPPER = fsm[s].Out; // step motor
 }
}


// Output: D3-D0 for STEPPER MOTOR
void Stepper_Init(void){
  SYSCTL_RCGC2_R |= 0x08; // 1) activate port D
  while ((SYSCTL_RCGC2_R&0x00000008)!=0x00000008){} // wait for the clock to be ready
  s = 0; 
                                    // 2) no need to unlock PD3-0
  GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
  GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTD_DIR_R |= 0x0F;   // 5) make PD3-0 out
  GPIO_PORTD_AFSEL_R &= ~0x0F;// 6) disable alt funct on PD3-0
  GPIO_PORTD_DR8R_R |= 0x0F;  // enable 8 mA drive
  GPIO_PORTD_DEN_R |= 0x0F;   // 7) enable digital I/O on PD3-0 
}


// **************SysTick_Init*********************
// Initialize SysTick periodic interrupts
// Input: interrupt period
//        Units of period are 62.5ns (assuming 16 MHz clock)
//        Maximum is 2^24-1
//        Minimum is determined by length of ISR
// Output: none
void SysTick_Init(unsigned long period){
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_RELOAD_R = period-1;// reload value
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
                              // enable SysTick with core clock and interrupts
  NVIC_ST_CTRL_R = 0x07;
  EnableInterrupts();
}


