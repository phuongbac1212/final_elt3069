 /***********************************
*       LIBRARIES DECLARATION
***********************************/
#include  <stdio.h>
//#include "LCD.c"
#include "main.h"
#include "MKL46Z4.h"
#include "Delay.h"

/***********************************
*       GLOBAL VARIABLES
***********************************/
uint8_t mode                    = 0;
uint64_t volatile timeNow       = 0;
uint64_t lastOperation          = 0;


/***********************************
*			FUNCTIONS' PROTOTYPE
***********************************/
extern void SegLCD_Init(void);
extern void SegLCD_DisplayHex(unsigned short);

void initLed();
void initSW();
void PORTC_PORTD_Interrupt_init();
void PIT_init(uint32_t);

int main(){
	//SegLCD_Init();
  initLed();
  initSW();
  init_SysTick_interrupt();
  PORTC_PORTD_Interrupt_init();
  //SegLCD_DisplayHex(15);
  Delay(500);
  PIT_init(5*20971520/2);

  while(1) {   
    if(msTicks - lastOperation > DELTA_TIME) {
      if (!mode) { // idle
        //printf("I\n");
        FPTE->PDOR |= LED1;
        FPTD->PDOR |= LED2;
      }
      else if(IS_BUZZING(mode)){ //buzz
              //printf("Bz\n")
        FPTE->PTOR |= LED1;
        FPTD->PTOR |= LED2;
      }
      else if (IS_SEATED(mode) & (IS_BELTED(mode))) { //belt
        MODE_TIMER_OFF(mode);
              //printf("B\n");
        FPTE->PCOR |= LED1;
        FPTD->PTOR |= LED2;
      }
      else if (IS_SEATED(mode)) { // seat
              //printf("S\n")
        FPTE->PTOR |= LED1;
        FPTD->PCOR |= LED2;
        MODE_TIMER_ON(mode);
      }
      lastOperation = msTicks;
    }
  }
}



/***********************************
* CONFIGURE CLOCK AND PIN MUXING FOR LEDs
***********************************/
void initLed() {
    /* Enable clock for PORTE & PORTD */
    SIM->SCGC5 |= ( SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

    /* Initialize the RED LED (PTE29)*/
    /* Set the PTE29 pin multiplexer to GPIO mode */
    PORTE->PCR[29] = PORT_PCR_MUX(1);
    /* Set the pin's direction to output */ 
    FPTE->PDDR |= LED1;
    /* Set the initial output state to high */
    FPTE->PSOR &= ~LED1;

    /* Initialize the Green LED (PTD5) */
    /* Set the PTD5 pin multiplexer to GPIO mode */
    PORTD->PCR[5]= PORT_PCR_MUX(1);
    /* Set the pin's direction to output */
    FPTD->PDDR |= LED2;
    /* Set the initial output state to high */
    FPTD->PSOR &= ~LED2;
}



/***********************************
* CONFIGURE CLOCK AND PIN MUXING FOR SWitch
***********************************/
void initSW() {
  // enable clock for port C
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  
  PORTC->PCR[12] &= ~(PORT_PCR_MUX_MASK);
  PORTC->PCR[12] |= PORT_PCR_MUX(1);
  PORTC->PCR[12] |= PORT_PCR_PE_MASK;
  PORTC->PCR[12] |= PORT_PCR_PS_MASK;
  //GPIO input
  GPIOC->PDDR &= ~(SW2);
  //IRQC (set interrupt DMA/request )
  PORTC->PCR[12] |= PORT_PCR_IRQC(EITHER_EDGE);
  
  PORTC->PCR[3] &= ~(PORT_PCR_MUX_MASK);
  PORTC->PCR[3] |= PORT_PCR_MUX(1);
  PORTC->PCR[3] |= PORT_PCR_PE_MASK;
  PORTC->PCR[3] |= PORT_PCR_PS_MASK;
  //GPIO input
  GPIOC->PDDR &= ~(SW1);
  //IRQC (set interrupt DMA/request )
  PORTC->PCR[3] |= PORT_PCR_IRQC(EITHER_EDGE);
}



/***********************************
* Interrupt configuration for 2 switches
**********************************/
void PORTC_PORTD_Interrupt_init(){
  NVIC_EnableIRQ(PORTC_PORTD_IRQn);
  NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
  NVIC_SetPriority(PORTC_PORTD_IRQn, SW1_PRIORITY);
}
void PORTC_PORTD_IRQHandler(){
  if(PORTC->ISFR ==ISW1){
         PORTC->ISFR |=SW1; // clear interrupt flag
         MODE_XOR_SEAT(mode);
         if (!IS_SEATED(mode)) mode =0;
	} 
  
  if(PORTC->ISFR==ISW2) {
         PORTC->ISFR |=SW2; // clear interrupt flag
         MODE_XOR_BELT(mode);
         MODE_TIMER_OFF(mode);
         MODE_BUZZER_OFF(mode);
  } 
}




/***********************************
*       PIT INIT & INTERRUPT SETUP
***********************************/
void PIT_init(uint32_t ldval) {
   SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;    // SIM_SCGC6: PIT=1 : Enable PIT Clock gate control
   PIT->MCR = 0x00U;                    // PIT_MCR: MDIS=0,FRZ=0 : Enable device clock
   PIT->CHANNEL[0].TCTRL = 0x00U;       // PIT_TCTRL0: CHN=0,TIE=0,TEN=0 :  Clear control register 
   PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;    // PIT_TFLG0: TIF=1: Clear timer flag register 
   PIT->CHANNEL[0].LDVAL = PIT_LDVAL_TSV(ldval);// PIT_LDVAL0: TSV=0xD1B6:  Set up load register 
   NVIC_SetPriority(PIT_IRQn, 2);             // Set priority for PIT IRQ22 */
   NVIC_EnableIRQ(PIT_IRQn);                  // Enable IRQ for PIT IRQ22. PIT_TCTRL0: CHN=0,TIE=1,TEN=1 
   PIT->CHANNEL[0].TCTRL = (PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK); 
}

void PIT_IRQHandler(){
    PIT->CHANNEL[0].TFLG |= 1; // Clear timer interrupt flag
    if (IS_TIMER_ENABLED(mode)) {
      MODE_XOR_BUZZ(mode);
      MODE_TIMER_OFF(mode);
    }
}


