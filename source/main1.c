 /***********************************
*       LIBRARIES DECLARATION
***********************************/
#include <MKL46z4.h>
#include <stdio.h>
#include "Delay.h"
#include "LCD.h"

/***********************************
*       GLOBAL VARIABLES
***********************************/
uint8_t mode                    = 0;
uint64_t volatile timeNow       = 0;
uint64_t lastOperation          = 0;
/***********************************
*       USER DEFINATIONS
***********************************/
#define LED1            (1 << 29)
#define LED2            (1 << 5)
#define SW1             (1 << 3)
#define SW2             (1 << 12)
#define SW1_PIN         (5)
#define SW2_PIN         (12)
#define EITHER_EDGE     (11)    
#define SW1_PRIORITY    (255)
#define ON              (1)
#define OFF             (0)
#define ISW1            (0x8)
#define ISW2            (0x1000)
#define DELTA_TIME      (200) 

#define IDLE            (0)
#define SEATED          (1)
#define BELTED          (2) 
#define BUZZER          (3)
/*
k | mean
--------
0 | time
1 | seat
2 | belt
3 | buzzer
*/

#define MODE_SEAT               (1)
#define MODE_TIMER              (0)
#define MODE_BELT               (2)
#define MODE_BUZZ               (3)

#define MODE_XOR_TIMER(m)       m = (m ^ (1 <<MODE_TIMER))
#define MODE_XOR_SEAT(m)        m = (m ^ (1 << MODE_SEAT))
#define MODE_XOR_BELT(m)        m = (m ^ (1 << MODE_BELT))
#define MODE_XOR_BUZZ(m)        m = (m ^ (1 << MODE_BUZZ))

#define IS_SEATED(m)            ((m >> MODE_SEAT) & 1)
#define IS_BELTED(m)            ((m >> MODE_BELT) & 1)
#define IS_TIMER_ENABLED(m)     ((m >>MODE_TIMER) & 1)
#define IS_BUZZING(m)           ((m >> MODE_BUZZ) & 1)

#define MODE_TIMER_ON(m)        { \
                                  m |= ON; \
                                  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK; \
                                }
#define MODE_TIMER_OFF(m)       { \
                                  m &= ~ON;\
                                  PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; \
                                }
#define MODE_BUZZER_OFF(m)      m &= ~(1 << MODE_BUZZ)

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

int main() {
  SegLCD_Init();
  while (1) {
      SegLCD_DisplayHex(10);

  }
}

int main0(){
  initSW();
  initLed();
  init_SysTick_interrupt();
  PORTC_PORTD_Interrupt_init();
  SegLCD_Init();
  PIT_init(20971520*5);
  //Delay(5000);
  while(1) {
    if(msTicks - lastOperation > DELTA_TIME) {
      //printf(": %x\n",mode);
      if (!mode) { // idle
        //printf("I\n");
        FPTE->PDOR |= LED1;
        FPTD->PDOR |= LED2;
      }
      else if(IS_BUZZING(mode)){ //buzz
        //printf("Bz\n");
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
        //printf("S\n");
        FPTE->PTOR |= LED1;
        FPTD->PCOR |= LED2;
        MODE_TIMER_ON(mode);
      }
     
      lastOperation = msTicks;
    }
  }
}