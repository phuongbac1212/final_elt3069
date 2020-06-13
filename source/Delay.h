#ifndef __DELAY_H__
#define __DELAY_H__

uint64_t volatile msTicks        = 0; 

void init_SysTick_interrupt()
{
  SysTick->LOAD = SystemCoreClock / 1000;
  //configured the SysTick to count in 1ms
  // Select Core Clock & Enable SysTick & Enable Interrupt 
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_ENABLE_Msk;
}

/*
*        SysTick interrupt Handler
*/
void SysTick_Handler (void) 
{
  msTicks++; // Increment counter 
  // uint64_t is enough for 584942 Millennium
}

/*
*       DELAY FUNCTION
*       @PARA TICK is time delay in ms.
*/
void Delay (uint32_t TICK) {
  while (msTicks < TICK); 
  msTicks = 0; 
  // Reset counter 
}

#endif
