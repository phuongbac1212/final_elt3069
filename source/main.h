#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************
*       USER DEFINATIONS
***********************************/
#define LED1            (1 << 29)
#define LED2            (1 << 5)

#define SW1_PIN         (3)
#define SW2_PIN         (12)
#define SW3_PIN         (13)
#define SW1             (1 << SW1_PIN)
#define SW2             (1 << SW2_PIN)
#define SW3             (1 << SW3_PIN)
#define ISW1            (SW1)
#define ISW2            (SW2)
#define ISW3            (SW3)

#define EITHER_EDGE     (11)  
#define FALLING_EDGE    (10)
#define SW1_PRIORITY    (255)
#define ON              (1)
#define OFF             (0)
#define DELTA_TIME      (200) 

#define BOUND_TIME      (250)

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
                                  PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK; \
                                  m |= ON; \
                                }
#define MODE_TIMER_OFF(m)       { \
                                  m &= ~ON;\
                                  PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK; \
                                }
#define MODE_BUZZER_OFF(m)      m &= ~(1 << MODE_BUZZ)

#endif
																