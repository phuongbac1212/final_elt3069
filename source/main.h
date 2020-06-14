#ifndef __MAIN_H__
#define __MAIN_H__

/***********************************
*       USER DEFINATIONS
***********************************/
#define LED1            (1 << 29)
#define LED2            (1 << 5)
#define SW1             (1 << 3)
#define SW2             (1 << 12)
#define SW3             (1 << 17)
#define SW1_PIN         (3)
#define SW2_PIN         (12)
#define SW3_PIN         (17)
#define EITHER_EDGE     (11)  
#define FALLING_EDGE    (12)
#define SW1_PRIORITY    (255)
#define ON              (1)
#define OFF             (0)
#define ISW1            (0x8)
#define ISW2            (0x1000)
#define DELTA_TIME      (200) 

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
																