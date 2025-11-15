#ifndef __KEY_TIMER_H
#define __KEY_TIMER_H
#include "stm32f1xx_hal.h"
#define KEY_PIN        GPIO_PIN_2
#define KEY_PORT       GPIOA

typedef enum {
    KEY_IDLE = 0,    
    KEY_PRESSED,     
    KEY_RELEASED     
} Key_StateTypeDef;

Key_StateTypeDef Key_GetState(void);
void Key_TimerScan(void);  

#endif
