#include "key_timer.h"

static uint8_t key_cnt = 0;          
static Key_StateTypeDef key_state = KEY_IDLE;
static Key_StateTypeDef last_state = KEY_IDLE;

void Key_TimerScan(void)
{
    uint8_t key_level = HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN);
    if (key_level != (last_state == KEY_PRESSED ? 1 : 0))
	{
        key_cnt++;
        if (key_cnt >= 3) 
		{
            key_state = (key_level == 1) ? KEY_PRESSED : KEY_RELEASED;
            last_state = key_state;
            key_cnt = 0;
        }
    }
	else 
	{
        key_cnt = 0;
    }
}

Key_StateTypeDef Key_GetState(void)
{
    Key_StateTypeDef temp = key_state;
    if (temp != KEY_IDLE)
	{
        key_state = KEY_IDLE;
    }
    return temp;
}
