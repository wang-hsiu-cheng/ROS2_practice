#include "stm32f10x.h"
#include "Serial.h"
#include "LED.h"

uint8_t RxData;
int main(void)
{
    Serial_Init();
    LED_Init();
    while (1)
    {
        if (Serial_GetRxFlag() == 1)
        {
            
            RxData = Serial_GetRxData();
            if(RxData=='A')
            {
                LED_ON();
            }
            else
            {    
                LED_OFF();
            }
        }
    }
}