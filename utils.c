

#include "cpu.h"
#include "uart.h"


#if defined(__GNUC__) && defined(__GNUC_MINOR__)


void debug_led_on(void)
{
  PORTB |= 0x01;
}

void debug_led_off(void)
{
  PORTB &= 0xfe;
}

#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)

void _delay_ms(u32 tick)
{
  u32 i,j;
  u32 load;
  RCC_ClocksTypeDef RCC_ClockFreq;
	
  RCC_GetClocksFreq(&RCC_ClockFreq);
	
  switch(RCC_ClockFreq.SYSCLK_Frequency)
  {
    case 72000000 : load = 10000; break;
    case 36000000 : load = 10000; break;
    case 20000000 : load = 10000; break;
    case 8000000  : load = 10000; break;
    default: load = 10000;
  }
	
  for(i=0;i<tick;i++)
  {
    for(j=0;j<load;j++) ;
  }
}


void debug_led_on(void)
{
  GPIO_SetBits(GPIOA, GPIO_Pin_3);
}

void debug_led_off(void)
{
  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}

#endif

/*
#define Debug   0 
#define Info    1
#define Warning 2
#define Error   3
*/

void logd(const char * function, char *info, int level)
{
  switch(level)
  {
    case 0: uart_puts("[Debug] "); break;
    case 1: uart_puts("[Info] "); break;
    case 2: uart_puts("[Warning] "); break;
    case 3: uart_puts("[Error] "); break;
    default: return;
  }
  
  uart_puts(function);
  uart_puts(": ");	
  uart_puts(info);	
}

