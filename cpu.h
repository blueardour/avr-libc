
#ifndef _CPU_H_
#define _CPU_H_

#include <avrlib_types.h>
#include <avrlib_micro.h>

#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
    defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
    defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
    defined (STM32F10X_XL) || defined (STM32F10X_CL)

#include "stm32f10x.h"

#define GPIO_APB2Periph(x) ((x==GPIOA)? RCC_APB2Periph_GPIOA:  \
                           ((x==GPIOB) ? RCC_APB2Periph_GPIOB: \
                           ((x==GPIOC) ? RCC_APB2Periph_GPIOC: \
                           ((x==GPIOD) ? RCC_APB2Periph_GPIOD:0) )))

#define PORT_APB_Periph(x)  (GPIO_APB2Periph(x))


typedef unsigned char BOOL;
typedef unsigned char u08;

#define PA0  GPIO_Pin_0
#define PA1  GPIO_Pin_1
#define PA2  GPIO_Pin_2
#define PA3  GPIO_Pin_3
#define PA4  GPIO_Pin_4
#define PA5  GPIO_Pin_5
#define PA6  GPIO_Pin_6
#define PA7  GPIO_Pin_7
#define PA8  GPIO_Pin_8
#define PA9  GPIO_Pin_9
#define PA10 GPIO_Pin_10
#define PA11 GPIO_Pin_11
#define PA12 GPIO_Pin_12
#define PA13 GPIO_Pin_13
#define PA14 GPIO_Pin_14
#define PA15 GPIO_Pin_15

#define PB0  PA0
#define PB1  PA1
#define PB2  PA2
#define PB3  PA3
#define PB4  PA4
#define PB5  PA5
#define PB6  PA6
#define PB7  PA7
#define PB8  PA8
#define PB9  PA9
#define PB10 PA10
#define PB11 PA11
#define PB12 PA12
#define PB13 PA13
#define PB14 PA14
#define PB15 PA15

#define PC0  PA0
#define PC1  PA1
#define PC2  PA2
#define PC3  PA3
#define PC4  PA4
#define PC5  PA5
#define PB6  PA6
#define PC7  PA7
#define PC8  PA8
#define PC9  PA9
#define PC10 PA10
#define PC11 PA11
#define PC12 PA12
#define PC13 PA13
#define PC14 PA14
#define PC15 PA15

#define PD0  PA0
#define PD1  PA1
#define PD2  PA2
#define PD3  PA3
#define PD4  PA4
#define PD5  PA5
#define PD6  PA6
#define PD7  PA7
#define PD8  PA8
#define PD9  PA9
#define PD10 PA10
#define PD11 PA11
#define PD12 PA12
#define PD13 PA13
#define PD14 PA14
#define PD15 PA15

void _delay_ms(u32);


#elif defined(__GNUC__) && defined(__GNUC_MINOR__)

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
  #error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif


#ifndef cli
  #define cli()	__asm__ __volatile__ ("cli" ::)
#endif
#ifndef sei
  #define sei()	__asm__ __volatile__ ("sei" ::)
#endif


#include <avr/pgmspace.h>
#include <avr/io.h>

#elif defined(__ICC_VERSION)


#ifndef _AVR_IO_H_
#define _AVR_IO_H_

#if defined (ATMega16)
#  include <iom16v.h>

/*
#elif defined (ATmega64)
#  include <iom64v.h>
#elif defined (ATmega64)
#  include <iom640v.h>
#elif defined (ATmega644)
#  include <iom644.h>
#elif defined (__AVR_ATmega644P__)
#  include <avr/iom644p.h>
#elif defined (__AVR_ATmega645__)
#  include <avr/iom645.h>
#elif defined (__AVR_ATmega6450__)
#  include <avr/iom6450.h>
#elif defined (__AVR_ATmega649__)
#  include <avr/iom649.h>
#elif defined (__AVR_ATmega6490__)
#  include <avr/iom6490.h>
#elif defined (__AVR_ATmega103__)
#  include <avr/iom103.h>
#elif defined (__AVR_ATmega32__)
#  include <avr/iom32.h>
#elif defined (__AVR_ATmega323__)
#  include <avr/iom323.h>
#elif defined (__AVR_ATmega324P__)
#  include <avr/iom324.h>
#elif defined (__AVR_ATmega325__)
#  include <avr/iom325.h>
#elif defined (__AVR_ATmega325P__)
#  include <avr/iom325.h>
#elif defined (__AVR_ATmega3250__)
#  include <avr/iom3250.h>
#elif defined (__AVR_ATmega3250P__)
#  include <avr/iom3250.h>
#elif defined (__AVR_ATmega328P__)
#  include <avr/iom328p.h>
#elif defined (__AVR_ATmega329__)
#  include <avr/iom329.h>
#elif defined (__AVR_ATmega329P__)
#  include <avr/iom329.h>
#elif defined (__AVR_ATmega3290__)
#  include <avr/iom3290.h>
#elif defined (__AVR_ATmega3290P__)
#  include <avr/iom3290.h>
#elif defined (__AVR_ATmega32HVB__)
#  include <avr/iom32hvb.h>
#elif defined (__AVR_ATmega406__)
#  include <avr/iom406.h>
#elif defined (__AVR_ATmega161__)
#  include <avr/iom161.h>
#elif defined (__AVR_ATmega162__)
#  include <avr/iom162.h>
#elif defined (__AVR_ATmega163__)
#  include <avr/iom163.h>
#elif defined (__AVR_ATmega164P__)
#  include <avr/iom164.h>
#elif defined (__AVR_ATmega165__)
#  include <avr/iom165.h>
#elif defined (__AVR_ATmega165P__)
#  include <avr/iom165p.h>
#elif defined (__AVR_ATmega168__)
#  include <avr/iom168.h>
#elif defined (__AVR_ATmega168P__)
#  include <avr/iom168p.h>
#elif defined (__AVR_ATmega169__)
#  include <avr/iom169.h>
#elif defined (__AVR_ATmega169P__)
#  include <avr/iom169p.h>
#elif defined (__AVR_ATmega8HVA__)
#  include <avr/iom8hva.h>
#elif defined (__AVR_ATmega16HVA__)
#  include <avr/iom16hva.h>
#elif defined (__AVR_ATmega8__)
#  include <avr/iom8.h>
#elif defined (__AVR_ATmega48__)
#  include <avr/iom48.h>
#elif defined (__AVR_ATmega48P__)
#  include <avr/iom48p.h>
#elif defined (__AVR_ATmega88__)
#  include <avr/iom88.h>
#elif defined (__AVR_ATmega88P__)
#  include <avr/iom88p.h>
#elif defined (__AVR_ATmega8515__)
#  include <avr/iom8515.h>
#elif defined (__AVR_ATmega8535__)
#  include <avr/iom8535.h>
#elif defined (__AVR_AT90S8535__)
#  include <avr/io8535.h>
#elif defined (__AVR_AT90C8534__)
#  include <avr/io8534.h>
#elif defined (__AVR_AT90S8515__)
#  include <avr/io8515.h>
#elif defined (__AVR_AT90S4434__)
#  include <avr/io4434.h>
#elif defined (__AVR_AT90S4433__)
#  include <avr/io4433.h>
#elif defined (__AVR_AT90S4414__)
#  include <avr/io4414.h>
#elif defined (__AVR_ATtiny22__)
#  include <avr/iotn22.h>
#elif defined (__AVR_ATtiny26__)
#  include <avr/iotn26.h>
#elif defined (__AVR_AT90S2343__)
#  include <avr/io2343.h>
#elif defined (__AVR_AT90S2333__)
#  include <avr/io2333.h>
#elif defined (__AVR_AT90S2323__)
#  include <avr/io2323.h>
#elif defined (__AVR_AT90S2313__)
#  include <avr/io2313.h>
#elif defined (__AVR_ATtiny2313__)
#  include <avr/iotn2313.h>
#elif defined (__AVR_ATtiny13__)
#  include <avr/iotn13.h>
#elif defined (__AVR_ATtiny13A__)
#  include <avr/iotn13a.h>
#elif defined (__AVR_ATtiny25__)
#  include <avr/iotn25.h>
#elif defined (__AVR_ATtiny45__)
#  include <avr/iotn45.h>
#elif defined (__AVR_ATtiny85__)
#  include <avr/iotn85.h>
#elif defined (__AVR_ATtiny24__)
#  include <avr/iotn24.h>
#elif defined (__AVR_ATtiny44__)
#  include <avr/iotn44.h>
#elif defined (__AVR_ATtiny84__)
#  include <avr/iotn84.h>
#elif defined (__AVR_ATtiny261__)
#  include <avr/iotn261.h>
#elif defined (__AVR_ATtiny461__)
#  include <avr/iotn461.h>
#elif defined (__AVR_ATtiny861__)
#  include <avr/iotn861.h>
#elif defined (__AVR_ATtiny43U__)
#  include <avr/iotn43u.h>
#elif defined (__AVR_ATtiny48__)
#  include <avr/iotn48.h>
#elif defined (__AVR_ATtiny88__)
#  include <avr/iotn88.h>
#elif defined (__AVR_ATtiny87__)
#  include <avr/iotn87.h>
#elif defined (__AVR_ATtiny167__)
#  include <avr/iotn167.h>
#elif defined (__AVR_AT90SCR100__)
#  include <avr/io90scr100.h>
#elif defined (__AVR_ATxmega16A4__)
#  include <avr/iox16a4.h>
#elif defined (__AVR_ATxmega16D4__)
#  include <avr/iox16d4.h>
#elif defined (__AVR_ATxmega32A4__)
#  include <avr/iox32a4.h>
#elif defined (__AVR_ATxmega32D4__)
#  include <avr/iox32d4.h>
#elif defined (__AVR_ATxmega64A1__)
#  include <avr/iox64a1.h>
#elif defined (__AVR_ATxmega64A3__)
#  include <avr/iox64a3.h>
#elif defined (__AVR_ATxmega64D3__)
#  include <avr/iox64d3.h>
#elif defined (__AVR_ATxmega128A1__)
#  include <avr/iox128a1.h>
#elif defined (__AVR_ATxmega128A3__)
#  include <avr/iox128a3.h>
#elif defined (__AVR_ATxmega128D3__)
#  include <avr/iox128d3.h>
#elif defined (__AVR_ATxmega192A3__)
#  include <avr/iox192a3.h>
#elif defined (__AVR_ATxmega256A3__)
#  include <avr/iox256a3.h>
#elif defined (__AVR_ATxmega256A3B__)
#  include <avr/iox256a3b.h>
#elif defined (__AVR_ATxmega256D3__)
#  include <avr/iox256d3.h>
#elif defined (__AVR_ATA6289__)
#  include <avr/ioa6289.h>
#elif defined (__AVR_ATtiny28__)
#  include <avr/iotn28.h>
#elif defined (__AVR_AT90S1200__)
#  include <avr/io1200.h>
#elif defined (__AVR_ATtiny15__)
#  include <avr/iotn15.h>
#elif defined (__AVR_ATtiny12__)
#  include <avr/iotn12.h>
#elif defined (__AVR_ATtiny11__)
#  include <avr/iotn11.h>
*/

#else
#  if !defined(__COMPILING_AVR_LIBC__)
#    warning "device type not defined"
#  endif
#endif


//#include <avr/portpins.h>
//#include <avr/common.h>
//#include <avr/version.h>
//#include <avr/fuse.h>
//#include <avr/lock.h>

#endif  // _AVR_IO_H_


#endif

#define Debug   0 
#define Info    1
#define Warning 2
#define Error   3

void logd(const char * , char *, int);
void debug_led_on(void);
void debug_led_off(void);

#endif

