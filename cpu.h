
#ifndef _CPU_H_
#define _CPU_H_

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


#include "avrlib_micro.h"
#include "avrlib_types.h"

#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

#endif

#define Debug   0 
#define Info    1
#define Warning 2
#define Error   3

void logd(const char * , char *, int);
void debug_led_on(void);
void debug_led_off(void);

#endif

