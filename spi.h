


#ifndef _SPI_H_
#define _SPI_H_

#include "cpu.h"

#define SPI_LSB         0x04
#define SPI_MSB         0x00
#define SCK_IDLE_HIGH   0x02
#define SCK_IDLE_LOW    0x00
#define SCK_PHASE_FIRST 0x00
#define SCK_PHASE_LAST  0x01

#define SPI_INTERRUPT_ENABLE 0

void spi_init(char);
u08  spi_master_setup(u08 div,u08 mode); // SPI_CLK = F_CLK / div;

void spi_cs_on(void);
void spi_cs_off(void);

void spi_master_nSendByte(u08 data);
u08  spi_master_SendByte(u08 data);
u08  spi_master_GetByte(void);

#if defined (__GNUC__) || defined (__ICC_VERSION) 

#if defined(__AVR_ATmega128__) || defined(ATMega128)

#define SPI_PORT PORTB
#define SPI_DIN  PINB
#define SPI_DDR  DDRB
#define SCK      PB1
#define MISO     PB3
#define MOSI     PB2
#define SS       PB0

#elif defined(__AVR_ATmega8__) || defined(ATMega8)

#define SPI_PORT PORTB
#define SPI_DIN  PINB
#define SPI_DDR  DDRB
#define SCK      PB5
#define MISO     PB4
#define MOSI     PB3
#define SS       PB2

#elif defined(__AVR_ATmega16__) || defined(ATMega16)

#define SPI_PORT PORTB
#define SPI_DIN  PINB
#define SPI_DDR  DDRB
#define SCK      PB7
#define MISO     PB6
#define MOSI     PB5
#define SS       PB4

#endif

#endif // #if defined (__GNUC__) || defined (__ICC_VERSION) 


#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)

#define Config_SPI1
#if defined(Config_SPI1)
#define SPI_SPI  SPI1
#define SPI_APB_Periph RCC_APB2Periph_SPI1
#define SPI_PORT GPIOA
#define MOSI     PA7
#define MISO     PA6
#define SCK      PA5
#define SS       PA4
#endif


#if defined(Config_SPI2)
#define SPI_SPI  SPI2
#define SPI_APB_Periph RCC_APB1Periph_SPI2
#define SPI_PORT GPIOB
#define MOSI     PB15
#define MISO     PB14
#define SCK      PB13
#define SS       PB12
#endif

#endif


#endif // #ifndef _SPI_H_
