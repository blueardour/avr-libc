
#ifndef _UART_H_
#define _UART_H_

#include <cpu.h>

extern BOOL UART_Ready;

void uart_init(u32);
void uart_putc(u08 c);
void uart_puth(u08);
void uart_puts(const char *);
u08  uart_getc(void);

void uart_flush(void);


// ******** Interrupt Enable ************//
#define UART_INTERRUPT_ENABLE    0
#define UART_Tx_INTERRUPT_ENABLE 0
#define UART_Rx_INTERRUPT_ENABLE 0


#if defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)

#define UART_PORT GPIOA
#define UART_UART USART1 
#define UART_APB_Periph RCC_APB2Periph_USART1

#endif // STM32F_XXX

#if defined(__GNUC__) || defined (__ICC_VERSION)

#define uart_printf printf

#define UART_DEFAULT_BAUD_RATE 9600
#define UART_BAUD_SELECT(baudRate, xtalCpu) ((xtalCpu)/(baudRate)/16-1)
#define UART_BAUD_SELECT_DOUBLE_SPEED(baudRate,xtalCpu) (((xtalCpu)/(baudRate)/8-1)|0x8000)

#if(UART_INTERRUPT_ENABLE)

#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 16
#endif

#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 16
#endif

#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
  #error RX buffer size is not a power of 2
#endif

#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
  #error TX buffer size is not a power of 2
#endif

// test if the size of the circular buffers fits into SRAM 
#if ( (UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE) >= ( RAMEND - 0x60 ) )
  #error "size of UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE larger than size of SRAM"
#endif

#endif // end of #if(UART_INTERRUPT_ENABLE)

#define UART_FRAME_ERROR      0x0800  /* Framing Error by UART       */
#define UART_OVERRUN_ERROR    0x0400  /* Overrun condition by UART   */
#define UART_BUFFER_OVERFLOW  0x0200  /* receive ringbuffer overflow */
#define UART_NO_DATA          0x0100  /* no receive data available   */


#if defined(__AVR_AT90S2313__) \
 || defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
 || defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__) \
 || defined(__AVR_ATmega103__)
 
  /* old AVR classic or ATmega103 with one UART */
  #define AT90_UART
  #define UART0_RECEIVE_INTERRUPT   UART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
  #define UART0_STATUS   USR
  #define UART0_CONTROL  UCR
  #define UART0_DATA     UDR  
  #define UART0_UDRIE    UDRIE
	
#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)

  /* old AVR classic with one UART */
  #define AT90_UART
  #define UART0_RECEIVE_INTERRUPT   UART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR 
  #define UART0_UDRIE    UDRIE
	
#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || \
	   defined(__AVR_ATmega32__) || defined(__AVR_ATmega323__)|| \
	   defined(ATMega8)  || defined(ATMega16) || \
	   defined(ATMega32) || defined(ATMega323) 
	
  /* ATmega with one USART */
  #define ATMEGA_USART
  #define UART0_RECEIVE_INTERRUPT   USART_RXC_vect
  #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR
  #define UART0_UDRIE    UDRIE
	
#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__) 

  /* ATmega with two USART */
  #define ATMEGA_USART0
  #define ATMEGA_USART1
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1
 
  #ifdef UART_USE_UART1
     #define UDR               UDR1
     #define UCR               UCSR1B
     #define UBRRL             UBRR1L
     #define UBRRH             UBRR1H
     #define SIG_UART_TRANS    SIG_UART1_TRANS
     #define SIG_UART_RECV     SIG_UART1_RECV
     #define SIG_UART_DATA     SIG_UART1_DATA
  #else
    #define UDR                UDR0
    #define UCR                UCSR0B
    #define UBRRL              UBRR0L
    #define UBRRH              UBRR0H
    #define SIG_UART_TRANS     SIG_UART0_TRANS
    #define SIG_UART_RECV      SIG_UART0_RECV
    #define SIG_UART_DATA      SIG_UART0_DATA
  #endif	
	
#elif  defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__)

  /* ATmega with one USART */
  #define ATMEGA_USART
  #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR
  #define UART0_UDRIE    UDRIE
	
#elif defined(__AVR_ATmega161__)

	// compatibility with dual-uart processors
  // (if you need to use both uarts, please use the uart2 library)
  #define UDR                UDR0
  #define UCR                UCSR0B
  #define UBRRL              UBRR0
  #define SIG_UART_TRANS     SIG_UART0_TRANS
  #define SIG_UART_RECV      SIG_UART0_RECV
  #define SIG_UART_DATA      SIG_UART0_DATA
	
#elif defined(__AVR_ATmega162__) 
  /* ATmega with two USART */
  #define ATMEGA_USART0
  #define ATMEGA_USART1
  #define UART0_RECEIVE_INTERRUPT   USART0_RXC_vect
  #define UART1_RECEIVE_INTERRUPT   USART1_RXC_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1
 
#elif defined(__AVR_ATmega163__)

  /* ATmega163 with one UART */
  #define ATMEGA_UART
  #define UART0_RECEIVE_INTERRUPT   UART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR
  #define UART0_UDRIE    UDRIE

#elif defined(__AVR_ATmega169__) 

  /* ATmega with one USART */
  #define ATMEGA_USART
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR
  #define UART0_UDRIE    UDRIE

#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
      defined(__AVR_ATmega328P__) 
			
  /* TLS-Added 48P/88P/168P/328P */
  /* ATmega with one USART */
  #define ATMEGA_USART0
  #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
 
#elif defined(__AVR_ATtiny2313__)

  #define ATMEGA_USART
  #define UART0_RECEIVE_INTERRUPT   USART_RX_vect 
  #define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
  #define UART0_STATUS   UCSRA
  #define UART0_CONTROL  UCSRB
  #define UART0_DATA     UDR
  #define UART0_UDRIE    UDRIE
 
#elif defined(__AVR_ATmega329__) ||\
      defined(__AVR_ATmega649__) ||\
      defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) ||\
      defined(__AVR_ATmega645__) ||defined(__AVR_ATmega6450__)
			
   /* ATmega with one USART */
   #define ATMEGA_USART0
   #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
   #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
   #define UART0_STATUS   UCSR0A
   #define UART0_CONTROL  UCSR0B
   #define UART0_DATA     UDR0
   #define UART0_UDRIE    UDRIE0
	 
#elif defined(__AVR_ATmega3290__) ||\
      defined(__AVR_ATmega6490__)
			
  /* TLS-Separated these two from the previous group because of inconsistency in the USART_RX */
  /* ATmega with one USART */
  #define ATMEGA_USART0
  #define UART0_RECEIVE_INTERRUPT   USART_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
	
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)

  /* ATmega with two USART */
  #define ATMEGA_USART0
  #define ATMEGA_USART1
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART1_RECEIVE_INTERRUPT   USART0_UDRE_vect
  #define UART0_TRANSMIT_INTERRUPT  USART1_RX_vect
  #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1  
	
#elif defined(__AVR_ATmega644__)
  /* ATmega with one USART */
  #define ATMEGA_USART0
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
	
#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__)
 
  /* ATmega with two USART */
  #define ATMEGA_USART0
  #define ATMEGA_USART1
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART1_RECEIVE_INTERRUPT   USART0_UDRE_vect
  #define UART0_TRANSMIT_INTERRUPT  USART1_RX_vect
  #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1
	
#else
  #error "no UART definition for MCU available"
#endif

#endif  // #if defined(__GNUC__) || defined (__ICC_VERSION)

#endif  // #ifndef _UART_H_

