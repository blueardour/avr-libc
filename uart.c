

#include "uart.h"

BOOL UART_Ready = 0;

void uart_puts(const char * str)
{
  while(*str)
   uart_putc((u08)(*str++));
}


#if defined(__GNUC__) && defined(__GNUC_MINOR__)

void uart_init(u32 baud)
{
  u32 baudrate;

  baudrate = UART_BAUD_SELECT(baud, F_CPU);

  //disable when setup
  UART0_CONTROL = 0x00;

#if defined(AT90_UART)
  /* set baud rate */
  UBRR = (u08)baudrate;

  /* enable UART receiver and transmmitter and receive complete interrupt */
  UART0_CONTROL = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);

#elif defined (ATMEGA_USART)
  /* Set baud rate */
  if( baudrate & 0x8000 )
  {
    UART0_STATUS = (1<<U2X);  //Enable 2x speed
    baudrate &= ~0x8000;
  }
  UBRRH = (u08)(baudrate>>8);
  UBRRL = (u08) baudrate;

  /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
  #ifdef URSEL
  UCSRC = (1<<URSEL)|(3<<UCSZ0);
  #else
  UCSRC = (3<<UCSZ0);
  #endif

  #if( UART_INTERRUPT_ENABLE | UART_Rx_INTERRUPT_ENABLE)
  //#warning "UART_Rx_INTERRUPT_ENABLE 1"
  UART0_CONTROL |= (1<<RXCIE);
  #endif 

  //* Enable USART receiver and transmitter 
  UART0_CONTROL |= (1<<RXEN)|(1<<TXEN);

#elif defined (ATMEGA_USART0 )
  // disable UART when setup
  UART0_CONTROL = 0x00;

  /* Set baud rate */
  if ( baudrate & 0x8000 )
  {
    UART0_STATUS = (1<<U2X0);  //Enable 2x speed
    baudrate &= ~0x8000;
  }
  UBRR0H = (unsigned char)(baudrate>>8);
  UBRR0L = (unsigned char) baudrate;

  #if( UART_INTERRUPT_ENABLE )
  UART0_CONTROL = _BV(RXCIE0);
  #endif 

  /* Enable USART receiver and transmitter */
  UART0_CONTROL = (1<<RXEN0)|(1<<TXEN0);

  /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
  #ifdef URSEL0
  UCSR0C = (1<<URSEL0)|(3<<UCSZ00);
  #else
  UCSR0C = (3<<UCSZ00);
  #endif

#elif defined ( ATMEGA_UART )
  /* set baud rate */
  if ( baudrate & 0x8000 )
  {
    UART0_STATUS = (1<<U2X);  //Enable 2x speed
    baudrate &= ~0x8000;
  }
  UBRRHI = (unsigned char)(baudrate>>8);
  UBRR   = (unsigned char) baudrate;

  /* Enable UART receiver and transmitter and receive complete interrupt */
  UART0_CONTROL = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);

#endif

  UART_Ready = 1;
}

void uart_flush(void)
{
  while(!(UART0_STATUS & (1<<UDRE))) ;
}
 
void uart_putc(u08 data)
{
  while(!(UART0_STATUS & (1<<UDRE))) ;
  UART0_DATA = data;
}

void uart_putc9(u16 data)
{
  while( !(UART0_STATUS &(1<<UDRE))) ;

  //copy 9th bit to UART0_CONTROL
  UART0_CONTROL &= ~(1<<TXB8);
  if(data & 0x0100) UART0_CONTROL |= (1<<TXB8);

  UART0_DATA = data;
}

u08 uart_getc()
{
  while(!(UART0_STATUS & (1<<RXC))) ;
  
  return UART0_DATA;
}

void uart_putp(const char * flash_str)
{
  u08 c;
  while((c = pgm_read_byte(flash_str++)))
   uart_putc(c);
}


void uart_puth(u08 c)
{
  u08 tmp;
  tmp = (c >> 4) & 0x0f;
  if(tmp<9) tmp = '0' + tmp;
  else tmp = 'a' + tmp - 10;
  uart_putc(tmp);

  tmp = c & 0x0f;
  if(tmp<9) tmp = '0' + tmp;
  else tmp = 'a' + tmp - 10; 
  uart_putc(tmp);
}

#if(UART_Rx_INTERRUPT_ENABLE)
SIGNAL ( UART0_RECEIVE_INTERRUPT )
{
  // demo, to debug INT0
  uart_puts("In UART0_RECEIVE_INTERRUPT !\r\n");

  DDRD = 0x0a;
  PORTD &=~(1<<PD3);
  uart_putc(UART0_DATA);
  PORTD |= (1<<PD3);

  uart_puts("\r\nExiting UART0_RECEIVE_INTERRUPT !\r\n");
}

#endif

#else

void uart_init(u32 baud)
{
  USART_InitTypeDef UASRT_Initstructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  UASRT_Initstructure.USART_BaudRate = baud;
  UASRT_Initstructure.USART_WordLength = USART_WordLength_8b;
  UASRT_Initstructure.USART_StopBits = USART_StopBits_1;
  UASRT_Initstructure.USART_Parity = USART_Parity_No;
  UASRT_Initstructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  UASRT_Initstructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
  RCC_APB2PeriphClockCmd(PORT_APB_Periph(UART_PORT) | UART_APB_Periph, ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(UART_PORT, &GPIO_InitStructure);
		
  USART_Init(UART_UART, &UASRT_Initstructure);
	
  /* Enable USART1 Receive and Transmit interrupts */
  USART_ITConfig(UART_UART, USART_IT_RXNE, ENABLE); 
		
  USART_Cmd(UART_UART, ENABLE);
  UART_Ready = 1;	
}	

void uart_putc(u08 data)
{
  while(USART_GetFlagStatus(UART_UART,USART_FLAG_TXE) == RESET) ;
  USART_SendData(UART_UART,data);
}

void uart_flush(void)
{
  while(USART_GetFlagStatus(UART_UART,USART_FLAG_TXE) == RESET) ;
}

#endif



