
#include "nrf2401.h"


#define Debug_20130117_

#if defined(Debug_20130117)

const char flash_str[10240] PROGMEM = " ############################################### ";

#endif

#define MAX_PACKAGE_ID 32
#define Device_Reciver

static u8 TX_Device_Addr[NRF24L01_TX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};

#if defined(Device_Reciver)

static u8 RX_Device_Addr[NRF24L01_RX_ADR_WIDTH] = {0x34,0x43,0x10,0x10,0x01};
static u8 nrf_rx_buff[32];
static int nrf_rx_len = 32;
static int package_id = 0;

#else

static u8 nrf_tx_buff[32] = "nrf24l01 demo - STM32F103!\r\n";
static int nrf_tx_len = 32;

#endif

#if defined(__GNUC__) && defined(__GNUC_MINOR__)


// Defines the port that holds the signals CE, CSN and IRQ
#define NRF_PORT_CSN PORTB // output PIN
#define NRF_DDR_CSN  DDRB
#define NRF_CSN PB4

#define NRF_CE  PB1
#define NRF_PORT_CE PORTB // output PIN
#define NRF_DDR_CE  DDRB

#define NRF_PIN_IRQ  PIND  // input PIN
#define NRF_PORT_IRQ PORTD// output PIN
#define NRF_DDR_IRQ  DDRD
#define NRF_IRQ INT0

// Definitions for selecting and enabling nRF24L01 module
#define SET_CSN_HIGH() NRF_PORT_CSN |= (1<<NRF_CSN)
#define SET_CSN_LOW()  NRF_PORT_CSN &=~(1<<NRF_CSN)
#define SET_CE_HIGH()  NRF_PORT_CE  |= (1<<NRF_CE)
#define SET_CE_LOW()   NRF_PORT_CE  &=~(1<<NRF_CE)


SIGNAL (SIG_INTERRUPT0)
{
  u8 status;

  status = nfr_GetStatus();
  if(status & NRF24L01_MASK_RX_DR)
  {
    SET_CE_LOW();
    if((status & NRF24L01_RX_P_NO) != NRF24L01_RX_P_NO)
    {
      nrf_ReadRxPload(nrf_rx_buff,nrf_rx_len);
    }
    package_id = (package_id + 1) & (MAX_PACKAGE_ID - 1);

    print("Expect Package ID: --------. FIFO:[0x%02x].Status:[Da Gt]. Interrupt-DR.\r",status);
  }
  else if(status & NRF24L01_MSAK_TX_DS)
  {
    nrf_FlushTx();
    print("Expect Package ID: --------. FIFO:[0x%02x].Status:[TX_DS]. Interrupt-DS.\r",status);
  }
  else if(status & NRF24L01_MSAK_MAX_RT)
  {
    print("Expect Package ID: --------. FIFO:[0x%02x].Status:[MAXRT]. Interrupt-RT.\r",status);
  }
  else
  {
    print("Unexpect interrupt-----------------------------------------Interrupt-Unexpect!\r");
  }
  // clear interrupt flag
  nrf_WriteReg(NRF24L01_STATUS,status);
  SET_CE_HIGH();

}

int nrf_init(void)
{
  cli();
	
  // Set CE and CSN as output
  NRF_DDR_CSN |= _BV(NRF_CSN);
  NRF_DDR_CE  |= _BV(NRF_CE);
  SET_CE_LOW();
  SET_CSN_HIGH();

  // set IRQ as input
  NRF_DDR_IRQ &= ~ _BV(NRF_IRQ); // set as input
  NRF_PORT_IRQ |= _BV(NRF_IRQ);  // enable interval pull up
	
  //Init SPI
  spi_init();

  // div:2-128/256; mode:
  if(spi_master_setup(128,SCK_IDLE_LOW | SCK_PHASE_FIRST |SPI_MSB) != 0) return -1; 
	
  // IRQ should high (should extral pull up via hardare) every time after init	
  {	// check this before enable the IRQ interrupt
    nrf_WriteReg(NRF24L01_STATUS,nfr_GetStatus()); // clear All Interrupt Flag
    if(NRF_PIN_IRQ & NRF_IRQ);
    else
    {    
      if(UART_Ready) uart_puts("NRF IRQ not pull up\r\n");
      return -2;
    }
  }
	
  // Initialize external interrupt(IRQ)
#if defined( __AVR_ATmega168__)

  EICRA = ((1<<ISC11)|(0<<ISC10)|(1<<ISC01)|(0<<ISC00));
  // Set external interupt on falling edge
  EIMSK  = ((0<<INT1)|(1<<INT0));

#elif defined(__AVR_ATmega16__)

    MCUCR |= 0x02;
    GICR  |= 0x40;

#endif

  sei();
  return 0;
}


void led_Test(void) // to test the SPI PIN
{
  uart_flush();
  DDRB = 0xfe;
  PORTB= 0x00;

  while(1)
  {
    PORTB = 0x00;
    _delay_ms(500);

    PORTB = 0xff;
    _delay_ms(500);
  }
}

#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)


// Defines the port that holds the signals CE, CSN and IRQ
#define NRF_PORT_CSN GPIOA // output PIN
#define NRF_PORT_CE  GPIOA // output PIN
#define NRF_PORT_IRQ GPIOB // input PIN

#define NRF_CE  PA3
#define NRF_CSN PA4
#define NRF_IRQ PB0

// Definitions for selecting and enabling nRF24L01 module
#define SET_CSN_HIGH()  GPIO_SetBits(NRF_PORT_CSN, NRF_CSN)
#define SET_CSN_LOW()   GPIO_ResetBits(NRF_PORT_CSN, NRF_CSN)
#define SET_CE_HIGH()   GPIO_SetBits(NRF_PORT_CE, NRF_CE)
#define SET_CE_LOW()    GPIO_ResetBits(NRF_PORT_CE, NRF_CE)


int nrf_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
		
  RCC_APB2PeriphClockCmd(PORT_APB_Periph(NRF_PORT_IRQ) | PORT_APB_Periph(NRF_PORT_CSN) | PORT_APB_Periph(NRF_PORT_CE), ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB ,ENABLE);
	
  // Set CE and CSN as output
  GPIO_InitStructure.GPIO_Pin = NRF_CE;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF_PORT_CE, &GPIO_InitStructure);
  SET_CE_LOW();
	
  GPIO_InitStructure.GPIO_Pin = NRF_CSN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(NRF_PORT_CSN, &GPIO_InitStructure);
  SET_CSN_HIGH();
	
  // set IRQ as input, should extral pull up via hardare
  GPIO_InitStructure.GPIO_Pin = NRF_IRQ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(NRF_PORT_IRQ, &GPIO_InitStructure);
 	
  //Init SPI
  spi_init();

  // div:2-128/256; mode:
  if(spi_master_setup(128,SCK_IDLE_LOW | SCK_PHASE_FIRST |SPI_MSB) != 0) return -1; 
	
	// IRQ should high (should extral pull up via hardare) every time after init
	{	// check this before enable the IRQ interrupt
	  nrf_WriteReg(NRF24L01_STATUS,nfr_GetStatus()); // clear All Interrupt Flag
    if(GPIO_ReadInputDataBit(NRF_PORT_IRQ,NRF_IRQ) == Bit_RESET)
    {
      if(UART_Ready) uart_puts("NRF IRQ not pull up\r\n");
      return -2;
    }
	}
	
  // Initialize external interrupt here
	
  return 0;
}

void led_Test(void)
{
}

#endif


/**************************************************/
u8 nfr_GetStatus(void)
{
  u8 status;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_NOP);
  SET_CSN_HIGH();
  return status;
}

u8 nrf_WriteReg(u8 addr, u8 data)
{
  u8 status;
  if((addr >= 0x18 && addr <= 0x1b) || addr >= 0x1d) return 255;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_WRITE_REG | addr);
  spi_master_SendByte(data);
  SET_CSN_HIGH();
  return status;
}

u8 nrf_WriteBuffer(u8 addr, u8 * data, int len)
{
  u8 status;
  int i;
  if((addr >= 0x18 && addr <= 0x1b) || addr >= 0x1d) return 255;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_WRITE_REG | addr);
  for(i=0; i<len ; i++) spi_master_SendByte(*(data+i));
  SET_CSN_HIGH();
  return status;
}

u8 nrf_ReadReg(u8 addr, u8 * data)
{
  u8 status;
  if((addr >= 0x18 && addr <= 0x1b) || addr >= 0x1d) return 255;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_READ_REG | addr);	
  (*data) = spi_master_GetByte();	
  SET_CSN_HIGH();
  return status;
}

u8 nrf_ReadBuffer(u8 addr, u8 * data, int len)
{
  u8 status;
  int i;
  if((addr >= 0x18 && addr <= 0x1b) || addr >= 0x1d) return 255;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_READ_REG | addr);
  for(i=0; i<len; i++)  (*(data+i)) = spi_master_GetByte();
  SET_CSN_HIGH();
  return status;
}

u8 nrf_FlushTx()
{
  u8 status;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_FLUSH_TX);
  SET_CSN_HIGH();
  return status;
}

u8 nrf_FlushRx()
{
  u8 status;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_FLUSH_RX);
  SET_CSN_HIGH();
  return status;
}

u8 nrf_ReadRxPload(u8 * data, int len)
{
  u8 status;
  int i;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_RD_RX_PLOAD);
  for(i=0; i<len; i++) (*(data+i)) = spi_master_GetByte();
  SET_CSN_HIGH();
  return status;
}

u8 nrf_WriteTxPload(u8 * data, int len)
{
  u8 status;
  int i;
  SET_CSN_HIGH();
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_WR_TX_PLOAD);
  for(i=0; i<len; i++) spi_master_SendByte(*(data+i));
  SET_CSN_HIGH();
  return status;
}


void nrf_Test(void)
{
  u8 status;
  int count;
 
  count = -1;

  #if defined(Debug_20130117)
  uart_putp(flash_str);
  #endif

  if(nrf_init() != 0)
  {
    uart_puts("NRF2401 Init Failed!\r\n");
    return;
  }
  else
  {
    uart_puts("NRF2401 Init OK!\r\n");
  }

  SET_CE_LOW();	
  uart_puts("NRF2401 Set CE Low!\r\n");
  _delay_ms(500);

  // Test Ok on STM32F103C6T8, but no on the ATmega16 
  //#define Debug_20130115
  #if defined(Debug_20130115)	

  print("SPCR:= %x\r\n",SPCR);
  print("SPSR:= %x\r\n",SPSR);
  print("DDRB:= %x\r\n",DDRB);
 
  SET_CSN_LOW();
  status = spi_master_SendByte(NRF24L01_READ_REG | NRF24L01_NOP );
  print("Status:%x \r\n",status);
	
  //SET_CSN_HIGH();
  //SET_CSN_LOW();
		
  status = spi_master_GetByte();
  print("Status:%x \r\n",status);	
	
  //while(1)
  //{
    //nrf_FlushTx();
    //nrf_FlushRx();
  //}

  SET_CSN_HIGH();
  SET_CSN_LOW();
  spi_master_SendByte(NRF24L01_READ_REG | NRF24L01_EN_AA );
  status = spi_master_GetByte();
  SET_CSN_HIGH();
  print("EN_AA:%x \r\n",status);

  _delay_ms(1);

  status = nfr_GetStatus();
  print("Status:%x \r\n",status);  //0x0e
  #endif

  #if(1)

  nrf_ReadReg(NRF24L01_CONFIG,&status);  //0x08
  print("Config:%x \r\n",status);

  nrf_WriteReg(NRF24L01_CONFIG,0x55);
  nrf_ReadReg(NRF24L01_CONFIG,&status);
  print("Config:%x \r\n",status);  //0x55

  SET_CE_LOW();	

  // check FIFO
  status = nfr_GetStatus();
  if((status & NRF24L01_RX_P_NO) == NRF24L01_RX_P_NO)
  {
    print("Rx FIFO empty!\r\n");
  }
  else
  {
    print("Rx FIFO not empty!\r\n");
  }
  
  if((status & NRF24L01_TX_FULL) == NRF24L01_TX_FULL)
  {
    print("Tx FIFO full!\r\n");
  }
  else
  {
    print("Tx FIFO not full!\r\n");
  }
  #endif // Test Ok on STM32F103C6T8, but no on the ATmega16 	

  #if defined(Device_Reciver)
	
  SET_CE_LOW();
  SET_CSN_HIGH();
	
  // 1. set RPIM_RT to high in CONFIG reg
  nrf_WriteReg(NRF24L01_CONFIG,0x0F); //enable CRC, 2bytes CRC, PWR UP, PRX

  // 1. All pipe that is reciveing data must be enable(EN_RXADDR)
  nrf_WriteReg(NRF24L01_EN_RXADDR,0x01); // enable Pipe0
  nrf_WriteReg(NRF24L01_EN_RXADDR,0x21); // enable Pipe0 /5

  // 1. if enable auto ack, must enable EN_AA reg
  nrf_WriteReg(NRF24L01_EN_AA,0x01);     // auto ack on Pipe0	
  nrf_WriteReg(NRF24L01_EN_AA,0x21);     // auto ack on Pipe0 / 5
	
  // 1. correct payload widths(RX_PW_Px reg)
  nrf_WriteReg(NRF24L01_SETUP_AW,0x03);  // address bytes width: 5 bytes	
  nrf_WriteReg(NRF24L01_RX_PW_P0,nrf_rx_len);  // expect package width
  nrf_WriteReg(NRF24L01_RX_PW_P5,nrf_rx_len);  // expect package width
	
  // 1. Address set as item 2
  //nrf_WriteBuffer(NRF24L01_RX_ADDR_P0,RX_Device_Addr,NRF24L01_RX_ADR_WIDTH); //write the receiver address
  nrf_WriteBuffer(NRF24L01_RX_ADDR_P0,TX_Device_Addr,NRF24L01_TX_ADR_WIDTH); //write the receiver address
  nrf_WriteBuffer(NRF24L01_RX_ADDR_P5,TX_Device_Addr,NRF24L01_TX_ADR_WIDTH); //write the receiver address

  // 2. common setup
  nrf_WriteReg(NRF24L01_RF_SETUP,0x07);  //1Mbps, 0dBm, LNA	
  nrf_WriteReg(NRF24L01_RF_CH,0x00);	// channel 0, Freq= 2.4G + CH MHz  
	
  //nrf_WriteBuffer(NRF24L01_TX_ADDR,TX_Device_Addr,NRF24L01_TX_ADR_WIDTH);  //write local device address

  SET_CE_HIGH(); // keep 10us to finish send
  _delay_ms(200);
	
  count = 1;
  nrf_rx_buff[31] = 0;
	
  debug_led_off();
  _delay_ms(1000);
	
  debug_led_on();
  _delay_ms(1000);
	
  debug_led_off();
  print("\r\n\r\n"); 
  print("**********************");
  print("\r\n"); 

  while(1)
  {
    status = nfr_GetStatus();
    if(status & NRF24L01_MASK_RX_DR)
    {
      debug_led_on();
      SET_CE_LOW();
      if((status & NRF24L01_RX_P_NO) != NRF24L01_RX_P_NO)
      {
        nrf_ReadRxPload(nrf_rx_buff,nrf_rx_len);
      }

      status = nfr_GetStatus();
      print("Expect Package ID: %-8x. FIFO:[0x%02x].Status:[RD_DR]\r",count,status);
      package_id = (package_id + 1) & (MAX_PACKAGE_ID - 1);
			
      // clear interrupt flag
      nrf_WriteReg(NRF24L01_STATUS,status);
    }	
    else 
    {
      print("Expect Package ID: %-8x. FIFO:[0x**].Status:[No Da]\r",count);
    }
        
    if(count == package_id) // expect package id: count
    {
      debug_led_on();
      count = (count + 1) & (MAX_PACKAGE_ID - 1);
      status = nfr_GetStatus();
      print("Expect Package ID: %-8x. FIFO:[0x%02x].Status:[New P]\r",count,status);
      SET_CE_HIGH();
    }

    _delay_ms(500);
  }
	
  #else
  SET_CE_LOW();
  SET_CSN_HIGH();
	
  nrf_tx_buff[31] = 0;
	
  // 1. Set PRIM_RX to low
  nrf_WriteReg(NRF24L01_CONFIG,0x0e); //enable CRC, 2bytes CRC, PWR UP, PTX for temp
	
  // 2.0 send the address for the receving node(TX_ADDR) to nrf24l01
  //  TX_ADDR does not have to rewriten if unchanged
  nrf_WriteReg(NRF24L01_SETUP_AW,0x03);  // address bytes width: 5 bytes
  nrf_WriteBuffer(NRF24L01_TX_ADDR,TX_Device_Addr,NRF24L01_TX_ADR_WIDTH);  //write local device address

  // 2.1 send payload data(TX_PLD) to nrf24l01
  status = nfr_GetStatus();
  if(status & NRF24L01_TX_FULL) // TX FIFO full
  {
    print("2.1.1 TX FIFO full\r\n");
  }
  else
  {
    nrf_WriteTxPload(nrf_tx_buff,nrf_tx_len);  // load tramsfer payload
    status = nfr_GetStatus();
    if(status & NRF24L01_TX_FULL) // TX FIFO full
    {
      print("2.1.2 TX FIFO full\r\n");
    }
  }
	
  // 2.3 if the auto ack is enabled, RX_ADDR_P0 should be TX_ADDR
  // (data of Tx) ---> (rx pipe of Rx) --ack--> (rx pipe of Tx)
  nrf_WriteReg(NRF24L01_EN_AA,0x01);     // auto ack on Pipe0
  nrf_WriteBuffer(NRF24L01_RX_ADDR_P0,TX_Device_Addr,NRF24L01_TX_ADR_WIDTH);  //write auto ack address

  // 3. common setup  
  nrf_WriteReg(NRF24L01_EN_RXADDR,0x01); // enable Pipe0
  nrf_WriteReg(NRF24L01_RF_CH,0x00);	   // channek 0, Freq= 2.4G + CH MHz
  nrf_WriteReg(NRF24L01_RF_SETUP,0x07);  //1Mbps, 0dBm, LNA	

  SET_CE_HIGH(); // keep 10us to finish send		
  _delay_ms(100);
	
  print("Send String:%s \r\n",nrf_tx_buff); 
  _delay_ms(2000);
  count = 1; 
	
  debug_led_off();
  _delay_ms(1000);
		
  debug_led_on();
  _delay_ms(1000);

  debug_led_off();
  _delay_ms(1000);
	
  while(1)
  {
    print("Transmitted %-8x. FIFO:[      ]. Status:[      ]!\r",count++);    
    	
    status = nfr_GetStatus();
    if(status & NRF24L01_MASK_RX_DR)
    {
      SET_CE_LOW();
      print("Transmitted %-8x. FIFO:[      ]. Status:[RX_DR ]!\r",count);
			
      // clear interrupt flag
      nrf_WriteReg(NRF24L01_STATUS,status);
    }
    else if(status & NRF24L01_MSAK_TX_DS)
    {
      SET_CE_LOW();
      print("Transmitted %-8x. FIFO:[      ]. Status:[TX_DS ]!\r",count);
			
      // clear interrupt flag
      nrf_WriteReg(NRF24L01_STATUS,status);
    }
    else if(status & NRF24L01_MSAK_MAX_RT)
    {
      SET_CE_LOW();
      print("Transmitted %-8x. FIFO:[      ]. Status:[MAX_RT]!\r",count);
			
      // clear interrupt flag
      nrf_WriteReg(NRF24L01_STATUS,status);			
    }
    else
    {
    }
		
    SET_CE_LOW();
    status = nfr_GetStatus();
    if(status & NRF24L01_TX_FULL) // TX FIFO full
    {
      print("Transmitted %-8x. FIFO:[Full  ].\r",count);
    }
    else
    {
      nrf_WriteTxPload(nrf_tx_buff,nrf_tx_len);  // load tramsfer payload
      status = nfr_GetStatus();
      if(status & NRF24L01_TX_FULL) // TX FIFO full
      {
        print("Transmitted %-8x. FIFO:[Full-R]!\r",count);
      }
    }		
		
    SET_CE_HIGH(); // keep 10us to finish send		
    _delay_ms(100);
		
    _delay_ms(2999);
  }
  #endif
}
	


