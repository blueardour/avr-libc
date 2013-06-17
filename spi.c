
#include "spi.h"

#if defined(__GNUC__) || defined (__ICC_VERSION)

// SPI interrupt service handler
#if(SPI_INTERRUPT_ENABLE)

static volatile u08 spi_TransferComplete;

SIGNAL(SIG_SPI)
{
  spi_TransferComplete = TRUE;
}

#endif

void spi_cs_on(void)
{
  SPI_DDR |=  (1<<SS);
  SPI_PORT |= (1<<SS);
}

void spi_cs_off(void)
{
  SPI_DDR |=   (1<<SS);
  SPI_PORT &= ~(1<<SS);
}


void spi_init(char bit)
{
  //set SCK SS MOSI ouput
  SPI_DDR  |= (1<<SCK) | (1<<MOSI);
  
  //set SS if desired
  if(bit != 0) SPI_DDR |= (1<<SS);
	
  // Pull up SCK, SS (no mater SS is an input or output)
  SPI_PORT |= (1<<SCK) | (1<<SS);
  
  // Set MISO input
  SPI_DDR  &= ~(1<<MISO);
  
  // clear SPCR
  SPCR = 0;
  SPSR = 0;

  #if(SPI_INTERRUPT_ENABLE)
  // enable SPI interrupt
  spi_TransferComplete = TRUE;
  sbi(SPCR, SPIE);
  #endif
}

u08 spi_master_setup(u08 div,u08 mode)
{
  u08 tmp[2];
  u08 tick;
  tmp[0] = SPCR;
  tmp[1] = SPSR;

  switch(div)
  {
    case 2:   cbi(SPCR,SPR1); cbi(SPCR,SPR0); sbi(SPSR,SPI2X); break;
    case 4:   cbi(SPCR,SPR1); cbi(SPCR,SPR0); cbi(SPSR,SPI2X); break;
    case 8:   cbi(SPCR,SPR1); sbi(SPCR,SPR0); sbi(SPSR,SPI2X); break;
    case 16:  cbi(SPCR,SPR1); sbi(SPCR,SPR0); cbi(SPSR,SPI2X); break;
    case 32:  sbi(SPCR,SPR1); cbi(SPCR,SPR0); sbi(SPSR,SPI2X); break;
    case 64:  sbi(SPCR,SPR1); cbi(SPCR,SPR0); cbi(SPSR,SPI2X); break;
    case 128: sbi(SPCR,SPR1); sbi(SPCR,SPR0); cbi(SPSR,SPI2X); break;
    default:  sbi(SPCR,SPR1); sbi(SPCR,SPR0); cbi(SPSR,SPI2X); break;  //128
  }
  switch(mode)
  {
    case 0:   cbi(SPCR,DORD); cbi(SPCR,CPOL); cbi(SPCR,CPHA); break;
    case 1:   cbi(SPCR,DORD); cbi(SPCR,CPOL); sbi(SPCR,CPHA); break;
    case 2:   cbi(SPCR,DORD); sbi(SPCR,CPOL); cbi(SPCR,CPHA); break;
    case 3:   cbi(SPCR,DORD); sbi(SPCR,CPOL); sbi(SPCR,CPHA); break;
    case 4:   sbi(SPCR,DORD); cbi(SPCR,CPOL); cbi(SPCR,CPHA); break;
    case 5:   sbi(SPCR,DORD); cbi(SPCR,CPOL); sbi(SPCR,CPHA); break;
    case 6:   sbi(SPCR,DORD); sbi(SPCR,CPOL); cbi(SPCR,CPHA); break;
    case 7:   sbi(SPCR,DORD); sbi(SPCR,CPOL); sbi(SPCR,CPHA); break;
    default:  SPCR = tmp[0]; SPSR = tmp[1]; return 255;
  }

  // if the SS is an input and driven low when this device is the SPI master
  // the SPIF will be set	
  if((SPI_DDR & (1<<SS)) != (1<<SS))
  {		
    //logd(__FUNCTION__,"SS port is set as an input!\r\n",Warning);
    if((SPI_DIN & (1<<SS)) != (1<<SS))
    {
      SPCR = tmp[0]; SPSR = tmp[1];
      //logd(__FUNCTION__,"SS is set low and will set the SPIF!\r\n",Error);
      return 254;
    }
  }

  // enable spi, master mode,
  SPCR |= (1<<SPE) | (1<<MSTR);

  // check data buffer
  if(SPSR & (1<<SPIF))
  {
    tick = 255;
    while(tick--) if(SPSR & (1<<SPIF)) continue;
    //logd(__FUNCTION__,"SPI Transmit bufffer not empty!\r\n",Error);
    if(tick == 0) return 253;
  }

  return 0;
}


void spi_master_nSendByte(u08 data)
{
#if(SPI_INTERRUPT_ENABLE)
  while(!spi_TransferComplete);
  spi_TransferComplete = FALSE;
  outb(SPDR,data);
#else
  outb(SPDR, data);
#endif
}

u08 spi_master_SendByte(u08 data)
{
#if(SPI_INTERRUPT_ENABLE)
  while(!spi_TransferComplete);
  spi_TransferComplete = FALSE;
   outb(SPDR, data);
  while(!spi_TransferComplete);
#else	
  outb(SPDR, data);	
  while(!(SPSR & (1<<SPIF))) ;		
#endif
  return inb(SPDR);
}


#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)


void spi_cs_on(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_PORT, &GPIO_InitStructure);
	
  GPIO_SetBits(SPI_PORT, SS);
}

void spi_cs_off(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_PORT, &GPIO_InitStructure);

  GPIO_ResetBits(SPI_PORT, SS);
}

void spi_init(void)
{
	
  GPIO_InitTypeDef GPIO_InitStructure;	
	
  RCC_APB2PeriphClockCmd(SPI_APB_Periph | PORT_APB_Periph(SPI_PORT), ENABLE);
	
  // Configure SPI1 pins: SCK, MISO and MOSI
  GPIO_InitStructure.GPIO_Pin = SCK | MISO | MOSI;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(SPI_PORT, &GPIO_InitStructure);

  // Configure I/O for Flash Chip select
  /*  
  GPIO_InitStructure.GPIO_Pin = SS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(PORT_APB_Periph, &GPIO_InitStructure);
  */
	
  #if(SPI_INTERRUPT_ENABLE)
  // enable SPI interrupt
  #endif
}

int spi_master_setup(u08 div,u08 mode)
{
  SPI_InitTypeDef  SPI_InitStructure;
	
  // SPI configuration 
  switch(div)
  {
    case 2:   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;   break;
    case 4:   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;   break;
    case 8:   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;   break;
    case 16:  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;  break;
    case 32:  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;  break;
    case 64:  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  break;
    case 128: SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; break;
    default:  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; break;
  }

  switch(mode)
  {
    case 0: SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
						break;
		
    case 1: SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
						break;
		
    case 2: SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
						break;
    
    case 3: SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
						break;
		
    case 4: SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
						break;

    case 5: SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
						break;
						
    case 6: SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
						break;
						
    case 7: SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
            SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
            SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;
						break;
						
    default:  return -2;
  }
			
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;  
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;    
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_SPI, &SPI_InitStructure);	
	
  // Enable SPI
  SPI_Cmd(SPI_SPI, ENABLE);
	
  // check TXE, RXNE, ERR bit in SPI->SR
  if(SPI_I2S_GetFlagStatus(SPI_SPI, SPI_I2S_FLAG_TXE) == RESET)
    logd(__FUNCTION__,"SPI Transmit bufffer not empty!\r\n",Error);
	
  if(SPI_I2S_GetFlagStatus(SPI_SPI, SPI_I2S_FLAG_RXNE)== SET) 
   logd(__FUNCTION__,"SPI Recieve buffer has vailed data!\r\n",Warning);
	
  return 0;
}

void spi_master_nSendByte(u08 data)
{
  while(SPI_I2S_GetFlagStatus(SPI_SPI, SPI_I2S_FLAG_TXE) == RESET) ; 	
  SPI_I2S_SendData(SPI_SPI, data);
}

u08  spi_master_SendByte(u08 data)
{
  u16 tmp;
  tmp = (u16) data;	
	
  // wait for last transfer compelte
  while(SPI_I2S_GetFlagStatus(SPI_SPI, SPI_I2S_FLAG_TXE) == RESET) ;
	
  SPI_I2S_SendData(SPI_SPI, tmp);	
	
  //wait for SPI Recieve buffer has vailed data
  while(SPI_I2S_GetFlagStatus(SPI_SPI, SPI_I2S_FLAG_RXNE)== RESET) ;
	
  // read the Recieve buffer, RXNE is clear at the same time
  tmp =  SPI_I2S_ReceiveData(SPI_SPI);		
	  
  return (u08) (tmp & 0x00ff);
}

#endif

u08  spi_master_GetByte(void)
{
  return spi_master_SendByte(0xff);
}



