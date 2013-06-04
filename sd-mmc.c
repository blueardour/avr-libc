
#include "sd-mmc.h"


void SD_SendCommand(u08 cmd, u32 arg, u08 crc, u08 * buffer, u16 len)
{
  u16 i;
  spi_cs_off();
  spi_master_SendByte(cmd);
  spi_master_SendByte(arg>>24);
  spi_master_SendByte(arg>>16);
  spi_master_SendByte(arg>>8);
  spi_master_SendByte(arg);
  spi_master_SendByte(crc);
  for(i=0;i<len;i++) buffer[i] = spi_master_SendByte(0xFF);
  spi_cs_on();
}
