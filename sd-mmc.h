
#ifndef SD_MMC_H
#define SD_MMC_H

#include "cpu.h"
#include "spi.h"
#include "uart.h"


void SD_SendCommand(u08 cmd, u32 arg, u08 crc, u08 * buffer, u16 len);


#endif // #ifndef SD-MMC_H
