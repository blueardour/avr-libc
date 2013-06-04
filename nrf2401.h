

#ifndef _NRF2401_H_
#define _NRF2401_H_

#include "cpu.h"

#include "uart.h"
#include "spi.h"
#include "print.h"

//****************************************************************//
// (nRF24L01) commands
#define NRF24L01_READ_REG        0x00  // Define read command to register
#define NRF24L01_WRITE_REG       0x20  // Define write command to register
#define NRF24L01_RD_RX_PLOAD     0x61  // Define RX payload register address
#define NRF24L01_WR_TX_PLOAD     0xA0  // Define TX payload register address
#define NRF24L01_FLUSH_TX        0xE1  // Define flush TX register command
#define NRF24L01_FLUSH_RX        0xE2  // Define flush RX register command
#define NRF24L01_REUSE_TX_PL     0xE3  // Define reuse TX payload register command
#define NRF24L01_NOP             0xFF  // Define No Operation, might be used to read status register

// (nRF24L01) registers(addresses)
#define NRF24L01_CONFIG          0x00  // 'Config' register address
#define NRF24L01_EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define NRF24L01_EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define NRF24L01_SETUP_AW        0x03  // 'Setup address width' register address
#define NRF24L01_SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define NRF24L01_RF_CH           0x05  // 'RF channel' register address
#define NRF24L01_RF_SETUP        0x06  // 'RF setup' register address
#define NRF24L01_STATUS          0x07  // 'Status' register address
#define NRF24L01_OBSERVE_TX      0x08  // 'Observe TX' register address
#define NRF24L01_CD              0x09  // 'Carrier Detect' register address
#define NRF24L01_RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define NRF24L01_RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define NRF24L01_RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define NRF24L01_RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define NRF24L01_RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define NRF24L01_RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define NRF24L01_TX_ADDR         0x10  // 'TX address' register address
#define NRF24L01_RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define NRF24L01_RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define NRF24L01_RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define NRF24L01_RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define NRF24L01_RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define NRF24L01_RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define NRF24L01_FIFO_STATUS     0x17  // 'FIFO Status Register' register address

#define NRF24L01_TX_ADR_WIDTH    5
#define NRF24L01_RX_ADR_WIDTH    5

#define NRF24L01_RETR(x,y)       (((x)/250-1)<< 4)|(y)

#define NRF24L01_MASK_RX_DR       0x40
#define NRF24L01_MSAK_TX_DS       0x20
#define NRF24L01_MSAK_MAX_RT      0x10

#define NRF24L01_RX_P_NO          0x0e
#define NRF24L01_TX_FULL          0x01
//#define NRF24L01_MSAK_MAX_RT      0x10


int   nrf_init(void);
void  nrf_Test(void);
void  led_Test(void);


u8 nrf_WriteTxPload(u8 * data, int len);
u8 nrf_ReadRxPload(u8 * data, int len);
u8 nrf_FlushRx(void);
u8 nrf_FlushTx(void);
u8 nrf_ReadBuffer(u8 addr, u8 * data, int len);
u8 nrf_ReadReg(u8 addr, u8 * data);
u8 nrf_WriteBuffer(u8 addr, u8 * data, int len);
u8 nrf_WriteReg(u8 addr, u8 data);
u8 nfr_GetStatus(void);

#endif

