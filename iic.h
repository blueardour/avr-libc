


#ifndef _IIC_H_

#include "cpu.h"


void iic_init(void);

#if defined(__GNUC__) && defined(__GNUC_MINOR__)

#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)


#define IIC_IIC  I2C1
#define IIC_PORT GPIOB
#define SDA PB7
#define SCL PB6
		

#endif


#endif


