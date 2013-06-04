

#include "iic.h"


#define IIC_INTERRUPT_ENABLE 0

#if defined(__GNUC__) && defined(__GNUC_MINOR__)


#elif defined (STM32F10X_LD) || defined (STM32F10X_LD_VL) || \
      defined (STM32F10X_MD) || defined (STM32F10X_MD_VL) || \
      defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || \
      defined (STM32F10X_XL) || defined (STM32F10X_CL)

void iic_init(void)
{
	/*
  I2C_InitTypeDef I2C_Init;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_ClocksTypeDef RCC_ClockFreq;
	
	//I2C_Init.I2C_ClockSpeed = xI2C_EE;
	I2C_Init.I2C_Mode = I2C_Mode_I2C;	
	I2C_Init.I2C_DutyCycle = I2C_DutyCycle_2;
	//I2C_Init.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
	I2C_Init.I2C_Ack = I2C_Ack_Enable;
	I2C_Init.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	// IIC IO configuration
	GPIO_InitStructure.GPIO_Pin = SCL | SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	
	GPIO_Init(PORT_APB_Periph(IIC_PORT),&GPIO_InitStructure);
	
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	I2C_DeInit(I2C1);
	I2C_Cmd(I2C1, DISABLE);
	I2C_Structure.I2C_Mode = I2C_Mode_I2C;
	I2C_Structure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_Structure.I2C_OwnAddress1 = 0xA6;
	I2C_Structure.I2C_Ack = I2C_Ack_Enable;
	I2C_Structure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Structure.I2C_ClockSpeed = 400000;
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_Structure);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	
	//I2C_Init(IIC_IIC,&I2C_Init);
	*/
}

#endif

