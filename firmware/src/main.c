#include <stm32f0xx_conf.h>
#include <stm32f0xx.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_i2c.h>
#include <stm32f0xx_spi.h>

//SHT31-D  declarations
#define SHD_SDA GPIO_Pin_11
#define SHD_SCL GPIO_Pin_10
#define SHD_GPIO GPIOB

#define SHD_SDA_PS GPIO_PinSource11
#define SHD_SCL_PS GPIO_PinSource10
#define SHD_PIN_AF GPIO_AF_1
#define SHD_I2C I2C2




//SHD-31-D I2C address - If Error Shift Address left by one due to libary fault
#define SHDAddr (0x44<<1)

//SHD-31  internal registers
//#define R_Config1 0x--
//#define R_Config2 0x--
//#define R_Mode 0x--
//#define R_Status 0x--
//#define R_XRegister 0x--

//Time keeping variable for later
volatile uint32_t MSec;

int SHD_Data[10];

GPIO_InitTypeDef GP;
I2C_InitTypeDef IT;


// add I2C stuff here

void I2C_WrReg(uint8_t Reg, uint8_t Val)
{
	//Wait until I2C isn't busy
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_BUSY) == SET);
	//and a lot of the code is from the Std peripheral library
	I2C_TransferHandling(SHD_I2C, SHDAddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	//Ensure the transmit interrupted flag is set
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TXIS) == RESET);
	//Send the address of the register we wish to write to
	I2C_SendData(SHD_I2C, Reg);
	//Ensure that the transfer complete reload flag is
	//set, essentially a standard TC flag
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TCR) == RESET);
	//Now that the SHD5883L knows which register
	//we want to write to, send the address again
	//and ensure the I2C peripheral doesn't add
	//any start or stop conditions
	I2C_TransferHandling(SHD_I2C, SHDAddr, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

	//Again, wait until the transmit interrupted flag is set
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TXIS) == RESET);

	//Send the value you wish you write to the register
	I2C_SendData(SHD_I2C, Val);

	//Wait for the stop flag to be set indicating
	//a stop condition has been sent
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_STOPF) == RESET);

	//Clear the stop flag for the next potential transfer
	I2C_ClearFlag(SHD_I2C, I2C_FLAG_STOPF);
	
}

uint8_t I2C_SHDRd(int8_t *Data, uint8_t DCnt)
{
	
	int8_t Cnt, SingleData = 0;
	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_BUSY) == SET);
	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the SHD device.
//	I2C_TransferHandling(SHD_I2C, SHDAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	//Wait until the transmit interrupt status is set
//	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TXIS) == RESET);
	//Send the address of the register you wish to read
//	I2C_SendData(SHD_I2C, (uint8_t)Reg);
	//Wait until transfer is complete!
//	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TC) == RESET);
	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(SHD_I2C, SHDAddr, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	//Read in DCnt pieces of data
	for(Cnt = 0; Cnt<DCnt; Cnt++)
	{
		//Wait until the RX register is full of luscious data!
		while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_RXNE) == RESET); 
		//If we're only reading one byte, place that data direct into the 
		//SingleData variable. If we're reading more than 1 piece of data 
		//store in the array "Data" (a pointer from main) 		
		if(DCnt > 1) Data[Cnt] = I2C_ReceiveData(SHD_I2C);
		else SingleData = I2C_ReceiveData(SHD_I2C);
	}

	//Wait for the stop condition to be sent
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_STOPF) == RESET);
	//Clear the stop flag for next transfers
	I2C_ClearFlag(SHD_I2C, I2C_FLAG_STOPF);
	//Return a single piece of data if DCnt was
	//less than 1, otherwise 0 will be returned.
	return SingleData;
	
}


uint8_t I2C_RdReg(int8_t Reg, int8_t *Data, uint8_t DCnt)
{
	
	int8_t Cnt, SingleData = 0;
	//As per, ensure the I2C peripheral isn't busy!
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_BUSY) == SET);
	//Again, start another tranfer using the "transfer handling"
	//function, the end bit being set in software this time
	//round, generate a start condition and indicate you will
	//be writing data to the SHD device.
	I2C_TransferHandling(SHD_I2C, SHDAddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	//Wait until the transmit interrupt status is set
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TXIS) == RESET);
	//Send the address of the register you wish to read
	I2C_SendData(SHD_I2C, (uint8_t)Reg);
	//Wait until transfer is complete!
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_TC) == RESET);
	//As per, start another transfer, we want to read DCnt
	//amount of bytes. Generate a start condition and
	//indicate that we want to read.
	I2C_TransferHandling(SHD_I2C, SHDAddr, DCnt, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	//Read in DCnt pieces of data
	for(Cnt = 0; Cnt<DCnt; Cnt++)
	{
		//Wait until the RX register is full of luscious data!
		while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_RXNE) == RESET); 
		//If we're only reading one byte, place that data direct into the 
		//SingleData variable. If we're reading more than 1 piece of data 
		//store in the array "Data" (a pointer from main) 		
		if(DCnt > 1) Data[Cnt] = I2C_ReceiveData(SHD_I2C);
		else SingleData = I2C_ReceiveData(SHD_I2C);
	}

	//Wait for the stop condition to be sent
	while(I2C_GetFlagStatus(SHD_I2C, I2C_FLAG_STOPF) == RESET);
	//Clear the stop flag for next transfers
	I2C_ClearFlag(SHD_I2C, I2C_FLAG_STOPF);
	//Return a single piece of data if DCnt was
	//less than 1, otherwise 0 will be returned.
	return SingleData;
}

void SPI_SendData(uint8_t adress, uint8_t data){
	 
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	 
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
	SPI_SendData8(SPI1, adress);
//	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
//	SPI_ReceiveData8(SPI1);
//	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE)); 
//	SPI_SendData8(SPI1, data);
//	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));
//	SPI_ReceiveData8(SPI1);
	 
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
 
//Convert an array to an unsigned byte/word/dword
//Type = 0 for u8
//Type = 1 for u16
//Type = 2 for u32
uint32_t AToU(uint8_t *D, uint8_t Type){
	uint32_t V = 0;
	uint8_t Cnt = 0;
	for(Cnt = 0; Cnt<(8<<Type); Cnt++)
	{
		V |= ((D[(8<<Type)-Cnt]&1)<<Cnt);
	}
	return V;
}

//Standard systick interrupt handler incrementing a variable named
//MSec (Milliseconds)
void SysTick_Handler(void)
{	
	MSec++;
		
	static uint16_t tick = 0;

	switch (tick++)
	{
  	case 500:
  		tick = 0;
  		GPIOA->ODR ^= (1 << 10);
//		I2C_WrReg(0xF3,0x2D);
//		Delay(1);
		 uint8_t DCnt = 4;	
//		I2C_RdReg(int8_t Reg, int8_t *Data, uint8_t DCnt) DCnt in bytes
//		I2C_SHDRd(SHD_Data,DCnt);
	
		break;
	}
	
}

void SPI1_Init(void)
{
	 
	 
	//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	 
	SPI_InitTypeDef SPI_InitTypeDefStruct;
 	 
	SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	 
	SPI_Init(SPI1, &SPI_InitTypeDefStruct);
	 
	//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB , ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitTypeDefStruct;
	 
	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);
	 
	//Setup nSS
	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitTypeDefStruct);
	//SEE TABLE 13. P.34 RM0360 for AF Table
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);
	//Chip Select Pin nSS PB12 
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	 
	 
	SPI_Cmd(SPI1, ENABLE);
	 
}

//Start I2C2 Bus All Clocks Must Be Started Before Calling
void I2C2_Init()
{
	//Configure the pins to the I2C AF
	GPIO_PinAFConfig(SHD_GPIO, SHD_SDA_PS, SHD_PIN_AF);
	GPIO_PinAFConfig(SHD_GPIO, SHD_SCL_PS, SHD_PIN_AF);

	//Set the pins SHD_SDA and SHD_SCL as alternate function GPIO pins
	GP.GPIO_Pin = SHD_SDA | SHD_SCL;
	GP.GPIO_Mode = GPIO_Mode_AF;
	GP.GPIO_OType = GPIO_OType_OD;
	GP.GPIO_Speed = GPIO_Speed_Level_1;
//	GP.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SHD_GPIO, &GP);
	//Setup the I2C struct. The timing variable is acquired
	//from the STM32F0 I2C timing calculator sheet. Pretty
	//standard stuff really, its using the Analog filter
	//to clean up any noisy edges (not really required though
	//if you wish to disable it, you will need a different
	//I2C_Timing value).
	IT.I2C_Ack = I2C_Ack_Enable;
	//find code for Nack 
	IT.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	IT.I2C_DigitalFilter = 0;
	IT.I2C_Mode = I2C_Mode_I2C;
	IT.I2C_OwnAddress1 = 0x00;
	IT.I2C_Timing = 0x20302E37; //32Mhz Clock
//	IT.I2C_Timing = 0x00201D2B;
	IT.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(SHD_I2C, &IT);
	I2C_Cmd(I2C2, ENABLE);

}

//Delay function: All it does is operate a nop instruction
//until "Time" amount of milliseconds has passed.
void Delay(uint32_t Time)
{
	volatile uint32_t MSStart = MSec;
	while((MSec-MSStart)<Time) asm volatile("nop");

}


int main(void)
{
	SysTick_Config(SystemCoreClock/1000);

	//Alternate Mask 
	//RCC->CR |= (uint32_t)0x00000001;
	//Enable GPIOB clock, required for the I2C output
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
	Delay(10);	
	//Enable the I2C peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* Reset I2Cx IP */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);

	Delay(10);
	/* Release reset signal of I2Cx IP */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);

	//Enable GPIOA clock, required for LED 
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 

	//SPI 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	//Set mode Register for GPIOA
	GPIOA->MODER = (1 << 20); //GPIO set mode register 

	I2C2_Init();
 	SPI1_Init();
	//GPIOB_AFRL Set B Pin 0-7
//	GPIOB->AFR[0] = (0x0);
	//GPIOB_AFRH Set B Pin 8-15
//	GPIOB->AFR[1] = (0x1100); 

	//SET GPIOB MODE MASK AFTER SETTING AF REGTISTERS TO AVOID UNWANTED PIN TRANSITIONS
//	GPIOB->MODER = (0x42A50A80);

//	I2C_RdReg(0,0,0);

	//PSUDO CODE
	//Read I2C Status register 
 	//Wait for Data Ready Flag
		//Read data Xbytes long and place in storage buffer
		//Place Data by type in seperate buffer. Read out in debugger
		//Pray
	
	while(1)
	{

	SPI_SendData(0x04,0x00);
	Delay(500);	
	}	
}
