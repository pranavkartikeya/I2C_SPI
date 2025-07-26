

#include <stdint.h>
#include "stm32f4xx.h"

#define DEVID_R (0X00)
#define DATA_FORMAT_R (0x31)
#define POWER_CTL_R (0X2D)
#define DATA_START_ADDR (0X32)
#define DEVICE_ADDR   (0X53)

#define FOUR_G (0X01)
#define RESET (0X00)
#define SET_MEASURE_B (0X08)


#define GPIOBEN (1U<<1)
#define I2C1EN (1U<<21)
#define I2C_100KHZ 80
#define SD_MODE_MAX_RISE_TIME 17


#define SR2_BUSY (1U<<1)
#define SR1_SB (1U<<0)
#define SR1_ADDR (1U<<1)
#define SR1_TXE (1U<<7)
#define SR1_RXNE (1U<<6)
#define SR1_BTF   (1U<<2)

#define CR1_START (1U<<8)
#define CR1_PE (1U<<0)
#define CR1_ACK (1U<<10)
#define CR1_STOP (1U<<9)

void I2C1_Init(void);
void I2C1_ByteRead(char saddr,char maddr,char* data);
void I2C1_BurstRead(char saddr,char maddr,int n,char* data);
void I2C1_BurstWrite(char saddr,char maddr,int n,char* data);
void adxl_read_values(uint8_t reg);
void adxl_write(uint8_t reg,char value);
void adxl_read_address(uint8_t reg);
void adxl_init(void);

char data;
uint8_t data_rec[6];
int16_t x,y,z;
float xg,yg,zg;

//PB8 -SCL
//PB7-SDA

int main(void)
{
	 adxl_init();
	 while(1)
	 {
		 adxl_read_values(DATA_START_ADDR);
		 x=((data_rec[1]<<8)|data_rec[0]);
		 y=((data_rec[3]<<8)|data_rec[2]);
		 z=((data_rec[5]<<8)|data_rec[4]);
		 		 xg=(x*0.0078);
		 		 yg=(y*0.0078);
		 		 zg=(y*0.0078);


	 }


}

void adxl_init(void)
{
	I2C1_Init();
	adxl_read_address(DEVID_R);
	adxl_write(DATA_FORMAT_R,FOUR_G);
	//REST ALL THE BITS
	adxl_write(POWER_CTL_R,RESET);

	adxl_write(POWER_CTL_R,SET_MEASURE_B);





}
void adxl_read_values(uint8_t reg)
{
	 I2C1_BurstRead(DEVICE_ADDR,reg,6,(char*)data_rec);
}
void adxl_write(uint8_t reg,char value)
{
	char data[1];
	data[0]=value;
	I2C1_BurstWrite(DEVICE_ADDR,reg,1,&data);

}

void adxl_read_address(uint8_t reg)

{
	I2C1_ByteRead(DEVICE_ADDR,reg ,&data);

}

void I2C1_Init(void)
{
	//Enable the clock access to PORT B -I2C1
	RCC->AHB1ENR |=GPIOBEN;

	//SET PB7 AND PB8 to alternate function
	GPIOB->MODER &=~(1U<<16);
	GPIOB->MODER |= (1U<<17);

	GPIOB->MODER &=~(1U<<18);
	GPIOB->MODER |= (1U<<19);

//SET OUTTYPE AS OPEN DRAIN
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);

	//SET PULLUP FOR BOTH
	GPIOB->PUPDR |=(1U<<16);
	GPIOB->PUPDR &=~(1U<<17);

	GPIOB->PUPDR |=(1U<<18);
	GPIOB->PUPDR &=~(1U<<19);



	//SET THE ALTERNATE FUNCTION TYPE FOR I2C
	GPIOB->AFR[1] &=~(1U<<0);
	GPIOB->AFR[1] &=~(1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &=~(1U<<3);

	GPIOB->AFR[1] &=~(1U<<4);
	GPIOB->AFR[1] &=~(1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &=~(1U<<7);

	//
	RCC->APB1ENR |=I2C1EN;
	//ENTER TO RESET MODE
	I2C1->CR1 |=(1U<<15);
	//COME OUR OF THE RESET MODE
	I2C1->CR1 &=~(1U<<15);
		//SET THE PERIPHERAL CLOCK FREQUENXT
	I2C1->CR2 =(1U<<4);

	I2C1->CCR = I2C_100KHZ;
	I2C1->TRISE =SD_MODE_MAX_RISE_TIME;

	I2C1->CR1|=CR1_PE;




}
//saddr-slave address
//maddr -memory addres
//dataa pointer
void I2C1_ByteRead(char saddr,char maddr,char* data)
{
	volatile int tmp;
	//Wait untill bus is not busy
	while(I2C1->SR2 & (SR2_BUSY)){}
	//Generate Start Condition
	I2C1->CR1 |=CR1_START;
	//Wait untill the start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){}

	//Transmit Slave address +write

	I2C1->DR=saddr<<1;

	//Wait untill teh addr flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	//Clear addr flag
	tmp=I2C1->SR2;

	//Send memory address
	I2C1->DR=maddr;

	//Wait untill teh transmitter is empty
	while(!(I2C1->SR1 & (SR1_TXE))){}

	//Generatr resart
	I2C1->CR1 |=CR1_START;

	//Wait untill start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){}


	//Transmit slave address+read
	I2C1->DR=saddr<<1 |1;

	//Wait untill the address flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){}

	//Disable Acknoweledgemnt
	I2C1->CR1 &= ~CR1_ACK;
	//cLEAR THE ADDR FLAG
	tmp=I2C1->SR2;

	//Generate stop after data received
	I2C1->CR1 |= CR1_STOP;


	//wait utill rxne flag is set
	while(!(I2C1->SR1 & (SR1_RXNE))){}

	//Read data from Data register

    *data++ =I2C1->DR;
}

void I2C1_BurstRead(char saddr,char maddr,int n,char* data)
{

	volatile int tmp;
		//Wait untill bus is not busy
		while(I2C1->SR2 & (SR2_BUSY)){}
		//Generate Start Condition
		I2C1->CR1 |=CR1_START;
		//Wait untill the start flag is set
		while(!(I2C1->SR1 & (SR1_SB))){}

		//Transmit Slave address +write

		I2C1->DR=saddr<<1;

		//Wait untill teh addr flag is set
		while(!(I2C1->SR1 & (SR1_ADDR))){}

		//Clear addr flag
		tmp=I2C1->SR2;



		//Wait untill teh transmitter is empty
		while(!(I2C1->SR1 & (SR1_TXE))){}
		//Send memory address
				I2C1->DR=maddr;
	    while(!(I2C1->SR1 & (SR1_TXE))){}

		//Generatr resart
		I2C1->CR1 |=CR1_START;

		//Wait untill start flag is set
		while(!(I2C1->SR1 & (SR1_SB))){}


		//Transmit slave address+read
		I2C1->DR=saddr<<1 |1;

		//Wait untill the address flag is set
		while(!(I2C1->SR1 & (SR1_ADDR))){}
		//cLEAR THE ADDR FLAG
		tmp=I2C1->SR2;

		//enable acknoweledgement
		I2C1->CR1|=CR1_ACK;
		while(n> 0U)
		{
			if (n==1U)
			{
				I2C1->CR1 &=~CR1_ACK;
				//Generate stop after data received
					I2C1->CR1 |= CR1_STOP;


					//wait utill rxne flag is set
					while(!(I2C1->SR1 & (SR1_RXNE))){}

					//Read data from Data register

				    *data++ =I2C1->DR;

                 break;

			}
			else
			{while(!(I2C1->SR1 & (SR1_RXNE))){}

			//Read data from Data register

		    (*data++) =I2C1->DR;
		    n--;

			}
		}

}

void I2C1_BurstWrite(char saddr,char maddr,int n,char* data)
{
	volatile int tmp;
			//Wait untill bus is not busy
			while(I2C1->SR2 & (SR2_BUSY)){}
			//Generate Start Condition
			I2C1->CR1 |=CR1_START;
			//Wait untill the start flag is set
			while(!(I2C1->SR1 & (SR1_SB))){}

			//Transmit Slave address +write

			I2C1->DR=saddr<<1;

			//Wait untill teh addr flag is set
			while(!(I2C1->SR1 & (SR1_ADDR))){}

			//Clear addr flag
			tmp=I2C1->SR2;


			//Wait untill teh transmitter is empty
			while(!(I2C1->SR1 & (SR1_TXE))){}

			//Send memory address
			I2C1->DR=maddr;

			for(int i=0; i<n ;i++)
			{
				//Wait untill teh transmitter is empty
						while(!(I2C1->SR1 & (SR1_TXE))){}
						I2C1->DR =*data++;
			}

			while(!(I2C1->SR1 & (SR1_BTF))){}
			//Generate stop after data received
								I2C1->CR1 |= CR1_STOP;
}
