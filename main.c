#include <stdint.h>
#include"stm32f4xx.h"

// MISO -> PA6
//MOSI -> PA7
//CLK ->PA5
//CS  ->PA9
//AF05

#define SPI1EN (1U<<12)
#define GPIOAEN (1U<<0)
#define  SR_TXE (1U<<1)
#define  SR_BUSY (1U<<7)
#define  SR_RXNE (1U<<0)
#define  MULTI_BYTE_EN 0X40

#define  READ_OPERATION 0X80


#define DEVID_R         (0X00)
#define DEVICE_ADDR     (0x53)
#define DATA_FORMAT_R   (0X31)
#define POWER_CNTRL_R   (0X2D)
#define DATA_START_ADDR (0X32)
#define DATA_FORMAR_R   (0X31)

#define FOUR_G          (0X01)
#define RESET           (0X00)
#define SET_MEASURE_B   (0X08)


void adxl_init(void);
void adxl_read_values(uint8_t reg);
void adxl_write(uint8_t reg,char value);
void adxl_read_address(uint8_t reg);
void spi_transmit(uint8_t *data ,uint32_t size);
void spi_receive(uint8_t *data ,uint32_t size);

void spi_gpio_init(void);
void spi_config(void);
void cs_enable(void);
void cs_disable(void);



int16_t x,y,z;
float xg,yg,zg;
//extern uint8_t data_rec[6];
char data;
uint8_t data_rec[6];

int main(void)
{
	adxl_init();
    /* Loop forever */
	while(1)
	{
		adxl_read(DATA_START_ADDR,data_rec);
		x=((data_rec[1]<<8)|data_rec[0]);
		y=((data_rec[3]<<8)|data_rec[2]);
		z=((data_rec[5]<<8)|data_rec[4]);

  xg=(x*0.0078);
  yg=(y*0.0078);
  zg=(z*0.0078);

	}
}



void adxl_read(uint8_t address, uint8_t *rxdata)
{

			 //set read operation
 address|=READ_OPERATION;
//ENABLE MULTI BYTE
 address|= MULTI_BYTE_EN;
 //pull cs line low to enable slave
 cs_enable();
 // Transmit data and  addrss
 spi_transmit(&address,1);

 spi_receive(rxdata,6);
 //pull sc line high to disbale
 cs_disable();


}

void adxl_write(uint8_t address,char value)
{
uint8_t data[2];
//ENABLE MULTI BYTE ,PLACE ADDRESS INTO BUFFER


data[0] =address|MULTI_BYTE_EN;
data[1] =value;

//pull cs line low to enable slave
cs_enable();
// Transmit data and  addrss
spi_transmit(data,2);
//pull sc line high to disbale
cs_disable();


// PLACE DATA INTO BUFFER




}

void adxl_read_values(uint8_t reg)
{



}

void adxl_init(void)
{
	// enable i2c init
	 spi_gpio_init();
	 spi_config();




	//set data format range
	adxl_write(DATA_FORMAT_R,FOUR_G);
	//reset all bits
	adxl_write(POWER_CNTRL_R,RESET);
	//config power control MEASURE	bits//
	adxl_write(POWER_CNTRL_R,SET_MEASURE_B);

}

void spi_gpio_init(void)
{
RCC->AHB1ENR |=GPIOAEN;

//PA5 * ALETERANTE MODE/
GPIOA->MODER &= ~(1U<<10); //SET 0
GPIOA->MODER |= (1U<<11); //SET 1

//PA6 * ALETERANTE MODE/
GPIOA->MODER &= ~(1U<<12); //SET 0
GPIOA->MODER |= (1U<<13); //SET 1

//PA7 * ALETERANTE MODE/
GPIOA->MODER &= ~(1U<<14); //SET 0
GPIOA->MODER |= (1U<<15); //SET 1

//PA9 AS OUTPUT PIN/
GPIOA->MODER |= (1U<<18); //SET 0
GPIOA->MODER &= ~(1U<<19); //SET 1

//SET  PA5  0101 -AF5
GPIOA->AFR[0] |=(1U<<20); //SET 1
GPIOA->AFR[0] &=~(1U<<21);//SET 0
GPIOA->AFR[0] |=(1U<<22);
GPIOA->AFR[0] &=~(1U<<23);
//PA6
GPIOA->AFR[0] |=(1U<<24);
GPIOA->AFR[0] &=~(1U<<25);
GPIOA->AFR[0] |=(1U<<26);
GPIOA->AFR[0] &=~(1U<<27);
//PA7
GPIOA->AFR[0] |=(1U<<28);
GPIOA->AFR[0] &=~(1U<<29);
GPIOA->AFR[0] |=(1U<<30);
GPIOA->AFR[0] &=~(1U<<31);

}

void spi_config(void)
{
	//ENABLE THE CLOCK ACCESS TO SPI1 MODULE
	RCC->APB2ENR |= SPI1EN;

	//SET CLOCK FPLCK/4
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	//SET CPOL TO 1 CPHA TO 1


	SPI1->CR1 |= (1U<<0);
	SPI1->CR1 |= (1U<<1);
//ENABLE FULL DUPLEX

	SPI1->CR1 &= ~(1U<<10);
	// SET MSB FIRST

	SPI1->CR1 &= ~(1U<<7);

	//SET MODE TO MASTER
	SPI1->CR1 |= (1U<<2);

	//SET 8 BIT DATA M0DE
	SPI1->CR1 &= ~ (1U<<11);

// select software slave management SSM=1 SSI=1
	SPI1->CR1 |= (1U<<8);
	SPI1->CR1 |= (1U<<9);

	//SPI ENABLE

	SPI1->CR1 |= (1U<<6);




}

void spi_transmit(uint8_t *data ,uint32_t size)
{
	uint32_t  i=0;
	uint8_t temp;
	while(i<size)
	{
		/*wait untill TXE IS SET/
		 *
		 */
		while(!(SPI1->SR & (SR_TXE))){}
		/* WRITE THE DAAT TO TEH DATA REGISTER
		 *
		 */
		SPI1->DR=data[i];
		i++;
	}
	// WAIT UNTILL TXE IS SET
	while(!(SPI1->SR & (SR_TXE))){}

	//WAIT UNTILL BUSY FLAG IS SET
	while(!(SPI1->SR & (SR_BUSY))){}

	//CLEAR OVR FLAG
	temp =SPI1->DR;
	temp =SPI1->SR;


}

void spi_receive(uint8_t *data ,uint32_t size)
{
while(size)
{
	// send dummmy data
	SPI1->DR=0;


///WAIT FOR RXNE TO SET
while(!(SPI1->SR & (SR_RXNE))){}
// READ THE DATA FROM DATA REG
*data++ =(SPI1->DR);
size--;
}

}

void cs_enable(void)
{
	GPIOA->ODR &= ~(1U<<9);

}

void cs_disable(void)
{
	GPIOA->ODR |= (1U<<9);

}
