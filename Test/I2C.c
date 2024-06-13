#include "Lib.h"
void I2C_Config(void)
{
	/* enable clock */ 
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // I2C1
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	
	/* config i2c gpio PB6 PB7 : alternate function */
	GPIOB->CRL &= ~0xFF000000;
	GPIOB->CRL |= GPIO_CRL_CNF6 | GPIO_CRL_CNF7;
	GPIOB->CRL |= GPIO_CRL_MODE6  | GPIO_CRL_MODE7;
	
	/*   */
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~ I2C_CR1_SWRST;
	
	I2C1->CR2 = 36;//dat cau hinh 36MHz
	I2C1->OAR1 |= (1 << 14);
	I2C1->CCR = 180;
	I2C1->TRISE = 37;	

	I2C1->CR1 |= I2C_CR1_PE;
}

void I2C_Start(void)
{
		
	I2C1->CR1 |= I2C_CR1_ACK;
	
	I2C1->CR1 |= I2C_CR1_START;
	Delay_ms(1);
	while(!(I2C1->SR1 & I2C_SR1_SB));
}

void I2C_Write(uint8_t data)
{
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = data;
	while(!(I2C1->SR1) & I2C_SR1_BTF);
}

void I2C_Address(uint8_t address)
{
	uint8_t addr = (uint8_t)(address << 1);
	I2C1->DR = addr;
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	
	(void) I2C1->SR1;
	(void) I2C1->SR2;
}

void I2C_Stop(void)
{
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_WriteMul(uint8_t *data, uint8_t size)
{
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	while(size)
	{
		while(!(I2C1->SR1 & I2C_SR1_TXE));
		I2C1->DR = *data;
		data++;
		size--;
	}
	while(!(I2C1->SR1 & I2C_SR1_BTF));
}
void I2C_ReadData(uint8_t address, uint8_t reg, int16_t* buffer, uint8_t length) {
	// Bat dau truyen
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));

	// Gui dia chi
	I2C1->DR = (address << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2; 

	// Gui thanh ghi
	I2C1->DR = reg;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// Khoi dong lai I2c de chuyen sang che do doc
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));

	// Gui dia chi bit doc
	I2C1->DR = (address << 1) | 1;
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2; // Doc SR2 de xoa co

	// Doc du lieu
	while (length) {
			if (length == 1) {
					I2C1->CR1 &= ~I2C_CR1_ACK; // Tat ACK
					I2C1->CR1 |= I2C_CR1_STOP; // Tao tín hieu STOP
			}

			if (I2C1->SR1 & I2C_SR1_RXNE) {
					*buffer = I2C1->DR;
					buffer++;
					length--;
			}
	}

	// Bat lai ACK cho lan doc sau
	I2C1->CR1 |= I2C_CR1_ACK;
}
void I2C_WriteData(uint8_t address, uint8_t reg, uint8_t data) {
	// Bat dau truyen
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));

	// Gui dia chi
	I2C1->DR = (address << 1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void)I2C1->SR2; 

	// Gui thanh ghi
	I2C1->DR = reg;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// Gui du lieu
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// Dung truyen
	I2C1->CR1 |= I2C_CR1_STOP;
}