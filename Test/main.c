#include "Lib.h"

void SystemClock_Config();
void SysTick_Init();
void LED_Init();
void Switch_Init();
void EXTI_Config();

#define MMA_ADDR 0x1C

int16_t buffer[6];
char buf[32]="";
volatile int16_t x,y,z;
volatile float a,ax,ay,az;
volatile bool fall = false;
// Trang thai cua he thong

volatile uint8_t systemState = 1;//1 chay
volatile uint8_t systemReset = 0;// ko reset
volatile uint32_t ticks = 0;

int main(void)
{
	SystemClock_Config();
	SysTick_Init();
	EXTI_Config();
	Delay_Config ();
	I2C_Config ();	
	lcd_init ();
	LED_Init();
	Switch_Init();
	
	lcd_clear();
	lcd_put_cur (0,0);
	lcd_send_string ("hello1");
	
	I2C_WriteData(MMA_ADDR,0x2B,0x40);//devide software reset
	Delay_ms(1000);
	I2C_WriteData(MMA_ADDR,0x2A,0x00);// devide stanby mode
	I2C_WriteData(MMA_ADDR,0x2D,0x01); //enable drdy interrupt
	I2C_WriteData(MMA_ADDR,0x2A,0x01); //enable device
	while (1)
	{
		
		Delay_ms(400);
		if (systemState)
		{
			
			I2C_ReadData(MMA_ADDR,0x01,buffer,6);
			x = (int16_t) (buffer[0]<<8 | buffer[1]);
			y = (int16_t) (buffer[2]<<8 | buffer[3]);
			z = (int16_t) (buffer[4]<<8 | buffer[5]);
			ax = (float) (x*x);
			ay = (float) (y*y);
			az = (float) (z*z);
			a= (float)sqrt(ax + ay + az) /16384 *9.8;
			if( a <=7) {
				fall = true;
			}
			ax = ((float) x) /16384 *9.81;
			ay = ((float) y)/16384 *9.81;
			az= ((float)z)/16384 *9.81;
			sprintf(buf,"Fall %d",fall);
			lcd_put_cur(0,0);
			lcd_send_string(buf);
			
			
			
		}
		if (systemReset)
		{
			systemState = 1;
			systemReset = 0;
			fall = false;
		}
	}
}

void SystemClock_Config() {
	// Kich hoat HSE (High-Speed External) oscillator
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY)){}; // Doi cho HSE san sang, HSERDY Flag set len 1

  // Cau hinh va kich hoat PLL
  RCC->CFGR |= RCC_CFGR_PLLSRC; // Chon HSE lam nguon PLL
  RCC->CFGR |= RCC_CFGR_PLLMULL9; // PLL x 9 de dat 72 MHz

  RCC->CR |= RCC_CR_PLLON; // Kich hoat PLL
  while (!(RCC->CR & RCC_CR_PLLRDY)); // Doi cho PLL san sang

  // Cau hinh cac bo chia AHB, APB1 v� APB2
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB = SYSCLK khong chia
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = HCLK chia 2
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK kh�ng chia

  // Chuyen he thong Clock nguon sang PLL
  FLASH->ACR |= FLASH_ACR_LATENCY_2; // Cau hinh Flash latency 2 wait states
  RCC->CFGR |= RCC_CFGR_SW_PLL; // Chon PLL lam SYSCLK
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Doi cho PLL duoc su dung lam SYSCLK
}
void SysTick_Init() {
    SysTick->CTRL = 0;
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}
void LED_Init() {
  // Bat clock cho GPIOB
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
  // Cau h�nh PB0 va PB1 l� output push-pull, toc do 2MHz
  GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1); 
  GPIOB->CRL |=  (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1); // 2 MHz
  GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1); // Push-pull
}
void Switch_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// Cau hinh PA0 PA1 la input pull up
  GPIOA->CRL &= ~0x0FF; // Reset
  GPIOA->CRL |= GPIO_CRL_CNF0_1; // Input pull up
  GPIOA->CRL |= GPIO_CRL_CNF1_1; // Input pull up
	GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1; 
}
void EXTI_Config()
{
  // Kich hoat clock cho AFIO de cau h�nh EXTI
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // C�u h�nh External Interrupt cho PA0 (EXTI0)
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0; // X�a c�c thiet lap cu
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Ch�n PA0 l�m ngu�n ngat cho EXTI0
	EXTI->IMR |= EXTI_IMR_MR0; // K�ch ho?t mask cho d�ng ng?t 0
	EXTI->FTSR |= EXTI_FTSR_TR0; // C?u h�nh ng?t theo c?nh xu?ng
	NVIC_SetPriority(EXTI0_IRQn,0);
	NVIC_EnableIRQ(EXTI0_IRQn); // K�ch ho?t ng?t trong NVIC

	// C?u h�nh External Interrupt cho PA1 (EXTI1)
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI1; // X�a c�c thi?t l?p cu
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // Ch?n PA1 l�m ngu?n ng?t cho EXTI1
	EXTI->IMR |= EXTI_IMR_MR1; // K�ch ho?t mask cho d�ng ng?t 1
	EXTI->FTSR |= EXTI_FTSR_TR1; // C?u h�nh ng?t theo c?nh xu?ng
	NVIC_SetPriority(EXTI1_IRQn,1);
	NVIC_EnableIRQ(EXTI1_IRQn); // K�ch ho?t ng?t trong NVIC
}
void SysTick_Handler(void) {
    static uint32_t ticksB0 = 0;
    static uint32_t ticksB1 = 0;

    ticks++;

    
    if ((++ticksB0 >= 1000) & (systemState == 1)) 
		{ 
			ticksB0 = 0;
      GPIOB->ODR ^= GPIO_ODR_ODR0; 
			
    } else if (systemState ==0) {
			GPIOB->ODR &= ~(1<<0);
		}
  	if ((++ticksB1 >= 500) & (fall)) 
		{ 
			ticksB1 = 0;
			GPIOB->ODR ^= GPIO_ODR_ODR1; 
		}
		else if (!fall) {
			GPIOB->ODR &= ~(1<<1);
		}	
		

    
    
}
void EXTI1_IRQHandler(void)//reset
{
	EXTI->PR |= EXTI_PR_PR1;
	systemReset = 1;
}
void EXTI0_IRQHandler(void)//switch state
{
	EXTI->PR |= EXTI_PR_PR0;
	systemState = !systemState;
}