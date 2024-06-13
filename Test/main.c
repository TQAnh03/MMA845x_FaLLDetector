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
	lcd_put_cur (1,0);
	lcd_send_string ("hello2");
	I2C_WriteData(MMA_ADDR,0x2B,0x40);//devide software reset
	Delay_ms(1000);// wait to reset
	I2C_WriteData(MMA_ADDR,0x0E,0x00); // scale +/-2g -> 1g = 16384/4 = 4096 count
	I2C_WriteData(MMA_ADDR,0x2B,0x02); // high resolution mode
	I2C_WriteData(MMA_ADDR,0x15,0x38); // freefall flags va freefall detection for x y z
	I2C_WriteData(MMA_ADDR,0x17,0x04);// Threshold Setting Value for the Freefall detection of  0.2g (4 * 0.063)
	I2C_WriteData(MMA_ADDR,0x18,0x20);// Set the debounce counter to 80 ms timer 
	
//	I2C_WriteData(MMA_ADDR,0x2A,0x00);// devide stanby mode
//	I2C_WriteData(MMA_ADDR,0x2D,0x01); //enable drdy interrupt
	I2C_WriteData(MMA_ADDR,0x2D,0x04); // Enable Motion/Freefall Interrupt
	I2C_WriteData(MMA_ADDR,0x2E,0x04);// Freefall interrupt routed to INT1
	I2C_WriteData(MMA_ADDR,0x2A,0x19); //enable device / chuyen sang active mode odr 100hz
	while (1)
	{
		
		Delay_ms(500);
		if (systemState)
		{
//			I2C_ReadData(MMA_ADDR,0x01,buffer,6);
//			x = (int16_t) (buffer[0]<<8 | buffer[1]);
//			y = (int16_t) (buffer[2]<<8 | buffer[3]);
//			z = (int16_t) (buffer[4]<<8 | buffer[5]);
//			ax = (float) (x*x);
//			ay = (float) (y*y);
//			az = (float) (z*z);
//			a= (float)sqrt(ax + ay + az) /16384;
//			if( a >=2 ) {
//				fall = true;
//			}
			lcd_clear();
			sprintf(buf,"Fall = %d",fall);
			
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

  // Cau hinh cac bo chia AHB, APB1 và APB2
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // AHB = SYSCLK khong chia
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 = HCLK chia 2
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 = HCLK không chia

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
	
  // Cau hình PB0 va PB1 là output push-pull, toc do 2MHz
  GPIOB->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1); 
  GPIOB->CRL |=  (GPIO_CRL_MODE0_1 | GPIO_CRL_MODE1_1); // 2 MHz
  GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1); // Push-pull
}
void Switch_Init() {
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// Cau hinh PA0 PA1  la input pull up 
  GPIOA->CRL &= ~0x0FF; // Reset
  GPIOA->CRL |= GPIO_CRL_CNF0_1; // Input pull up
  GPIOA->CRL |= GPIO_CRL_CNF1_1; // Input pull up
	GPIOA->ODR |= GPIO_ODR_ODR0 | GPIO_ODR_ODR1; 
	
	GPIOA->CRL &= ~(GPIO_CRL_MODE2); // Clear mode bits for PA2
  GPIOA->CRL &= ~(GPIO_CRL_CNF2_0); // Clear CNF2_0 bit for PA2
  GPIOA->CRL |= (GPIO_CRL_CNF2_1);
}
void EXTI_Config()
{
  // Kich hoat clock cho AFIO de cau hình EXTI
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  // Câu hình External Interrupt cho PA0 (EXTI0)
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0; // Xóa các thiet lap cu
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; // Chân PA0 làm nguôn ngat cho EXTI0
	EXTI->IMR |= EXTI_IMR_MR0; // Kích ho?t mask cho dòng ng?t 0
	EXTI->FTSR |= EXTI_FTSR_TR0; // C?u hình ng?t theo canh xuong
	NVIC_EnableIRQ(EXTI0_IRQn); // Kích ho?t ng?t trong NVIC

	// C?u hình External Interrupt cho PA1 (EXTI1)
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI1; // Xóa các thiet lap cu
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // Chan PA1 làm nguon ngat cho EXTI1
	EXTI->IMR |= EXTI_IMR_MR1; // Kích hoat mask cho dòng ngat 1
	EXTI->FTSR |= EXTI_FTSR_TR1; // Cau hình ngat theo canh xuong
	NVIC_EnableIRQ(EXTI1_IRQn); // Kích hoat ngat trong NVIC
	
	//Cau hinh external Interrupt cho PA2 (EXTI2)
	AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI1; // Xóa các thi?t l?p cu
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI2_PA; // Ch?n PA2 làm ngu?n ng?t cho EXTI2
	EXTI->IMR |= EXTI_IMR_MR2; // Kích hoat mask cho dòng ngat 2
	EXTI->FTSR |= EXTI_FTSR_TR2; // Cau hình ngat theo canh xuong
	NVIC_EnableIRQ(EXTI2_IRQn); // Kích hoat ngat trong NVIC
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

void EXTI2_IRQHandler(void) {
    
	EXTI->PR |= EXTI_PR_PR2;     // Xóa c? ng?t EXTI2
  fall = true;  
}