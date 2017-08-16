#include "stm32f0xx_conf.h"
#include "stm32f0xx.h"


void SysTick_Handler(void) {
  static uint16_t tick = 0;

  switch (tick++) {
  	case 100:
  		tick = 0;
  		GPIOA->ODR ^= (1 << 10);
  		break;
  }
}
// add I2C stuff here
int main(void)
{

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 	// enable the clock to GPIOC
						//(RM0091 lists this as IOPCEN, not GPIOCEN)

	GPIOA->MODER = (1 << 20); //GPIO set mode register 

	SysTick_Config(SystemCoreClock/100);

	while(1);

}
