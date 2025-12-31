
#include "../Inc/stm32f103xx_gpio_driver.h"
void GPIO_Init(GPIO_Handle_s *pGPIOx_Handle);
int main(void){

	GPIO_Handle_s gpioled;


	GPIO_Clk_Control(GPIOC,ENABLE);
	gpioled.pGPIOx = GPIOC;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_50MHz;
	gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


   /*
    *GPIO_Init(&gpioled);
    */

	while(1){
	GPIO_ToggleOutputPin( GPIOC, GPIO_PIN_NO_13);

	}

}


