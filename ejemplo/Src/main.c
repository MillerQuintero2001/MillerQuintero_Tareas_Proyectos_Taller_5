/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQUintero2001
 * @brief          : Configuracion Básica de un proyecto
 ******************************************************************************
 * Generacion del archivo de configuración por defecto
 * como plantilla para los proyectos funcionales
 ******************************************************************************
 */

#include <stdint.h>
#include <stm32f4xx.h>

#include "GPIOxDriver.h"

GPIO_Handler_t handlerOnBoardLed = {0};

int main(void)
{
    handlerOnBoardLed.pGPIOx = GPIOA;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinNumber = PIN_5;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinMode 	= GPIO_MODE_OUT;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinSpeed 	= GPIO_OSPEED_FAST;
    handlerOnBoardLed.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OTYPE_PUSHPULL;

    GPIO_Config(&handlerOnBoardLed);

    GPIO_WritePin(&handlerOnBoardLed, SET);

    /* Configuración del timer */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 &= ~TIM_CR1_DIR;

    TIM2->PSC = 16000; //El pre-escale se puso en 16000 ya que como la F del MCU es 16MHz, con esto obtengo el incremento del counter cada 1 ms

    TIM2->CNT = 0;

    TIM2->ARR = 250;

    TIM2->CR1 |= TIM_CR1_CEN;

   while(1){

	   if(TIM2->SR & TIM_SR_UIF){
		   GPIOxTooglePin(&handlerOnBoardLed);
		   TIM2->SR &= ~TIM_SR_UIF; //Bajando la bandera
	   }

   }
   	   return 0;
}
