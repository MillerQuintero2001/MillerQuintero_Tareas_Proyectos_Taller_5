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
#include "stm32f411xx_hal.h"
/**
 * Funcion principal del programa
 * Esta funcion es el corazon del programa
 *
 */
int main(void)
{
	//Configuración inicial del MCU
	RCC->AHB1ENR &= ~(1 << 0); //Borrar la posiciones cero
	RCC->AHB1ENR |= (1 << 0); //Activando la señal de reloj del GPIOA
	//Para colocar un 1 en el primer bit del advance high performance bus 1 enable, esto se hace con una máscara para no borrar la info de los otros bits

	// Configurar MODER
	GPIOA->MODER &= ~(0b11 << 10); //Limpiando la posición correspondiente al pinA5
	GPIOA->MODER |= (0b01 << 10); //Configurando el pinA5 como salida general

	//COnfigurar OTYPE
	GPIOA->OTYPER &= ~(0b1 << 5); //pinA5 configurado como salida Push-Pull

	//Configurar el OSPEED
	GPIOA->OSPEEDR &= ~(0b11 << 10); //Limpiando las posiciones correspondientes al pinA5
	GPIOA->OSPEEDR |= (0b10 << 10); //Configurando la velocidad como Fast

	GPIOA->ODR &= ~(0b1 << 5); //Limpiando posición 5 -LED apagado
	GPIOA->ODR |= (0b01 << 5); //Escribiendo 1 en posición 5, el LED debe encender
    /* Loop forever */
	while(1){

	}

	return 0;
}

//Este proyecto de IntroGPIO hace lo mismo que se hace en el del Add Header, que es configurar el PIN 5 del A como salida en alto.
