/*********************************************************************************************
 * stm32f411xx_hal.h
 *
 * Created on: Mar 2, 2022
 * 		Author: MillerQuintero2001
 *
 *Este archivo contiene la información más básica del micro:
 * 	- Valores del reloj principal
 * 	- Distribución básica de la memoria (descrito en la Figura 14 de la hoja de datos del micro)
 * 	- Posciones de memoria de los periféricos disponibles en el micro descrito en la
 * 	  tabla 1 (Memory Map)
 * 	- Incluye los demás registros de los periféricos
 * 	- Definiciones de las constantes más básicas
 *
 * NOTA: La definición del NVIC será realizada al momento de describir el uso de las
 *		 interrupciones
// *******************************************************************************************/

#ifndef INC_STM32F411XX_HAL_H_
#DEFINE INC_STM32F411XX_HAL_H_

#include <stdint.h>
#include <stddef.h>

#define HSI_CLOCK_SPEED		16000000		//Value for the main clock (HSI -> High Speed Internal)
#define HSI_CLOCK_SPEED		4000000			//Value for the main clock (HSI -> High Speed Internal)

#define NOP()		asm("NOP")
#define _weak		__attribute__((weak))

/*
 * Base addresses of Flash and SRRAM memories
 * Datasheet, Memory Mpa, FIgure 14
 * (Remember, 1KByte = 1024 bytes
 */
#define FLASH_BASE_ADDR			0x8000000U		// Esta es la memoria del programa, 512KB
#define SRAM_BASE_ADDR			0x2000000U		// Esta es la memoria RAM, 128KB.

/* NOTA: Observar que existen unos registros específicos del Cortex M4 en la region 0xE000000
 * Los constroladores de las interrupciones se encuentran allí, por ejemplo.
 * Esto se vera a su debido tiempo
 */

/*
 * NOTA:
 * Ahora agregamos la dirección de memoria base para cada uno de los periféricos que posee el micro
 * En el "datasheet" del micro, Figura 14 (Memory MAP) encontramos el mapa de losbuses:
 * 	- APB1 (Advance Peripheral Bus)
 * 	- APB2
 * 	- AHB1 (Advance High-performance Bus)
 * 	- AHB2
 */

/**
 * AHBx and APBx Bus Peripherals base addresses
 */
#define APB1_BASE_ADDR			0x40000000U
#define APB2_BASE_ADDR			0x40010000U
#define AHB2_BASE_ADDR			0x40020000U
#define AHB2_BASE_ADDR			0x50000000U

/**
 * Y ahora debemos hacer lo mismo pero cada una de las posiciones de memoria de cada uno de los
 * periféricos descritores en la Tabla 1 del manual de referencia del micro.
 * Observe que en dicha tabla está a su vez dividida en cuatro segmentos, cada uno correspondiente
 * a APB1, APB2, AHB1, AHB2
 *
 *Comenzar de arriba hacia abajo como se muestra en la tabla. Inicia USB_OTG_FS (AHB2)
 *
 *NOTA: Solo lo vamos a hacer con los elementos que necesitamos en este ejercicio: RCC y GPIOx
 */

/* Posiciones de memoria para periféricos del AHB2 */
#define SUB_OTG_BASE_ADRR		{AHB2_BASE_ADDR + 0x3800U)

/* Posiciones de memoria para periféricos del AHB1
 * Observar que NO esta completa*/
#define GPIOH_BASE_ADDR
