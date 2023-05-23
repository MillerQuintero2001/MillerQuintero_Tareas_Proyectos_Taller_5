/*
 * PLLDriver.c
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 *
 * 	Este driver permite manipular la frecuencia de la señal de reloj del microncontrolador,
 * 	como el tiempo apremia, de momento solo va a ser para la frecuencia de 80 MHz
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <PLLDriver.h>


/** Función de configuración del PLL según la frecuencia en MHz*/
void configPLL(int PLLFreqMHz){

	// Se verifica antes que todo que el HSI sea el oscilador usado por el PLL
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);

	// 1. Activamos el Power Interface Clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// 4. Seleccionamos el regulador para la frecuencia deseada
	//PWR->CR &= ~PWR_CR_VOS;
	PWR->CR |= (0x2 << PWR_CR_VOS_Pos);

	/* 5. Cambiamos los registros necesarios para poder acceder a la memoria flash */

	// 5.1 Prefetch, Data e Instruction Cache adecuados
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_ICEN;
	FLASH->ACR |= FLASH_ACR_DCEN;

	// 5.2 Configuramos la respectiva latencia para 80 MHz que es 2 Wait States para 2.7 a 3.6 Voltios
	FLASH->ACR &= ~ FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS;

	/* 6. Ahora escogemos los pre-escaler adecuados */

	// 6.2 Definimos cual va hacer el valor del PLLM, pre-escaler que divide la frecuencia que recibe el PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos); // Escogemos 8 y al VCO le llegan 2MHz, 16/8=2

	// 6.3 Definimos cual va a ser el valor del PLLN, pre-escaler que multiplica lo que entra al VCO
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	RCC->PLLCFGR |= (160 <<  RCC_PLLCFGR_PLLN_Pos); // Con esto si usamos 160 del VCO salen 320 MHz

	// 6.4 Definimos cual va a ser el valor del PLLP, pre-escaler que divide lo que sale del VCO y llega al SysClk
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	RCC->PLLCFGR |= (0b01 <<RCC_PLLCFGR_PLLP_Pos); 	// Con esto dividimos por 4 y obtenemos 80 MHz, 320/4=80

	// 6.5 Ahora configuramos para AHB
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Con esto para el bus AHB dividimos por 1, queda en 80 MHz

	// 6.6 Para APB1
	RCC->CFGR &= ~RCC_CFGR_PPRE1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // Si escogemos que se divide por 2, dan 40 MHz
									  // esto para no exceder el límite de 50MHz
	// 6.7 Para APB2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // No se divide para valor menores a 4, se queda en 80 MHz, ya que admite hasta 100 MHz


	// 7. Personalmente quiero habilitar la posbilidad de usar un pin del micro para observar la señal
	RCC -> CFGR |= RCC_CFGR_MCO1; // Con este para el MCO1 uso la PLL de 80 MHz
	RCC -> CFGR &= ~RCC_CFGR_MCO1PRE;
	RCC -> CFGR |= RCC_CFGR_MCO1PRE; // Con esta macro, divido los 80MHz por 5, para tener 16MHz en el pin MCO1


	// 8. Ahora activamos el PLL
	RCC->CR |= RCC_CR_PLLON;

	// 8.1 Esperamos hasta que el hardware indique que el PLL esta desbloqueado
	while( !(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}

	// Ahora convertimos nuestro PLL de 80MHz en nuestro System Clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_1;


}

/** Función que retorna la frecuencia configurada calculada con la frecuencia del HSI y los pre-escaler */
uint64_t getConfigPLL(void){
	uint64_t freq = 0;
	/* Vamos a leer los registros para obtener el valor de los pre-escaler
	 * entendiendo que para que esta función sea correcta el PLLP debe ser siempre 4
	 * lo que hacemos es conseguir el valor del PLLM y el PLLN que son sencillos
	 * obteniendo el valor en la posición, y luego desplazando a la izquierda
	 * en esa misma posición */
	uint32_t PLLM = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos);
	uint32_t PLLN = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos);
	freq = (HSI_FREQUENCY*(PLLN))/(4*PLLM);
	return freq;
}
