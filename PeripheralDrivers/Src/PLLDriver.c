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
#include <PLLDriver.h>



/** Función de configuración del PLL según la frecuencia en MHz*/
void configPLL(uint8_t freq){

	// 1. Activación del HSI
	RCC->CR |= RCC_CR_HSION;

	// 1.1 Esperamos hasta que esté activo
	while(!(RCC->CR &= RCC_CIR_HSERDYIE)){
		__NOP();
	}

	// 2. Verificamos que el PLL está inactivo
	RCC->CR &= ~RCC_CR_PLLON;

	// 3. Activamos el Power Interface Clock
	RCC->APB1ENR &= ~RCC_APB1ENR_PWREN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// 4. Seleccionamos el regulador para la frecuencia deseada
	PWR->CR &= ~PWR_CR_VOS;
	PWR->CR |= (0b10 << PWR_CR_VOS_Pos);

	/* 5. Cambiamos los registros necesarios para poder acceder a la memoria flash */

	// 5.1 Prefetch, Data e Instruction Cache adecuados
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_ICEN;
	FLASH->ACR |= FLASH_ACR_DCEN;

	// 5.2 Configuramos la respectiva latencia para 80 MHz que es 2 Wait States para 2.7 a 3.6 Voltios
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR &= FLASH_ACR_LATENCY_2WS;

	/* 6. Ahora escogemos los pre-escaler adecuados */

	// 6.2 Definimos cual va hacer el valor del PLLM, pre-escaler que divide la frecuencia que recibe el PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM; // Limpio
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos); // Así al VCO le llegan 2MHz, 16/8=2

	// 6.3 Definimos cual va a ser el valor del PLLN, pre-escaler que multiplica lo que entra al VCO
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN; // Limpio
	RCC->PLLCFGR |= (160 << RCC_PLLCFGR_PLLN_Pos); // Con esto del VCO salen 320 MHz

	// 6.4 Definimos cual va a ser el valor del PLLP, pre-escaler que divide lo que sale del VCO y llega al SysClk
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; // Limpio
	RCC->PLLCFGR |= (4 << RCC_PLLCFGR_PLLP_Pos); // Con esto obtenemos 80 MHz, 320/4=80

	// 6.5 Ahora configuramos para AHB
	RCC->CFGR &= ~RCC_CFGR_HPRE; // Con esto indicamos que no se va a dividir

	// 6.6 Para APB1
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 	// Limpio
	RCC->CFGR |= RCC_CFGR_PPRE1_2; 	// La macro escribe un 1000 en los bits, indicando que se divide por 2, teniendo así 40 MHz
									// esto para no exceder el límite de 50MHz
	// 6.7 Para APB2
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // No se divide, se queda en 80 MHz, ya que admite hasta 100 MHz

	// 6.8 Definimos el HSI como el reloj base para el PLL
	RCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLSRC_HSE_Pos);

	// 7. Ahora activamos el PLL
	RCC->CR |= RCC_CR_PLLON;

	// 7.1 Esperamos hasta que el hardware indique que el PLL esta desbloqueado
	while( !(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}

	// Ahora convertimos nuestro PLL de 80MHz en nuestro System Clock
	RCC->CFGR &= ~RCC_CFGR_SW_PLL;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Esperamos hasta que el hardware indique que efectivamente el PLL es el System Clock
	while( ((RCC->CR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL)){
		__NOP();
	}

}

/** Función que retorna la frecuencia configurada calculada con la frecuencia del HSI y los pre-escaler */
uint64_t getConfigPLL(void){
	uint64_t freq = 0;
	//uint64_t HSI = 16000000;
	return freq;
}
