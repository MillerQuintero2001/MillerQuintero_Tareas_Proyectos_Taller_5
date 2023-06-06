/*
 * PLLDriver.c
 *
 *  Created on: 26/04/2023
 *      Author: MillerQuintero2001
 *
 * 	Este driver permite manipular la frecuencia de la señal de reloj del microncontrolador,
 * 	obtener información de esta, y emplear el pin MC01 para medir las señales.
 */

#include <stm32f4xx.h>
#include <stdint.h>
#include <PLLDriver.h>


/** Función de configuración del PLL según la frecuencia en MHz*/
void configPLL(uint16_t PLLFreqMHz){

	// 0. Se verifica antes que todo que el HSI sea el oscilador usado por el PLL
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);

	// 1. Seleccionamos el regulador para la frecuencia deseada
	if(PLLFreqMHz<=64){
		PWR->CR |= (0x1 << PWR_CR_VOS_Pos);
	}
	else if((64<PLLFreqMHz)&&(PLLFreqMHz<=84)){
		PWR->CR |= (0x2 << PWR_CR_VOS_Pos);
	}
	else if((84<PLLFreqMHz)&&(PLLFreqMHz<=100)){
		PWR->CR |= (0x3 << PWR_CR_VOS_Pos);
	}

	/* 2. Cambiamos los registros necesarios para poder acceder a la memoria flash */

	// 2.1 Prefetch, Data e Instruction Cache adecuados
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR |= FLASH_ACR_ICEN;
	FLASH->ACR |= FLASH_ACR_DCEN;

	// 2.2 Configuramos la respectiva latencia para PLLFreqMHz con X Wait States para 2.7 a 3.6 Voltios
	FLASH->ACR &= ~ FLASH_ACR_LATENCY;
	if((0<PLLFreqMHz)&&(PLLFreqMHz<=30)){
		FLASH->ACR |= FLASH_ACR_LATENCY_0WS;
	}
	else if((30<PLLFreqMHz)&&(PLLFreqMHz<=64)){
		FLASH->ACR |= FLASH_ACR_LATENCY_1WS;
	}
	else if((64<PLLFreqMHz)&&(PLLFreqMHz<=90)){
		FLASH->ACR |= FLASH_ACR_LATENCY_2WS;
	}
	else if((90<PLLFreqMHz)&&(PLLFreqMHz<=100)){
		FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
	}
	else{
		__NOP();
	}

	/* 3. Ahora escogemos los pre-escaler adecuados */

	// 3.2 Definimos cual va hacer el valor del PLLM, pre-escaler que divide la frecuencia que recibe el PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos); // Escogemos 8 y al VCO le llegan 2MHz, 16/8=2

	// 3.3 Definimos cual va a ser el valor del PLLN, pre-escaler que multiplica lo que entra al VCO
	/* NOTA: Como dejamos el PLLM en 8, y el PLLP en 2, el valor del PLLN siempre será igual
	 * a la PLLFreqMHz deseada, limitando que no pasemos de 100 MHz*/
	if(PLLFreqMHz<=100){
		RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
		RCC->PLLCFGR |= (PLLFreqMHz <<  RCC_PLLCFGR_PLLN_Pos); // Con esto si usamos 80 del VCO salen 160 MHz
	}
	else{
		__NOP();
	}

	// 3.4 Definimos cual va a ser el valor del PLLP, pre-escaler que divide lo que sale del VCO y llega al SysClk
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	RCC->PLLCFGR |= (0b00 <<RCC_PLLCFGR_PLLP_Pos); 	// Con esto dividimos por 2 y obtenemos 80 MHz, 160/2=80

	// 3.5 Ahora configuramos para AHB
	RCC->CFGR &= ~RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; // Con esto para el bus AHB dividimos por 1, queda en 80 MHz, o lo mismo del PLLFreqMHz

	// 3.6 Para APB1
	if(PLLFreqMHz>50){
		RCC->CFGR &= ~RCC_CFGR_PPRE1;
		// Si escogemos que se divide por 2, dan 40 MHz, esto para no exceder el límite de 50MHz
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	}
	else if(PLLFreqMHz<=50){
		RCC->CFGR &= ~RCC_CFGR_PPRE1;
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;
	}
	else{
		__NOP();
	}

	// 3.7 Para APB2
	// No se divide para valores en el registro menores a 4(binario), se queda en PLLFreqMHz, ya que admite hasta 100 MHz
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;


	// 4. Ahora activamos el PLL
	RCC->CR |= RCC_CR_PLLON;

	// 5. Esperamos hasta que el hardware indique que el PLL esta desbloqueado
	while( !(RCC->CR & RCC_CR_PLLRDY)){
		__NOP();
	}

	/* 6. Ahora convertimos nuestro PLL en nuestro System Clock, solo si la PLLFreqMHz no supera los 100MHz
	 * de lo contrario se queda con el HSI */
	if(PLLFreqMHz<=100){
		RCC->CFGR &= ~RCC_CFGR_SW;
		RCC->CFGR |= RCC_CFGR_SW_1;
	}
	else{
		__NOP();
	}

}

/** Función que retorna la frecuencia configurada calculada con la frecuencia del HSI y los pre-escaler */
uint64_t getConfigPLL(void){
	uint64_t freq = 0;
	/* Vamos a leer los registros para obtener el valor de los pre-escaler
	 * entendiendo que para que esta función sea correcta el PLLP debe ser siempre 2
	 * lo que hacemos es conseguir el valor del PLLM y el PLLN que son sencillos
	 * obteniendo el valor en la posición, y luego desplazando a la izquierda
	 * en esa misma posición */
	if(RCC->CFGR & RCC_CFGR_SW){
		uint32_t PLLM = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos);
		uint32_t PLLN = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos);
		freq = (HSI_FREQUENCY*(PLLN))/(2*PLLM);
	}
	else{
		freq = HSI_FREQUENCY;
	}
	return freq;
}

/** Función para definir la fuente de reloj y el pre-escaler para el pin usado como Microcontroller Clock Output 1*/
void changeMCO1(uint8_t sourceClock, uint8_t preEscaler){
		RCC->CFGR &= ~RCC_CFGR_MCO1; 						// Limpio
		RCC->CFGR |= (sourceClock << RCC_CFGR_MCO1_Pos); 	// Selecciona el reloj
		RCC->CFGR &= ~RCC_CFGR_MCO1PRE; 					// Limpio
		RCC->CFGR |= (preEscaler << RCC_CFGR_MCO1PRE_Pos); 	// Selecciono el pre-escaler para el MCO1
}
