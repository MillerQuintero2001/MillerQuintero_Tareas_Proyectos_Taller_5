/**
 ******************************************************************************
 * @file           : main.c
 * @author         : MillerQuintero2001
 * @brief          : Archivo comentado y con código del contador binario
 ******************************************************************************
 * Tarea #2 de Taller V
 * DriverGPIOx
 ******************************************************************************
 */

//NOTA: Sí por algún motivo no funciona, quedo atento a sus comentarios, muchas gracias por su tiempo y atención
#include "stm32f411xx_hal.h"
#include "GPIOxDriver.h"
#include <stdint.h>

/*Definición e inicialización de variables */
uint8_t valor = 1; //Variable que almacena el valor del conteo
uint8_t p0, p1, p2, p3, p4, p5, p6 = 0; // Variables que almacenan el estado para un pin según los bits del valor de conteo.

/*Definición prototipo de funcion*/
void configPinesSegunValor(uint8_t valor, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6);

/*Definición de objetos tipo handler para los pines*/
GPIO_Handler_t handlerUsedAsBit0 = {0}; // Inicializo handler del PA7
GPIO_Handler_t handlerUsedAsBit1 = {0}; // Inicializo handler del PC8
GPIO_Handler_t handlerUsedAsBit2 = {0}; // Inicializo handler del PC7
GPIO_Handler_t handlerUsedAsBit3 = {0}; // Inicializo handler del PA6
GPIO_Handler_t handlerUsedAsBit4 = {0}; // Inicializo handler del PB8
GPIO_Handler_t handlerUsedAsBit5 = {0}; // Inicializo handler del PC6
GPIO_Handler_t handlerUsedAsBit6 = {0}; // Inicializo handler del PC9
GPIO_Handler_t handlerUsedAsUser = {0}; // Inicializo handler del USER_BUTTON, perteneciente al PC13
/**
 * Funcion principal del programa
 * Esta función es el corazón del programa
 *
 */
int main(void)
{

	/* 1er Punto, error en la funcion GPIO_ReadPin()
	 *	a) El error en este punto radica en que la función nos está retornando la información de todo el Input
	 *	Data Register (IDR), cuando nuestra intención es leer el estado de un pin en particular y nada más. Si
	 *	esta función no se corrige, podría estar retornando números como 11001, y un número de esos no me indica
	 *	el estado de una entrada específica, ya que este solo puede tomar 2 valores, "0" ó "1".
	 *
	 * 	b) Para solucionar este error es necesario aplicar una máscara que limpie todos
	 * 	los bits, excepto el primer bit, esta máscara se aplica con un &(and bitwise)
	 *
	 * 	c) Para corroborar que esta solución es correcta configuré un PIN como entrada
	 * 	y puse su estado en 1, comprobando efectivamente la lectura correcta. De cualquier forma
	 * 	esta función la empleo más adelante en el 3er punto y esto verifica su corrección.
	 */


	/*----------------------------------------------------------------------------------------------------
	 * 2do. Punto, la función Toogle fue creada en el archivo GPIOxDriver.c y la Macro-Definición
	 * esta en el archivo GPIOxDriver.h, en un main la función solo es invocada, así como se hizo con la
	 * función GPIO_Config, GPIO_WritePin y GPIO_ReadPin
	 *---------------------------------------------------------------------------------------------------*/

	/* 3er Punto, Contador binario up-down
	 * Lo primero de todo es configurar los pines de alimentación como salidas de próposito general
	 * y el pin correspondiente al USER_BUTTON como entrada. Para esto vamos crear primero los
	 * handler para cada uno de estos pines */

	//Configuración del PA7
	handlerUsedAsBit0.pGPIOx								= GPIOA;
	handlerUsedAsBit0.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerUsedAsBit0.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit0.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit0.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit0.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PC8
	handlerUsedAsBit1.pGPIOx								= GPIOC;
	handlerUsedAsBit1.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerUsedAsBit1.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit1.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit1.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit1.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PC7
	handlerUsedAsBit2.pGPIOx								= GPIOC;
	handlerUsedAsBit2.GPIO_PinConfig.GPIO_PinNumber			= PIN_7;
	handlerUsedAsBit2.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit2.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit2.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit2.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PA6
	handlerUsedAsBit3.pGPIOx								= GPIOA;
	handlerUsedAsBit3.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerUsedAsBit3.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit3.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit3.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit3.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PB8
	handlerUsedAsBit4.pGPIOx								= GPIOB;
	handlerUsedAsBit4.GPIO_PinConfig.GPIO_PinNumber			= PIN_8;
	handlerUsedAsBit4.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit4.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit4.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit4.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PC6
	handlerUsedAsBit5.pGPIOx								= GPIOC;
	handlerUsedAsBit5.GPIO_PinConfig.GPIO_PinNumber			= PIN_6;
	handlerUsedAsBit5.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit5.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit5.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit5.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PC9
	handlerUsedAsBit6.pGPIOx								= GPIOC;
	handlerUsedAsBit6.GPIO_PinConfig.GPIO_PinNumber			= PIN_9;
	handlerUsedAsBit6.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT; //Salida
	handlerUsedAsBit6.GPIO_PinConfig.GPIO_PinOPType			= GPIO_OTYPE_PUSHPULL; //Que pueda suministrar corriente
	handlerUsedAsBit6.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST; //Rápido
	handlerUsedAsBit6.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING; //Sin resistencias PUPD

	//Configuración del PC13 (USER_BUTTON)
	handlerUsedAsUser.pGPIOx								= GPIOC; //El botón está en el grupo GPIOC
	handlerUsedAsUser.GPIO_PinConfig.GPIO_PinNumber			= PIN_13; //El botón está en el PIN13
	handlerUsedAsUser.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_IN; //Entrada

	//Cargamos la configuración de todos los pines
	GPIO_Config(&handlerUsedAsBit0);
	GPIO_Config(&handlerUsedAsBit1);
	GPIO_Config(&handlerUsedAsBit2);
	GPIO_Config(&handlerUsedAsBit3);
	GPIO_Config(&handlerUsedAsBit4);
	GPIO_Config(&handlerUsedAsBit5);
	GPIO_Config(&handlerUsedAsBit6);
	GPIO_Config(&handlerUsedAsUser);


    /* Loop forever */
	while(1){
		if(GPIO_ReadPin(&handlerUsedAsUser) == 1){
			//Llamo la función encargada de configurar los pines seǵun valor, esta función está creada después del main.
			configPinesSegunValor(valor,p0,p1,p2,p3,p4,p5,p6);
			//Aumento el valor para este caso en 1
			valor+=1;
			if(valor > 60){
				//Si el valor de conteo superó a 60, reincia en 1
				valor = 1;
			}
			else{
				//De lo contrario continua igual
				valor = valor;
			}
		}
		//Cuando el USER_BUTTON esta presionado su estado es 0, entra aquí
		else{
			//Llamo nuevamente la funcíon de configuración de pines
			configPinesSegunValor(valor,p0,p1,p2,p3,p4,p5,p6);
			//Disminuyo el valor para este caso en 1
			valor-=1;
			if(valor < 1){
				//Si el valor de conteo es menor a 1, reincia en 60
				valor = 60;
			}
			else{
				//De lo contrario continua igual
				valor = valor;
			}
		}
		//Delay de 1 sec aproximadamente
		for(int i = 0; i<1250000; i++){
			asm("NOP");
		}
	}
}

/**Función encargada de configurar los pines según el valor del contador*/
void configPinesSegunValor(uint8_t contador, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6){
	//Se aplica una máscara a la variable valor que guarda el conteo, y se obtiene la info. del bit necesario, usando un AND BITWISE (&)
	//Luego para poder emplear el valor rescatado en la función GPIO_WritePin es necesario hacer un SHIFT RIGHT con el número de posiciones
	b0 = (valor&(0b1))>>0;
	b1 = (valor&(0b10))>>1;
	b2 = (valor&(0b100))>>2;
	b3 = (valor&(0b1000))>>3;
	b4 = (valor&(0b10000))>>4;
	b5 = (valor&(0b100000))>>5;
	b6 = (valor&(0b1000000))>>6;
	//Escribo el estado en cada pin
	GPIO_WritePin(&handlerUsedAsBit0, b0);
	GPIO_WritePin(&handlerUsedAsBit1, b1);
	GPIO_WritePin(&handlerUsedAsBit2, b2);
	GPIO_WritePin(&handlerUsedAsBit3, b3);
	GPIO_WritePin(&handlerUsedAsBit4, b4);
	GPIO_WritePin(&handlerUsedAsBit5, b5);
	GPIO_WritePin(&handlerUsedAsBit6, b6);
}
