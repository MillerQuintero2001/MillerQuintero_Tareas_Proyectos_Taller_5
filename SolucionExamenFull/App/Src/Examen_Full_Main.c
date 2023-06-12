/**
 ********************************************************************************
 * @file         : Examen_Full_Main.c
 * @author       : Miller Quintero - miquinterog@unal.edu.co - MillerQuintero2001
 * @brief        : Solución del Examen
 ********************************************************************************
 * El equipo cargado con este programa deberá ,
 *
 ********************************************************************************
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <stm32f4xx.h>

#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"
#include "SysTickDriver.h"
#include "I2CDriver.h"
#include "PLLDriver.h"
#include "AdcDriver.h"
#include "PwmDriver.h"

#include "arm_math.h"

// Macro-definiciones específicas para el sensor
#define ACCEL_ADDRESS 	0b1101000; // 0xD2 -> Dirección del sensor Acelerómetro con ADO=0
#define ACCEL_XOUT_H	59	// 0x3B
#define ACCEL_XOUT_L	60	// OX3C
#define ACCEL_YOUT_H	61	// 0x3D
#define ACCEL_YOUT_L 	62	// 0x3E
#define ACCEL_ZOUT_H 	63	// 0x3F
#define ACCEL_ZOUT_L 	64	// 0x40
#define PWR_MGMT_1		107	// 0x6B
#define WHO_AM_I		117	// 0x75

/* Definición de los handlers necesarios */

// Elementos para el Blinky LED
GPIO_Handler_t handlerStatePin = 			{0};	// LED de estado del Pin H1
GPIO_Handler_t handlerBlinkyPin = 			{0}; 	// LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0};	// Timer del LED de estado

// Elemento para el Pin MC01 para chequear la señal
GPIO_Handler_t handlerMCO = {0};

// Elementos para el muestreo a 200Hz
BasicTimer_Handler_t handlerSamplingTimer = {0}; 	// Timer del muestreo de datos
uint8_t arraySaveData[6] = {0};						// Array que guarda los datos leídos de los registros de forma multiple
float32_t arrayXdata[1024] = {0};					// Array que guarda la información de datos eje X, capacidad de 2 segundos de muestreo
float32_t arrayYdata[1024] = {0};					// Array que guarda la información de datos eje Y, capacidad de 2 segundos de muestreo
float32_t arrayZdata[1024] = {0};					// Array que guarda la información de datos eje Z, capacidad de 2 segundos de muestreo
uint8_t flag200HzSamplingData = 0;					// Bandera para indicar tomar dato cada 5ms
uint16_t counter_ms = 0;							// Contador de milisegundos con el Timer 4

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};	// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};	// Pin de recepción de datos
USART_Handler_t usartComm =  {0};	// Comunicación serial
uint8_t usartRxData = 0; 			// Variable en la que se guarda el dato transmitido
char bufferData[128] = {0}; 			// Buffer de datos como un arreglo de caracteres
char userMsg[] = "Funciona por comandos c: \n";
char bufferReception[32] = {0};
char cmd[16] = {0};
uint16_t counterReception = 0;
bool stringComplete = false;
unsigned int firstParameter = 0;
unsigned int secondParameter = 0;

// Elementos para utilizar comunicación I2C con Acelerometro
GPIO_Handler_t handlerI2C_SDA = {0};
GPIO_Handler_t handlerI2C_SCL = {0};
I2C_Handler_t handlerAccelerometer = {0};
uint8_t i2cBuffer = 0;
uint8_t arrayRegisters[6] = {ACCEL_XOUT_L,ACCEL_XOUT_H, ACCEL_YOUT_L, ACCEL_YOUT_H, ACCEL_ZOUT_L, ACCEL_ZOUT_H};
uint8_t captureAccel = 0;

// Elementos para la ADC
ADC_Multichannel_Config_t handlerDualChannelADC = {0};
uint8_t orderChannelADC[2] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
uint16_t freqADC = 10;
uint16_t samplingArray[2] = {ADC_SAMPLING_PERIOD_84_CYCLES, ADC_SAMPLING_PERIOD_84_CYCLES};
uint16_t dataChannelADC0[1024] = {0};
uint16_t dataChannelADC1[1024]  = {0};
PWM_Handler_t handlerPwmEventADC = {0};
uint16_t counterDataADC = 0;
uint8_t indicatorADC = 0;
uint8_t adcComplete = 0;

// Elementos para transformada de Fourier

uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 config_Rfft_fast_f32;
arm_cfft_radix4_instance_f32 configRadix4_f32;
arm_status statusInitFFT = ARM_MATH_ARGUMENT_ERROR;
uint16_t fftSize = 1024;

#define SIGNAL_DATA_SIZE	1024					// Tamaño del arreglo de datos
float32_t transformedSignal[SIGNAL_DATA_SIZE];		// Arreglo donde se guardan los datos de la señal transformada
float32_t absFourierData[SIGNAL_DATA_SIZE];			// Arreglo donde se guardan los datos de la señal transformada luego de aplicar valor absoluto
float32_t finalFourierData[SIGNAL_DATA_SIZE];		// Arreglo donde se guardan los datos reales, del valor absoluto de la transformada
float32_t maxFourierValue = 0;						// Valor máximo qen el arreglo que retorna la transformada
uint16_t maxFourierIndex = 0;						// Indíce correspondiente al valor máximo
float fundFrequencyFFT = 0;							// Variable que guarda la frecuencia fundamental



/* Inicializo variables a emplear */
unsigned int freq = 0;				// Variable en la que guardo los Hz de reloj del micro, entregados por getConfigPLL()
uint16_t clock100 = 100;			// Variable que guarda 100, para argumento de la función configPLL(), y poner a 100MHz

/* Definición de las cabeceras de funciones del main */
void initSystem(void); 								// Función que inicializa los periféricos básicos
void commandBuild(void);							// Función que construye el string del comando USART
void commandUSART(char* arrayCmd);					// Función encargada de gestionar el comando ingresado por USART
void saveData(void);								// Función encargada de guardar los datos acelerómetro
void initLSE(void);									// Función encargada de iniciar el Low Speed External, cristal resonador
void initFourier(void);								// Función encargada de inicializar la función de FFT

/** Función principal del programa
 * ¡Esta función es el corazón del programa! */
int main(void){

	// Inicializamos todos los elementos del sistema
	initSystem();
	// Cargo en variable el valor en Hz del reloj MCU
	freq = (unsigned int)getConfigPLL();


    /* Loop forever */
	while(1){

		// Hacemos un "eco" con el valor que nos llega por el serial
		if(usartRxData != '\0'){
			writeChar(&usartComm, usartRxData);
			commandBuild();
		}

		// Verificamos si se ha dado por completado del comando
		if(stringComplete){
			commandUSART(bufferReception);
			stringComplete = false;
		}

		// Verificamos si el ADC ha terminado sus operaciones para imprimirlas
		if(adcComplete){
			adcComplete = 0;
			for(uint16_t i = 0; i<1024; i++){
				sprintf(bufferData, "%u\t%u\n", dataChannelADC0[i],dataChannelADC1[i]);
				writeMsg(&usartComm, bufferData);
			}
		}

		// Muestreo de datos constante cada 5ms
		if(flag200HzSamplingData){
			// Controlo el contador de cada 5 milisegundos, para controlar los datos
			if(counter_ms > 1024){
				counter_ms = 0;
				captureAccel = 0;
				writeMsg(&usartComm, "Accelerometer data capture complete, ready for Fourier \n");
			}
			saveData();					// Guardo los datos del acelerómetro
			flag200HzSamplingData = 0; 	// Bajo bandera porque ya se tomaron datos
		}

		// Algoritmo de la transformada rápida de Fourier (FFT)
		if(statusInitFFT == ARM_MATH_SUCCESS){
			int i = 0;

			sprintf(bufferData, "FFT \n");
			writeMsg(&usartComm, bufferData);
			arm_rfft_fast_f32(&config_Rfft_fast_f32, arrayZdata, transformedSignal, ifftFlag);
			arm_abs_f32(transformedSignal, absFourierData, fftSize);
			for(i = 1; i < fftSize; i++){
				if(i%2){
					finalFourierData[i] = absFourierData[i];
				}
			}

			// Finalmente rescatamos el valor máximo del arreglo de datos finales
			maxFourierValue = finalFourierData[0];
			maxFourierIndex = 0;
			for (i = 1; i < fftSize; i++){
				if(maxFourierValue < finalFourierData[i]){
					maxFourierValue = finalFourierData[i];
					maxFourierIndex = i;
				}
			}
			/* Calculamos el valor máximo con la fórmula: Indice * FrecMuestreo * (2*fftSize)
			 * se multiplica fftSize por 2 porque al tomar solo los indices pares del arreglo
			 * retornado, quedo la mitad del tamaño original de la FFT*/
			fundFrequencyFFT = (float)(maxFourierIndex* 200 /(fftSize))/2;
			sprintf(bufferData, "La frecuencia fundamental es %#.6f Hz \n", fundFrequencyFFT);
			writeMsg(&usartComm, bufferData);
			statusInitFFT = ARM_MATH_ARGUMENT_ERROR;
		}

	}
	return 0;
}

/** Función encargada de iniciar hardware para un pin*/
void initSystem(void){

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* Configuramos el equipo a 100MHz, usando el driver del PLL */
	configPLL(clock100);

	/* Calibración manual del HSI para tener mayor precisión:
	 * Midiendo con el LED de estado y haciendo cálculos, considerando el pre-escaler y auto-reload escogidos
	 * para el timer que controla el blinky, se llegó a una frecuencia real de 101044394,86532803052349080092 Hz
	 * según la bibliografía consultada, los bits HSITRIM[4:0] que son los del 3 al 7 del RCC_CR, están por
	 * defecto en un valor de 16, incrementar en 1 binario aumenta X% del HSI la frecuencia real, y decrementar
	 * en 1 binario, disminuye X% del HSI la frecuencia real.
	 * Haciendo pruebas se llego a que este es el valor con el que queda mejor calibrado */
	RCC->CR &= ~(0b11111 << RCC_CR_HSITRIM_Pos); 	// Limpio
	RCC->CR |= (13 << RCC_CR_HSITRIM_Pos);			// Escribo

	//Esperamos hasta que el HSI vuelva a ser estable
	while(!(RCC->CR & RCC_CR_HSIRDY)){
		__NOP();
	}
	// Fin de la calibración

	/* Configuración del Pin para el MC01 */
	handlerMCO.pGPIOx								= GPIOA;
	handlerMCO.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerMCO.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerMCO.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerMCO.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerMCO.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_FAST;
	handlerMCO.GPIO_PinConfig.GPIO_PinAltFunMode	= AF0;
	GPIO_Config(&handlerMCO);

	/* GPIO y Timer del Blinky Led de Estado PH1 */
	handlerStatePin.pGPIOx								= GPIOH;
	handlerStatePin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_1;
	handlerStatePin.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_OUT;
	handlerStatePin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerStatePin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerStatePin);
	// Pongo estado en alto
	GPIO_WritePin(&handlerStatePin, SET);

	/* GPIO y Timer del Blinky Led de Estado PA5 */
	handlerBlinkyPin.pGPIOx								= GPIOA;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerBlinkyPin);
	// Pongo estado en alto
	GPIO_WritePin(&handlerBlinkyPin, SET);

	// Atributos para el Timer 4 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM4;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	// Fin del GPIO y Timer del LED de estado

	/*----------------------------------------------------------------------------------------*/

	/* Configuración del Timer 3 para controlar el muestreo*/
	handlerSamplingTimer.ptrTIMx							= TIM3;
	handlerSamplingTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerSamplingTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerSamplingTimer.TIMx_Config.TIMx_period			= 50;
	handlerSamplingTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerSamplingTimer);

	/* Configuración de pines para el USART1 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_9;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinTX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_10;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_PUSHPULL;
	handlerPinRX.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usartComm.ptrUSARTx							= USART1;
	usartComm.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usartComm.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usartComm.USART_Config.USART_parity			= USART_PARITY_NONE;
	usartComm.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usartComm.USART_Config.USART_mode			= USART_MODE_RXTX;
	usartComm.USART_Config.USART_enableIntRX	= USART_RX_INTERRUP_ENABLE;
	usartComm.USART_Config.USART_enableIntTX	= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usartComm);

	/* ------------ Configuraciones del I2C1 para el acelerómetro ------------ */

	/* Configuración del pin SCL del I2C1 */
	handlerI2C_SCL.pGPIOx								= GPIOB;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinNumber		= PIN_8;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SCL);

	/* Configuración del pin SDA del I2C1 */
	handlerI2C_SDA.pGPIOx								= GPIOB;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinNumber		= PIN_9;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinMode			= GPIO_MODE_ALTFN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinOPType		= GPIO_OTYPE_OPENDRAIN;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinSpeed			= GPIO_OSPEED_HIGH;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl	= GPIO_PUPDR_NOTHING;
	handlerI2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode 	= AF4;
	GPIO_Config(&handlerI2C_SDA);

	/* Configuración para el I2C1 */
	handlerAccelerometer.ptrI2Cx		= I2C1;
	handlerAccelerometer.modeI2C		= I2C_MODE_FM;
	handlerAccelerometer.slaveAddress	= ACCEL_ADDRESS;
	i2c_config(&handlerAccelerometer);

	/* ------------ Configuración del ADC y el evento disparador del ADC ------------ */

	/* Configuración multicanal del ADC, la vamos a hacer con canales 0 y 1, es decir
	 * usando los pines PA0 y PA1 */
	handlerDualChannelADC.orderADC 			= orderChannelADC;
	handlerDualChannelADC.samplingPeriod	= samplingArray;
	handlerDualChannelADC.resolution		= ADC_RESOLUTION_12_BIT;
	handlerDualChannelADC.dataAlignment		= ADC_ALIGNMENT_RIGHT;
	handlerDualChannelADC.extTriggerEnable	= ADC_EXTEN_RISING_EDGE;
	handlerDualChannelADC.extTriggerSelect	= ADC_EXTSEL_TIM2_CC2;
	// Paso el elemento ADC la función de configuración, indicando el número de canales
	adc_ConfigMultichannel(&handlerDualChannelADC, 2);

	/* Configuración del PWM usado como disparador de la conversión ADC */
	// Utilizo el canal 3 para PWM del Timer 5 ya que el 1 y 2 están ocupados por ADC
	handlerPwmEventADC.ptrTIMx						= TIM2;
	handlerPwmEventADC.PWMx_Config.PWMx_Channel		= PWM_CHANNEL_2;
	handlerPwmEventADC.PWMx_Config.PWMx_Prescaler	= BTIMER_PLL_100MHz_SPEED_1us;
	handlerPwmEventADC.PWMx_Config.PWMx_Period		= freqADC;
	handlerPwmEventADC.PWMx_Config.PWMx_DuttyCicle	= freqADC/2;
	pwm_Config(&handlerPwmEventADC);

	/* Inicio el cristal LSE como RTC activo */
	initLSE();

	/* Reset del sensor por si no toma datos */
	i2c_writeSingleRegister(&handlerAccelerometer, PWR_MGMT_1, 0x00);

	//Fin initSystem

}

/** Función encargada de construir el string con el comando */
void commandBuild(void){
	bufferReception[counterReception] = usartRxData;
	counterReception++;

	// Aqui hacemmos la instrucción que detine la recepción del comando
	if(usartRxData == '\r'){
		stringComplete = true;

		//Sustituyo el último caracter de @ por un null
		bufferReception[counterReception] = '\0';
		counterReception = 0;
	}
	if(usartRxData == '\b'){
		counterReception--;
		counterReception--;
	}

	// Volvemos a null para terminar
	usartRxData = '\0';
}

/** Función encargada de gestionar e identificar el comando ingresado, recibe el puntero al string */
void commandUSART(char* ptrBufferReception){

	/* El funcionamiento es de la siguiente forma: Empleamos el puntero al buffer para
	 * acceder a los elementos del string, y por medio de la función sscanf se almacena
	 * en 3 elemetos diferentes, el string del comando "cmd", y dos números enteros llamados
	 * "firstParameter" y "SecondParameter". De esta froma, podemos introducir información
	 * al micro desde el puerto serial */
	sscanf(ptrBufferReception, "%s %u %u %s", cmd, &firstParameter, &secondParameter, userMsg);

	/* Usamos la funcion strcmp, string compare, que me retorna un 0 si los 2 strings son iguales */

	// Este primer comando imprime una lista con los otros comando que tiene el equipo
	if(strcmp(cmd, "help") == 0){
		writeMsg(&usartComm, "\nHelp Menu CMDs:\n");
		writeMsg(&usartComm, "1) help				-- Print this menu \n");
		writeMsg(&usartComm, "2) freq				-- Print current MCU frequency \n");
		writeMsg(&usartComm, "3) MCO #Source #Prescaler \n"
				" -- Source: HSE = 0; LSE = 1, PLL = 3 \n"
				" -- Pre-escaler from 1 to 5 \n");
		writeMsg(&usartComm, "4) reset				-- Reset MCO config, as HSI with Prescaler = 1 \n");
		writeMsg(&usartComm, "9) sampling #Freq[Hz]	-- ADC sampling 40 times more config, #A indicates the sampling frequency between 800 to 3000 Hz \n");
		writeMsg(&usartComm, "10) show				-- Show ADC data saved in arrays \n");
		writeMsg(&usartComm, "11) capture			-- Launch capture of Accelerometer data \n");
		writeMsg(&usartComm, "12) fourier			-- Show frequency data with CMSIS-FFT \n");

	}

	else if(strcmp(cmd, "freq") == 0){
		writeMsg(&usartComm, "\nCMD: freq \n");
		sprintf(bufferData, "The clock frequency is %u Hz \n", freq);
		writeMsg(&usartComm, bufferData);
	}

	else if(strcmp(cmd, "MCO") == 0){
		writeMsg(&usartComm, "\nCMD: MCO \n");
		if((0 <= firstParameter)&&(firstParameter < 6)&&(firstParameter != 2)&&(secondParameter<6)){
			changeMCO1(firstParameter, secondParameter+2);
			writeMsg(&usartComm, "MCO1 configuration succesfull \n");
			firstParameter = 0;
			secondParameter = 0;
		}
		else{
			writeMsg(&usartComm, "Wrong source or prescaler \n");
		}
	}

	else if(strcmp(cmd, "reset") == 0){
		writeMsg(&usartComm, "\nCMD: reset \n");
		changeMCO1(MCO1_HSI_CLOCK, MCO1_DIVIDED_BY_1);
		writeMsg(&usartComm, "The MC01 source is HSI with prescaler of 1 \n");
	}

	else if(strcmp(cmd, "sampling") == 0){
		writeMsg(&usartComm, "\nCMD: sampling \n");
		/* Calculamos el valor a cargar en el ARR de forma que se garantize un
		 * muestreo con el PWM, a una frecuencia 10 veces mayor a la deseada */
		if((firstParameter>=800)&&(firstParameter<=3000)){
			freqADC = (uint16_t)(((float)(1.0/firstParameter))*1000000)/40;
			updatePeriod(&handlerPwmEventADC, freqADC);
			updateDuttyCycle(&handlerPwmEventADC, freqADC/2);
			sprintf(bufferData, "The new sampling frequency is %u Hz, I choose 40 times more for good sampling \n", firstParameter*40);
			writeMsg(&usartComm, bufferData);
			firstParameter = 0;
		}
		else{
			writeMsg(&usartComm, "Incorrect frequency, must be an integer value between 800 Hz and 3000 Hz \n");
		}
	}

	else if(strcmp(cmd, "show") == 0){
		writeMsg(&usartComm, "\nCMD: sampling \n");
		startPwmSignal(&handlerPwmEventADC);
		enableOutput(&handlerPwmEventADC);
		writeMsg(&usartComm, "Wait... We are taking 256 data from ADC\n");
	}

	else if(strcmp(cmd, "capture") == 0){
		writeMsg(&usartComm, "\nCMD: capture \n");
		// Levanto la bandera para tomar 1024 datos a 200 Hz en el main, para la FFT
		captureAccel = 1;
		writeMsg(&usartComm, "Wait... We are taking 1024 data from Accelerometer with I2C\n");
	}

	else if(strcmp(cmd, "fourier") == 0){
		writeMsg(&usartComm, "\nCMD: fourier \n");
		// Inicializo la función para hacer la FFT
		initFourier();
	}
	else{
		writeMsg(&usartComm, "\nWrong command \n");
	}
}

/** Función encargada de guardar los datos tomados cada 1ms en los respectivos arreglos */
void saveData(void){

	i2c_readMultipleRegisters(&handlerAccelerometer, ACCEL_XOUT_H, 6, arraySaveData);

	if(captureAccel){
		int16_t AccelX = arraySaveData[0] << 8 | arraySaveData[1]; // Aquí lo que se hace es básicamente concatenar los valores
		arrayXdata[counter_ms] = (float32_t)(AccelX/1.0);

		int16_t AccelY = arraySaveData[2] << 8 | arraySaveData[3]; // Aquí lo que se hace es básicamente concatenar los valores
		arrayYdata[counter_ms] = (float32_t)(AccelY/1.0);

		int16_t AccelZ = arraySaveData[4] << 8 | arraySaveData[5]; // Aquí lo que se hace es básicamente concatenar los valores
		arrayZdata[counter_ms] = (float32_t)(AccelZ/1.0);
		counter_ms++;
	}

}

/** Función encargada de iniciar el cristal LSE */
void initLSE(void){
	// Activamos el Power Interface Clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	// Permitimos escritura en el LSE
	PWR->CR |= PWR_CR_DBP;
	// Activamos el LSE
	RCC->BDCR |= RCC_BDCR_LSEON;
	// Esperamos hasta que sea estable
	while(!(RCC->BDCR & RCC_BDCR_LSERDY)){
		__NOP();
	}
	// Seleccionamos el LSE como RTC
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;
	// Activamos el RTC
	RCC->BDCR |= RCC_BDCR_RTCEN;
}

void initFourier(void){
	statusInitFFT = arm_rfft_fast_init_f32(&config_Rfft_fast_f32, fftSize);

	if(statusInitFFT == ARM_MATH_SUCCESS){
		sprintf(bufferData, "Fourier initialization ... SUCCESS! \n");
		writeMsg(&usartComm, bufferData);
	}
}


/* Interrupción del timer blinky LED */
void BasicTimer4_Callback(void){
	GPIOxTooglePin(&handlerStatePin);	//Cambio el estado del LED PH1
	GPIOxTooglePin(&handlerBlinkyPin);	//Cambio el estado del LED PA5
}

/* Interrupción del timer de muestreo */
void BasicTimer3_Callback(void){
	flag200HzSamplingData = 1; 	// Levanto bandera para tomar datos cada 1ms
}

/* Interrupción del USART1 */
void usart1Rx_Callback(void){
	usartRxData = getRxData();
}

/* Interrupción del ADC */
void adcComplete_Callback(void){
	if(indicatorADC == 0){
		dataChannelADC0[counterDataADC] = getADC();
		indicatorADC++;
	}
	else{
		dataChannelADC1[counterDataADC] = getADC();
		indicatorADC = 0;
		counterDataADC++;

	}
	if(counterDataADC > 1023){
		counterDataADC = 0;
		disableOutput(&handlerPwmEventADC);
		stopPwmSignal(&handlerPwmEventADC);
		adcComplete = 1;
	}
}

