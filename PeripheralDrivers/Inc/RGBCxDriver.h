/*
 * RGBCxDriver.h
 *
 *  Created on: 12/05/2023
 *      Author: MillerQuintero2001
 */

#ifndef RGBCXDRIVER_H_
#define RGBCXDRIVER_H_

#include <stm32f4xx.h>
#include <stdint.h>
#include "I2CDriver.h"


// Elementos para comunicación I2C con el Sensor RGB
#define RGB_SLAVE_ADDRESS	0b0101001	// 0x29
#define ENABLE_REGISTER		0			// 0x00
#define TIMING_REGISTER		1			// 0x01
#define WAIT_TIME_REGISTER	3			// 0x03
#define CONTROL_REGISTER	15			// 0x0F
#define ID_REGISTER			18			// 0x12
#define CLEAR_DATA_LOW		20			// 0x14
#define CLEAR_DATA_HIGH		21			// 0x15
#define RED_DATA_LOW		22			// 0x16
#define RED_DATA_HIGH		23			// 0x17
#define GREEN_DATA_LOW		24			// 0x18
#define GREEN_DATA_HIGH		25			// 0x19
#define BLUE_DATA_LOW		26			// 0x1A
#define BLUE_DATA_HIGH		27			// 0x1B

// Bit de comando, esencial para el sensor TC2S34725
#define COMMAND_BIT			0x80		//

#endif /* RGBCXDRIVER_H_ */
