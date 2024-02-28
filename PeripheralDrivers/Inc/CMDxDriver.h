/*
 * CMDxDriver.h
 *
 *  Created on: 28/02/2024
 *      Author: MillerQuintero2001
 */

#ifndef CMDXDRIVER_H_
#define CMDXDRIVER_H_

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <stm32f4xx.h>

#include "USARTxDriver.h"

void commandConfig(void);
void commandBuild(void);
void commandUSART(char* ptrBufferReception);
void command_1(void);
void command_2(void);
void command_3(void);
void command_4(void);
void command_5(void);
void command_6(void);


#endif /* CMDXDRIVER_H_ */
