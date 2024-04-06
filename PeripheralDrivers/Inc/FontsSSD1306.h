/*
 * FontsSSD1306.h
 *
 *  Created on: Aug 6, 2023
 *      Author: MillerQuintero2001
 */

#ifndef FONTSSSD1306_H_
#define FONTSSSD1306_H_

#include <stdint.h>

/* Estructura para definir la fuente */
typedef struct {
    uint8_t FontWidth;    /* Tamaño del ancho en pixeles */
    uint8_t FontHeight;   /* Tamaño del alto en pixeles */
    uint16_t *data;       /* Puntero a matriz de datos con el tamaño*/
} FontDef;


/* Exportamos las 3 fuentes disponibles */
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif /* FONTSSSD1306_H_ */
