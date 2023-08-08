/*
 * SSD1306.h (OLED 0.96)
 *
 *  Created on: Aug 6, 2023
 *      Author: MillerQuintero2001
 */

#ifndef SSD1306_H_
#define SSD1306_H_

#include "FontsSSD1306.h"
#include "I2CDriver.h"
#include "SysTickDriver.h"

#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_COM_LR_REMAP    0
#define SSD1306_COM_ALTERNATIVE_PIN_CONFIG    1

#define COMMAND_OLED 	0x80
#define DATA_OLED 		0x01

/*  Númeración para los estados pixeles en la OLED */
typedef enum {
    Black = 0x00,   // Color negro, pixel en estado bajo
    White = 0x01,   // COlor blanco, pixel en estado alto
} SSD1306_COLOR;


/* Estructura para almacenar transformaciones */
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} SSD1306_t;

/* Prototipos de funciones */
uint8_t ssd1306WriteCommand(I2C_Handler_t *ptrHandlerOLED, uint8_t command);
uint8_t ssd1306Init(I2C_Handler_t *ptrHandlerOLED);
void ssd1306Fill(SSD1306_COLOR color);
void ssd1306UpdateScreen(I2C_Handler_t *ptrHandlerOLED);
void ssd1306DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
char ssd1306WriteChar(char character, FontDef font, SSD1306_COLOR color);
void ssd1306WriteString(char* str, FontDef Font, SSD1306_COLOR color);
void ssd1306InvertColors(void);
void ssd1306SetCursor(uint8_t x, uint8_t y);

#endif /* SSD1306_H_ */
