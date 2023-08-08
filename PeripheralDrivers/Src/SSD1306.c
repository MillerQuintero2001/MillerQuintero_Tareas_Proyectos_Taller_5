/*
 * SSD1306.c (OLED 0.96)
 *
 *  Created on: Aug 6, 2023
 *      Author: MillerQuintero2001
 */

#include "SSD1306.h"
#include "FontsSSD1306.h"
#include "I2CDriver.h"
#include "SysTickDriver.h"

/* Buffer del display, el tamaño de este buffer se calcula de esta forma
 * ya que cada 8 pixeles de la OLED son un byte de memoria, porque cada
 * pixel es un bit en 0 o 1 según el color blanco/negro */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Objeto de la OLED, para acceder a la información
static SSD1306_t SSD1306;


/** Función para escribir comandos en la OLED */
uint8_t ssd1306WriteCommand(I2C_Handler_t *ptrHandlerOLED, uint8_t command){
	i2c_writeSingleRegister(ptrHandlerOLED, COMMAND_OLED, command);
	return 0;
}


/** Función para inicializar la pantalla OLED */
uint8_t ssd1306Init(I2C_Handler_t *ptrHandlerOLED){
    // Esperamos a que la pantalla arranque
    delay_ms(100);
    int status = 0;

    // Iniciamos la OLED, siguiendo la tabla de comandos de la datasheet

    status += ssd1306WriteCommand(ptrHandlerOLED, 0xAE);   // Display en off, sin energía en los diodos emisores de luz orgánicos (OLED)
    /* Hacemos un Set Memory Addressing Mode, donde: 00 es Horizontal Addressing Mode(este es el estándar;
      * 01 es Vertical Addressing Mode;10 es Page Addressing Mode (RESET) y 11 es Invalid*/
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x20);	// Comando
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x10);   	// Valor el último byte es 00, indicando modo horizontal

    status += ssd1306WriteCommand(ptrHandlerOLED, 0xB0);   	// Hacemos Set Page Start Address, comenzando desde la primera
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xC8);   	// Establecemos dirección de escaneo de los comunes (COM) en los OLED's, C8 es el estándar
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x00);   	// Definimos la columna más baja por estándar (Set low column address), acto protocolario
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x10);   	// Definimos la columna más baja por estándar (Set low column address), acto protocolario
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x40);   	// Establecemos la línea en la que empieza el cursor de la OLED, con 0x40 es en la 1ra línea
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x81);  	// Establecemos el contraste con un valor de 0 a 255, con 0x81 (129) es un buen contraste
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xFF);	// Comando dummy, necesario para iniciar la OLED
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xA1);  	// Set segment re-map 0 to 127, (creo que es modo invertido) !CAUTION POSSIBLE ERROR!
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xA6);  	// Modo de iluminación normal en los pixeles (0 = OFF, 1 = ON, Set normal display)

    status += ssd1306WriteCommand(ptrHandlerOLED, 0xA8);   // Establecemos la relación multiplex que indica el número de filas
    status += ssd1306WriteCommand(ptrHandlerOLED, SSD1306_HEIGHT - 1); // Contando desde el 0, son 63 líneas en una OLED 128x64

    /* Configuramos con 0xA4 para que los pixeles en alto correspondan a la info de la GDDRAM, con 0xA5 la ignoran, puede ser otra forma de invertir colores */
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xA4);

    status += ssd1306WriteCommand(ptrHandlerOLED, 0xD3);   // Comando para establecer un desplazamiento inicial en la pantalla  (Set display offset)
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x00);   // No offset

    /* Establecemos con este comando la frecuencia de reloj del oscilador, el primer byte es el pre-escaler que divide
     * pudiendo escoger por tanto valores de 1 hasta 16. El segundo byte es el valor del oscilador, también hasta de 16 */
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xD5);   // Comando
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xF0);   // Valor, con esto tenemos la mayor frecuencia

    /* Establecemos ahora el período de pre-carga, primero el comando y luego el valor;
     * en dicho valor el primer byte es es el número de ciclos de reloj por fila de fase 1,
     * el segundo byte es los ciclos de reloj por fila de fase 2. La fase 1 se refiere
     * al tiempo de descarga de los pixeles para mostrar la nueva imagen, mientras que
     * la fase 2 al tiempo de carga conectados a la alimentación */
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xD9);   	// Comando
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x22);	// Con esto tenemos 2 ciclos de reloj en carga y descarga

    /* Definimos el orden de las filas en la matriz OLED, hay múltiples optciones,
     * hasta lineas alternadas, lo mejor es consultar la documentación */
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xDA);   // Comando, Set COM pins
    status += ssd1306WriteCommand(ptrHandlerOLED, SSD1306_COM_LR_REMAP << 5 | SSD1306_COM_ALTERNATIVE_PIN_CONFIG << 4 | 0x02); // Estándar, alternativo y sin remap

    /* Ajustams el voltaje para la salida del regulador que alimenta la línea común
     * con el sustrato de la OLED, primero el comando de acción y luego el valor */
    status += ssd1306WriteCommand(ptrHandlerOLED, 0xDB);   // Comando, set voltage COMH
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x20);   // Valor 0x20,con este obtenemos un 0.77*Vcc

    status += ssd1306WriteCommand(ptrHandlerOLED, 0x8D);   // Set DC-DC enable
    status += ssd1306WriteCommand(ptrHandlerOLED, 0x14);   // IDK

    status += ssd1306WriteCommand(ptrHandlerOLED, 0xAF);   // Display ON

    if (status != 0) {
        return 1;
    }

    // Escribimos en el buffer del Display un valor de solo un color
    ssd1306Fill(Black);

    // Actualizamos la pantalla, pintando los valores
    ssd1306UpdateScreen(ptrHandlerOLED);

    // Establecemos el origen
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    SSD1306.Initialized = 1;

    return 0;
}

/** Función para pintar cada pixel con el color seleccionado,
 *  puede usarse como un reseteo de la pantalla sin iniciar
 *  todo de nuevo */
void ssd1306Fill(SSD1306_COLOR color){
    // Llenamos los valores del buffer del Display con el oclor indicado
    uint32_t i;

    for(i = 0; i < sizeof(SSD1306_Buffer); i++)
    {
    	/* Usamos el evaluador condicional para asignar color
    	 * negro (estado bajo) a los pixeles que están en blanco */
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}


/** Función para escribir la información del buffer del Display, actualizando la pantalla */
void ssd1306UpdateScreen(I2C_Handler_t *ptrHandlerOLED){
    uint8_t i;
    uint8_t* pointerArray[8];

    // Llenamos el arreglo de punteros con las respectivas posiciones
    for (i = 0; i < 8; i++){
    	pointerArray[i] = &SSD1306_Buffer[SSD1306_WIDTH * i];
    }

    // Ciclo para escribir los datos de la GDDRAM
    for (i = 0; i < 8; i++) {
    	// Nos movemos en las páginas, la OLED tiene 8 Páginas
        ssd1306WriteCommand(ptrHandlerOLED, 0xB0 + i);
        // Nos movemos a la columna baja
        ssd1306WriteCommand(ptrHandlerOLED, 0x00);
        // Nos movemos a la columna fija
        ssd1306WriteCommand(ptrHandlerOLED, 0x10);

        // Pasamos el elemento correspondiente del arreglo de punteros, que es en sí un puntero
        i2c_writeMultipleRegisters(ptrHandlerOLED, 0x40, SSD1306_WIDTH, pointerArray[i]);

        //HAL_I2C_Mem_Write(ptrHandlerOLED, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}

/** Función para pintar un pixel */
void ssd1306DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
    {
        // No escribimos por fuera del buffer (límite de la OLED)
        return;
    }

    // Verificamos si estamos en modo invertido
    if (SSD1306.Inverted)
    {
        color = (SSD1306_COLOR)!color;
    }

    // Pintamos con el color correcto (en realidad estamos escribiendo en el buffer GDDRAM)
    if (color == White)
    {
    	/* Operación OR bitwise sobre el arreglo, buscamos la posción correspondiente a la columna
    	 * y sumamos el cociente de la fila divido 8 (por ser cada 8 bits como se divide) que fue
    	 * multiplicado por el ancho total, así se obtiene la posición*/
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    }
    else
    {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}


/** Función para dibujar un carácter simple en la OLED */
char ssd1306WriteChar(char character, FontDef font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // Verificamos el espacio disponible según la posición XY y el tamaño
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + font.FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + font.FontHeight))
    {
        // No hay espacio suficiente, retornamos un cero
        return 0;
    }

    // Lógica para llevar la fuente al buffer del Display
    for (i = 0; i < font.FontHeight; i++)
    {
        b = font.data[(character - 32) * font.FontHeight + i];
        for (j = 0; j < font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                ssd1306DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)color);
            }
            else
            {
                ssd1306DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // Actualizamos las coordenadas
    SSD1306.CurrentX += font.FontWidth;

    // Retornamos el carácter escrito, para validaciones
    return character;
}


/** Función para escribir toda una cadena de carácteres */
void ssd1306WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
    /* Ciclo para escribir hasta el carácter nulo,
     * nótese que se emplea el operador * que es el
     * de desreferencia, con esto obtenemos el valor
     * al que apunta el puntero al arreglo tipo char,
     * y como sabemos, en C todo lo que sea diferente
     * de 0 (carácter nulo) es como un 1 */
    while (*str)
    {
    	/* Usamos la función de escribir un carácter simple, si el carácter es válido,
    	 * lo dibuja y retorna, para hacer la comparación */
        if (ssd1306WriteChar(*str, Font, color) != *str)
        {
            // El carácter no puede ser escrito
        	__NOP();
        }

        // Aumentamos en +1 el puntero al string
        str++;
    }
}


/** Función para invertir los colores del display */
void ssd1306InvertColors(void)
{
    SSD1306.Inverted = !SSD1306.Inverted;
}

/** Función para establecer posición del cursor */
void ssd1306SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}
