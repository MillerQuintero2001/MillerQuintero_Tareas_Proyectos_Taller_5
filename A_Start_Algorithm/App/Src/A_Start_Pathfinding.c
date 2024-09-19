/*
 ============================================================================
 Name        : A_star_pathfinding.c
 Author      : namontoy
 Version     :
 Copyright   : Programming to learn A*
 Description : Learning how to write my first A*...
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "Single_Cell.h"

#include "arm_math.h"

#include "PLLDriver.h"
#include "GPIOxDriver.h"
#include "BasicTimer.h"
#include "USARTxDriver.h"


/* Definición de los handlers necesarios de los periféricos del MCU */

// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};				// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};				// Pin de recepción de datos
USART_Handler_t usartCmd =  {0};				// Comunicación serial
uint8_t usartData = 0; 						// Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; 						// Buffer de datos como un arreglo de caracteres

#define MAP_GRID_ROWS		8
#define MAP_GRID_COLS		10
#define MAP_CELLS_COUNT		(MAP_GRID_ROWS * MAP_GRID_COLS)
#define ROW_MAP_DATA_LEN	20
#define MAX_NEIGHBOURS		8

/* Elementos del sistema */
uint8_t grid_rows = MAP_GRID_ROWS;
uint8_t grid_cols = MAP_GRID_COLS;

Cell_map_t grid_map_cells[MAP_CELLS_COUNT] = {0};
Cell_map_t empty_cell = {0};
Cell_map_t *ptr_current_cell;
Cell_map_t *ptr_goal_cell;
Cell_map_t *ptr_start_cell;

Cell_map_t* open_list[MAP_CELLS_COUNT];
uint8_t open_list_index = 0;

Cell_map_t* closed_list[MAP_CELLS_COUNT];
uint8_t closed_list_index = 0;

char* map_string[MAP_GRID_ROWS];

/* Prototipos de las funciones del main */

// Related with A* pathfinding
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void print_cells_info(Cell_map_t *cellArray);
void print_single_cell_info(Cell_map_t *singleCell);
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map);
Cell_map_t* get_cell_start(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows);
void addTo_open_list(Cell_map_t *working_cell);
void identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check);
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
uint16_t get_G_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell);
uint16_t get_F_cost(Cell_map_t *working_cell);
void update_F_cost(Cell_map_t *working_cell);
void order_open_list(uint8_t index_last);
void init_empty_openlist(Cell_map_t* empty_cell);
uint8_t get_count_item_open_list(void);
void removeFrom_open_list(Cell_map_t *working_cell);
Cell_map_t* get_next_item(void);
void print_path(Cell_map_t *working_cell);
void A_star_algorithm(void);

// Related with peripherals
void initSystem(void);

/** Función que ejecuta paso a paso el algoritmo A* */
int main(void) {

	initSystem();
	/* prints !!!Hello World!!! */
	//printf("!!!Hello World!!!\n");
	writeMsg(&usartCmd, "!!!Hello World!!!\n");

	//printf("A* pathfinding\n");
	writeMsg(&usartCmd, "A* pathfinding\n");

	/* 1. Crea todas las celdas vacias */
	init_empty_grid_map(grid_cols, grid_rows, grid_map_cells);

	/* Estoy creando un "objeto" tipo cell_amp, el cual estará siempre vacio, para pruebas */
	empty_cell = create_cell(11,11);

	/* Inicializo todo el arreglo de punteros del open_list apuntando a la empty_cell */
	init_empty_openlist(&empty_cell);


	/* 2. Llena el mapa con la descripcion para el ejercicio
	 * En el caso del MCU, el string se debe recibir por el puerto serial,
	 * al igual que la indicación de a qué fila del mapa corresponde
	 * */
	populate_grid(". . . . . G . . . . ", 0, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 1, grid_map_cells);
	populate_grid(". . # # # # # # . . ", 2, grid_map_cells);
	populate_grid(". # # . . . . # # . ", 3, grid_map_cells);
	populate_grid(". # . . . . . . # . ", 4, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 5, grid_map_cells);
	populate_grid(". . . . . . . . . . ", 6, grid_map_cells);
	populate_grid(". . . . S . . . . . ", 7, grid_map_cells);

	// Imprime la informacion más simple de todas las celdas del grid
	/* YA FUNCIONA ESTA FUNCIÓN */
	//print_cells_info(grid_map_cells);

	/* 3. Imprime en pantalla el mapa que se envió a por los comandos del USART,
	 * para verificar que en efecto el sistema tiene el mapa correcto, o que el mapa
	 * fue correctamente recibido
	 * */
	/* YA FUNCIONA ESTA FUNCIÓN */
	print_map(grid_cols, grid_rows, grid_map_cells);

	/* 4. Ejecución del algoritmo A*
	 * Al llamar esta funcion, que basicamente ejecuta el pseudocodigo, se debe
	 * obtener al final la solución para la ruta.
	 * */
//	A_star_algorithm();

/* == Espacio para pruebas simples... == */
//	/* Incluye la celda "Start" en la lista open_list, utilizando para esto al puntero "current_cell" */
	ptr_start_cell = get_cell_start(grid_map_cells, grid_cols, grid_rows);
	addTo_open_list(ptr_start_cell);
//
//	print_single_cell_info(ptr_current_cell);
//	identify_cell_neighbours(grid_map_cells, &grid_map_cells[66]);
//	update_H_cost(ptr_goal_cell, &grid_map_cells[57]);
//	update_G_cost(ptr_current_cell, &grid_map_cells[57]);
//	update_F_cost(&grid_map_cells[57]);
//
//	addTo_open_list(&grid_map_cells[57]);
//	removeFrom_open_list(ptr_current_cell);
/* == Final de las pruebas simples... == */

//	printf("END\n");
	writeMsg(&usartCmd, "\nEND\n");

	return 0;
}

/**
 * Esta función se encarga de devolver la distancia "directa" entre la celda X en la que se
 * está trabajando y la celda definida como "Goal". En este primer caso será una heurística
 * del tipo Pitagoras h = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ).
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los cálculos con las funciones de CMSIS que con las
 * de C estándar ya que son mucho más eficientes.
 * */
uint16_t get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell){

    // Escribir código...
	float32_t ptrSource[2] = {0};
	ptrSource[0] = (float32_t)(goal_cell->coordinateX - working_cell->coordinateX);
	ptrSource[1] = (float32_t)(goal_cell->coordinateY - working_cell->coordinateY);
	float32_t inputSqrt = 0.00f;
	float32_t output = 0.00f;
	arm_power_f32(ptrSource, 2, &inputSqrt);
	//const float32_t input = pow((goal_cell->coordinateX - working_cell->coordinateX),2)+pow((goal_cell->coordinateY - working_cell->coordinateY),2);

	if(arm_sqrt_f32(inputSqrt, &output) == ARM_MATH_SUCCESS){
		uint16_t aux_result = (uint16_t)output;
		return aux_result;
	}

	else{
		printf("Error updating the H cost.\n");
		return 0;
	}
}

/*
 * Actualiza el valor del H_cost de la celda que se pasa como parametro, con respecto
 * a la celda "Goal" que se pasa tambien como parametro
 * */
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell){
	working_cell->Hcost = get_H_cost(goal_cell, working_cell);
}

/**
 * Esta función se encarga de devolver la distancia "directa" entre la celda X y su siguiente
 * vecino, lo cual es el costo de viaje entre ambos. En este primer caso será una heurística
 * del tipo Pitagoras G_cost_cell = sqrt( (X1 - X2)^2 + (Y1 - Y2)^2 ) + G_cost_vecino.
 *
 * Con este valor debera ser suficiente.
 *
 * NOTA: Observar que para este calculo es preferible tener la unidad FPU activa en el MCU
 * además de ser mas conveniente hacer los calculos con las funciones de CMSIS que con las
 * de C standar ya que son mucho mas eficientes.
 * */
uint16_t get_G_cost(Cell_map_t *neighbour_cell, Cell_map_t *working_cell){

	// Escribir código...
	float32_t output = 0.00f;
	const float32_t input = (float32_t)(pow((neighbour_cell->coordinateX - working_cell->coordinateX),2)+pow((neighbour_cell->coordinateY - working_cell->coordinateY),2));
	if(arm_sqrt_f32(input, &output) == ARM_MATH_SUCCESS){
		uint16_t aux_result = (uint16_t)output + (neighbour_cell->Gcost);
		return aux_result;
	}
	else{
		printf("Error updating the G cost.\n");
		return 0;
	}
}

/*
 * Actualiza el valor del G_cost de la celda que se pasa como parámetro, con respecto
 * a la celda vecina que se pasa tambien como parámetro
 *
 * Para el caso especial de la celda "start" el G_cost siempre es 0 (ya que desde allí se inicia).
 * Esta caracteristica debe quedar incluida en esta función.
 * */
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell){
	// Check if the working cell is the start cell, in that case his G cost is zero and doesn't have any parent
    if(working_cell->typeOfCell == 'S'){
    	working_cell->Gcost = 0;
    	working_cell->ptr_parent = NULL;
    }
    // Otherwise, we update his G cost and his parent
    else{
    	working_cell->Gcost = get_G_cost(parent_cell, working_cell);
    	working_cell->ptr_parent = parent_cell;
    }
}


/**
 * Esta función se encarga de retornar el valor de la función de costo completa de la celda que se
 * está analizando.
 *
 * F = G_cost + H_cost
 *
 * */
uint16_t get_F_cost(Cell_map_t *working_cell){
	return (working_cell->Gcost + working_cell->Hcost);
}

/** Actualiza el valor F_cost de la celda que se pasa como parámetro */
void update_F_cost(Cell_map_t *working_cell){
	working_cell->Fcost = get_F_cost(working_cell);
}


/**
 * Esta función busca cual es la celda que esta designada como el inicio (start)
 * y se apunta a ella con el puntero "current_cell" para comenzar a hacer el análisis,
 * además se le organizan los parámetros G, H y F adecuadamente...
 *
 * Este es de los primeros paso del pseudocódigo
 *
 * Se imprime un error si la celda no es encontrada en el arreglo.
 *
 * */
Cell_map_t* get_cell_start(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows) {

    // Escribir código...
	Cell_map_t *ptr_search = grid_map;
	uint8_t i = 0;
	while((ptr_search->typeOfCell != 'S')||(i < gridCols*gridRows)){
		i++;
		ptr_search = grid_map + i;
	}
	// The start cell has been found
	if(i < gridCols*gridRows){
		ptr_search->Gcost = 0;
		ptr_search->Hcost = get_H_cost(ptr_goal_cell, ptr_search);
		ptr_search->Fcost = get_F_cost(ptr_search);
		ptr_search->ptr_parent = NULL;
	    // Esta función retorna un valor...
		return ptr_search;
	}
	else{
		//printf("Start cell hasn't been found.\n");
		writeMsg(&usartCmd, "Start cell hasn't been found.\n");
		return NULL;
	}
}

/**
 * Esta función busca cual es la celda que esta designada como el "objetivo" (Goal),
 * además se le organizan los parametros G, H y F adecuadamente...
 * */
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols,uint8_t gridRows) {
    // Escribir código...
	Cell_map_t *ptr_search = grid_map;
	uint8_t i = 0;
	while((ptr_search->typeOfCell != 'G')||(i < gridCols*gridRows)){
		i++;
		ptr_search = grid_map + i;
	}
	//If goal cell has been found
	if(i < gridCols*gridRows){
		ptr_search->Gcost = get_G_cost(ptr_search->ptr_parent, ptr_search);
		ptr_search->Hcost = 0;
		ptr_search->Fcost = get_F_cost(ptr_search);
	    // Esta función retorna un valor...
		return ptr_search;
	}
	else{
		//printf("Goal cell hasn't been found.\n");
		writeMsg(&usartCmd, "Goal cell hasn't been found.\n");
		return NULL;
	}
}

/**
 * Esta funcion debe organizar los elementos en la lista en orden, y el criterio es
 * primero F_cost y luego H_cost, si ambos son igual, se deja quien estaba primero.
 *
 * Este sistema implica que se debe poder mover hacia arriba y hacia abajo los elementos
 * dependiendo de la posición que se le deba dar.
 *
 * Esta funcion utiliza un index, el cual solo puede ser modificado desde esta función
 * ya que este index se encarga de "mover" arriba y abajo el indicador de cuantos
 * elementos se encuentran abiertos.
 *
 * Además, tambien se tiene un puntero auxiliar, para poder comparar entre elementos y
 * saber quien debe ir en que posicion.
 *
 * Recordar que el elemento "open_list" es un arreglo de punteros de tipo Cell_map_t,
 * o sea esta lista NO almacena de nuevo las celdas, solo almacena la referencia a ellas
 * en el orden adecuado... punteros, queridos punteros...
 * */
void addTo_open_list(Cell_map_t *working_cell){

    // Escribir código...

}

/*
 * Entrega el puntero al elemento mas arriba de la lista open_list
 * */
Cell_map_t* get_next_item(void){
    return *open_list; // Escribir código...
}

/**
 * Esta función identifica cuantos elementos activos hay en la lista open_list */
uint8_t get_count_item_open_list(void) {
	Cell_map_t *ptr_element = *open_list;
	uint8_t counter = 0;
	while(ptr_element->typeOfCell != 'e'){
		counter++;
		// Sum 1 to the pointer to desplace at the next element
		ptr_element++;
	}
    // Esta función retorna un valor...
	return counter;
}
/*
 * Remueve el elemento indicado por el puntero working_cell, que además deberia ser el elemento
 * más arriba en la lista de open_list
 * */
void removeFrom_open_list(Cell_map_t *working_cell){
	Cell_map_t *ptr_element = *open_list;
	uint8_t i = 0;
	while(ptr_element->typeOfCell != 'e'){
		*(open_list+i) = *(open_list+i+1);
		i++;
		ptr_element = *(open_list+i);
	}
}

/*
 * Agrega el elemento pasado como parámetro (puntero) en la lista de elementos cerrados
 * */
void addTo_closed_list(Cell_map_t *working_cell){

    // Escribir código...

}

/**
 * Carga la lista Open con el valor conocido de una celda vacia, y que además está fuera de las
 * fronteras del mapa. Esto lo hago con el objetivo de saber hasta donde debo recorrer el
 * arreglo open_list, de forma que pueda saber donde ya no hay nuevos elementos.

 * Es como la idea del caracter "NULL" en los strings...
 * */
void init_empty_openlist(Cell_map_t* empty_cell){
	open_list[0] = empty_cell;
}

/**
 * Función encargada de ordenar el arreglo OpenList con respecto a los valores F y H
 * de cada elemento al que se apunta en la lista.
 *
 * Esta función es "llamada" desde la función "addTo_open_list(...)"
 * */
void order_open_list(uint8_t index_last){

    // Escribir código...

}

/**
 * Un paso fundamental en el Algoritmo es identificar a los vecinos alrededor de la celda
 * que se encuentra activa en ese momento.
 * En este caso, deseo utilizar la posición x,y como guia para encontrar los vecinos alrededor
 * de la celda[IDx] selecionada. Para esto vale la pena tener claro que los vecinos alrededor
 * de la celda son 8:
 * 0. Diagonal left above
 * 1. Above
 * 2. Diagonal right above
 * 3. Left
 * 4. Right
 * 5. Diagonal left below
 * 6. Below
 * 7. Diagonal right below
 *
 * Viendo la celda X en una posición general en el mapa:
 *						0 1 2
 * 						3 X 4
 * 						5 6 7
 * Se obtiene así entonces los vecinos de la celda activa.
 * */
void identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check){

	// First, we found the index (position in the array) of the pointer "cell_to_check"
	int8_t index = 0;
	while((grid_map+index) != cell_to_check){
		index++;
	}

	// This part is only correct with cells that are not on the border, in the next code block it will be corrected
	int8_t neighbourIndex = 0;

	// 0. Establish diagonal left above neighbour
	neighbourIndex = index - (MAP_GRID_COLS + 1);
	cell_to_check->neighbours[0] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex) : NULL;

	// 1. Establish above neighbour
	neighbourIndex = index - MAP_GRID_COLS;
	cell_to_check->neighbours[1] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 2. Establish diagonal right above neighbour
	neighbourIndex = index - (MAP_GRID_COLS - 1);
	cell_to_check->neighbours[2] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 3. Establish left neighbour
	neighbourIndex = index - 1;
	cell_to_check->neighbours[3] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 4. Establish right neighbour
	neighbourIndex = index + 1;
	cell_to_check->neighbours[4] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 5. Establish diagonal left below neighbour
	neighbourIndex = index + (MAP_GRID_COLS - 1);
	cell_to_check->neighbours[5] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 6. Establish below neighbour
	neighbourIndex = index + MAP_GRID_COLS;
	cell_to_check->neighbours[6] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);

	// 7. Establish diagonal left below neighbour
	neighbourIndex = index + (MAP_GRID_COLS + 1);
	cell_to_check->neighbours[7] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):(NULL);


	/* Now, it mends mistakes in the case of border cells */

	// Check the X coordinate to establish in a secure way the NULL neighbours
    if(cell_to_check->coordinateX == 0){
    	cell_to_check->neighbours[0] = NULL;
    	cell_to_check->neighbours[3] = NULL;
    	cell_to_check->neighbours[5] = NULL;
    }
    else if(cell_to_check->coordinateX == MAP_GRID_COLS){
    	cell_to_check->neighbours[2] = NULL;
    	cell_to_check->neighbours[4] = NULL;
    	cell_to_check->neighbours[7] = NULL;
    }

    // Check the Y coordinate to establish in a secure way the NULL neighbours
    if(cell_to_check->coordinateY == 0){
    	cell_to_check->neighbours[0] = NULL;
    	cell_to_check->neighbours[1] = NULL;
    	cell_to_check->neighbours[2] = NULL;
    }
    else if(cell_to_check->coordinateY == MAP_GRID_ROWS){
    	cell_to_check->neighbours[5] = NULL;
    	cell_to_check->neighbours[6] = NULL;
    	cell_to_check->neighbours[7] = NULL;
    }

}

/**
 * Esta función debería utilizar la información entregada de alguna manera,
 * como por ejemplo un string con todos los datos o quizas en un JSON.
 *
 * En la primer idea se emplea el envio de datos línea a línea (string con los
 * caracteres de cada fila (row) indicando además cual es la línea que se desea
 * actualizar (0 a 7).
 * */
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map){
    // Escribir código...
	uint8_t k = 0;
	uint8_t index = 0;
	while(k < (MAP_GRID_COLS*2)){
		if(*(row_data + k) != ' '){
			(grid_map + index + grid_row*MAP_GRID_COLS)->typeOfCell = *(row_data + k);
			index++;
		}
		else{
			__NOP();
		}
		k++;
	}
}

/**
 * Esta función se encarga de crear un grid_map vacio, solo con los
 * elementos que corresponden creados por defecto (vacios, sin costos y
 * sin indicar de qué tipo son)
 */
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray){
    // Escribir código...
	uint8_t index = 0;
	for(uint8_t i = 0; i < gridRows; i++){
		for(uint8_t j = 0; j < gridCols; j++){
			*(cellArray + index) = create_cell(j, i);
			index++;
		}
	}
}

/**
 * Me imprime la información de una celda X, para verificar la información que ella contiene
 * y así poder mirar si todo esta funcionando correctamente.
 *
 * */
void print_single_cell_info(Cell_map_t *singleCell) {
//	printf("Cell's identification is:\n"
//			"Columna: %c\n"
//			"Fila: %hu\n", singleCell->identifier[0], singleCell->identifier[1]);
//	printf("The X & Y coordinates are: X = %hu, Y = %hu\n", singleCell->coordinateX, singleCell->coordinateY);
//	printf("The type of cell is: %c.\n",singleCell->typeOfCell);
//
//	printf("The G cost is: %hu\n", singleCell->Gcost);
//	printf("The H cost is: %hu\n", singleCell->Hcost);
//	printf("The F cost is: %hu\n", singleCell->Fcost);
//
//	printf("The parent's identifier is: %c%hu.\n\n", singleCell->ptr_parent->identifier[0], singleCell->ptr_parent->identifier[1]);

	// With the MCU
	sprintf(bufferMsg,"Cell's identification is:\n"
			"Columna: %c\n"
			"Fila: %hu\n", singleCell->identifier[0], singleCell->identifier[1]);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The X & Y coordinates are: X = %.3f, Y = %.3f\n", singleCell->coordinateX, singleCell->coordinateY);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The type of cell is: %c.\n",singleCell->typeOfCell);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The G cost is: %.3f\n", singleCell->Gcost);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The H cost is: %.3f\n", singleCell->Hcost);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The F cost is: %.3f\n", singleCell->Fcost);
	writeMsg(&usartCmd, bufferMsg);

	sprintf(bufferMsg,"The parent's identifier is: %c%hu.\n\n", singleCell->ptr_parent->identifier[0], singleCell->ptr_parent->identifier[1]);
	writeMsg(&usartCmd, bufferMsg);
}

/**
 * Imprime la información de todas las celdas (en general del mapa)
 * */
void print_cells_info(Cell_map_t *cellArray){
    for(uint8_t i = 0; i < MAP_CELLS_COUNT; i++){
    	print_single_cell_info(cellArray+i);
    }
}


/**
 * Esta función se encarga de imprimir el mapa con sus características
 * . default, empty cell
 * # obstacle
 * G Goal
 * S start
 * o Open
 * c Closed
 * El mapa se escribe fila a fila, por lo cual una idea es hacer fija la fila
 * e imprimir los elementos (columnas) de cada fila
 * */
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray) {
	for(uint8_t j = 0; j < gridCols*gridRows; j++){
		if((j%(gridCols)) == 0){
			//printf("\n");
			writeChar(&usartCmd, '\n');
		}
		else{
			__NOP();
		}
		//printf("%c ", (cellArray+j)->typeOfCell);
		char c = (cellArray+j)->typeOfCell;
		writeChar(&usartCmd, c);
		writeChar(&usartCmd, ' ');
	}
}


/**
 * Esta función debe recibir como parámetro el puntero a la ultima celda, que debe ser la "goal"
 * Con la información del ID_parent y entendiendo que todas las celdas visitadas
 * ya deben estar en el arreglo closed_list, se debe poder buscar la ruta y presentarla en pantalla
 *
 * */
void print_path(Cell_map_t *working_cell){
    // Escribir código...
	Cell_map_t *ptr_backward_path = working_cell;
	printf("The path is:\n");
	while(ptr_backward_path != NULL){
		printf("%c%hu", working_cell->identifier[0], working_cell->identifier[1]);
		ptr_backward_path = ptr_backward_path->ptr_parent;
	}
}

/**
 * == A* algorithm ==
 * Aca es donde se hace toda la magia, todo lo de arriba es necesario, pero
 * el algoritmo se ejecuta es en esta funcion.
 *
 * Esta función es la descripción literal del pseudocodigo...
  * */
void A_star_algorithm(void){

// Escribir código...
}



/* ---			Funciones de los periféricos del MCU			--- */

/** Función encargada de iniciar hardware */
void initSystem(void){

	configPLL(100);

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* GPIO y Timer del Blinky Led de Estado */
	handlerBlinkyPin.pGPIOx								= GPIOA;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerBlinkyPin);
	// Pongo estado en alto
	GPIO_WritePin(&handlerBlinkyPin, SET);
	// Atributos para el Timer 2 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM2;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/


	/* Configuración de pines para el USART2 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_2;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_3;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usartCmd.ptrUSARTx							= USART2;
	usartCmd.USART_Config.USART_baudrate 		= USART_BAUDRATE_115200;
	usartCmd.USART_Config.USART_datasize		= USART_DATASIZE_8BIT;
	usartCmd.USART_Config.USART_parity			= USART_PARITY_NONE;
	usartCmd.USART_Config.USART_stopbits		= USART_STOPBIT_1;
	usartCmd.USART_Config.USART_mode			= USART_MODE_RXTX;
	usartCmd.USART_Config.USART_enableIntRX		= USART_RX_INTERRUP_ENABLE;
	usartCmd.USART_Config.USART_enableIntTX		= USART_TX_INTERRUP_DISABLE;
	USART_Config(&usartCmd);
}


/** Interrupción del timer blinky LED*/
void BasicTimer2_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin); //Cambio el estado del LED PA5
}


/** Interrupción del USART2 */
void usart2Rx_Callback(void){
	usartData = getRxData();	// Pongo en alto la variable bandera del USART2 para el main
}


