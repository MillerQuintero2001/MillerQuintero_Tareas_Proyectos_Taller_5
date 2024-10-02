/*
 ============================================================================
 Name        : A_Star_By_Commands.c
 Author      : MillerQuintero2001
 Version     : 3.0
 Description : A* algorithm with map send by USART commands
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
#include "MotorDriver.h"


#define MAP_GRID_ROWS		8
#define MAP_GRID_COLS		10
#define MAP_CELLS_COUNT		(MAP_GRID_ROWS * MAP_GRID_COLS)
#define ROW_MAP_DATA_LEN	20
#define MAX_NEIGHBOURS		8

/* Definición de los handlers necesarios de los periféricos del MCU */
// Elementos para el Blinky LED
GPIO_Handler_t handlerBlinkyPin = 			{0}; // LED de estado del Pin A5
BasicTimer_Handler_t handlerBlinkyTimer = 	{0}; // Timer del LED de estado

// Elementos para hacer la comunicación serial
GPIO_Handler_t handlerPinTX = {0};										// Pin de transmisión de datos
GPIO_Handler_t handlerPinRX = {0};										// Pin de recepción de datos
USART_Handler_t usartCmd =  {0};										// Comunicación serial
uint8_t usartData = 0; 													// Variable en la que se guarda el dato transmitido
char bufferMsg[64] = {0}; 												// Buffer de datos como un arreglo de caracteres
char bufferReception[MAP_GRID_ROWS][ROW_MAP_DATA_LEN + 1] = {0};		// Buffer para guardar caracteres ingresados para el mapa
uint8_t counterReception = 0;											// Contador de carácteres para la recepción


/* Variables y elementos para A* */
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

char map_string[MAP_GRID_ROWS][ROW_MAP_DATA_LEN + 1];
bool flagMap = false;
uint8_t line = 0;

int8_t oppyIndicator = -1;
bool flagDoIt = false;

/* Functions Proto-types */

// Related with A* pathfinding
void init_empty_grid_map(uint8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);			// Ready
void print_cells_info(Cell_map_t *cellArray);													// Ready
void print_single_cell_info(Cell_map_t *singleCell);											// Ready
void print_map(int8_t gridCols, uint8_t gridRows, Cell_map_t *cellArray);						// Ready
void populate_grid(char *row_data, uint8_t grid_row, Cell_map_t *grid_map);						// Ready
Cell_map_t* get_cell_start(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);			// Ready
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows);			// Ready
void addTo_open_list(Cell_map_t *working_cell);													// Ready
void identify_cell_neighbours(Cell_map_t *grid_map, Cell_map_t *cell_to_check);					// Ready
float get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);								// Ready
void update_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell);							// Ready
float get_G_cost(Cell_map_t *neighbour_cell, Cell_map_t *working_cell);							// Ready
void update_G_cost(Cell_map_t *parent_cell, Cell_map_t *working_cell);							// Ready
float get_F_cost(Cell_map_t *working_cell);														// Ready
void update_F_cost(Cell_map_t *working_cell);													// Ready
void order_open_list(uint8_t index_last);														// Need more harder and extreme tests
void init_empty_openlist(Cell_map_t* empty_cell);												// Ready
uint8_t get_count_item_open_list(void);															// Ready
void removeFrom_open_list(Cell_map_t *working_cell);											// Ready
Cell_map_t* get_next_item(void);																// Ready
void addTo_closed_list(Cell_map_t *working_cell);												// Wrote but not checked
void print_path(Cell_map_t *working_cell);														// Ready
int8_t A_star_algorithm(void);

// Related with peripherals
void initSystem(void);
void catchMap(void);
void oppyPath(void);


/** Función que ejecuta paso a paso el algoritmo A* */
int main(void) {

	initSystem();

	while(1){
		catchMap();

		if(flagMap){

			//printf("A* pathfinding\n");
			writeMsg(&usartCmd, "\nA* pathfinding.\n");

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
			for(uint8_t j = 0; j < MAP_GRID_ROWS; j++){
				populate_grid(map_string[j], j, grid_map_cells);
			}

			/* 3. Imprime en pantalla el mapa que se envió a por los comandos del USART,
			 * para verificar que en efecto el sistema tiene el mapa correcto, o que el mapa
			 * fue correctamente recibido */
			ptr_goal_cell = get_cell_goal(grid_map_cells, grid_cols, grid_rows);
			ptr_start_cell = ptr_current_cell = get_cell_start(grid_map_cells, grid_cols, grid_rows);
			print_map(grid_cols, grid_rows, grid_map_cells);

			/* 4. Preparativos previos al A*, son:
			 * 	- Agregar el puntero a la celda de inicio a la Open List
			 * 	- Definir el H cost en cada una de las celdas del mapa ya que esta medida si es absoluta
			 * 	- En el mismo ciclo de definir el H cost, identificar a los vecinos */
			addTo_open_list(ptr_current_cell);
			for(uint8_t k = 0; k < MAP_CELLS_COUNT; k++){
				update_H_cost(ptr_goal_cell, (grid_map_cells + k));
				identify_cell_neighbours(grid_map_cells, (grid_map_cells+k));
			}

			/* 5. Ejecución del algoritmo A*
			 * Al llamar esta funcion, que basicamente ejecuta el pseudocodigo, se debe
			 * obtener al final la solución para la ruta.
			 * */
			oppyIndicator = A_star_algorithm();

			print_map(grid_cols, grid_rows, grid_map_cells);

		//	printf("END\n");
			writeMsg(&usartCmd, "\nEND\n");
			flagMap = false;
		}


		else{
			__NOP();
		}

		if(flagDoIt){
			if(oppyIndicator == -1){
				writeMsg(&usartCmd, "\nThere's no path found, then, the Oppy remains still.\n");
				flagDoIt = false;
			}
			else{
				oppyPath();
				oppyIndicator = -1;
				flagDoIt = false;
			}
		}
		else{
			__NOP();
		}

	}
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
float get_H_cost(Cell_map_t *goal_cell, Cell_map_t *working_cell){
	float32_t ptrSource[2] = {0};
	ptrSource[0] = (float32_t)(goal_cell->coordinateX - working_cell->coordinateX);
	ptrSource[1] = (float32_t)(goal_cell->coordinateY - working_cell->coordinateY);
	float32_t inputSqrt = 0.00f;
	float32_t output = 0.00f;
	arm_power_f32(ptrSource, 2, &inputSqrt);
	if(arm_sqrt_f32(inputSqrt, &output) == ARM_MATH_SUCCESS){
		float aux_result = (float)output;
		return aux_result;
	}

	else{
		//printf("Error updating the H cost.\n");
		writeMsg(&usartCmd, "Error updating the H cost.\n");
		return 0.00f;
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
float get_G_cost(Cell_map_t *neighbour_cell, Cell_map_t *working_cell){

	float32_t ptrSource[2] = {0};
	ptrSource[0] = (float32_t)(neighbour_cell->coordinateX - working_cell->coordinateX);
	ptrSource[1] = (float32_t)(neighbour_cell->coordinateY - working_cell->coordinateY);
	float32_t inputSqrt = 0.00f;
	float32_t output = 0.00f;
	arm_power_f32(ptrSource, 2, &inputSqrt);
	if(arm_sqrt_f32(inputSqrt, &output) == ARM_MATH_SUCCESS){
		float aux_result = (float)output + (neighbour_cell->Gcost);
		return aux_result;
	}
	else{
		//printf("Error updating the G cost.\n");
		writeMsg(&usartCmd, "Error updating the G cost.\n");
		return 0.00f;
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
    	working_cell->Gcost = 0.00f;
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
float get_F_cost(Cell_map_t *working_cell){
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
	while((ptr_search->typeOfCell != 'S')&&(i < gridCols*gridRows)){
		ptr_search++;
		i++;
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
Cell_map_t* get_cell_goal(Cell_map_t *grid_map, uint8_t gridCols, uint8_t gridRows) {
    // Escribir código...
	Cell_map_t *ptr_search = grid_map;
	uint8_t i = 0;
	while(((ptr_search->typeOfCell) != 'G')&&(i < MAP_CELLS_COUNT)){
		ptr_search++;
		i++;
	}
	//If goal cell has been found
	if(i < gridCols*gridRows){
		ptr_search->Gcost = 25000.00f;
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
	working_cell->typeOfCell = 'o';
	// To introduce the new element to the list, we will first move the existing ones
	for(uint8_t i = open_list_index; i > 0; i--){
		*(open_list + i) = *(open_list + i - 1);
	}
	// Now, the first element is the element to introduce
	*open_list = working_cell;

	// Sum 1 to the open list index
	open_list_index++;

	// Order the open list
	order_open_list(open_list_index);
}

/** Entrega el puntero al elemento mas arriba de la lista open_list */
Cell_map_t* get_next_item(void){
    return *open_list;
}

/** Esta función identifica cuantos elementos activos hay en la lista open_list sin contar el elemento vacío del final */
uint8_t get_count_item_open_list(void) {
	Cell_map_t *ptr_element = *open_list;
	uint8_t counter = 0;
	while(ptr_element->typeOfCell != 'e'){
		counter++;
		// Sum 1 to the pointer to displace at the next element
		ptr_element = *(open_list + counter);
	}
	return counter;
}
/** Remueve el elemento indicado por el puntero working_cell, que además debería ser el elemento más arriba en la lista de open_list */
void removeFrom_open_list(Cell_map_t *working_cell){
	Cell_map_t *ptr_element = *open_list;
	uint8_t i = 0;
	// As long as the ptr element is different from the final empty cell or is not the element being searched for
	while((ptr_element->typeOfCell != 'e')&&(ptr_element != working_cell)){
		i++;
		ptr_element = *(open_list+i);
	}
	if(ptr_element == working_cell){
		while(ptr_element->typeOfCell != 'e'){
			*(open_list+i) = *(open_list+i+1);
			i++;
			ptr_element = *(open_list+i);
		}
		*(open_list+i) = NULL;
	}
	else{
		//printf("Error: The parameter indicated in 'removeFrom_open_list' function doesn't exists.\n");
		writeMsg(&usartCmd, "Error: The parameter indicated in 'removeFrom_open_list' function doesn't exists.\n");
	}
}

/** Agrega el elemento pasado como parámetro (puntero) en la lista de elementos cerrados */
void addTo_closed_list(Cell_map_t *working_cell){
	working_cell->typeOfCell = 'c';
	closed_list[closed_list_index] = working_cell;
	closed_list_index++;
}

/**
 * Carga la lista Open con el valor conocido de una celda vacia, y que además está fuera de las
 * fronteras del mapa. Esto lo hago con el objetivo de saber hasta donde debo recorrer el
 * arreglo open_list, de forma que pueda saber donde ya no hay nuevos elementos.

 * Es como la idea del caracter "NULL" en los strings...
 * */
void init_empty_openlist(Cell_map_t* empty_cell){
	open_list[0] = empty_cell;
	open_list_index++;
}

/**
 * Función encargada de ordenar el arreglo OpenList con respecto a los valores F y H
 * de cada elemento al que se apunta en la lista.
 *
 * Esta función es "llamada" desde la función "addTo_open_list(...)"
 * */
void order_open_list(uint8_t index_last){
	Cell_map_t *ptrAux = *(open_list);
	Cell_map_t *ptrCopy = NULL;
	for(uint8_t i = 0; i < index_last; i++){
		if (ptrAux->Fcost > (*(open_list + i + 1))->Fcost){
			ptrCopy = *(open_list+ i + 1);
			*(open_list + i + 1) = ptrAux;
			*(open_list + i) = ptrCopy;
		}
		else if (ptrAux->Fcost == (*(open_list + i + 1))->Fcost){
			uint8_t k = 0;
			while(ptrAux->Fcost == (*(open_list + i + k + 1))->Fcost){
				// If the H cost is greater or equal, then, swap the pointer's position
				if (ptrAux->Hcost >= (*(open_list + i + k + 1))->Hcost){
					ptrCopy = *(open_list+ i + k + 1);
					*(open_list + i + k + 1) = ptrAux;
					*(open_list + i + k) = ptrCopy;
				}
				// Otherwise, H cost is lower, so it goes first, and there is nothing to do, for that reason, the loop is break
				else{
					break;
				}
				k++;
			}
		}
		// Then, there is nothing to do and is time to break the loop to finish with it
		else{
			break;
		}
	}
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
	cell_to_check->neighbours[0] = ((neighbourIndex >= 0)||(neighbourIndex < MAP_CELLS_COUNT)) ? (grid_map+neighbourIndex):NULL;

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
    else if(cell_to_check->coordinateX == FLOOR_TILE_SIZE*(MAP_GRID_COLS-1)){
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
    else if(cell_to_check->coordinateY == FLOOR_TILE_SIZE*(MAP_GRID_ROWS-1)){
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
	uint8_t k = 0;
	uint8_t index = 0;
	while(k < ROW_MAP_DATA_LEN){
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
//	printf("\nCell's identification is:\n"
//			"Columna: %c\n"
//			"Fila: %hu\n", singleCell->identifier[0], singleCell->identifier[1]);
//	printf("The X & Y coordinates are: X = %hu, Y = %hu\n", singleCell->coordinateX, singleCell->coordinateY);
//	printf("The type of cell is: %c.\n",singleCell->typeOfCell);
//
//	printf("The G cost is: %hu\n", singleCell->Gcost);
//	printf("The H cost is: %hu\n", singleCell->Hcost);
//	printf("The F cost is: %hu\n", singleCell->Fcost);
//
//	if(singleCell->ptr_parent == NULL){
//		printf("The parent's identifier is: NULL.\n");
//	}
//	else{
//		printf("The parent's identifier is: %c%hu.\n\n", singleCell->ptr_parent->identifier[0], singleCell->ptr_parent->identifier[1]);
//	}

	// With the MCU
	sprintf(bufferMsg,"\nCell's identification is:\n"
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

	if(singleCell->ptr_parent == NULL){
		writeMsg(&usartCmd, "The parent's identifier is: NULL.\n");
	}
	else{
		sprintf(bufferMsg,"The parent's identifier is: %c%hu.\n\n", singleCell->ptr_parent->identifier[0], singleCell->ptr_parent->identifier[1]);
		writeMsg(&usartCmd, bufferMsg);
	}
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
	ptr_goal_cell->typeOfCell = 'G';
	ptr_start_cell->typeOfCell = 'S';
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
	//printf("\n");
	writeChar(&usartCmd, '\n');
}


/** Esta función debe recibir como parámetro el puntero a la última celda, que debe ser la "goal"
 * Con la información del ID_parent y entendiendo que todas las celdas visitadas
 * ya deben estar en el arreglo closed_list, se debe poder buscar la ruta y presentarla en pantalla */
void print_path(Cell_map_t *working_cell){
	Cell_map_t *ptr_backward_path = working_cell;
//	printf("The path in backward is:\n");
	writeMsg(&usartCmd, "The path in backward is:\n");
	while(ptr_backward_path != NULL){
		if ((ptr_backward_path->typeOfCell != 'G')&&(ptr_backward_path->typeOfCell != 'S')){
			ptr_backward_path->typeOfCell = '*';
		}
		else{
			__NOP();
		}
//		printf("\n%c%hu-> ,", ptr_backward_path->identifier[0], ptr_backward_path->identifier[1]);
		sprintf(bufferMsg,"%c%hu->", ptr_backward_path->identifier[0], ptr_backward_path->identifier[1]);
		writeMsg(&usartCmd, bufferMsg);
		ptr_backward_path = ptr_backward_path->ptr_parent;
	}
}

/**
 * == A* algorithm ==
 * Aca es donde se hace toda la magia, todo lo de arriba es necesario, pero
 * el algoritmo se ejecuta es en esta función.
 *
 * Esta función es la descripción literal del pseudocodigo...
  * */
int8_t A_star_algorithm(void){
	while(1){
		// Check if the current cell pointer is the empty cell, it would mean that there is no walkable path to the goal
		if (ptr_current_cell->typeOfCell == 'e'){
//			printf("\nThere's no possible walkable path.\n")
			writeMsg(&usartCmd, "\nThere's no possible walkable path.\n");
			return -1;
		}
		// Check if the goal has been reached to finish A* algorithm
		else if (ptr_current_cell == ptr_goal_cell){
//			printf("\nThe path has been found");
			writeMsg(&usartCmd, "\nThe path has been found.\n");
			print_path(ptr_current_cell);
			return 0;
		}
		// Do the actions of exam neighbours and select the best candidate
		else{
			Cell_map_t *ptrNeighbour = NULL;
			// Iterate neighbours
			for(uint8_t j = 0; j < MAX_NEIGHBOURS; j++){
				ptrNeighbour = ptr_current_cell->neighbours[j];
				// Check if the neighbor is valid and isn't an obstacle or if isn't in the Closed List, furthermore, the new G cost is lower than the current one
				if ((ptrNeighbour != NULL)&&(ptrNeighbour->typeOfCell != '#')&&(ptrNeighbour->typeOfCell != 'c')&&(get_G_cost(ptr_current_cell, ptrNeighbour) < ptrNeighbour->Gcost)){
					update_G_cost(ptr_current_cell, ptrNeighbour);
					update_F_cost(ptrNeighbour);
					// Verify if the neighbour isn't in the Open List
					if(ptrNeighbour->typeOfCell != 'o'){
						addTo_open_list(ptrNeighbour);
					}
					else{
						__NOP();
					}
				}
				else{
					__NOP();
				}
			}

			// With the best neighbour, is time to remove the current cell from the open list, and add it to the closed list
			removeFrom_open_list(ptr_current_cell);
			addTo_closed_list(ptr_current_cell);
			// So, now the best neighbour selected, will be the currrent cell pointer and will be add to the open list.
			ptr_current_cell = get_next_item();
		}
	}
}



/* ---			Funciones de los periféricos del MCU			--- */

/** Function responsible for starting the hardware */
void initSystem(void){

	configPLL(100);

	/* Activamos el Coprocesador Matemático - FPU */
	SCB->CPACR |= (0XF << 20);

	/* GPIO y Timer del Blinky Led de Estado */
	handlerBlinkyPin.pGPIOx								= GPIOC;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinNumber 		= PIN_5;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinSpeed 		= GPIO_OSPEED_FAST;
	handlerBlinkyPin.GPIO_PinConfig.GPIO_PinOPType 		= GPIO_OTYPE_PUSHPULL;
	// Cargo la configuración
	GPIO_Config(&handlerBlinkyPin);
	// Pongo estado en alto
	GPIO_WritePin(&handlerBlinkyPin, SET);
	// Atributos para el Timer 2 del LED de estado
	handlerBlinkyTimer.ptrTIMx								= TIM3;
	handlerBlinkyTimer.TIMx_Config.TIMx_mode				= BTIMER_MODE_UP;
	handlerBlinkyTimer.TIMx_Config.TIMx_speed				= BTIMER_PLL_100MHz_SPEED_100us;
	handlerBlinkyTimer.TIMx_Config.TIMx_period				= 2500;
	handlerBlinkyTimer.TIMx_Config.TIMx_interruptEnable 	= BTIMER_INTERRUP_ENABLE;
	handlerBlinkyTimer.TIMx_Config.TIMx_priorityInterrupt	= 6;
	BasicTimer_Config(&handlerBlinkyTimer);
	startBasicTimer(&handlerBlinkyTimer);
	/* Fin del GPIO y Timer del LED de estado
	 * ----------------------------------------*/


	/* Configuración de pines para el USART2 */
	handlerPinTX.pGPIOx								= GPIOA;
	handlerPinTX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_9;
	handlerPinTX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinTX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinTX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinTX);

	handlerPinRX.pGPIOx								= GPIOA;
	handlerPinRX.GPIO_PinConfig.GPIO_PinNumber 		= PIN_10;
	handlerPinRX.GPIO_PinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
	handlerPinRX.GPIO_PinConfig.GPIO_PinSpeed		= GPIO_OSPEED_FAST;
	handlerPinRX.GPIO_PinConfig.GPIO_PinAltFunMode	= AF7;
	GPIO_Config(&handlerPinRX);

	/* Configuración de la comunicación serial */
	usartCmd.ptrUSARTx								= USART1;
	usartCmd.USART_Config.USART_baudrate 			= USART_BAUDRATE_19200;
	usartCmd.USART_Config.USART_datasize			= USART_DATASIZE_8BIT;
	usartCmd.USART_Config.USART_parity				= USART_PARITY_NONE;
	usartCmd.USART_Config.USART_stopbits			= USART_STOPBIT_1;
	usartCmd.USART_Config.USART_mode				= USART_MODE_RXTX;
	usartCmd.USART_Config.USART_enableIntRX			= USART_RX_INTERRUP_ENABLE;
	usartCmd.USART_Config.USART_enableIntTX			= USART_TX_INTERRUP_DISABLE;
	usartCmd.USART_Config.USART_priorityInterrupt	= 6;
	USART_Config(&usartCmd);

	configMotors();
	updateDuttyCycle(&handlerPwmRight, 12000);
	updateDuttyCycle(&handlerPwmLeft, 12975);
}


/** Function to enter the map via serial port */
void catchMap(void){
	while((!flagMap)&&(!flagDoIt)){
		if(usartData != '\0'){
			bufferReception[line][counterReception] = usartData;
			counterReception++;

			// Aqui hacemmos la instrucción que detine la recepción del comando
			if(usartData == '@'){
				writeChar(&usartCmd, '\n');
				//Sustituyo el último caracter de \r por un null
				bufferReception[line][counterReception - 1] = '\0';
				for(uint8_t n = 0; n < (ROW_MAP_DATA_LEN + 1); n++){
					map_string[line][n] = bufferReception[line][n];
				}
				line++;
				counterReception = 0;
			}
			else{
				__NOP();
			}

			// Para borrar lo que se haya digitado en la terminal
			if(usartData == '\b'){
				counterReception--;
				counterReception--;
			}
			else{
				__NOP();
			}

			// Volvemos a null para terminar
			usartData = '\0';
			flagMap = (line == MAP_GRID_ROWS) ? (true):(false);
			line = (line == MAP_GRID_ROWS) ? (0):(line);
		}
		else{
			__NOP();
		}
	}
}


/** Function responsible for manage Oppy movement according to A* Algorithm */
void oppyPath(void){

	// In this block, we count how many cells are in the path founded
	Cell_map_t *ptrAux = ptr_goal_cell;
	uint8_t numberOfCells = 1;
	while(ptrAux->ptr_parent != NULL){
		numberOfCells++;
		ptrAux = ptrAux->ptr_parent;
	}
	sprintf(bufferMsg, "The number of path cells is: %hu", numberOfCells);
	writeMsg(&usartCmd, bufferMsg);

	// Now, we create and array or pointers, to save the reference to each path cell, this process is saving in reverse, from the end to the begin.
	Cell_map_t *ptrArray[numberOfCells];
	ptrAux = ptr_goal_cell;
	for(uint8_t i = numberOfCells; i > 0; i--){
		ptrArray[i-1] = ptrAux;
		ptrAux = ptrAux->ptr_parent;
	}


	Cell_map_t *ptrNext = NULL;

	uint8_t dirIndicator = 0;

	int16_t globalAngle = 0;
	int16_t differentialAngle = 0;
	differentialAngle++;

	// Loop to move through the path, in begins in 1, because start cell doesn't count
	for(uint8_t path = 1; path < numberOfCells; path++){
		// Update the pointers
		ptrAux = ptrArray[path-1];
		ptrNext = ptrArray[path];

		// This loop is to get the direction indicator
		for(uint j = 0; j < MAX_NEIGHBOURS; j++){
			if(ptrAux->neighbours[j] == ptrNext){
				dirIndicator = j;
				break;
			}
			else{
				__NOP();
			}
		}

		// Switch-case block to do the movement according to the direction indicator
		switch (dirIndicator) {
			// 0. Diagonal left above
			case 0: {
				differentialAngle = (abs(45 - globalAngle) > 180) ? ((45 - globalAngle) + 360):(45 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = 45;
				pathSegment(DIAGONAL_LENGTH);
				break;
			}
			// 1. Above
			case 1: {
				differentialAngle = (abs(0 - globalAngle) > 180) ? ((0 - globalAngle) + 360):(0 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = 0;
				pathSegment(STRAIGHT_LENGTH);
				break;
			}
			// 2. Diagonal right above
			case 2: {
				differentialAngle = (abs(-45 - globalAngle) > 180) ? ((-45 - globalAngle) + 360):(-45 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = -45;
				pathSegment(DIAGONAL_LENGTH);
				break;
			}
			// 3. Left
			case 3: {
				differentialAngle = (abs(90 - globalAngle) > 180) ? ((90 - globalAngle) + 360):(90 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = 90;
				pathSegment(STRAIGHT_LENGTH);
				break;
			}
			// 4. Right
			case 4: {
				differentialAngle = (abs(-90 - globalAngle) > 180) ? ((-90 - globalAngle) + 360):(-90 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = -90;
				pathSegment(STRAIGHT_LENGTH);
				break;
			}
			// 5. Diagonal left below
			case 5: {
				differentialAngle = (abs(135 - globalAngle) > 180) ? ((135 - globalAngle) + 360):(135 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = 135;
				pathSegment(DIAGONAL_LENGTH);
				break;
			}
			// 6. Below
			case 6: {
				differentialAngle = (globalAngle < 0) ? (-180 - globalAngle):(180 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = 180;
				pathSegment(STRAIGHT_LENGTH);
				break;
			}
			// 7. Diagonal right below
			case 7: {
				differentialAngle = (abs(-135 - globalAngle) > 180) ? ((-135 - globalAngle) + 360):(-135 - globalAngle);
				rotateOppy(differentialAngle);
				globalAngle = -135;
				pathSegment(DIAGONAL_LENGTH);
				break;
			}
			default:{
				__NOP();
				break;
			}
		}
	}

	// Finally, on the gol, Oppy rotates -globalAngle to set up itself to default orientation
	rotateOppy(-globalAngle);
}

/** Interrupción del timer blinky LED*/
void BasicTimer3_Callback(void){
	GPIOxTooglePin(&handlerBlinkyPin);
}


/** Interrupción del USART2 */
void usart1Rx_Callback(void){
	usartData = getRxData();
	writeChar(&usartCmd, usartData);
	// Simple instruction
	if(usartData == 'M'){
		writeChar(&usartCmd, '\n');
		flagDoIt = true;
	}
	else{
		__NOP();
	}
}


