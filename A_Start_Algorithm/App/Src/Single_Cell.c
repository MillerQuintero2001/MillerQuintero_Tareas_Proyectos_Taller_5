/*
 * Single_Cell.c
 *
 *  Created on: Mar 8, 2024
 *      Author: namontoy
 *
 * Este archivo posee las funciones para crear celdas vacias (casilllas), de forma
 * que luego cada una de ellas pueda ser completada con la información que le pasamos
 * a nuestro equipo por medio de la comunicación RS-232 inalambrica.
 */


#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "Single_Cell.h"

/* Prototipos privados */
Cell_map_t init_empty_cell(Cell_map_t dummyCell);

/**
 * Esta función se encarga de inicializar las celdas que hacen parte
 * del grid del mapa que se desea estudiar.
 *
 * La idea es que cada celda debe ser capaz de reconocer a sus vecinos
 * por la posicion x,y que tienen en el grid_map.
 *
 * Como tal, al inicio, cada celda comienza completamente vacia y se va llenando a medida
 * que le enviamos la información del mapa a nuestro equipo.
*/
Cell_map_t init_empty_cell(Cell_map_t dummyCell){
    // aca se escribe toda la info de la celda...}
	dummyCell.identifier[0] = 0;
	dummyCell.identifier[1] = 0;
	dummyCell.coordinateX = 0.00f;
	dummyCell.coordinateY = 0.00f;
	// Write 'e' to represente an empty no walkable cell
	dummyCell.typeOfCell = 'e';
	// This great number is only to simbolize an infinite value
	dummyCell.Gcost = 25000.00f;
	dummyCell.Hcost = 25000.00f;
	dummyCell.Fcost = 25000.00f;
	for(uint8_t i = 0; i < 8; i++){
		dummyCell.neighbours[i] = NULL;
	}
	dummyCell.ptr_parent = NULL;
    return dummyCell;
}

/*
 *	Esta función es utilizada para crear TODAS las celdas del mapa inicializadas
 *	como vacias pero en la posición X, Y que se pasan como parametros, o sea, como
 *	tal esta función crea todos los elementos del mapa en sus posiciones correctas
 *	pero sin organizar sus caracteristicas mas específicas.
 *
 * */
Cell_map_t create_cell(uint8_t matrix_pos_x, uint8_t matrix_pos_y){
	Cell_map_t unknowCell = {0};
	unknowCell = init_empty_cell(unknowCell);
	unknowCell.identifier[0] = matrix_pos_x + 'A';
	unknowCell.identifier[1] = matrix_pos_y;
	unknowCell.coordinateX = matrix_pos_x*FLOOR_TILE_SIZE;
	unknowCell.coordinateY = matrix_pos_y*FLOOR_TILE_SIZE;
    return  unknowCell;
}
