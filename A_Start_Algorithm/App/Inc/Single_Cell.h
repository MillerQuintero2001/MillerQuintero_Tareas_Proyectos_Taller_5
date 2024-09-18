/*
 * Single_Cell.h
 *
 *  Created on: Sep 16, 2024
 *      Author: MillerQuintero2001
 *
 * Este archivo esta pensado describir a una casilla (celda) del mapa,
 * en un intento de pensar a C y sus estructuras como los objetos de la
 * OOP.
 */

#ifndef SINGLE_CELL_H_
#define SINGLE_CELL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Esta estructura "describe" todos los elementos y caracteristicas de una celda X,Y
 * del mapa, como por ejemplo:
 * - Identificación
 * - Ubicación
 * - Tipo de celda
 * - Valores G, H y F
 * - Lista de sus vecinos
 * - Indicador de quien es su parent.
 * */
typedef struct Cell_map_t
{
	char identifier[2];						// The identifier is an array with 2 elements, the column letter and row number
	uint16_t coordinateX;					// X coordinate in the map
	uint16_t coordinateY;					// Y coordinate in the map
	uint8_t typeOfCell;						// Type of Cell, S = start, G = goal, . = empty walkable cell, # = obstacle, o = Open, c = Closed, e = empty no walkable cell
	uint16_t Gcost;							// Displacement distance from the start point to the cell itself
	uint16_t Hcost;							// Distance between the particular cell and the target
	uint16_t Fcost;							// Sum of G cost and H cost, F = G + H
	struct Cell_map_t *neighbours[8];		/* Array with the 8 neighbours of the cell, the order regarding to the cell is:
											diagonal left above, above, diagonal right above, left, right, diagonal left below,
											below and diagonal right below */
	struct Cell_map_t *ptr_parent;			// Pointer to the parent cell
}Cell_map_t;

/** Prototipos públicos */
Cell_map_t create_cell(uint8_t pos_x, uint8_t pos_y);

#endif /* SINGLE_CELL_H_ */
