/**
 * grid_traversal.h
 * 
 * Grid pathfinding with multi-start greedy search and lookahead optimization.
 * Finds paths that maximize coverage of unblocked cells within movement constraints.
 * 
 * License: MIT
 */

#ifndef GRID_TRAVERSAL_H
#define GRID_TRAVERSAL_H

#include <stdbool.h>

// Forward declaration
typedef struct BitArray BitArray;

// Coordinate structure
typedef struct {
    int x;
    int y;
} Coordinate;

// Grid structure
typedef struct {
    int rows;
    int cols;
    BitArray *blocked;
    int total_unblocked;
} Grid;

// Path structure
typedef struct {
    Coordinate *coords;
    int length;
    int capacity;
    int unique_count;
} Path;

// Public API
Grid* create_grid(int rows, int cols, int num_blocked, Coordinate *blocked_squares);
void destroy_grid(Grid *grid);
void solve_and_print(Grid *grid, int movement_points);

#endif
