/**
 * grid_traversal.c
 * 
 * Implementation of grid pathfinding algorithm using multi-start greedy approach
 * with 2-step lookahead. Uses BitArray for memory-efficient storage.
 * 
 * Algorithm: O(k) time complexity where k = movement points
 * Strategy: Tries multiple starting positions (center, corners, edges) and picks best
 * 
 * License: MIT
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <limits.h>
#include "grid_traversal.h"

// Movement directions: N, E, S, W
const int dx[] = {-1, 0, 1, 0};
const int dy[] = {0, 1, 0, -1};

// =============================================================================
// UTILITY: BitArray - pack 64 bools into one 64-bit word
// =============================================================================

typedef unsigned long long BitWord;
#define BITS_PER_WORD (sizeof(BitWord) * 8)

struct BitArray {
    BitWord *words;
    int size;
    int num_words;
};

BitArray* create_bitarray(int size) {
    BitArray *ba = (BitArray*)malloc(sizeof(BitArray));
    if (!ba) return NULL;
    
    ba->size = size;
    ba->num_words = (size + BITS_PER_WORD - 1) / BITS_PER_WORD;  // How many 64-bit words needed?
    ba->words = (BitWord*)calloc(ba->num_words, sizeof(BitWord));
    if (!ba->words) {
        free(ba);
        return NULL;
    }
    return ba;
}

void destroy_bitarray(BitArray *ba) {
    if (ba) {
        free(ba->words);
        free(ba);
    }
}

// Set bit at index to 1 (mark as blocked/visited)
static inline void bitarray_set(BitArray *ba, int index) {
    ba->words[index / BITS_PER_WORD] |= (1ULL << (index % BITS_PER_WORD));
}

// Check if bit at index is 1
static inline bool bitarray_get(BitArray *ba, int index) {
    return (ba->words[index / BITS_PER_WORD] & (1ULL << (index % BITS_PER_WORD))) != 0;
}

// =============================================================================
// UTILITY: Grid Operations
// =============================================================================

Grid* create_grid(int rows, int cols, int num_blocked, Coordinate *blocked_squares) {
    Grid *grid = (Grid*)malloc(sizeof(Grid));
    if (!grid) return NULL;
    
    grid->rows = rows;
    grid->cols = cols;
    
    grid->blocked = create_bitarray(rows * cols);
    if (!grid->blocked) {
        free(grid);
        return NULL;
    }
    
    // Mark all the blocked cells
    for (int i = 0; i < num_blocked; i++) {
        int x = blocked_squares[i].x;
        int y = blocked_squares[i].y;
        if (x >= 0 && x < rows && y >= 0 && y < cols) {
            bitarray_set(grid->blocked, x * cols + y);
        }
    }
    
    // Calculate total unblocked cells for quick access
    grid->total_unblocked = rows * cols - num_blocked;
    
    return grid;
}

void destroy_grid(Grid *grid) {
    if (!grid) return;
    if (grid->blocked) {
        destroy_bitarray(grid->blocked);
    }
    free(grid);
}

// Can I legally move to this cell?
static inline bool is_valid(Grid *grid, int x, int y) {
    return x >= 0 && x < grid->rows &&      // Not above/below grid
           y >= 0 && y < grid->cols &&      // Not left/right of grid
           !bitarray_get(grid->blocked, x * grid->cols + y);  // Not blocked
}

// =============================================================================
// UTILITY: Path Operations
// =============================================================================

Path* create_path(int capacity) {
    Path *path = (Path*)malloc(sizeof(Path));
    if (!path) return NULL;
    
    path->coords = (Coordinate*)malloc(capacity * sizeof(Coordinate));
    if (!path->coords) {
        free(path);
        return NULL;
    }
    
    path->length = 0;
    path->capacity = capacity;
    path->unique_count = 0;
    return path;
}

void destroy_path(Path *path) {
    if (path) {
        free(path->coords);
        free(path);
    }
}

static inline void add_to_path(Path *path, int x, int y) {
    // Out of space? Double the array size
    if (path->length >= path->capacity) {
        int new_capacity = path->capacity * 2;
        Coordinate *new_coords = (Coordinate*)realloc(path->coords, new_capacity * sizeof(Coordinate));
        if (!new_coords) return;
        path->coords = new_coords;
        path->capacity = new_capacity;
    }
    path->coords[path->length].x = x;
    path->coords[path->length].y = y;
    path->length++;
}

// Count how many unique cells we actually visited (ignoring backtracking)
int calculate_unique(Path *path, Grid *grid) {
    BitArray *seen = create_bitarray(grid->rows * grid->cols);
    int unique = 0;
    
    for (int i = 0; i < path->length; i++) {
        int idx = path->coords[i].x * grid->cols + path->coords[i].y;
        if (!bitarray_get(seen, idx)) {
            bitarray_set(seen, idx);
            unique++;
        }
    }
    
    destroy_bitarray(seen);
    return unique;
}

void print_path(Path *path) {
    for (int i = 0; i < path->length; i++) {
        printf("(%d,%d)", path->coords[i].x + 1, path->coords[i].y + 1);
        if (i < path->length - 1) printf(", ");
    }
    printf(", %d\n", path->unique_count);
}

// =============================================================================
// ALGORITHM: Greedy Solver with Lookahead
// =============================================================================

// Look ahead and score a position based on future opportunities
static int evaluate_position(Grid *grid, int x, int y, BitArray *visited, int depth) {
    if (depth == 0) {  // Base case: count available neighbors
        int score = 0;
        for (int dir = 0; dir < 4; dir++) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];
            if (is_valid(grid, nx, ny) && !bitarray_get(visited, nx * grid->cols + ny)) {
                score++;
            }
        }
        return score;
    }
    
    int best = 0;
    for (int dir = 0; dir < 4; dir++) {
        int nx = x + dx[dir];
        int ny = y + dy[dir];
        int idx = nx * grid->cols + ny;
        if (is_valid(grid, nx, ny) && !bitarray_get(visited, idx)) {
            int score = 1 + evaluate_position(grid, nx, ny, visited, depth - 1);
            if (score > best) best = score;
        }
    }
    return best;
}

static int solve_from_position(Grid *grid, int start_x, int start_y, int k, Path *path, BitArray *visited) {
    // Reset path and visited
    path->length = 0;
    memset(visited->words, 0, visited->num_words * sizeof(BitWord));
    
    add_to_path(path, start_x, start_y);
    bitarray_set(visited, start_x * grid->cols + start_y);
    
    int curr_x = start_x;
    int curr_y = start_y;
    int moves = 0;
    int unique_visited = 1;
    int moves_since_new_cell = 0;
    int total_unblocked = grid->total_unblocked;
    
    while (moves < k) {
        if (unique_visited >= total_unblocked) break;  // Visited everything!
        if (moves_since_new_cell > 10) break;  // Stuck in a loop, give up
        
        int best_x = -1, best_y = -1;
        int best_score = -1;
        bool found_unvisited = false;
        
        // First pass: look for unvisited neighbors (greedy)
        for (int dir = 0; dir < 4; dir++) {
            int nx = curr_x + dx[dir];
            int ny = curr_y + dy[dir];
            int idx = nx * grid->cols + ny;
            
            if (is_valid(grid, nx, ny) && !bitarray_get(visited, idx)) {
                int score = evaluate_position(grid, nx, ny, visited, 2);
                if (score > best_score) {
                    best_score = score;
                    best_x = nx;
                    best_y = ny;
                    found_unvisited = true;
                }
            }
        }
        
        // No unvisited cells? Backtrack to find new areas
        if (!found_unvisited) {
            for (int dir = 0; dir < 4; dir++) {
                int nx = curr_x + dx[dir];
                int ny = curr_y + dy[dir];
                
                if (is_valid(grid, nx, ny)) {
                    int score = evaluate_position(grid, nx, ny, visited, 1);
                    if (score > best_score) {
                        best_score = score;
                        best_x = nx;
                        best_y = ny;
                    }
                }
            }
        }
        
        if (best_x == -1) break;
        
        add_to_path(path, best_x, best_y);
        int idx = best_x * grid->cols + best_y;
        if (!bitarray_get(visited, idx)) {
            bitarray_set(visited, idx);
            unique_visited++;
            moves_since_new_cell = 0;
        } else {
            moves_since_new_cell++;
        }
        curr_x = best_x;
        curr_y = best_y;
        moves++;
    }
    
    return unique_visited;
}

// Find good starting positions to try (center, corners, edges)
static void find_candidate_starts(Grid *grid, Coordinate *candidates, int *num_candidates) {
    *num_candidates = 0;
    
    // Try center first (usually best for coverage)
    int center_x = grid->rows / 2;
    int center_y = grid->cols / 2;
    if (is_valid(grid, center_x, center_y)) {
        candidates[(*num_candidates)++] = (Coordinate){center_x, center_y};
    }
    
    // Try four corners
    int corners[4][2] = {
        {0, 0},                           // Top-left
        {0, grid->cols - 1},              // Top-right
        {grid->rows - 1, 0},              // Bottom-left
        {grid->rows - 1, grid->cols - 1}  // Bottom-right
    };
    for (int i = 0; i < 4; i++) {
        int x = corners[i][0];
        int y = corners[i][1];
        if (is_valid(grid, x, y)) {
            candidates[(*num_candidates)++] = (Coordinate){x, y};
        }
    }
    
    // Try edge midpoints
    int edges[4][2] = {
        {0, grid->cols / 2},              // Top edge
        {grid->rows / 2, 0},              // Left edge
        {grid->rows / 2, grid->cols - 1}, // Right edge
        {grid->rows - 1, grid->cols / 2}  // Bottom edge
    };
    for (int i = 0; i < 4; i++) {
        int x = edges[i][0];
        int y = edges[i][1];
        if (is_valid(grid, x, y)) {
            candidates[(*num_candidates)++] = (Coordinate){x, y};
        }
    }
    
    // Fallback: scan top-left area if no candidates found
    if (*num_candidates == 0) {
        for (int i = 0; i < grid->rows && i < 10; i++) {
            for (int j = 0; j < grid->cols && j < 10; j++) {
                if (is_valid(grid, i, j)) {
                    candidates[(*num_candidates)++] = (Coordinate){i, j};
                    return;
                }
            }
        }
    }
}

void solve_and_print(Grid *grid, int k) {
    if (!grid || k < 0) {
        printf("Invalid input\n");
        return;
    }
    
    // Get candidate starting positions
    Coordinate candidates[10];  // Max 9 candidates (center + 4 corners + 4 edges)
    int num_candidates = 0;
    find_candidate_starts(grid, candidates, &num_candidates);
    
    if (num_candidates == 0) {
        printf("No unblocked squares\n");
        return;
    }
    
    // Try each starting position, keep whichever gives best coverage
    int initial_capacity = (k + 1 < 10000) ? k + 1 : 10000;
    Path *temp_path = create_path(initial_capacity);
    Path *best_path = create_path(initial_capacity);
    BitArray *temp_visited = create_bitarray(grid->rows * grid->cols);
    
    int best_unique = 0;
    int best_start_idx = 0;
    
    // Try each candidate starting position
    for (int i = 0; i < num_candidates; i++) {
        int unique = solve_from_position(grid, candidates[i].x, candidates[i].y, k, temp_path, temp_visited);
        
        // Better than what we have? Save it
        if (unique > best_unique) {
            best_unique = unique;
            best_start_idx = i;
            
            // Copy path to best_path
            best_path->length = temp_path->length;
            if (best_path->capacity < temp_path->length) {
                best_path->coords = (Coordinate*)realloc(best_path->coords, temp_path->length * sizeof(Coordinate));
                best_path->capacity = temp_path->length;
            }
            memcpy(best_path->coords, temp_path->coords, temp_path->length * sizeof(Coordinate));
        }
    }
    
    // Print the best path found
    best_path->unique_count = best_unique;
    print_path(best_path);
    
    // Cleanup
    destroy_path(temp_path);
    destroy_path(best_path);
    destroy_bitarray(temp_visited);
}

// =============================================================================
// MAIN: Command Line Interface
// =============================================================================

#ifndef GRID_TRAVERSAL_LIB

int main(int argc, char *argv[]) {
    if (argc < 4) {
        printf("Usage: %s <rows> <cols> <k> [blocked_coords...]\n", argv[0]);
        printf("\nArguments:\n");
        printf("  rows            : Number of rows in the grid\n");
        printf("  cols            : Number of columns in the grid\n");
        printf("  k               : Maximum movement points\n");
        printf("  blocked_coords  : Optional blocked cell coordinates (1-indexed)\n");
        printf("                    Format: row,col (e.g., 3,2 4,2 5,2)\n");
        printf("\nExample:\n");
        printf("  %s 8 8 25 3,2 4,2 5,2\n", argv[0]);
        printf("  Creates 8x8 grid with 25 moves, blocking cells (3,2), (4,2), (5,2)\n");
        printf("\n");
        return 1;
    }
    
    // Parse grid dimensions and movement points
    int rows = atoi(argv[1]);
    int cols = atoi(argv[2]);
    int k = atoi(argv[3]);
    
    // Validate input
    if (rows <= 0 || cols <= 0 || k < 0) {
        printf("Error: Invalid parameters. Rows, cols must be positive, k must be non-negative.\n");
        return 1;
    }
    
    // Parse blocked squares from remaining arguments
    int num_blocked = argc - 4;
    Coordinate *blocked = NULL;
    
    if (num_blocked > 0) {
        blocked = (Coordinate*)malloc(num_blocked * sizeof(Coordinate));
        if (!blocked) {
            printf("Error: Memory allocation failed\n");
            return 1;
        }
        
        for (int i = 0; i < num_blocked; i++) {
            int parsed = sscanf(argv[4 + i], "%d,%d", &blocked[i].x, &blocked[i].y);
            if (parsed != 2) {
                printf("Error: Invalid blocked coordinate format '%s'. Use row,col (e.g., 3,2)\n", argv[4 + i]);
                free(blocked);
                return 1;
            }
            
            // Convert from 1-indexed (user input) to 0-indexed (internal)
            blocked[i].x--;
            blocked[i].y--;
            
            // Validate coordinates
            if (blocked[i].x < 0 || blocked[i].x >= rows || 
                blocked[i].y < 0 || blocked[i].y >= cols) {
                printf("Error: Blocked coordinate (%d,%d) is out of bounds for %dx%d grid\n", 
                       blocked[i].x + 1, blocked[i].y + 1, rows, cols);
                free(blocked);
                return 1;
            }
        }
    }
    
    // Create grid
    Grid *grid = create_grid(rows, cols, num_blocked, blocked);
    if (!grid) {
        printf("Error: Failed to create grid\n");
        free(blocked);
        return 1;
    }
    
    // Solve and print solution
    solve_and_print(grid, k);
    
    // Cleanup
    destroy_grid(grid);
    free(blocked);
    
    return 0;
}

#endif