/**
 * grid_traversal.c
 * 
 * Algorithm: BFS with state memoization - explores all reachable states systematically
 * 
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

void print_path(Path *path) {
    for (int i = 0; i < path->length; i++) {
        printf("(%d,%d)", path->coords[i].x + 1, path->coords[i].y + 1);
        if (i < path->length - 1) printf(", ");
    }
    printf(", %d\n", path->unique_count);
}

// =============================================================================
// ALGORITHM: Optimal BFS State-Space Search
// =============================================================================

// State structure for BFS exploration
typedef struct {
    int x, y;           // Current position
    int moves_left;     // Remaining movement points
    BitArray *visited;  // Visited cells (owned by this state)
    int unique_count;   // Number of unique cells visited
    Path *path;         // Path taken to reach this state
} State;

// Queue structure for BFS
typedef struct QueueNode {
    State *state;
    struct QueueNode *next;
} QueueNode;

typedef struct {
    QueueNode *front;
    QueueNode *rear;
} Queue;

Queue* create_queue() {
    Queue *q = (Queue*)malloc(sizeof(Queue));
    if (!q) return NULL;
    q->front = q->rear = NULL;
    return q;
}

void enqueue(Queue *q, State *state) {
    QueueNode *node = (QueueNode*)malloc(sizeof(QueueNode));
    if (!node) return;
    node->state = state;
    node->next = NULL;
    
    if (q->rear == NULL) {
        q->front = q->rear = node;
    } else {
        q->rear->next = node;
        q->rear = node;
    }
}

State* dequeue(Queue *q) {
    if (q->front == NULL) return NULL;
    
    QueueNode *temp = q->front;
    State *state = temp->state;
    q->front = q->front->next;
    
    if (q->front == NULL) {
        q->rear = NULL;
    }
    
    free(temp);
    return state;
}

bool is_queue_empty(Queue *q) {
    return q->front == NULL;
}

void destroy_queue(Queue *q) {
    while (!is_queue_empty(q)) {
        State *s = dequeue(q);
        if (s) {
            destroy_bitarray(s->visited);
            destroy_path(s->path);
            free(s);
        }
    }
    free(q);
}

State* create_state(Grid *grid, int x, int y, int moves_left, 
                    BitArray *parent_visited, Path *parent_path, int parent_unique) {
    State *s = malloc(sizeof(State));
    if (!s) return NULL;
    
    s->x = x;
    s->y = y;
    s->moves_left = moves_left;
    
    // Copy parent's visited cells
    s->visited = create_bitarray(grid->rows * grid->cols);
    if (!s->visited) {
        free(s);
        return NULL;
    }
    if (parent_visited) {
        memcpy(s->visited->words, parent_visited->words, 
               parent_visited->num_words * sizeof(BitWord));
    }
    
    // Mark current cell and count it if new
    int idx = x * grid->cols + y;
    bool is_new = !parent_visited || !bitarray_get(parent_visited, idx);
    bitarray_set(s->visited, idx);
    s->unique_count = parent_unique + (is_new ? 1 : 0);
    
    // Copy parent path and add current position
    int new_len = parent_path ? parent_path->length + 1 : 1;
    s->path = create_path(new_len);
    if (!s->path) {
        destroy_bitarray(s->visited);
        free(s);
        return NULL;
    }
    if (parent_path) {
        memcpy(s->path->coords, parent_path->coords, 
               parent_path->length * sizeof(Coordinate));
        s->path->length = parent_path->length;
    }
    add_to_path(s->path, x, y);
    
    return s;
}

void destroy_state(State *s) {
    if (!s) return;
    if (s->visited) destroy_bitarray(s->visited);
    if (s->path) destroy_path(s->path);
    free(s);
}

void solve_and_print(Grid *grid, int k) {
    if (!grid || k < 0) {
        printf("Invalid input\n");
        return;
    }
    
    State *best = NULL;
    int best_count = 0;
    
    for (int sx = 0; sx < grid->rows; sx++) {
        for (int sy = 0; sy < grid->cols; sy++) {
            if (!is_valid(grid, sx, sy)) continue;
            
            Queue *q = create_queue();
            State *start = create_state(grid, sx, sy, k, NULL, NULL, 0);
            if (!start) continue;
            enqueue(q, start);
            
            int memo_size = grid->rows * grid->cols * (k + 1);
            int *memo = malloc(memo_size * sizeof(int));
            if (!memo) {
                destroy_queue(q);
                continue;
            }
            memset(memo, -1, memo_size * sizeof(int));
            
            while (!is_queue_empty(q)) {
                State *curr = dequeue(q);
                
                if (curr->unique_count > best_count) {
                    if (best) destroy_state(best);
                    best = curr;
                    best_count = curr->unique_count;
                    curr = NULL;
                }
                
                if (curr && curr->moves_left > 0) {
                    for (int d = 0; d < 4; d++) {
                        int nx = curr->x + dx[d];
                        int ny = curr->y + dy[d];
                        
                        if (!is_valid(grid, nx, ny)) continue;
                        
                        State *next = create_state(grid, nx, ny, curr->moves_left - 1, 
                                                   curr->visited, curr->path, curr->unique_count);
                        if (!next) continue;
                        
                        int idx = nx * grid->cols * (k + 1) + ny * (k + 1) + next->moves_left;
                        if (memo[idx] >= next->unique_count) {
                            destroy_state(next);
                            continue;
                        }
                        memo[idx] = next->unique_count;
                        
                        if (next->unique_count >= grid->total_unblocked) {
                            if (next->unique_count > best_count) {
                                if (best) destroy_state(best);
                                best = next;
                                best_count = next->unique_count;
                                next = NULL;
                            } else {
                                destroy_state(next);
                            }
                            continue;
                        }
                        
                        enqueue(q, next);
                    }
                }
                
                if (curr) destroy_state(curr);
            }
            
            destroy_queue(q);
            free(memo);
            
            if (best_count >= grid->total_unblocked) {
                goto done;
            }
        }
    }
    
done:
    if (best) {
        best->path->unique_count = best_count;
        print_path(best->path);
        destroy_state(best);
    } else {
        printf("No solution found\n");
    }
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

#endif // GRID_TRAVERSAL_LIB
