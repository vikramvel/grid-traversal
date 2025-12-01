# Grid Traversal

A C implementation for finding optimal paths through a grid to maximize cell coverage within movement constraints.

## The Problem

Given an N×M grid with some blocked cells and a maximum number of movement points (k), find a path that visits the highest number of unique unblocked cells. You can backtrack over previously visited cells, but each cell only counts once toward your unique visit count.

**Movement Rules:**
- Only N/E/W/S directions (no diagonals)
- Each move costs 1 movement point
- Can start from any unblocked cell

## Algorithm

**Approach:** Multi-start greedy with 2-step lookahead  
**Time Complexity:** O(k) where k = movement points  
**Space Complexity:** O(N×M) 

**Strategy:**
1. Try multiple strategic starting positions (center, corners, edges)
   - Why? Different starts work better for different obstacle layouts
2. From each start, use greedy selection with lookahead to pick best moves
   - Why? Lookahead avoids dead ends that pure greedy would get stuck in
3. Return the path with maximum coverage
   - Why? We pick the best result from all the starting positions we tried

## Example

**Input:** 8×8 grid, 25 movement points, blocked cells at (3,2), (4,2), (5,2)

```
Grid visualization:
. . . . . . . .
. . . . . . . .
. X . . . . . .  <- Row 3, column 2 is blocked
. X . . . . . .
. X . . . . . .
. . . . . . . .
. . . . . . . .
. . . . . . . .

X = blocked
. = open
```

**Output:**
```
(5,5), (4,5), (3,5), (2,5), (2,6), (2,7), (3,7), (4,7), (5,7), (6,7), (7,7), (7,6), (6,6), (6,5), (7,5), (7,4), (6,4), (6,3), (5,3), (4,3), (3,3), (2,3), (1,3), (1,2), (2,2), (2,1), 26
```

**Result:** Visited **26 unique cells** in 25 moves

## Compilation

```bash
gcc -O2 -std=c99 grid_traversal.c -o grid_traversal
```

## Usage

```bash
./grid_traversal <rows> <cols> <k> [blocked_coordinates...]
```

**Arguments:**
- `rows` - Number of rows in grid
- `cols` - Number of columns in grid  
- `k` - Maximum movement points
- `blocked_coords` - Optional blocked cells (1-indexed), format: `row,col`
  - If omitted, all cells are unblocked (open grid)

**Examples:**
```bash
# Simple 4×4 grid with no obstacles
./grid_traversal 4 4 10

# 5×5 grid with center blocked
./grid_traversal 5 5 15 3,3

# 8×8 grid with vertical wall
./grid_traversal 8 8 25 3,2 4,2 5,2

# Large 100×100 grid
./grid_traversal 100 100 500
```

## Sample Test File

Create `test_input.txt`:
```
8 8 25 3,2 4,2 5,2
```

Run:
```bash
./grid_traversal $(cat test_input.txt)
```

## How It Works

**1. BitArray Optimization**
- Packs 64 boolean values into one 64-bit word, helps in saving memory.
- Example: 20000×20000 grid uses only 50MB instead of 400MB

**2. Multi-Start Strategy**
- Tries 9 strategic positions: center, 4 corners, 4 edge midpoints
- Picks the start that gives best coverage
- Handles grids with obstacles better than single-start

**3. Greedy with Lookahead**
- At each step, evaluates all 4 neighbors
- Looks 2 moves ahead to avoid dead ends
- Prefers unvisited cells, backtracks if needed

## Code Structure

```
grid_traversal.h    - Public API and structures
grid_traversal.c    - Implementation
  ├── BitArray      - Memory-efficient bit storage
  ├── Grid Ops      - Grid creation and validation
  ├── Path Ops      - Path management
  ├── Algorithm     - Greedy solver with lookahead
  └── Main          - Command-line interface
```

## Trade-offs

**Pros:**
- Very fast: O(k) vs exhaustive O((N×M)³)
- Memory efficient: BitArray saves 8× space
- Practical: Achieves high coverage in real scenarios
- Scalable: Handles millions of cells efficiently

**Cons:**
- Not guaranteed optimal (greedy heuristic)
- Limited to 9 starting positions

For most practical applications, the speed/coverage trade-off is excellent!
