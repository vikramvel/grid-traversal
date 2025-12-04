# Grid Traversal

Find the path that visits the most cells in a grid with limited moves.

## Problem

You have an N×M grid with some blocked cells and k movement points. Find a path that visits as many unique cells as possible.

**Rules:**
- Move only up/down/left/right (no diagonal)
- Each move costs 1 point
- Can revisit cells but they only count once
- Start anywhere on the grid

## Algorithm

Uses BFS (Breadth-First Search) with memoization to explore all possible paths systematically.


**Why BFS?**
- Explores all reachable states level-by-level
- Memoization prevents checking the same state twice
- Guaranteed to find optimal solution

## Build

```bash
gcc -O2 grid_traversal.c -o grid_traversal
```

## Usage

```bash
./grid_traversal <rows> <cols> <k> [blocked_cells...]
```

**Examples:**

Simple 3×3 grid, 5 moves:
```bash
./grid_traversal 3 3 5
```

8×8 grid with blocked cells at (3,2), (4,2), (5,2):
```bash
./grid_traversal 8 8 25 3,2 4,2 5,2
```

Using input file:
```bash
echo "8 8 25 3,2 4,2 5,2" > test.txt
./grid_traversal $(cat test.txt)
```

## Output Format

```
(1,3), (2,3), (3,3), (3,2), (2,2), (1,2), 6
```

Shows the path taken and final unique cell count (6 in this example).

## How It Works

**BitArray** - Stores visited cells using 1 bit per cell instead of 1 byte. Saves memory.

**BFS Queue** - Explores all paths systematically. Tries every starting position and every possible move.

**Memoization** - At each (x, y, moves_left) state, remembers the best coverage found. Skips states that can't improve the result.

**Pruning** - Stops early if all cells are visited.

## Code Structure

- `BitArray` - Bit packing for visited cells
- `Grid` - Grid with blocked cells
- `State` - BFS search state (position, moves, visited, path)
- `Queue` - FIFO queue for BFS
- `solve_and_print()` - Main BFS loop

## Performance

Tested on:
- 3×3 grid: instant
- 8×8 grid: < 1 second
- 10×10 grid: ~2 seconds
- 20×20 grid: ~30 seconds (depends on k)

For larger grids or high k values, runtime increases. This is expected for optimal solutions.
