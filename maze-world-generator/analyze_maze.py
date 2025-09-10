from collections import deque


def is_fully_connected(grid, start_position):
    """
    Determines if all cells in the grid are reachable from the start position using BFS.

    Args:
        grid: 2D list of cell objects.
        start_position: Tuple (x, y) representing the starting coordinates.

    Returns:
        True if all cells in the grid are reachable from the start position, False otherwise.
    """
    grid_width = len(grid[0])
    grid_height = len(grid)
    total_cells = grid_width * grid_height
    
    # Track visited cells to avoid cycles
    visited_cells = set()
    cells_to_explore = deque([start_position])

    # Define movement directions with corresponding wall checks
    movement_directions = [
        (0, -1, 'top'),     
        (0, 1, 'bottom'),   
        (-1, 0, 'left'),   
        (1, 0, 'right'),    
    ]

    while cells_to_explore:
        current_x, current_y = cells_to_explore.popleft()
        
        # Skip if already visited
        if (current_x, current_y) in visited_cells:
            continue
            
        visited_cells.add((current_x, current_y))
        current_cell = grid[current_y][current_x]

        # Explore all possible directions from the current cell
        for delta_x, delta_y, wall_direction in movement_directions:
            neighbor_x = current_x + delta_x
            neighbor_y = current_y + delta_y
            
            is_within_bounds = (0 <= neighbor_x < grid_width and 
                                0 <= neighbor_y < grid_height)
            if not is_within_bounds:
                continue
                
            is_passage_open = not current_cell.walls[wall_direction]
            if is_passage_open:
                cells_to_explore.append((neighbor_x, neighbor_y))

    # Grid is fully connected if all cells were visited
    return len(visited_cells) == total_cells


def find_shortest_path(grid, start, end):
    """
    Finds the shortest path from start to end using BFS.

    Args:
        grid: 2D list of cell objects.
        start: Tuple (x, y) representing the starting coordinates.
        end: Tuple (x, y) representing the target coordinates.

    Returns:
        List of tuples representing the path from start to end, inclusive.
        Returns an empty list if no path exists.
    """
    grid_height = len(grid)
    grid_width = len(grid[0])
    
    # Matrix to track visited cells to avoid cycles and redundant processing
    visited = [[False] * grid_width for _ in range(grid_height)]
    
    # Dictionary to store the parent of each cell for path reconstruction
    parent = {}
    
    cells_to_explore = deque([start])
    visited[start[1]][start[0]] = True

    movement_directions = {
        'top': (0, -1),
        'bottom': (0, 1),
        'left': (-1, 0),
        'right': (1, 0)
    }

    while cells_to_explore:
        current_x, current_y = cells_to_explore.popleft()
        
        # Early exit if the target cell is reached
        if (current_x, current_y) == end:
            break

        current_cell = grid[current_y][current_x]
        
        for direction, (dx, dy) in movement_directions.items():
            # Check if there's no wall in the current direction
            if not current_cell.walls[direction]:
                neighbor_x = current_x + dx
                neighbor_y = current_y + dy
                
                # Check if the neighbor is within grid bounds and not visited
                if (0 <= neighbor_x < grid_width and 
                    0 <= neighbor_y < grid_height and 
                    not visited[neighbor_y][neighbor_x]):
                    
                    # Mark neighbor as visited and record its parent
                    visited[neighbor_y][neighbor_x] = True
                    parent[(neighbor_x, neighbor_y)] = (current_x, current_y)
                    cells_to_explore.append((neighbor_x, neighbor_y))

    # Reconstruct the path by backtracking from the end cell to the start cell
    path = []
    current_cell = end
    
    # Backtrack until reach the start cell
    while current_cell != start:
        path.append(current_cell)
        current_cell = parent.get(current_cell)
        
        # If current_cell is None, the start cell was never reached
        if current_cell is None:
            return []  # Return empty list indicating no path exists
    
    # Add the start cell and reverse the path to get start-to-end order
    path.append(start)
    path.reverse()
    return path