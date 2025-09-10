import random
from analyze_maze import is_fully_connected
from draw_maze import draw_maze


class Cell:
    """Represents a single cell in the maze with wall properties and visited state."""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False


def generate_maze(width, height, num_rooms=3, max_attempts=20, chest_probability=0.4, room_size=3):
    """Generate a valid maze with rooms and paths.
    
    Attempts to create a fully connected maze up to max_attempts times.
    Returns the last attempt if no valid maze is found.
    """
    for attempt in range(max_attempts):
        grid, start, end, chests = _generate_single_maze(width, height, num_rooms, chest_probability, room_size)
        if is_fully_connected(grid, start):
            print(f"Maze valid after {attempt + 1} attempt(s).")
            return grid, start, end, chests
        print(f"Maze not fully connected (attempt {attempt + 1}), retrying...")

    print("Failed to generate a fully connected maze after max attempts.")
    return grid, start, end, chests  # Fallback: return last attempt


def _generate_single_maze(width, height, num_rooms, chest_probability, room_size):
    """Generate a single maze attempt with rooms, paths, and chests."""
    grid = _initialize_grid(width, height)
    room_cells = []
    room_entrances = []
    chest_locations = []

    # Direction mappings: (dx, dy, current_wall, neighbor_wall)
    directions = [
        (0, -1, 'top', 'bottom'),
        (0, 1, 'bottom', 'top'),
        (-1, 0, 'left', 'right'),
        (1, 0, 'right', 'left')
    ]

    def is_within_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def carve_passages(x, y):
        """Recursively carve paths using DFS from position (x, y)."""
        current_cell = grid[y][x]
        current_cell.visited = True
        random.shuffle(directions)

        for dx, dy, current_wall, neighbor_wall in directions:
            next_x, next_y = x + dx, y + dy
            if not is_within_bounds(next_x, next_y):
                continue
            if (next_x, next_y) in room_cells:
                continue

            neighbor_cell = grid[next_y][next_x]
            if not neighbor_cell.visited:
                current_cell.walls[current_wall] = False
                neighbor_cell.walls[neighbor_wall] = False
                carve_passages(next_x, next_y)

    def try_add_chest(room_center_x, room_center_y):
        """Attempt to add a chest at room center based on probability."""
        if random.random() < chest_probability:
            chest_locations.append((room_center_x, room_center_y))

    # Create start and end rooms
    _create_room(grid, 0, 0, room_size, room_cells, room_entrances, is_start_room=True)
    _create_room(grid, width - room_size, height - room_size, room_size, room_cells, room_entrances, is_start_room=False)
    
    # Add random rooms and their entrances
    _place_random_rooms(grid, width, height, num_rooms, room_cells, room_entrances, try_add_chest)

    # Visualize maze after room creation
    draw_maze(grid, "maze_step1_rooms_only.png", chests=chest_locations)

    # Start carving from start room entrance
    start_entrance_x, start_entrance_y = room_entrances[0]
    carve_passages(start_entrance_x, start_entrance_y)

    # Visualize maze after path generation
    draw_maze(grid, "maze_step2_after_carving.png", chests=chest_locations)

    # Define start and end points (center of start/end rooms)
    start = (room_size // 2, room_size // 2)
    end = (width - room_size // 2 - 1, height - room_size // 2 - 1)

    return grid, start, end, chest_locations


def _initialize_grid(width, height):
    """Create a 2D grid of unvisited Cells."""
    return [[Cell(x, y) for x in range(width)] for y in range(height)]


def _create_room(grid, start_x, start_y, room_size, room_cells, room_entrances, is_start_room):
    """Create a room at specified location and add its entrance."""
    for y in range(start_y, start_y + room_size):
        for x in range(start_x, start_x + room_size):
            cell = grid[y][x]
            cell.visited = True
            room_cells.append((x, y))

            # Remove interior walls
            if y > start_y: cell.walls['top'] = False
            if y < start_y + room_size - 1: cell.walls['bottom'] = False
            if x > start_x: cell.walls['left'] = False
            if x < start_x + room_size - 1: cell.walls['right'] = False

    # Create entrance/exit for the room
    center_x = start_x + room_size // 2
    if is_start_room:
        # Start room entrance at bottom center
        grid[start_y + room_size - 1][center_x].walls['bottom'] = False
        grid[start_y + room_size][center_x].walls['top'] = False
        room_entrances.append((center_x, start_y + room_size))
    else:
        # End room entrance at top center
        grid[start_y][center_x].walls['top'] = False
        grid[start_y - 1][center_x].walls['bottom'] = False
        room_entrances.append((center_x, start_y - 1))


def _place_random_rooms(grid, width, height, num_rooms, room_cells, room_entrances, add_chest_callback):
    """Place random rooms with entrances and potential chests."""
    directions = {
        'top': (0, -1, 'top', 'bottom'),
        'bottom': (0, 1, 'bottom', 'top'),
        'left': (-1, 0, 'left', 'right'),
        'right': (1, 0, 'right', 'left')
    }

    rooms_placed = 0
    attempts = 0
    max_attempts = 1000

    while rooms_placed < num_rooms and attempts < max_attempts:
        attempts += 1
        room_x = random.randint(1, width - 4)
        room_y = random.randint(1, height - 4)

        # Check for overlapping rooms
        if any((x, y) in room_cells 
               for y in range(room_y, room_y + 3) 
               for x in range(room_x, room_x + 3)):
            continue

        # Create 3x3 room
        for y in range(room_y, room_y + 3):
            for x in range(room_x, room_x + 3):
                cell = grid[y][x]
                cell.visited = True
                room_cells.append((x, y))
                # Remove interior walls
                if y > room_y: cell.walls['top'] = False
                if y < room_y + 2: cell.walls['bottom'] = False
                if x > room_x: cell.walls['left'] = False
                if x < room_x + 2: cell.walls['right'] = False

        # Add 1-4 random entrances
        room_center_x = room_x + 1
        room_center_y = room_y + 1
        possible_entrances = random.sample(list(directions.keys()), random.randint(1, 4))
        
        for entrance_side in possible_entrances:
            dx, dy, wall_curr, wall_next = directions[entrance_side]
            entrance_x = room_center_x + dx * 2
            entrance_y = room_center_y + dy * 2

            if not (0 <= entrance_x < width and 0 <= entrance_y < height):
                continue

            # Remove walls between room and maze
            grid[room_center_y + dy][room_center_x + dx].walls[wall_curr] = False
            grid[entrance_y][entrance_x].walls[wall_next] = False
            room_entrances.append((entrance_x, entrance_y))

        add_chest_callback(room_center_x, room_center_y)
        rooms_placed += 1