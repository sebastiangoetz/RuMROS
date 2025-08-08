import random
from collections import deque
from draw_maze import draw_maze


class Cell:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False


def generate_maze(width, height, n_rooms=3, max_attempts=20):
    for attempt in range(max_attempts):
        grid, start, end = _generate_single_maze(width, height, n_rooms)
        if is_fully_connected(grid, start):
            print(f"Maze valid after {attempt + 1} attempt(s).")
            return grid, start, end
        else:
            print(f"Maze not fully connected (attempt {attempt + 1}), retrying...")

    print("Failed to generate a fully connected maze after max attempts.")
    return grid, start, end  # return last attempt anyway


def _generate_single_maze(width, height, n_rooms=3):
    grid = [[Cell(x, y) for x in range(width)] for y in range(height)]
    room_positions = []
    room_entrances = []

    directions = [
        (0, -1, 'top', 'bottom'),
        (0,  1, 'bottom', 'top'),
        (-1, 0, 'left', 'right'),
        (1,  0, 'right', 'left')
    ]

    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def carve(x, y):
        current = grid[y][x]
        current.visited = True
        random.shuffle(directions)
        for dx, dy, wall_curr, wall_next in directions:
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny) and (nx, ny) not in room_positions:
                neighbor = grid[ny][nx]
                if not neighbor.visited:
                    current.walls[wall_curr] = False
                    neighbor.walls[wall_next] = False
                    carve(nx, ny)

    # -----------------------
    # Start room (top-left)
    # -----------------------
    for y in range(3):
        for x in range(3):
            cell = grid[y][x]
            cell.visited = True
            room_positions.append((x, y))
            if y > 0: cell.walls['top'] = False
            if y < 2: cell.walls['bottom'] = False
            if x > 0: cell.walls['left'] = False
            if x < 2: cell.walls['right'] = False
    # Entry into maze
    grid[2][1].walls['bottom'] = False
    grid[3][1].walls['top'] = False
    room_entrances.append((3, 1))

    # -----------------------
    # Goal room (bottom-right)
    # -----------------------
    for y in range(height - 3, height):
        for x in range(width - 3, width):
            cell = grid[y][x]
            cell.visited = True
            room_positions.append((x, y))
            if y > height - 3: cell.walls['top'] = False
            if y < height - 1: cell.walls['bottom'] = False
            if x > width - 3: cell.walls['left'] = False
            if x < width - 1: cell.walls['right'] = False
    # Entry into maze
    grid[height - 3][width - 2].walls['top'] = False
    grid[height - 4][width - 2].walls['bottom'] = False
    room_entrances.append((height - 4, width - 2))

    # -----------------------
    # Random rooms (3x3)
    # -----------------------
    def place_random_rooms(n_rooms):
        attempts = 0
        rooms_placed = 0

        while rooms_placed < n_rooms and attempts < 1000:
            attempts += 1
            rx = random.randint(1, width - 4)
            ry = random.randint(1, height - 4)

            # Avoid overlap
            overlap = any((x, y) in room_positions for y in range(ry, ry + 3) for x in range(rx, rx + 3))
            if overlap:
                continue

            for y in range(ry, ry + 3):
                for x in range(rx, rx + 3):
                    room_positions.append((x, y))
                    cell = grid[y][x]
                    cell.visited = True
                    if y > ry: cell.walls['top'] = False
                    if y < ry + 2: cell.walls['bottom'] = False
                    if x > rx: cell.walls['left'] = False
                    if x < rx + 2: cell.walls['right'] = False

            # Create 1–4 random entrances
            openings = random.sample(['top', 'bottom', 'left', 'right'], random.randint(1, 4))
            cx, cy = rx + 1, ry + 1

            for side in openings:
                if side == 'top' and ry > 0:
                    grid[ry][cx].walls['top'] = False
                    grid[ry - 1][cx].walls['bottom'] = False
                    room_entrances.append((ry - 1, cx))
                elif side == 'bottom' and ry + 2 < height - 1:
                    grid[ry + 2][cx].walls['bottom'] = False
                    grid[ry + 3][cx].walls['top'] = False
                    room_entrances.append((ry + 3, cx))
                elif side == 'left' and rx > 0:
                    grid[cy][rx].walls['left'] = False
                    grid[cy][rx - 1].walls['right'] = False
                    room_entrances.append((cy, rx - 1))
                elif side == 'right' and rx + 2 < width - 1:
                    grid[cy][rx + 2].walls['right'] = False
                    grid[cy][rx + 3].walls['left'] = False
                    room_entrances.append((cy, rx + 3))

            rooms_placed += 1

    place_random_rooms(n_rooms)
    draw_maze(grid, "maze_step1_rooms_only.png")

    # -----------------------
    # Maze carving
    # -----------------------
    carve(3, 3)
    draw_maze(grid, "maze_step2_after_carving.png")

    start = (1, 1)
    end = (height - 2, width - 2)
    return grid, start, end


def is_fully_connected(grid, start):
    width, height = len(grid[0]), len(grid)
    visited = set()
    queue = deque([start])

    directions = [
        (0, -1, 'top', 'bottom'),
        (0, 1, 'bottom', 'top'),
        (-1, 0, 'left', 'right'),
        (1, 0, 'right', 'left'),
    ]

    while queue:
        x, y = queue.popleft()
        if (x, y) in visited:
            continue
        visited.add((x, y))
        cell = grid[y][x]

        for dx, dy, wall_curr, wall_next in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                neighbor = grid[ny][nx]
                if not cell.walls[wall_curr] and not neighbor.walls[wall_next]:
                    queue.append((nx, ny))

    return len(visited) == width * height
