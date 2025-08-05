import random
import matplotlib.pyplot as plt

class Cell:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False

def generate_maze(width, height, n_rooms=3):
    grid = [[Cell(x, y) for x in range(width)] for y in range(height)]
    room_positions = []

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
    # Start-Raum (oben links)
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
    # Eingang ins Maze
    grid[2][1].walls['bottom'] = False
    grid[3][1].walls['top'] = False

    # -----------------------
    # Ziel-Raum (unten rechts)
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
    # Eingang ins Maze
    grid[height - 3][width - 2].walls['top'] = False
    grid[height - 4][width - 2].walls['bottom'] = False

    # --------------------------------
    # Zufällige Räume (3x3, konfigurierbar)
    # --------------------------------
    def place_random_rooms(n_rooms):
        attempts = 0
        rooms_placed = 0

        while rooms_placed < n_rooms and attempts < 1000:
            attempts += 1
            rx = random.randint(1, width - 4)
            ry = random.randint(1, height - 4)

            # Kollision mit bestehenden Räumen vermeiden
            overlap = False
            for y in range(ry, ry + 3):
                for x in range(rx, rx + 3):
                    if (x, y) in room_positions:
                        overlap = True
            if overlap:
                continue

            # Raum markieren & innen öffnen
            for y in range(ry, ry + 3):
                for x in range(rx, rx + 3):
                    room_positions.append((x, y))
                    cell = grid[y][x]
                    cell.visited = True
                    if y > ry: cell.walls['top'] = False
                    if y < ry + 2: cell.walls['bottom'] = False
                    if x > rx: cell.walls['left'] = False
                    if x < rx + 2: cell.walls['right'] = False

            # Eingänge (mind. 1, max. 4)
            openings = random.sample(['top', 'bottom', 'left', 'right'], random.randint(1, 2))
            center_x, center_y = rx + 1, ry + 1

            for side in openings:
                if side == 'top' and ry > 0:
                    grid[ry][center_x].walls['top'] = False
                    grid[ry - 1][center_x].walls['bottom'] = False
                elif side == 'bottom' and ry + 2 < height - 1:
                    grid[ry + 2][center_x].walls['bottom'] = False
                    grid[ry + 3][center_x].walls['top'] = False
                elif side == 'left' and rx > 0:
                    grid[center_y][rx].walls['left'] = False
                    grid[center_y][rx - 1].walls['right'] = False
                elif side == 'right' and rx + 2 < width - 1:
                    grid[center_y][rx + 2].walls['right'] = False
                    grid[center_y][rx + 3].walls['left'] = False

            rooms_placed += 1

    # Räume platzieren
    place_random_rooms(n_rooms)

    # Maze schnitzen ab Punkt außerhalb der Räume
    carve(3, 3)

    start = (1, 1)
    end = (width - 2, height - 2)
    return grid, start, end

# Verwendung
# Generieren + Pfad finden + Zeichnen
from draw_maze import draw_maze, find_path
maze, start, end = generate_maze(20, 20, n_rooms=4)
path = find_path(maze, start, end)
draw_maze(maze, path=path, start=start, end=end)

