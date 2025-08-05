import random
import matplotlib.pyplot as plt

class Cell:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False

def generate_maze(width, height):
    grid = [[Cell(x, y) for x in range(width)] for y in range(height)]

    directions = [
        (0, -1, 'top', 'bottom'),
        (0,  1, 'bottom', 'top'),
        (-1, 0, 'left', 'right'),
        (1,  0, 'right', 'left')
    ]

    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def in_start_or_end_area(x, y):
        return (0 <= x < 3 and 0 <= y < 3) or (width - 3 <= x < width and height - 3 <= y < height)

    def carve(x, y):
        current = grid[y][x]
        current.visited = True
        random.shuffle(directions)
        for dx, dy, wall_curr, wall_next in directions:
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny) and not in_start_or_end_area(nx, ny):
                neighbor = grid[ny][nx]
                if not neighbor.visited:
                    current.walls[wall_curr] = False
                    neighbor.walls[wall_next] = False
                    carve(nx, ny)

    # Maze schnitzen (außerhalb der Räume)
    carve(3, 3)

    # ----------------------
    # Start-Raum (oben links)
    # ----------------------
    for y in range(3):
        for x in range(3):
            cell = grid[y][x]
            cell.visited = True
            # Wände innerhalb des Raums entfernen
            if y > 0:
                cell.walls['top'] = False
            if y < 2:
                cell.walls['bottom'] = False
            if x > 0:
                cell.walls['left'] = False
            if x < 2:
                cell.walls['right'] = False
    # Eingang zur Maze (unten, mittig)
    grid[2][1].walls['bottom'] = False
    grid[3][1].walls['top'] = False

    # ----------------------
    # Ziel-Raum (unten rechts)
    # ----------------------
    for y in range(height - 3, height):
        for x in range(width - 3, width):
            cell = grid[y][x]
            cell.visited = True
            # Wände innerhalb des Raums entfernen
            if y > height - 3:
                cell.walls['top'] = False
            if y < height - 1:
                cell.walls['bottom'] = False
            if x > width - 3:
                cell.walls['left'] = False
            if x < width - 1:
                cell.walls['right'] = False
    # Ausgang zur Maze (oben, mittig)
    grid[height - 3][width - 2].walls['top'] = False
    grid[height - 4][width - 2].walls['bottom'] = False

    return grid

def draw_maze(grid):
    height = len(grid)
    width = len(grid[0])
    fig, ax = plt.subplots(figsize=(6, 6))

    for y in range(height):
        for x in range(width):
            cell = grid[y][x]
            if cell.walls['top']:
                ax.plot([x, x+1], [y, y], color='black')
            if cell.walls['right']:
                ax.plot([x+1, x+1], [y, y+1], color='black')
            if cell.walls['bottom']:
                ax.plot([x, x+1], [y+1, y+1], color='black')
            if cell.walls['left']:
                ax.plot([x, x], [y, y+1], color='black')

    # Start- und Zielbereiche markieren (Zentrum der Bereiche)
    ax.scatter(1.5, 1.5, color='red', label='Start', s=100)
    ax.scatter(width - 1.5, height - 1.5, color='green', label='End', s=100)

    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.xticks([]), plt.yticks([])
    plt.savefig("maze.png")

# Verwendung
print("Generating maze...")
maze = generate_maze(20, 20)
draw_maze(maze)
