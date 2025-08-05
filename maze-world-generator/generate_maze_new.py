import random
import matplotlib.pyplot as plt

class Cell:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.walls = {'top': True, 'right': True, 'bottom': True, 'left': True}
        self.visited = False

def generate_maze(width, height):
    # Gitter aus Cell-Objekten
    grid = [[Cell(x, y) for x in range(width)] for y in range(height)]

    # Bewegungsrichtungen mit Wandreferenzen
    directions = [
        (0, -1, 'top', 'bottom'),   # nach oben
        (0,  1, 'bottom', 'top'),   # nach unten
        (-1, 0, 'left', 'right'),   # nach links
        (1,  0, 'right', 'left')    # nach rechts
    ]

    def in_bounds(x, y):
        return 0 <= x < width and 0 <= y < height

    def carve(x, y):
        current = grid[y][x]
        current.visited = True
        random.shuffle(directions)
        for dx, dy, wall_curr, wall_next in directions:
            nx, ny = x + dx, y + dy
            if in_bounds(nx, ny):
                neighbor = grid[ny][nx]
                if not neighbor.visited:
                    current.walls[wall_curr] = False
                    neighbor.walls[wall_next] = False
                    carve(nx, ny)

    carve(0, 0)
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

    # Start und Ende markieren
    ax.scatter(0.5, 0.5, color='red', label='Start', s=100)
    ax.scatter(width - 0.5, height - 0.5, color='green', label='End', s=100)

    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.xticks([]), plt.yticks([])
    plt.show()

# Verwendung
print("Generating maze...")
maze = generate_maze(20, 20)
draw_maze(maze)
