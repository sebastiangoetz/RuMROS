from matplotlib import pyplot as plt
from collections import deque


def draw_maze(grid, path=None, start=None, end=None):
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

    # Pfad plotten
    if path:
        px, py = zip(*[(x + 0.5, y + 0.5) for x, y in path])
        ax.plot(px, py, color='red', linewidth=2, label='Lösungsweg')

    # Start und Ziel markieren
    if start:
        ax.scatter(start[0] + 0.5, start[1] + 0.5, color='red', label='Start', s=100)
    if end:
        ax.scatter(end[0] + 0.5, end[1] + 0.5, color='green', label='Ziel', s=100)

    ax.set_aspect('equal')
    ax.invert_yaxis()
    plt.xticks([]), plt.yticks([])
    plt.savefig("maze.png")


def find_path(grid, start, end):
    height = len(grid)
    width = len(grid[0])
    visited = [[False]*width for _ in range(height)]
    parent = {}

    queue = deque()
    queue.append(start)
    visited[start[1]][start[0]] = True

    directions = {
        'top': (0, -1),
        'bottom': (0, 1),
        'left': (-1, 0),
        'right': (1, 0)
    }

    while queue:
        x, y = queue.popleft()
        if (x, y) == end:
            break

        cell = grid[y][x]
        for wall, (dx, dy) in directions.items():
            if not cell.walls[wall]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                    visited[ny][nx] = True
                    parent[(nx, ny)] = (x, y)
                    queue.append((nx, ny))

    # Pfad zurückverfolgen
    path = []
    current = end
    while current != start:
        path.append(current)
        current = parent.get(current)
        if current is None:
            return []  # Kein Pfad gefunden
    path.append(start)
    path.reverse()
    return path
