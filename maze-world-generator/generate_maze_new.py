import random
import matplotlib.pyplot as plt

def generate_maze(width, height):

    # Anfang: alles Wände (1 = Wand, 0 = Weg)
    maze = [[1 for _ in range(width)] for _ in range(height)]

    # Bewegungsrichtungen: (dy, dx)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def carve(x, y):
        maze[y][x] = 0
        dirs = directions[:]
        random.shuffle(dirs)
        for dy, dx in dirs:
            nx, ny = x + dx * 2, y + dy * 2
            if 0 < ny < height and 0 < nx < width and maze[ny][nx] == 1:
                maze[y + dy][x + dx] = 0
                carve(nx, ny)

    # Startpunkt
    carve(1, 1)

    return maze

print("Generating maze...")
maze = generate_maze(19,19)

plt.figure(figsize=(6,6))
plt.imshow(maze, cmap="binary")
plt.scatter(1, 1, color='red', label='Start', s=100)
plt.scatter(len(maze)-2, len(maze)-2, color='green', label='End', s=100)
plt.xticks([]), plt.yticks([])
plt.show()