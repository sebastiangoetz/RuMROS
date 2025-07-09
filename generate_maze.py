import random

# Maze-Größe in Zellen
SIZE = 20
CELL_SIZE = 1.0
WALL_MODEL = "wall_1m"

# Pfad zur Ausgabedatei
OUTPUT_FILE = "generated_maze.txt"

# Jede Zelle bekommt 4 Wände (rechts, unten) falls nicht verbunden
maze = [[0 for _ in range(SIZE)] for _ in range(SIZE)]
visited = [[False for _ in range(SIZE)] for _ in range(SIZE)]

# Richtungen: (dx, dy, bitmask)
DIRS = [
    (0, -1, 1),  # oben
    (1, 0, 2),   # rechts
    (0, 1, 4),   # unten
    (-1, 0, 8),  # links
]

# Rückwärts-Masken für Nachbarn
OPPOSITE = {1: 4, 2: 8, 4: 1, 8: 2}


def in_bounds(x, y):
    return 0 <= x < SIZE and 0 <= y < SIZE


def carve(x, y):
    visited[y][x] = True
    dirs = DIRS[:]
    random.shuffle(dirs)

    for dx, dy, mask in dirs:
        nx, ny = x + dx, y + dy
        if in_bounds(nx, ny) and not visited[ny][nx]:
            maze[y][x] |= mask
            maze[ny][nx] |= OPPOSITE[mask]
            carve(nx, ny)


def wall_block(x, y, rotation, name_id):
    pose = f"{x:.1f} {y:.1f} 0 0 0 {rotation:.4f}"
    return f"""  <include>
    <name>maze_wall_{name_id}</name>
    <uri>model://{WALL_MODEL}</uri>
    <pose>{pose}</pose>
  </include>"""


def generate_maze():
    carve(0, 0)
    wall_id = 0
    sdf_blocks = []

    for y in range(SIZE):
        for x in range(SIZE):
            cx = -SIZE / 2 + x + 0.5
            cy = -SIZE / 2 + y + 0.5
            cell = maze[y][x]

            # Rechte Wand, wenn nicht nach rechts offen
            if not (cell & 2):
                sdf_blocks.append(wall_block(cx + 0.5, cy, 1.5708, wall_id))
                wall_id += 1

            # Untere Wand, wenn nicht nach unten offen
            if not (cell & 4):
                sdf_blocks.append(wall_block(cx, cy + 0.5, 0, wall_id))
                wall_id += 1

    # Rand rechts und unten
    for i in range(SIZE):
        cx = -SIZE / 2 + i + 0.5
        sdf_blocks.append(wall_block(cx, -SIZE / 2, 0, wall_id))  # unterer Rand
        wall_id += 1
        cy = -SIZE / 2 + i + 0.5
        sdf_blocks.append(wall_block(SIZE / 2, cy, 1.5708, wall_id))  # rechter Rand
        wall_id += 1

    return sdf_blocks


if __name__ == "__main__":
    print(f"Generiere {SIZE}x{SIZE} Maze ...")
    maze_blocks = generate_maze()
    with open(OUTPUT_FILE, "w") as f:
        f.write("<!-- AUTOMATISCH GENERIERTES MAZE -->\n")
        for block in maze_blocks:
            f.write(block + "\n")
    print(f"✔️ Maze geschrieben nach {OUTPUT_FILE}")
