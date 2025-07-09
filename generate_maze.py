import random

# Maze parameters
maze_width = 20
maze_height = 20
cell_size = 1.0

# Bitmasks for directions
N, E, S, W = 1, 2, 4, 8
DX = {E: 1, W: -1, N: 0, S: 0}
DY = {E: 0, W: 0, N: -1, S: 1}
OPPOSITE = {E: W, W: E, N: S, S: N}

# Larger segments improve performance
SEGMENT_SIZES = [8, 4, 2, 1]

origin_x = -maze_width * cell_size / 2.0
origin_y = -maze_height * cell_size / 2.0

maze = [[0 for _ in range(maze_width)] for _ in range(maze_height)]

def carve(x, y):
    dirs = [N, S, E, W]
    random.shuffle(dirs)
    for d in dirs:
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < maze_width and 0 <= ny < maze_height and maze[ny][nx] == 0:
            maze[y][x] |= d
            maze[ny][nx] |= OPPOSITE[d]
            carve(nx, ny)

def choose_segments(length):
    segments = []
    for s in SEGMENT_SIZES:
        while length >= s:
            segments.append(s)
            length -= s
    return segments

def generate_wall_block(x, y, orientation, segment, index):
    model = f"wall_{segment}m"
    name = f"{model}_{index}"
    if orientation == "H":
        cx = origin_x + x + (segment / 2.0) * cell_size
        cy = origin_y + y
        yaw = 0
    else:
        cx = origin_x + x
        cy = origin_y + y + (segment / 2.0) * cell_size
        yaw = 1.5708
    pose = f"{cx:.2f} {cy:.2f} 0 0 0 {yaw}"
    return f"""  <include>
    <name>{name}</name>
    <uri>model://{model}</uri>
    <pose>{pose}</pose>
  </include>"""

carve(0, 0)

output = []
used = set()
index = 0

# Horizontal walls (south edge)
for y in range(maze_height + 1):
    x = 0
    while x < maze_width:
        if y < maze_height and not (maze[y][x] & S):
            if (x, y, "H") in used:
                x += 1
                continue
            length = 0
            while x + length < maze_width and y < maze_height and not (maze[y][x + length] & S):
                used.add((x + length, y, "H"))
                length += 1
            segments = choose_segments(length)
            offset = 0
            for s in segments:
                block = generate_wall_block(x + offset, y + 1, "H", s, index)
                output.append(block)
                index += 1
                offset += s
            x += length
        else:
            x += 1

# Vertical walls (east edge)
for x in range(maze_width + 1):
    y = 0
    while y < maze_height:
        if x < maze_width and not (maze[y][x] & E):
            if (x, y, "V") in used:
                y += 1
                continue
            length = 0
            while y + length < maze_height and x < maze_width and not (maze[y + length][x] & E):
                used.add((x, y + length, "V"))
                length += 1
            segments = choose_segments(length)
            offset = 0
            for s in segments:
                block = generate_wall_block(x + 1, y + offset, "V", s, index)
                output.append(block)
                index += 1
                offset += s
            y += length
        else:
            y += 1

with open("maze_wall_includes.txt", "w") as f:
    f.write("<!-- Generated maze wall segments -->\n")
    for line in output:
        f.write(line + "\n")

print(f"[✔] Maze generated with {index} wall segments.")
print("[→] Output written to 'maze_wall_includes.txt'")
