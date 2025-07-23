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
    return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{pose}</pose>
    </include>"""

# Maze generation using recursive backtracking (depth-first search)
carve(0, 0)

wall_blocks = []
used = set()
index = 0

# Horizontal walls
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
                wall_blocks.append(block)
                index += 1
                offset += s
            x += length
        else:
            x += 1

# Vertical walls
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
                wall_blocks.append(block)
                index += 1
                offset += s
            y += length
        else:
            y += 1

# Write full .world file
with open("maze_world.world", "w") as f:
    f.write('<?xml version="1.0"?>\n')
    f.write('<sdf version="1.8">\n')
    f.write('  <world name="maze_world">\n')

    # Plugins and physics
    f.write("""    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin name='ignition::gazebo::systems::Physics'
      filename='libignition-gazebo-physics-system.so' />
    <plugin name='ignition::gazebo::systems::UserCommands'
      filename='libignition-gazebo-user-commands-system.so' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster'
      filename='libignition-gazebo-scene-broadcaster-system.so' />
    <plugin name='ignition::gazebo::systems::Imu' filename='ignition-gazebo-imu-system' />
    <plugin name='ignition::gazebo::systems::Sensors' filename='ignition-gazebo-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>\n""")

    # Sunlight and ground
    f.write("""    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <name>ground</name>
      <uri>model://ground</uri>
    </include>\n""")

    # Maze wall includes
    f.write("    <!-- Generated maze wall segments -->\n")
    for block in wall_blocks:
        f.write(block + "\n")

    # Close world and sdf
    f.write("  </world>\n")
    f.write("</sdf>\n")

print(f"[✔] Maze world generated with {index} wall segments.")
print("[→] Output written to 'maze_world.world'")
