import random

# Maze parameters
maze_width = 20
maze_height = 20
cell_size = 1.0
ROOM_SIZE = 3  # 3x3 cells for rooms
NUM_ADDITIONAL_ROOMS = 2  # Number of random rooms to add

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

def iterative_carve(start_points):
    stack = start_points[:]
    while stack:
        x, y = stack[-1]
        dirs = [N, E, S, W]
        random.shuffle(dirs)
        found = False
        for d in dirs:
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < maze_width and 0 <= ny < maze_height and maze[ny][nx] == 0:
                maze[y][x] |= d
                maze[ny][nx] |= OPPOSITE[d]
                stack.append((nx, ny))
                found = True
                break
        if not found:
            stack.pop()

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
        cx = origin_x + (x + segment / 2.0) * cell_size
        cy = origin_y + y * cell_size
        yaw = 0
    else:
        cx = origin_x + x * cell_size
        cy = origin_y + (y + segment / 2.0) * cell_size
        yaw = 1.5708
    pose = f"{cx:.2f} {cy:.2f} 0 0 0 {yaw}"
    return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{pose}</pose>
    </include>"""

def create_room(room_x, room_y):
    """Create a room at specified top-left coordinates with entrances"""
    room_cells = []
    # Mark all cells in the room as open
    for y in range(room_y, room_y + ROOM_SIZE):
        for x in range(room_x, room_x + ROOM_SIZE):
            if 0 <= x < maze_width and 0 <= y < maze_height:
                maze[y][x] = 15  # All walls open
                room_cells.append((x, y))
    
    # Determine possible entrance sides
    possible_sides = []
    if room_y > 0: possible_sides.append("N")
    if room_x + ROOM_SIZE < maze_width: possible_sides.append("E")
    if room_y + ROOM_SIZE < maze_height: possible_sides.append("S")
    if room_x > 0: possible_sides.append("W")
    
    # Ensure at least one entrance
    if not possible_sides:
        # If room is completely isolated, force one entrance (shouldn't happen normally)
        possible_sides = ["N", "E", "S", "W"]
    
    # Create entrances (1-4)
    num_entrances = random.randint(1, min(4, len(possible_sides)))
    entrance_sides = random.sample(possible_sides, num_entrances)
    
    entrance_points = []
    
    for side in entrance_sides:
        if side == "N":  # North side
            entrance_x = random.randint(room_x, room_x + ROOM_SIZE - 1)
            entrance_y = room_y
            maze[entrance_y][entrance_x] &= ~S  # Remove south wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "E":  # East side
            entrance_x = room_x + ROOM_SIZE - 1
            entrance_y = random.randint(room_y, room_y + ROOM_SIZE - 1)
            maze[entrance_y][entrance_x] &= ~W  # Remove west wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "S":  # South side
            entrance_x = random.randint(room_x, room_x + ROOM_SIZE - 1)
            entrance_y = room_y + ROOM_SIZE - 1
            maze[entrance_y][entrance_x] &= ~N  # Remove north wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "W":  # West side
            entrance_x = room_x
            entrance_y = random.randint(room_y, room_y + ROOM_SIZE - 1)
            maze[entrance_y][entrance_x] &= ~E  # Remove east wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
    
    return room_cells, entrance_points

# Create fixed start and goal rooms
start_room, start_entrances = create_room(0, 0)
goal_room, goal_entrances = create_room(maze_width - ROOM_SIZE, maze_height - ROOM_SIZE)

# Create random rooms
all_rooms = [start_room, goal_room]
all_entrances = start_entrances + goal_entrances
additional_rooms_info = []

for _ in range(NUM_ADDITIONAL_ROOMS):
    while True:
        # Ensure rooms don't overlap and have space for entrances
        room_x = random.randint(1, maze_width - ROOM_SIZE - 1)
        room_y = random.randint(1, maze_height - ROOM_SIZE - 1)
        
        # Check for overlap with existing rooms
        overlap = False
        for room in all_rooms:
            for (x, y) in room:
                if (room_x <= x < room_x + ROOM_SIZE and 
                    room_y <= y < room_y + ROOM_SIZE):
                    overlap = True
                    break
            if overlap:
                break
        
        if not overlap:
            room_cells, entrances = create_room(room_x, room_y)
            all_rooms.append(room_cells)
            all_entrances.extend(entrances)
            additional_rooms_info.append((room_x, room_y, entrances))
            break

# Find starting points for maze generation from all entrances
start_points = all_entrances

# Generate maze using iterative DFS
iterative_carve(start_points)

wall_blocks = []
used = set()
index = 0

# Generate outer boundary walls (always present)
# Bottom wall (y=0)
segments = choose_segments(maze_width)
offset = 0
for s in segments:
    block = generate_wall_block(offset, 0, "H", s, index)
    wall_blocks.append(block)
    index += 1
    offset += s

# Top wall (y = maze_height)
segments = choose_segments(maze_width)
offset = 0
for s in segments:
    block = generate_wall_block(offset, maze_height, "H", s, index)
    wall_blocks.append(block)
    index += 1
    offset += s

# Left wall (x=0)
segments = choose_segments(maze_height)
offset = 0
for s in segments:
    block = generate_wall_block(0, offset, "V", s, index)
    wall_blocks.append(block)
    index += 1
    offset += s

# Right wall (x = maze_width)
segments = choose_segments(maze_height)
offset = 0
for s in segments:
    block = generate_wall_block(maze_width, offset, "V", s, index)
    wall_blocks.append(block)
    index += 1
    offset += s

# Generate inner walls
# Horizontal walls (between rows, y from 1 to maze_height-1)
for y in range(1, maze_height):
    x = 0
    while x < maze_width:
        # Check if wall should be placed between (x, y-1) and (x, y)
        if not (maze[y-1][x] & S):
            if (x, y, "H") in used:
                x += 1
                continue
            length = 0
            while (x + length < maze_width and 
                   not (maze[y-1][x + length] & S) and 
                   (x + length, y, "H") not in used):
                used.add((x + length, y, "H"))
                length += 1
            segments = choose_segments(length)
            offset = 0
            for s in segments:
                block = generate_wall_block(x + offset, y, "H", s, index)
                wall_blocks.append(block)
                index += 1
                offset += s
            x += length
        else:
            x += 1

# Vertical walls (between columns, x from 1 to maze_width-1)
for x in range(1, maze_width):
    y = 0
    while y < maze_height:
        # Check if wall should be placed between (x-1, y) and (x, y)
        if not (maze[y][x-1] & E):
            if (x, y, "V") in used:
                y += 1
                continue
            length = 0
            while (y + length < maze_height and 
                   not (maze[y + length][x-1] & E) and 
                   (x, y + length, "V") not in used):
                used.add((x, y + length, "V"))
                length += 1
            segments = choose_segments(length)
            offset = 0
            for s in segments:
                block = generate_wall_block(x, y + offset, "V", s, index)
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

# Print room information
print(f"[✔] Maze world generated with {index} wall segments.")
print(f"[→] Start room: bottom-left (0,0) with entrances at {start_entrances}")
print(f"[→] Goal room: top-right ({maze_width-ROOM_SIZE},{maze_height-ROOM_SIZE}) with entrances at {goal_entrances}")
for i, (x, y, entrances) in enumerate(additional_rooms_info, 1):
    print(f"[→] Additional room {i}: top-left ({x},{y}) with entrances at {entrances}")
print("[→] Output written to 'maze_world.world'")