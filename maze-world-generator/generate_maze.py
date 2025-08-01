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

def initialize_maze():
    return [[0 for _ in range(maze_width)] for _ in range(maze_height)]

def initialize_region_map():
    return [[0 for _ in range(maze_width)] for _ in range(maze_height)]

def can_place_room(room_x, room_y, rooms):
    """Check if a room can be placed without overlapping existing rooms"""
    # Check bounds
    if room_x < 0 or room_y < 0 or room_x + ROOM_SIZE > maze_width or room_y + ROOM_SIZE > maze_height:
        return False
    
    # Check overlap with existing rooms
    for rx, ry, _ in rooms:
        if not (room_x + ROOM_SIZE <= rx or rx + ROOM_SIZE <= room_x or
                room_y + ROOM_SIZE <= ry or ry + ROOM_SIZE <= room_y):
            return False
    return True

def create_room(room_x, room_y, region_id, maze, region_map):
    """Create a room at specified top-left coordinates and mark its region"""
    # Mark all cells in the room as open
    for y in range(room_y, room_y + ROOM_SIZE):
        for x in range(room_x, room_x + ROOM_SIZE):
            if 0 <= x < maze_width and 0 <= y < maze_height:
                maze[y][x] = 15  # All walls open
                region_map[y][x] = region_id
    
    # Create entrances to connect room to maze
    entrance_points = []
    possible_sides = []
    if room_y > 0: possible_sides.append("N")
    if room_x + ROOM_SIZE < maze_width: possible_sides.append("E")
    if room_y + ROOM_SIZE < maze_height: possible_sides.append("S")
    if room_x > 0: possible_sides.append("W")
    
    if not possible_sides:
        return entrance_points
    
    # Create entrances (1-4)
    num_entrances = random.randint(1, min(4, len(possible_sides)))
    entrance_sides = random.sample(possible_sides, num_entrances)
    
    for side in entrance_sides:
        if side == "N":  # North side
            entrance_x = room_x + random.randint(0, ROOM_SIZE-1)
            entrance_y = room_y
            maze[entrance_y][entrance_x] &= ~N  # Remove north wall
            if entrance_y > 0:
                maze[entrance_y-1][entrance_x] &= ~S  # Remove south wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "E":  # East side
            entrance_x = room_x + ROOM_SIZE - 1
            entrance_y = room_y + random.randint(0, ROOM_SIZE-1)
            maze[entrance_y][entrance_x] &= ~E  # Remove east wall
            if entrance_x < maze_width-1:
                maze[entrance_y][entrance_x+1] &= ~W  # Remove west wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "S":  # South side
            entrance_x = room_x + random.randint(0, ROOM_SIZE-1)
            entrance_y = room_y + ROOM_SIZE - 1
            maze[entrance_y][entrance_x] &= ~S  # Remove south wall
            if entrance_y < maze_height-1:
                maze[entrance_y+1][entrance_x] &= ~N  # Remove north wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
            
        elif side == "W":  # West side
            entrance_x = room_x
            entrance_y = room_y + random.randint(0, ROOM_SIZE-1)
            maze[entrance_y][entrance_x] &= ~W  # Remove west wall
            if entrance_x > 0:
                maze[entrance_y][entrance_x-1] &= ~E  # Remove east wall of outside cell
            entrance_points.append((entrance_x, entrance_y))
    
    return entrance_points

def iterative_carve(start_x, start_y, region_id, maze, region_map):
    """Carve a maze starting from a point using DFS and assign region ID"""
    stack = [(start_x, start_y)]
    maze[start_y][start_x] = 0
    region_map[start_y][start_x] = region_id
    
    while stack:
        x, y = stack[-1]
        dirs = [N, E, S, W]
        random.shuffle(dirs)
        found = False
        
        for d in dirs:
            nx, ny = x + DX[d], y + DY[d]
            if (0 <= nx < maze_width and 0 <= ny < maze_height and 
                maze[ny][nx] == 0 and region_map[ny][nx] == 0):
                
                # Carve path
                maze[y][x] |= d
                maze[ny][nx] |= OPPOSITE[d]
                region_map[ny][nx] = region_id
                stack.append((nx, ny))
                found = True
                break
        
        if not found:
            stack.pop()

def find_connectors(region_map):
    """Find all potential connectors between regions"""
    connectors = []
    
    for y in range(maze_height):
        for x in range(maze_width):
            # Only consider solid walls as potential connectors
            if region_map[y][x] != 0:
                continue
                
            # Check all directions for different regions
            neighbor_regions = set()
            
            for d in [N, E, S, W]:
                nx, ny = x + DX[d], y + DY[d]
                if 0 <= nx < maze_width and 0 <= ny < maze_height:
                    region_id = region_map[ny][nx]
                    if region_id != 0:
                        neighbor_regions.add(region_id)
            
            # If we found at least two different regions, this is a connector
            if len(neighbor_regions) >= 2:
                regions = list(neighbor_regions)
                # Choose a random direction to open
                for d in [N, E, S, W]:
                    nx, ny = x + DX[d], y + DY[d]
                    if 0 <= nx < maze_width and 0 <= ny < maze_height:
                        if region_map[ny][nx] != 0:
                            connectors.append((x, y, d, regions[0], regions[1]))
                            break
    
    return connectors

def connect_regions(connectors, maze, region_map):
    """Connect regions using the minimum spanning tree approach"""
    if not connectors:
        print("Warning: No connectors found. Trying to force connections.")
        return
    
    # Get all unique regions
    regions = set()
    for _, _, _, a, b in connectors:
        regions.add(a)
        regions.add(b)
    
    if not regions:
        # Collect regions directly from region map
        regions = set()
        for y in range(maze_height):
            for x in range(maze_width):
                rid = region_map[y][x]
                if rid != 0:
                    regions.add(rid)
        
        if not regions:
            print("Critical error: No regions found!")
            return
    
    # Start with a random region
    connected = set([random.choice(list(regions))])
    
    while len(connected) < len(regions):
        # Find connectors that bridge connected and unconnected regions
        candidates = []
        for c in connectors:
            _, _, _, a, b = c
            if (a in connected and b not in connected) or (b in connected and a not in connected):
                candidates.append(c)
        
        if not candidates:
            # If we still have unconnected regions, try any connector
            if regions - connected:
                candidates = connectors
            else:
                break
        
        # Choose a random candidate
        chosen = random.choice(candidates)
        x, y, d, a, b = chosen
        
        # Open the connection
        maze[y][x] |= d
        region_map[y][x] = a  # Assign to one of the regions
        nx, ny = x + DX[d], y + DY[d]
        if 0 <= nx < maze_width and 0 <= ny < maze_height:
            maze[ny][nx] |= OPPOSITE[d]
        
        # Update regions
        connected.add(a)
        connected.add(b)
        
        # Remove used connector
        if chosen in connectors:
            connectors.remove(chosen)

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

def generate_ascii_maze(maze, room_info, filename="maze_layout.txt"):
    """Generate an accurate ASCII representation of the maze"""
    # Create a set of room cells for faster lookup
    room_cells = {}
    for info in room_info:
        name, room_x, room_y, region, entrances = info
        for y in range(room_y, room_y + ROOM_SIZE):
            for x in range(room_x, room_x + ROOM_SIZE):
                if 0 <= x < maze_width and 0 <= y < maze_height:
                    if name == "Start":
                        room_cells[(x, y)] = 'S'
                    elif name == "Goal":
                        room_cells[(x, y)] = 'G'
                    else:
                        room_cells[(x, y)] = '.'

    # Build the ASCII grid
    grid = [[' ' for _ in range(maze_width * 2 + 1)] 
            for _ in range(maze_height * 2 + 1)]
    
    # Draw outer walls
    for x in range(maze_width * 2 + 1):
        grid[0][x] = '#'
        grid[maze_height * 2][x] = '#'
    for y in range(maze_height * 2 + 1):
        grid[y][0] = '#'
        grid[y][maze_width * 2] = '#'
    
    # Draw walls and paths
    for y in range(maze_height):
        for x in range(maze_width):
            # Calculate grid positions
            gx = x * 2 + 1
            gy = y * 2 + 1
            
            # Draw room/path center
            if (x, y) in room_cells:
                grid[gy][gx] = room_cells[(x, y)]
            elif maze[y][x] != 0:  # Path
                grid[gy][gx] = ' '
            else:  # Wall
                grid[gy][gx] = '#'
            
            # Draw north wall
            if maze[y][x] & N:
                grid[gy-1][gx] = '#'
            
            # Draw south wall
            if maze[y][x] & S:
                grid[gy+1][gx] = '#'
            
            # Draw west wall
            if maze[y][x] & W:
                grid[gy][gx-1] = '#'
            
            # Draw east wall
            if maze[y][x] & E:
                grid[gy][gx+1] = '#'
    
    # Save to file
    with open(filename, "w") as f:
        for row in grid:
            f.write(''.join(row) + '\n')

# Main generation logic ========================================================
maze = initialize_maze()
region_map = initialize_region_map()

# Place rooms
rooms = []
room_info = []
region_id = 1

# Fixed start room
start_room = (0, 0)
if can_place_room(*start_room, rooms):
    entrances = create_room(*start_room, region_id, maze, region_map)
    rooms.append((*start_room, region_id))
    room_info.append(("Start", start_room[0], start_room[1], region_id, entrances))
    region_id += 1

# Fixed goal room
goal_room = (maze_width - ROOM_SIZE, maze_height - ROOM_SIZE)
if can_place_room(*goal_room, rooms):
    entrances = create_room(*goal_room, region_id, maze, region_map)
    rooms.append((*goal_room, region_id))
    room_info.append(("Goal", goal_room[0], goal_room[1], region_id, entrances))
    region_id += 1

# Additional rooms
for i in range(NUM_ADDITIONAL_ROOMS):
    placed = False
    for _ in range(100):  # Try up to 100 times to place room
        room_x = random.randint(0, maze_width - ROOM_SIZE - 1)
        room_y = random.randint(0, maze_height - ROOM_SIZE - 1)
        
        if can_place_room(room_x, room_y, rooms):
            entrances = create_room(room_x, room_y, region_id, maze, region_map)
            rooms.append((room_x, room_y, region_id))
            room_info.append((f"Room {i+1}", room_x, room_y, region_id, entrances))
            region_id += 1
            placed = True
            break
    
    if not placed:
        print(f"Could not place additional room {i+1}")

# Fill remaining areas with mazes
for y in range(maze_height):
    for x in range(maze_width):
        if maze[y][x] == 0 and region_map[y][x] == 0:
            iterative_carve(x, y, region_id, maze, region_map)
            region_id += 1

# Connect all regions
connectors = find_connectors(region_map)
connect_regions(connectors, maze, region_map)

# Generate ASCII maze visualization
generate_ascii_maze(maze, room_info)

# Generate wall segments ======================================================
wall_blocks = []
used = set()
index = 0

# Generate outer boundary walls
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
for info in room_info:
    name, x, y, region, entrances = info
    print(f"[→] {name}: top-left ({x},{y}) with entrances at {entrances}")
print("[→] Gazebo world written to 'maze_world.world'")
print("[→] ASCII visualization written to 'maze_layout.txt'")