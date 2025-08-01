import random
from collections import defaultdict

# Maze parameters
maze_width = 20
maze_height = 20
cell_size = 1.0
ROOM_SIZE = 3  # 3x3 cells for rooms
NUM_ADDITIONAL_ROOMS = 2  # Number of random rooms to add
DEAD_END_REMOVAL_CHANCE = 0.5  # Chance to keep interesting dead ends

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
region_map = [[0 for _ in range(maze_width)] for _ in range(maze_height)]

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

def create_room(room_x, room_y, region_id):
    """Create a room at specified top-left coordinates and mark its region"""
    room_cells = []
    # Mark all cells in the room as open
    for y in range(room_y, room_y + ROOM_SIZE):
        for x in range(room_x, room_x + ROOM_SIZE):
            if 0 <= x < maze_width and 0 <= y < maze_height:
                maze[y][x] = 15  # All walls open
                region_map[y][x] = region_id
                room_cells.append((x, y))
    return room_cells

def iterative_carve(start_x, start_y, region_id):
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

def find_connectors():
    """Find all potential connectors between regions (improved version)"""
    connectors = []
    
    for y in range(maze_height):
        for x in range(maze_width):
            # Nur feste Wände können Verbindungspunkte sein
            if maze[y][x] != 0:
                continue
                
            # Prüfe alle 4 Richtungen nach Regionen
            neighbor_regions = set()
            valid_directions = []
            
            for d in [N, E, S, W]:
                nx, ny = x + DX[d], y + DY[d]
                if 0 <= nx < maze_width and 0 <= ny < maze_height:
                    region_id = region_map[ny][nx]
                    if region_id != 0:
                        neighbor_regions.add(region_id)
                        valid_directions.append(d)
            
            # Wenn mindestens 1 Region gefunden wurde
            if len(neighbor_regions) >= 1:
                # Wähle eine zufällige Richtung
                d = random.choice(valid_directions) if valid_directions else N
                connectors.append((x, y, d, 0, list(neighbor_regions)[0]))
    
    return connectors

def connect_regions(connectors):
    """Connect regions with improved error handling"""
    if not connectors:
        print("Warnung: Keine Verbindungspunkte gefunden. Erzwinge direkte Verbindungen.")
        return
    
    regions = set()
    for _, _, _, a, b in connectors:
        if a != 0: regions.add(a)
        if b != 0: regions.add(b)
    
    if not regions:
        # Sammle Regionen direkt aus der Region-Map
        regions = set()
        for y in range(maze_height):
            for x in range(maze_width):
                rid = region_map[y][x]
                if rid != 0:
                    regions.add(rid)
        
        if not regions:
            print("Kritischer Fehler: Keine Regionen gefunden!")
            return
    
    # Starte mit einer zufälligen Region
    connected = set([random.choice(list(regions))])

def remove_dead_ends():
    """Remove dead ends while preserving interesting paths"""
    changed = True
    while changed:
        changed = False
        for y in range(maze_height):
            for x in range(maze_width):
                # Skip solid cells and rooms
                if maze[y][x] == 0:
                    continue
                
                # Count open directions
                count = 0
                for d in [N, E, S, W]:
                    if maze[y][x] & d:
                        count += 1
                
                # Remove dead ends (except with random chance to keep some)
                if count == 1 and random.random() > DEAD_END_REMOVAL_CHANCE:
                    # Find the open direction
                    for d in [N, E, S, W]:
                        if maze[y][x] & d:
                            # Close this cell and neighbor connection
                            nx, ny = x + DX[d], y + DY[d]
                            if 0 <= nx < maze_width and 0 <= ny < maze_height:
                                maze[ny][nx] &= ~OPPOSITE[d]
                            maze[y][x] = 0
                            changed = True
                            break

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

# Main generation logic ========================================================

# Place rooms
rooms = []
room_info = []

# Fixed start room
start_room = (0, 0)
if can_place_room(*start_room, rooms):
    region_id = 1
    create_room(*start_room, region_id)
    rooms.append((*start_room, region_id))
    room_info.append(("Start", start_room[0], start_room[1], region_id))
    region_id += 1

# Fixed goal room
goal_room = (maze_width - ROOM_SIZE, maze_height - ROOM_SIZE)
if can_place_room(*goal_room, rooms):
    create_room(*goal_room, region_id)
    rooms.append((*goal_room, region_id))
    room_info.append(("Goal", goal_room[0], goal_room[1], region_id))
    region_id += 1

# Additional rooms
for i in range(NUM_ADDITIONAL_ROOMS):
    placed = False
    for _ in range(100):  # Try up to 100 times to place room
        room_x = random.randint(0, maze_width - ROOM_SIZE - 1)
        room_y = random.randint(0, maze_height - ROOM_SIZE - 1)
        
        if can_place_room(room_x, room_y, rooms):
            create_room(room_x, room_y, region_id)
            rooms.append((room_x, room_y, region_id))
            room_info.append((f"Room {i+1}", room_x, room_y, region_id))
            region_id += 1
            placed = True
            break
    
    if not placed:
        print(f"Could not place additional room {i+1}")

# Fill remaining areas with mazes
for y in range(maze_height):
    for x in range(maze_width):
        if maze[y][x] == 0 and region_map[y][x] == 0:
            iterative_carve(x, y, region_id)
            region_id += 1

# Connect all regions
connectors = find_connectors()
connect_regions(connectors)

# Remove boring dead ends
remove_dead_ends()

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
for name, x, y, region in room_info:
    print(f"[→] {name}: top-left ({x},{y})")
print("[→] Output written to 'maze_world.world'")