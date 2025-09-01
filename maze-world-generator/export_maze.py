import math
import random

WALL_SEGMENT_SIZES = [8, 4, 2, 1]
ROOM_SIZE = 3
LAMP_SPACING = 6
LAMP_HEIGHT_OFFSET = 1.3
CHEST_HEIGHT_OFFSET = 0.1


def generate_multi_floor_world(mazes, floor_count, floor_height, filename="maze_world.world", cell_size=1.0):
    """Generates a complete multi-floor maze world in SDF format."""
    with open(filename, "w") as world_file:
        _write_world_header(world_file)
        
        for floor_number, (maze_grid, chest_locations) in enumerate(mazes):
            z_offset = floor_number * floor_height
            _add_floor_to_world(world_file, maze_grid, chest_locations, cell_size, z_offset, floor_number)
            
            if floor_number < floor_count - 1:
                _add_ramp_between_floors(world_file, maze_grid, cell_size, z_offset, floor_number)
                
            if floor_number > 0:
                _add_floor_with_hole(world_file, cell_size, z_offset, floor_number)

        _write_world_footer(world_file)
    
    print(f"Exported {floor_count} floors to '{filename}'")


def _write_world_header(file):
    """Writes the static world header content."""
    file.write('<?xml version="1.0"?>\n')
    file.write('<sdf version="1.10">\n')
    file.write('  <world name="maze_world">\n')
    file.write("""    <include>
      <uri>model://ground</uri>
    </include>
    <light type="directional" name="sun">
      <visualize>false</visualize>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse> 
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range> 
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>\n""")


def _write_world_footer(file):
    """Writes the closing tags for the world file."""
    file.write("  </world>\n</sdf>\n")


def _add_floor_to_world(file, maze_grid, chest_locations, cell_size, z_offset, floor_number):
    """Adds a complete floor with walls, chests and lamps to the world."""
    grid_width = len(maze_grid[0])
    grid_height = len(maze_grid)
    
    x_center_offset = grid_width * cell_size / 2.0
    y_center_offset = grid_height * cell_size / 2.0
    
    file.write(f"    <!-- Floor {floor_number} -->\n")
    
    for chest_xml in _generate_chest_xml(chest_locations, cell_size, x_center_offset, y_center_offset, z_offset, floor_number):
        file.write(chest_xml + "\n")
    
    for wall_xml in _generate_wall_segments_xml(maze_grid, cell_size, z_offset, floor_number):
        file.write(wall_xml + "\n")
    
    for lamp_xml in _generate_lamp_xml(maze_grid, cell_size, x_center_offset, y_center_offset, z_offset, floor_number):
        file.write(lamp_xml + "\n")


def _add_ramp_between_floors(file, maze_grid, cell_size, z_offset, floor_number):
    """Adds a ramp connection between consecutive floors."""
    grid_width = len(maze_grid[0])
    grid_height = len(maze_grid)
    
    # Determine ramp position based on floor parity
    if floor_number % 2 == 0:
        ramp_x = grid_width - ROOM_SIZE // 2 - 1
        ramp_y = grid_height - ROOM_SIZE // 2 - 1
    else:
        ramp_x = ROOM_SIZE // 2
        ramp_y = ROOM_SIZE // 2
    
    x_center_offset = grid_width * cell_size / 2.0
    y_center_offset = grid_height * cell_size / 2.0
    
    world_x = (ramp_x + 0.5) * cell_size - x_center_offset
    world_y = (ramp_y + 0.5) * cell_size - y_center_offset
    
    file.write(f"""    <!-- Ramp from Floor {floor_number} to {floor_number + 1} -->
    <include>
      <name>ramp_{floor_number}</name>
      <uri>model://ramp</uri>
      <pose>{world_x:.2f} {world_y:.2f} {z_offset:.2f} 0 0 0</pose>
    </include>\n""")


def _add_floor_with_hole(file, cell_size, z_offset, floor_number):
    """Adds a floor plate with hole for vertical connections."""
    file.write(f"""    <!-- Floor with hole for Level {floor_number} -->
    <include>
      <name>ground_hole_{floor_number}</name>
      <uri>model://ground_hole_{(floor_number % 2) + 1}</uri>
      <pose>0 0 {z_offset:.2f} 0 0 0</pose>
    </include>\n""")


def _generate_wall_segments_xml(grid, cell_size, z_offset, floor_number):
    """Generates XML definitions for all wall segments in the maze."""
    grid_width = len(grid[0])
    grid_height = len(grid)
    wall_segments = []
    segment_counter = 0

    x_center_offset = grid_width * cell_size / 2.0
    y_center_offset = grid_height * cell_size / 2.0

    def generate_segment(x, y, length, orientation):
        nonlocal segment_counter
        model_name = f"wall_{length}m"
        segment_name = f"{model_name}_{floor_number}_{segment_counter}"
        
        if orientation == "HORIZONTAL":
            center_x = (x + length / 2.0) * cell_size - x_center_offset
            center_y = y * cell_size - y_center_offset
            yaw = 0
        else:
            center_x = x * cell_size - x_center_offset
            center_y = (y + length / 2.0) * cell_size - y_center_offset
            yaw = math.pi / 2

        pose = f"{center_x:.2f} {center_y:.2f} {z_offset} 0 0 {yaw:.4f}"
        segment_counter += 1
        
        return f"""    <include>
      <name>{segment_name}</name>
      <uri>model://{model_name}</uri>
      <pose>{pose}</pose>
    </include>"""

    # Process horizontal walls
    for row_idx in range(grid_height):
        col_idx = 0
        while col_idx < grid_width:
            if grid[row_idx][col_idx].walls['top']:
                segment_added = False
                for segment_size in WALL_SEGMENT_SIZES:
                    if (col_idx + segment_size <= grid_width and 
                        all(grid[row_idx][col_idx + i].walls['top'] for i in range(segment_size))):
                        wall_segments.append(generate_segment(col_idx, row_idx, segment_size, "HORIZONTAL"))
                        col_idx += segment_size
                        segment_added = True
                        break
                if not segment_added:
                    col_idx += 1
            else:
                col_idx += 1

    # Process vertical walls
    for col_idx in range(grid_width):
        row_idx = 0
        while row_idx < grid_height:
            if grid[row_idx][col_idx].walls['left']:
                segment_added = False
                for segment_size in WALL_SEGMENT_SIZES:
                    if (row_idx + segment_size <= grid_height and 
                        all(grid[row_idx + i][col_idx].walls['left'] for i in range(segment_size))):
                        wall_segments.append(generate_segment(col_idx, row_idx, segment_size, "VERTICAL"))
                        row_idx += segment_size
                        segment_added = True
                        break
                if not segment_added:
                    row_idx += 1
            else:
                row_idx += 1

    # Add bottom boundary walls
    row_idx = grid_height
    col_idx = 0
    while col_idx < grid_width:
        if grid[grid_height-1][col_idx].walls['bottom']:
            segment_added = False
            for segment_size in WALL_SEGMENT_SIZES:
                if (col_idx + segment_size <= grid_width and 
                    all(grid[grid_height-1][col_idx + i].walls['bottom'] for i in range(segment_size))):
                    wall_segments.append(generate_segment(col_idx, row_idx, segment_size, "HORIZONTAL"))
                    col_idx += segment_size
                    segment_added = True
                    break
            if not segment_added:
                col_idx += 1
        else:
            col_idx += 1

    # Add right boundary walls
    col_idx = grid_width
    row_idx = 0
    while row_idx < grid_height:
        if grid[row_idx][grid_width-1].walls['right']:
            segment_added = False
            for segment_size in WALL_SEGMENT_SIZES:
                if (row_idx + segment_size <= grid_height and 
                    all(grid[row_idx + i][grid_width-1].walls['right'] for i in range(segment_size))):
                    wall_segments.append(generate_segment(col_idx, row_idx, segment_size, "VERTICAL"))
                    row_idx += segment_size
                    segment_added = True
                    break
            if not segment_added:
                row_idx += 1
        else:
            row_idx += 1

    return wall_segments


def _generate_chest_xml(chest_locations, cell_size, x_offset, y_offset, z_offset, floor_number):
    """Generates XML definitions for all treasure chests."""
    chest_definitions = []
    
    for chest_index, (x_coord, y_coord) in enumerate(chest_locations):
        world_x = (x_coord + 0.5) * cell_size - x_offset
        world_y = (y_coord + 0.5) * cell_size - y_offset
        chest_z = z_offset + CHEST_HEIGHT_OFFSET
        
        chest_definitions.append(f"""    <include>
      <name>chest_{floor_number}_{chest_index}</name>
      <uri>model://treasure_chest</uri>
      <pose>{world_x:.2f} {world_y:.2f} {chest_z:.2f} 0 0 0</pose>
    </include>""")
    
    return chest_definitions


def _generate_lamp_xml(grid, cell_size, x_offset, y_offset, z_offset, floor_number):
    """Generates XML definitions for wall-mounted lamps."""
    lamp_definitions = []
    grid_width = len(grid[0])
    grid_height = len(grid)
    
    for row_idx in range(grid_height):
        for col_idx in range(grid_width):
            if col_idx % LAMP_SPACING != 0 or row_idx % LAMP_SPACING != 0:
                continue
                
            cell = grid[row_idx][col_idx]
            available_walls = []
            
            if cell.walls['top'] and row_idx > 0:
                available_walls.append('top')
            if cell.walls['left'] and col_idx > 0:
                available_walls.append('left')
            if cell.walls['right'] and col_idx < grid_width - 1:
                available_walls.append('right')
            if cell.walls['bottom'] and row_idx < grid_height - 1:
                available_walls.append('bottom')
            
            if not available_walls:
                continue
                
            chosen_wall = random.choice(available_walls)
            
            if chosen_wall == 'top':
                world_x = (col_idx + 0.5) * cell_size - x_offset
                world_y = row_idx * cell_size - y_offset - 0.1
                yaw = 0
            elif chosen_wall == 'bottom':
                world_x = (col_idx + 0.5) * cell_size - x_offset
                world_y = (row_idx + 1) * cell_size - y_offset + 0.1
                yaw = math.pi
            elif chosen_wall == 'left':
                world_x = col_idx * cell_size - x_offset - 0.1
                world_y = (row_idx + 0.5) * cell_size - y_offset
                yaw = -math.pi / 2
            else:  # right wall
                world_x = (col_idx + 1) * cell_size - x_offset + 0.1
                world_y = (row_idx + 0.5) * cell_size - y_offset
                yaw = math.pi / 2
            
            lamp_z = z_offset + LAMP_HEIGHT_OFFSET
            
            lamp_definitions.append(f"""    <include>
      <name>wall_lamp_{floor_number}_{col_idx}_{row_idx}_{chosen_wall}</name>
      <uri>model://wall_lamp</uri>
      <pose>{world_x:.2f} {world_y:.2f} {lamp_z:.2f} 0 0 {yaw:.4f}</pose>
    </include>""")
    
    return lamp_definitions