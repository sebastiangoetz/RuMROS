import math

SEGMENT_SIZES = [8, 4, 2, 1]  # Use largest possible segments

def cell_maze_to_gazebo_world(grid, chests=None, filename="maze_world.world", cell_size=1.0):
    width = len(grid[0])
    height = len(grid)

    wall_segments = []
    chest_includes = []
    index = 0

    offset_x = width * cell_size / 2.0
    offset_y = height * cell_size / 2.0

    def generate_wall(x, y, length, orientation, index):
        """Generate a Gazebo <include> block for a wall segment of given length."""
        model = f"wall_{length}m"
        name = f"{model}_{index}"

        if orientation == "H":
            cx = (x + length / 2.0) * cell_size - offset_x
            cy = y * cell_size - offset_y
            yaw = 0
        else:
            cx = x * cell_size - offset_x
            cy = (y + length / 2.0) * cell_size - offset_y
            yaw = math.pi / 2

        pose = f"{cx:.2f} {cy:.2f} 0 0 0 {yaw:.4f}"
        return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{pose}</pose>
    </include>"""

    def generate_chest(cx_cell, cy_cell, chest_id):
        """Generate a Gazebo <include> block for a chest at maze cell center."""
        world_x = (cx_cell + 0.5) * cell_size - offset_x
        world_y = (cy_cell + 0.5) * cell_size - offset_y
        world_z = 0.2
        return f"""    <include>
      <name>chest_{chest_id}</name>
      <uri>model://treasure_chest</uri>
      <pose>{world_x:.2f} {world_y:.2f} {world_z:.2f} 0 0 0</pose>
    </include>"""

    # --- Combine horizontal walls (top and bottom)
    for y in range(height + 1):
        for x in range(width):
            if y < height and grid[y][x].walls['top']:
                for size in SEGMENT_SIZES:
                    if x + size <= width and all(
                        grid[y][x + i].walls['top'] for i in range(size)
                    ):
                        for i in range(size):
                            grid[y][x + i].walls['top'] = False
                        wall_segments.append(generate_wall(x, y, size, "H", index))
                        index += 1
                        break
            if y > 0 and y <= height and grid[y - 1][x].walls['bottom']:
                for size in SEGMENT_SIZES:
                    if x + size <= width and all(
                        grid[y - 1][x + i].walls['bottom'] for i in range(size)
                    ):
                        for i in range(size):
                            grid[y - 1][x + i].walls['bottom'] = False
                        wall_segments.append(generate_wall(x, y, size, "H", index))
                        index += 1
                        break

    # --- Combine vertical walls (left and right)
    for x in range(width + 1):
        for y in range(height):
            if x < width and grid[y][x].walls['left']:
                for size in SEGMENT_SIZES:
                    if y + size <= height and all(
                        grid[y + i][x].walls['left'] for i in range(size)
                    ):
                        for i in range(size):
                            grid[y + i][x].walls['left'] = False
                        wall_segments.append(generate_wall(x, y, size, "V", index))
                        index += 1
                        break
            if x > 0 and x <= width and grid[y][x - 1].walls['right']:
                for size in SEGMENT_SIZES:
                    if y + size <= height and all(
                        grid[y + i][x - 1].walls['right'] for i in range(size)
                    ):
                        for i in range(size):
                            grid[y + i][x - 1].walls['right'] = False
                        wall_segments.append(generate_wall(x, y, size, "V", index))
                        index += 1
                        break

    if chests:
        for i, (cx_cell, cy_cell) in enumerate(chests):
            chest_includes.append(generate_chest(cx_cell, cy_cell, i))

    # Write world file
    with open(filename, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.8">\n')
        f.write('  <world name="cell_maze_world">\n')

        f.write("""    <include>
      <uri>model://ground</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>
    <include>
      <name>target_marker</name>
      <uri>model://marker_green</uri>
      <pose>8.5 8.5 0.01 0 0 0</pose>
    </include>\n""")

        f.write("    <!-- Maze walls -->\n")
        for wall in wall_segments:
            f.write(wall + "\n")

        if chest_includes:
            f.write("    <!-- Chests -->\n")
            for chest in chest_includes:
                f.write(chest + "\n")

        f.write("  </world>\n</sdf>\n")

    print(f"Exported maze with {index} wall segments and {len(chest_includes)} chests to '{filename}'")
