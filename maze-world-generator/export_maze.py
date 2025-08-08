import math

SEGMENT_SIZES = [8, 4, 2, 1]  # Use largest possible segments

def cell_maze_to_gazebo_world(grid, filename="maze_world.world", cell_size=1.0):
    width = len(grid[0])
    height = len(grid)

    wall_segments = []
    index = 0

    offset_x = width * cell_size / 2.0
    offset_y = height * cell_size / 2.0

    def generate_wall(x, y, length, orientation, index):
        """Generate a Gazebo <include> block for a wall segment of given length."""
        model = f"wall_{length}m"
        name = f"{model}_{index}"

        # Position: wall center in world coordinates, centered at origin
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

    # --- Combine horizontal walls (top and bottom)
    for y in range(height + 1):
        for x in range(width):
            # Check top wall at (x, y)
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
            # Check bottom wall at (x, y-1)
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
            # Check left wall at (x, y)
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
            # Check right wall at (x-1, y)
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
    </include>\n""")

        f.write("    <!-- Maze walls -->\n")
        for wall in wall_segments:
            f.write(wall + "\n")

        f.write("  </world>\n</sdf>\n")

    print(f"Exported maze with {index} wall segments to '{filename}'")
