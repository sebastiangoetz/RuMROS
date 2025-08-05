import math

def cell_maze_to_gazebo_world(grid, filename="maze_world.world", cell_size=1.0):
    width = len(grid[0])
    height = len(grid)
    
    wall_segments = []
    index = 0
    origin_x = -width * cell_size / 2.0
    origin_y = -height * cell_size / 2.0

    def generate_wall(x, y, orientation, index):
        """Generate a Gazebo <include> block for a wall"""
        model = "wall_1m"
        name = f"{model}_{index}"
        if orientation == "H":
            cx = origin_x + (x + 0.5) * cell_size
            cy = origin_y + y * cell_size
            yaw = 0
        else:
            cx = origin_x + x * cell_size
            cy = origin_y + (y + 0.5) * cell_size
            yaw = math.pi / 2
        pose = f"{cx:.2f} {cy:.2f} 0 0 0 {yaw:.4f}"
        return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{pose}</pose>
    </include>"""

    # Wände erzeugen
    for y in range(height):
        for x in range(width):
            cell = grid[y][x]
            if cell.walls['top']:
                wall_segments.append(generate_wall(x, y, "H", index))
                index += 1
            if cell.walls['left']:
                wall_segments.append(generate_wall(x, y, "V", index))
                index += 1
            # Nur rechts/bottom für Randzellen
            if x == width - 1 and cell.walls['right']:
                wall_segments.append(generate_wall(x + 1, y, "V", index))
                index += 1
            if y == height - 1 and cell.walls['bottom']:
                wall_segments.append(generate_wall(x, y + 1, "H", index))
                index += 1

    # Schreibe world-Datei
    with open(filename, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.8">\n')
        f.write('  <world name="cell_maze_world">\n')

        # Umgebungs-Plugins und Licht
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

    print(f"[✔] Exported maze with {index} wall segments to '{filename}'")
