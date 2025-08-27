import math
import random

SEGMENT_SIZES = [8, 4, 2, 1]
ROOM_SIZE = 3

def generate_multi_floor_world(mazes, n_floors, floor_height, filename="maze_world.world", cell_size=1.0):
    with open(filename, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.8">\n')
        f.write('  <world name="maze_world">\n')
        
        # Grundbeleuchtung und Himmel
        f.write("""    <include>
      <uri>model://ground</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>\n""")

        # Füge jede Etage hinzu
        for floor_num, (maze, chests) in enumerate(mazes):
            offset_z = floor_num * floor_height
            add_floor_to_world(f, maze, chests, cell_size, offset_z, floor_num)
            
            # Füge Rampe hinzu (außer für die oberste Etage)
            if floor_num < n_floors - 1:
                add_ramp_between_floors(f, maze, cell_size, offset_z, floor_num)
                
            # Füge Boden mit Loch hinzu (außer für die unterste Etage)
            if floor_num > 0:
                add_floor_with_hole(f, cell_size, offset_z, floor_num)

        f.write("  </world>\n</sdf>\n")
    print(f"Exportierte {n_floors} Etagen nach '{filename}'")

def add_floor_to_world(f, grid, chests, cell_size, offset_z, floor_num):
    width = len(grid[0])
    height = len(grid)

    offset_x = width * cell_size / 2.0
    offset_y = height * cell_size / 2.0
    
    wall_segments = generate_wall_segments(grid, cell_size, offset_z, floor_num)
    chest_includes = generate_chest_includes(chests, cell_size, offset_x, offset_y, offset_z, floor_num)
    lamp_includes = generate_lamp_includes(grid, cell_size, offset_x, offset_y, offset_z, floor_num)
    
    f.write(f"    <!-- Etage {floor_num} -->\n")
    for chest in chest_includes:
        f.write(chest + "\n")
    for wall in wall_segments:
        f.write(wall + "\n")
    for lamp in lamp_includes:
        f.write(lamp + "\n")

def add_ramp_between_floors(f, grid, cell_size, offset_z, floor_num):
    width = len(grid[0])
    height = len(grid)
    
    # Bestimme Position basierend auf Etagennummer (gerade/ungerade)
    if floor_num % 2 == 0:  # Gerade Etage - Rampe oben rechts
        ramp_x = width - ROOM_SIZE // 2 - 1
        ramp_y = height - ROOM_SIZE // 2 - 1
    else:  # Ungerade Etage - Rampe unten links
        ramp_x = ROOM_SIZE // 2
        ramp_y = ROOM_SIZE // 2
    
    # Weltkoordinaten
    offset_x = width * cell_size / 2.0
    offset_y = height * cell_size / 2.0
    world_x = (ramp_x + 0.5) * cell_size - offset_x
    world_y = (ramp_y + 0.5) * cell_size - offset_y
    world_z = offset_z
    
    f.write(f"""    <!-- Rampe von Etage {floor_num} zu {floor_num + 1} -->
    <include>
      <name>ramp_{floor_num}</name>
      <uri>model://ramp</uri>
      <pose>{world_x:.2f} {world_y:.2f} {world_z:.2f} 0 0 0</pose>
    </include>\n""")

def add_floor_with_hole(f, cell_size, offset_z, floor_num):
    # Boden mit Loch wird einfach in der Mitte platziert (wie gewünscht)
    # Die Höhe wird entsprechend der Etage angepasst
    world_z = offset_z  # Auf der Höhe der aktuellen Etage
    
    f.write(f"""    <!-- Boden mit Loch für Etage {floor_num} -->
    <include>
      <name>ground_hole_{floor_num}</name>
      <uri>model://ground_hole_{(floor_num % 2) + 1}</uri>
      <pose>0 0 {world_z:.2f} 0 0 0</pose>
    </include>\n""")

def generate_wall_segments(grid, cell_size, offset_z, floor=0):
    width = len(grid[0])
    height = len(grid)
    wall_segments = []
    index = 0

    offset_x = width * cell_size / 2.0
    offset_y = height * cell_size / 2.0

    def generate_wall(x, y, length, orientation, index, floor=0):
        model = f"wall_{length}m"
        name = f"{model}_{floor}_{index}"

        if orientation == "H":
            cx = (x + length / 2.0) * cell_size - offset_x
            cy = y * cell_size - offset_y
            yaw = 0
        else:
            cx = x * cell_size - offset_x
            cy = (y + length / 2.0) * cell_size - offset_y
            yaw = math.pi / 2

        pose = f"{cx:.2f} {cy:.2f} {offset_z} 0 0 {yaw:.4f}"
        return f"""    <include>
      <name>{name}</name>
      <uri>model://{model}</uri>
      <pose>{pose}</pose>
    </include>"""

    # Process horizontal walls (top walls only)
    for y in range(height):
        x = 0
        while x < width:
            if grid[y][x].walls['top']:
                for size in SEGMENT_SIZES:
                    if x + size <= width and all(grid[y][x + i].walls['top'] for i in range(size)):
                        wall_segments.append(generate_wall(x, y, size, "H", index, floor))
                        index += 1
                        x += size
                        break
                else:
                    x += 1
            else:
                x += 1

    # Process vertical walls (left walls only)
    for x in range(width):
        y = 0
        while y < height:
            if grid[y][x].walls['left']:
                for size in SEGMENT_SIZES:
                    if y + size <= height and all(grid[y + i][x].walls['left'] for i in range(size)):
                        wall_segments.append(generate_wall(x, y, size, "V", index, floor))
                        index += 1
                        y += size
                        break
                else:
                    y += 1
            else:
                y += 1

    # Add bottom wall of last row
    y = height
    x = 0
    while x < width:
        if grid[height-1][x].walls['bottom']:
            for size in SEGMENT_SIZES:
                if x + size <= width and all(grid[height-1][x + i].walls['bottom'] for i in range(size)):
                    wall_segments.append(generate_wall(x, y, size, "H", index, floor))
                    index += 1
                    x += size
                    break
            else:
                x += 1
        else:
            x += 1

    # Add right wall of last column
    x = width
    y = 0
    while y < height:
        if grid[y][width-1].walls['right']:
            for size in SEGMENT_SIZES:
                if y + size <= height and all(grid[y + i][width-1].walls['right'] for i in range(size)):
                    wall_segments.append(generate_wall(x, y, size, "V", index, floor))
                    index += 1
                    y += size
                    break
            else:
                y += 1
        else:
            y += 1

    return wall_segments

def generate_chest_includes(chests, cell_size, offset_x, offset_y, offset_z, floor=0):
    includes = []
    if not chests:
        return includes
           
    for i, (cx, cy) in enumerate(chests):
        world_x = (cx + 0.5) * cell_size - offset_x
        world_y = (cy + 0.5) * cell_size - offset_y
        includes.append(f"""    <include>
      <name>chest_{floor}_{i}</name>
      <uri>model://treasure_chest</uri>
      <pose>{world_x:.2f} {world_y:.2f} {offset_z:.2f} 0 0 0</pose>
    </include>""")
    return includes

def generate_lamp_includes(grid, cell_size, offset_x, offset_y, offset_z, floor=0):
    includes = []
    width = len(grid[0])
    height = len(grid)
    
    # Platziere Lampen an den Wänden in regelmäßigen Abständen
    lamp_spacing = 4
    
    for y in range(height):
        for x in range(width):
            # Prüfe, ob wir an dieser Position eine Lampe platzieren sollen
            if x % lamp_spacing == 0 and y % lamp_spacing == 0:
                cell = grid[y][x]
                
                # Bestimme verfügbare Wände für Lampen
                available_walls = []
                if cell.walls['top'] and y > 0:
                    available_walls.append('top')
                if cell.walls['left'] and x > 0:
                    available_walls.append('left')
                if cell.walls['right'] and x < width - 1:
                    available_walls.append('right')
                if cell.walls['bottom'] and y < height - 1:
                    available_walls.append('bottom')
                
                # Wähle eine zufällige Wand für die Lampe
                if available_walls:
                    wall_side = random.choice(available_walls)
                    
                    # Berechne Position und Ausrichtung basierend auf der Wand
                    if wall_side == 'top':
                        world_x = (x + 0.5) * cell_size - offset_x
                        world_y = y * cell_size - offset_y - 0.1  # Leicht vor der Wand
                        yaw = 0
                        # Lichtquelle 0.2 Einheiten von der Lampe entfernt
                        light_x = world_x
                        light_y = world_y - 0.2
                    elif wall_side == 'bottom':
                        world_x = (x + 0.5) * cell_size - offset_x
                        world_y = (y + 1) * cell_size - offset_y + 0.1  # Leicht vor der Wand
                        yaw = math.pi
                        # Lichtquelle 0.2 Einheiten von der Lampe entfernt
                        light_x = world_x
                        light_y = world_y + 0.2
                    elif wall_side == 'left':
                        world_x = x * cell_size - offset_x - 0.1  # Leicht vor der Wand
                        world_y = (y + 0.5) * cell_size - offset_y
                        yaw = -math.pi / 2
                        # Lichtquelle 0.2 Einheiten von der Lampe entfernt
                        light_x = world_x - 0.2
                        light_y = world_y
                    elif wall_side == 'right':
                        world_x = (x + 1) * cell_size - offset_x + 0.1  # Leicht vor der Wand
                        world_y = (y + 0.5) * cell_size - offset_y
                        yaw = math.pi / 2
                        # Lichtquelle 0.2 Einheiten von der Lampe entfernt
                        light_x = world_x + 0.2
                        light_y = world_y
                    
                    # Höhe der Lampe
                    world_z = offset_z + 1.3  # 1.3m über dem Boden
                    
                    includes.append(f"""    <include>
      <name>wall_lamp_{floor}_{x}_{y}_{wall_side}</name>
      <uri>model://wall_lamp</uri>
      <pose>{world_x:.2f} {world_y:.2f} {world_z:.2f} 0 0 {yaw:.4f}</pose>
    </include>
    <light type="point" name="wall_lamp_light_{floor}_{x}_{y}_{wall_side}">
        <pose>{light_x:.2f} {light_y:.2f} {world_z:.2f} 0 0 {yaw:.4f}</pose>
        <diffuse>1 0.8 0.5 1</diffuse>
        <specular>0.9 0.9 0.9 1</specular>
        <attenuation>
            <range>3</range>
            <constant>0.5</constant>
            <linear>0.01</linear>
            <quadratic>0.001</quadratic>
        </attenuation>
        <cast_shadows>true</cast_shadows>
    </light>
    """)
    
    return includes