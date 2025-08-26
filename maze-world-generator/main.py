from generate_maze import generate_maze
from find_path import find_path
from draw_maze import draw_maze
from export_maze import generate_multi_floor_world

width = 20
height = 20
n_rooms = 5
max_attempts = 20
chest_probability = 0.3
n_floors = 3  # Anzahl der Etagen
floor_height = 2.0  # Höhe pro Etage in Metern (2m für Wandhöhe)

# Generiere Mazes für alle Etagen
mazes = []
for i in range(n_floors):
    maze, start, end, chests = generate_maze(width, height, n_rooms, max_attempts, chest_probability)
    path = find_path(maze, start, end)
    draw_maze(maze, f"maze_floor_{i}.png", path, start, end, chests)
    mazes.append((maze, chests))

# Exportiere alle Etagen in eine Weltdatei
generate_multi_floor_world(mazes, n_floors, floor_height)