from generate_maze import generate_maze
from find_path import find_path
from draw_maze import draw_maze
from export_maze import cell_maze_to_gazebo_world

width = 20
height = 20
n_rooms = 2

maze, start, end = generate_maze(width, height, n_rooms)
path = find_path(maze, start, end)
draw_maze(maze, "maze_step3_solution.png", path, start, end)
cell_maze_to_gazebo_world(maze)