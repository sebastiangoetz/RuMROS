import generate_maze_new  # dein Cell-basiertes Maze
import export_maze

maze = generate_maze_new.generate_maze(20, 20)  # Cell[][]-Struktur
export_maze.cell_maze_to_gazebo_world(maze)
