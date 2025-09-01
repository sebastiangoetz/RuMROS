from generate_maze import generate_maze
from analyze_maze import find_shortest_path
from draw_maze import draw_maze
from export_maze import generate_multi_floor_world


def generate_multi_floor_maze():
    """Generate a multi-floor maze with visualization and world export."""
    # Configuration constants
    MAZE_WIDTH = 20
    MAZE_HEIGHT = 20
    NUM_ROOMS = 5
    MAX_GENERATION_ATTEMPTS = 20
    CHEST_SPAWN_PROBABILITY = 0.6
    NUM_FLOORS = 1
    FLOOR_HEIGHT_METERS = 2.0  # Wall height per floor in meters

    floors_data = _generate_all_floors(
        MAZE_WIDTH, 
        MAZE_HEIGHT, 
        NUM_ROOMS, 
        MAX_GENERATION_ATTEMPTS, 
        CHEST_SPAWN_PROBABILITY, 
        NUM_FLOORS
    )
    
    _export_world(floors_data, NUM_FLOORS, FLOOR_HEIGHT_METERS)


def _generate_all_floors(width, height, num_rooms, max_attempts, chest_probability, num_floors):
    """Generate mazes for all floors and return their data."""
    floors_data = []
    
    for floor_number in range(num_floors):
        maze, start_position, end_position, chest_locations = generate_maze(
            width, height, num_rooms, max_attempts, chest_probability
        )
        
        shortest_path = find_shortest_path(maze, start_position, end_position)
        
        _visualize_floor(
            maze, 
            shortest_path, 
            start_position, 
            end_position, 
            chest_locations, 
            floor_number
        )
        
        floors_data.append((maze, chest_locations))
    
    return floors_data


def _visualize_floor(maze, path, start, end, chests, floor_number):
    """Create a visual representation of a single floor."""
    image_filename = f"maze_floor_{floor_number}.png"
    draw_maze(maze, image_filename, path, start, end, chests)
    print(f"Visualization for floor {floor_number} saved as {image_filename}")


def _export_world(floors_data, num_floors, floor_height):
    """Export all floors to a world file."""
    generate_multi_floor_world(floors_data, num_floors, floor_height)
    print(f"Exported {num_floors} floor(s) to world file")


if __name__ == "__main__":
    generate_multi_floor_maze()