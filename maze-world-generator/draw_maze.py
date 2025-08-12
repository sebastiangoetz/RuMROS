from matplotlib import pyplot as plt


def draw_maze(grid, file_name="maze.png", path=None, start=None, end=None, chests=None, show_legend=False):
    height = len(grid)
    width = len(grid[0])
    fig, ax = plt.subplots(figsize=(6, 6))

    # Draw maze walls
    for y in range(height):
        for x in range(width):
            cell = grid[y][x]
            if cell.walls['top']:
                ax.plot([x, x + 1], [y, y], color='black')
            if cell.walls['right']:
                ax.plot([x + 1, x + 1], [y, y + 1], color='black')
            if cell.walls['bottom']:
                ax.plot([x, x + 1], [y + 1, y + 1], color='black')
            if cell.walls['left']:
                ax.plot([x, x], [y, y + 1], color='black')

    # Draw path
    if path:
        px, py = zip(*[(x + 0.5, y + 0.5) for x, y in path])
        ax.plot(px, py, color='red', linewidth=2, label='Solution Path')

    # Mark start and target
    if start:
        ax.scatter(start[0] + 0.5, start[1] + 0.5, color='red', label='Start', s=100)
    if end:
        ax.scatter(end[0] + 0.5, end[1] + 0.5, color='green', label='Target', s=100)

    # Draw chests (small yellow squares)
    if chests:
        for i, (cx, cy) in enumerate(chests):
            ax.add_patch(plt.Rectangle(
                (cx + 0.25, cy + 0.25),  # bottom-left corner
                0.5, 0.5,               # width and height
                facecolor='yellow',
                edgecolor='gold',
                linewidth=1,
                label='Chest' if i == 0 else None  # add label only once for legend
            ))

    ax.set_aspect('equal')
    plt.xticks([]), plt.yticks([])
    if show_legend:
        plt.legend(loc='upper left')
    plt.savefig(file_name, bbox_inches='tight')
    plt.close(fig)
