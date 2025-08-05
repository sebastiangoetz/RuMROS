from collections import deque


def find_path(grid, start, end):
    height = len(grid)
    width = len(grid[0])
    visited = [[False]*width for _ in range(height)]
    parent = {}

    queue = deque()
    queue.append(start)
    visited[start[1]][start[0]] = True

    directions = {
        'top': (0, -1),
        'bottom': (0, 1),
        'left': (-1, 0),
        'right': (1, 0)
    }

    while queue:
        x, y = queue.popleft()
        if (x, y) == end:
            break

        cell = grid[y][x]
        for wall, (dx, dy) in directions.items():
            if not cell.walls[wall]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height and not visited[ny][nx]:
                    visited[ny][nx] = True
                    parent[(nx, ny)] = (x, y)
                    queue.append((nx, ny))

    # Pfad zurückverfolgen
    path = []
    current = end
    while current != start:
        path.append(current)
        current = parent.get(current)
        if current is None:
            print("No path found!")
            return []  # Kein Pfad gefunden
    path.append(start)
    path.reverse()
    return path