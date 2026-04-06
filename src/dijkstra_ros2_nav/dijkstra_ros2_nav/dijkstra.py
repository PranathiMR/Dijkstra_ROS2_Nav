import heapq

def calculate_dijkstra_path(grid, start, goal):
    """
    Calculates the shortest path on a 2D grid using Dijkstra's algorithm.
    grid: 2D list where 0 is free space, 1 is obstacle.
    start: tuple (row, col)
    goal: tuple (row, col)
    """
    rows = len(grid)
    cols = len(grid[0])
    
    if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
        return None

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)

        if current == goal:
            return path

        if current in visited:
            continue
        visited.add(current)

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0:
                if (nx, ny) not in visited:
                    heapq.heappush(queue, (cost + 1, (nx, ny), path + [(nx, ny)]))

    return None