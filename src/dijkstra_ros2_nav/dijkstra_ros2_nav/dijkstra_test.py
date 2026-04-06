from dijkstra import calculate_dijkstra_path

def print_grid(grid, path, start, goal):
    for r in range(len(grid)):
        row_str = ""
        for c in range(len(grid[0])):
            if (r, c) == start:
                row_str += " S "
            elif (r, c) == goal:
                row_str += " G "
            elif path and (r, c) in path:
                row_str += " * "
            elif grid[r][c] == 1:
                row_str += " █ "
            else:
                row_str += " . "
        print(row_str)

def main():
    map_grid = [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0]
    ]

    start_node = (0, 0)
    goal_node = (4, 3)

    print("Running Dijkstra's Algorithm...\n")
    path = calculate_dijkstra_path(map_grid, start_node, goal_node)

    if path:
        print(f"Path found! Length: {len(path)} steps.")
        print(f"Coordinates: {path}\n")
        print("Visualizing Map:")
        print_grid(map_grid, path, start_node, goal_node)
    else:
        print("No path could be found. The goal is unreachable.")

if __name__ == '__main__':
    main()