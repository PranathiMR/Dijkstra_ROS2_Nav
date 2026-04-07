from dijkstra_adjacency import calculate_dijkstra_graph

def main():
    # The Adjacency List representing the graph
    graph_map = {
        'A': {'B': 3, 'C': 8, 'D': 6},
        'B': {'E': 9},
        'C': {'E': 4, 'F': 7},
        'D': {'C': 1},
        'E': {'F': 2},
        'F': {}  
    }

    start_node = 'A'
    goal_node = 'F'

    print(f"Running Dijkstra on Topological Graph from '{start_node}' to '{goal_node}'...\n")
    
    path, total_cost = calculate_dijkstra_graph(graph_map, start_node, goal_node)

    if path:
        print(f"✅ Path found!")
        print(f"Route: {' -> '.join(path)}")
        print(f"Total Cost: {total_cost}")
    else:
        print("❌ No path could be found.")

if __name__ == '__main__':
    main()