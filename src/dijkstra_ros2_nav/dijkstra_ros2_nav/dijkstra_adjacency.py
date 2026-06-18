import heapq

def calculate_dijkstra_graph(graph, start, goal):
    """
    Calculates the shortest path on a node-based graph (Adjacency List).
    """
    # Priority Queue: stores tuples of total_cost, current_node, path_so_far
    queue = [(0, start, [start])]
    visited = set()

    while queue:
        cost, current, path = heapq.heappop(queue)

        if current == goal:
            return path, cost

        if current in visited:
            continue
        visited.add(current)

        # Explore neighbors using the dictionary
        neighbors = graph.get(current, {})
        for neighbor, weight in neighbors.items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))

    return None, 0