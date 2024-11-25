import heapq
from itertools import permutations
from collections import deque

# Graph Input
# graph = {
#     "Depot": {"Customer 1": 5, "Customer 3": 20},
#     "Customer 1": {"Depot": 5, "Customer 2": 10},
#     "Customer 2": {"Customer 1": 10, "Customer 3": 15},
#     "Customer 3": {"Depot": 20, "Customer 2": 15}
# }

# blocked_roads = {("Customer 1", "Customer 2")}
# traffic_data = {
#     ("Depot", "Customer 1"): "low",
#     ("Depot", "Customer 3"): "high_traffic"
# }

# Helper Functions
def build_graph_excluding_blocked(graph, blocked_roads):
    new_graph = {node: {} for node in graph}
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            if (node, neighbor) not in blocked_roads and (neighbor, node) not in blocked_roads:
                new_graph[node][neighbor] = weight
    return new_graph

# TSP: Brute Force
def tsp_brute_force(graph, start):
    nodes = list(graph.keys())
    nodes.remove(start)
    min_distance = float('inf')
    best_route = []
    
    for perm in permutations(nodes):
        route = [start] + list(perm) + [start]
        try:
            # Calculate distance for the current route
            distance = sum(graph[route[i]][route[i+1]] for i in range(len(route) - 1))
            if distance < min_distance:
                min_distance = distance
                best_route = route
        except KeyError:
            # Skip invalid routes where an edge is missing
            continue
    
    if best_route:
        return best_route, min_distance
    else:
        return None, float('inf')  # Return None if no valid route is found


# Dijkstra's Algorithm
def dijkstra(graph, start, end):
    pq = []
    heapq.heappush(pq, (0, start))
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    parents = {node: None for node in graph}

    while pq:
        current_distance, current_node = heapq.heappop(pq)
        if current_node == end:
            break
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                parents[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
    
    # Reconstruct path
    path = []
    node = end
    while node:
        path.insert(0, node)
        node = parents[node]
    
    return path, distances[end]

# Prim's Algorithm for MST
def prim_mst(graph):
    start_node = next(iter(graph))
    visited = set()
    mst_edges = []
    total_cost = 0
    pq = []

    visited.add(start_node)
    for neighbor, weight in graph[start_node].items():
        heapq.heappush(pq, (weight, start_node, neighbor))
    
    while pq:
        weight, from_node, to_node = heapq.heappop(pq)
        if to_node not in visited:
            visited.add(to_node)
            mst_edges.append((from_node, to_node, weight))
            total_cost += weight
            for neighbor, edge_weight in graph[to_node].items():
                if neighbor not in visited:
                    heapq.heappush(pq, (edge_weight, to_node, neighbor))
    
    return mst_edges, total_cost

# BFS Traversal
def bfs(graph, start):
    visited = set()
    queue = deque([start])
    traversal = []

    while queue:
        node = queue.popleft()
        if node not in visited:
            visited.add(node)
            traversal.append(node)
            for neighbor in graph[node]:
                if neighbor not in visited:
                    queue.append(neighbor)
    
    return traversal

# DFS Traversal
def dfs(graph, start, visited=None, traversal=None):
    if visited is None:
        visited = set()
        traversal = []
    
    visited.add(start)
    traversal.append(start)
    for neighbor in graph[start]:
        if neighbor not in visited:
            dfs(graph, neighbor, visited, traversal)
    
    return traversal

# Main Execution
# if __name__ == "__main__":
#     # Build graph excluding blocked roads
#     updated_graph = build_graph_excluding_blocked(graph, blocked_roads)

#     # TSP
#     tsp_route, tsp_distance = tsp_brute_force(updated_graph, "Depot")
#     print("TSP Route:", tsp_route)
#     print("TSP Distance:", tsp_distance)

#     # Dijkstra
#     shortest_path, shortest_distance = dijkstra(updated_graph, "Depot", "Customer 3")
#     print("Shortest Path (Depot -> Customer 3):", shortest_path)
#     print("Shortest Distance:", shortest_distance)

#     # MST
#     mst_edges, mst_cost = prim_mst(updated_graph)
#     print("MST Edges:", mst_edges)
#     print("Total MST Cost:", mst_cost)

#     # BFS
#     bfs_traversal = bfs(updated_graph, "Depot")
#     print("BFS Traversal:", bfs_traversal)

#     # DFS
#     dfs_traversal = dfs(updated_graph, "Depot")
#     print("DFS Traversal:", dfs_traversal)
