import osmnx as ox
import networkx as nx

def dijkstra(start, goal, graph):
    unvisited = {n: float('infinity') for n in graph.nodes()}
    unvisited[start] = 0

    visited = {}
    predecessors = {}

    while unvisited:
        minNode = min(unvisited, key=unvisited.get)
        visited[minNode] = unvisited[minNode]

        if minNode == goal:
            break

        for neighbor in graph.neighbors(minNode):
            if neighbor in visited:
                continue
            tempDist = unvisited[minNode] + graph[minNode][neighbor]['length']
            if tempDist < unvisited[neighbor]:
                unvisited[neighbor] = tempDist
                predecessors[neighbor] = minNode

        unvisited.pop(minNode)

    shortest_path = [goal]
    node = goal
    while node != start:
        node = predecessors[node]
        shortest_path.append(node)
    shortest_path.reverse()

    return shortest_path

place_name = "Berkeley, California"
map_graph = ox.graph_from_place(place_name, network_type='drive')

origin_point = (37.8743, -122.277)

origin = ox.distance.nearest_nodes(map_graph, origin_point[1], origin_point[0])
#pata nahi aisa karnese kyu hua ab
destination = list(map_graph.nodes())[-1]


shortest_path = nx.shortest_path(map_graph, origin, destination, weight='length')
fig, ax = ox.plot_graph_route(map_graph, shortest_path)

path_nodes = dijkstra(origin, destination, map_graph)
fig, ax = ox.plot_graph_route(map_graph, path_nodes)
'''origin_point = (37.8743, -122.277)
destination_point = 
origin = ox.distance.nearest_nodes(map_graph, origin_point[1], origin_point[0])
destination = ox.distance.nearest_nodes(map_graph, destination_point[1], destination_point[0])'''
