
def dijkstra(start, goal, graph):
    unvisited = {n: float('infinity') for n in graph.keys()}
    unvisited[start] = 0

    visited = {}
    predecessors = {}

    while unvisited:
        minNode = min(unvisited, key=unvisited.get)
        visited[minNode] = unvisited[minNode]

        if minNode == goal:
            break

        for neighbor in graph[minNode]:
            if neighbor in visited:
                continue
            tempDist = unvisited[minNode] + graph[minNode][neighbor]
            if tempDist < unvisited[neighbor]:
                unvisited[neighbor] = tempDist
                predecessors[neighbor] = minNode

        unvisited.pop(minNode)

    shortest_path = [goal]
    node = goal
    while node != start:
        if node not in predecessors:
                print(f"No path found from {start} to {goal}")
                return visited, []
        node = predecessors[node]
        shortest_path.append(node)
    shortest_path.reverse()  #

    return shortest_path, visited
graph={
     'S':{'C':2, 'A':5, 'B':7},
        'C':{'E':8},
        'B':{'E':3},
        'A':{'D':2,'B':1},
        'E':{'D':7},
        'D':{'E':7,'T':1},
        'T':{'E':4}
      }
start=str(input("start node:"))
goal=str(input("goal node:"))
path, costs = dijkstra(start, goal, graph)

print("Shortest path:", path)
print("Cost to each node:", costs)
