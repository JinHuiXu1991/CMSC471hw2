import sys


def bfs(graph, start, end):
    # maintain a queue of paths
    queue = []
    # store all the found paths
    paths = []
    # push the first path into the queue
    queue.append([start])
    path = [start]
    if start == end:
        return path
    if start not in graph:
        return []
    while queue:
        # first path from the queue
        path = queue.pop(0)
        # last node from the path
        last_node = path[-1]
        # path found then add it into paths list
        if last_node == end:
            paths.append(path)
        # enumerate all adjacent nodes, construct a new path and push it into the queue
        for node in graph.get(last_node, []):
            if node not in path:
                new_path = []
                new_path = path + [node]
                queue.append(new_path)

    return paths


def dfs(graph, start, end, path=[]):
    paths = []
    path = path + [start]
    if start == end:
        return [path]
    if not graph.has_key(start):
        return []
    for node in graph[start]:
        if node not in path:
            new_paths = dfs(graph, node, end, path)
            for new_path in new_paths:
                paths.append(new_path)

    return paths


def shortest_path(graph, paths):
    min = 1000000
    for each_path in paths:
        cost = sum(graph[i][j] for i,j in zip(each_path,each_path[1::]))
        if cost < min:
            min = cost
            min_path = each_path

    return min_path


if __name__ == "__main__":

    paths = []
    graph = {}
    with open(sys.argv[1]) as input:
        for line in input:
            (node1, node2, edge) = line.split()
            if node1 in graph:
                graph[node1].update({node2: int(edge)})
            else:
                graph[node1] = {node2: int(edge)}
    input.close()

    algorithm = sys.argv[3]
    start = sys.argv[4]
    end = sys.argv[5]

    if algorithm == "breadth":
    	paths = bfs(graph, start, end)
	path = shortest_path(graph, paths)

    if algorithm == "depth":
        paths = dfs(graph, start, end)
        path = shortest_path(graph, paths)

    output = open(sys.argv[2], 'w')
    for node in path:
        output.write(str(node)+ '\n')
    cost = sum(graph[i][j] for i,j in zip(path,path[1::]))
    output.write(str(cost))
    output.close()

    print graph

