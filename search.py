# CMCS471 homework 2
# Author: Jin Hui Xu
# Description: This project will implement the following search algorithms: breadth first search, 
#              depth first search, uniform cost search, best first search and A* search


import sys


# breadth first search function
def breadth(graph, start, end):
    # maintain a queue of paths
    queue = []
    # push the first path into the queue
    queue.append([start])
    # store the start node in to path
    path = [start]

    if start == end:
        return path
    if start not in graph:
        return None

    while queue:
        # first path from the queue
        path = queue.pop(0)
        # last node from the path
        last_node = path[-1]
        # path found then return path
        if last_node == end:
            return path

        # enumerate all neighbor nodes, construct a new path and push it into the queue
        for node in graph.get(last_node, []):
            if node not in path:
                new_path = []
                new_path = path + [node]
                queue.append(new_path)


# depth first search function
def depth(graph, start, end, path=[]):
    # store the start node in to path and keep adding the successor
    path = path + [start]

    if start == end:
        return path
    if start not in graph:
        return None
    
    # enumerate all neighbor nodes, construct a new path recursively
    for node in graph[start]:
        if node not in path:
            new_path = depth(graph, node, end, path)
            if new_path:
                return new_path
    return None


# shortest path function for uniform cost search
def shortest_path(unvisited):
    lowest = 999
    lowest_node = None
    for node1 in unvisited:
        if unvisited[node1] <= lowest:
            lowest = unvisited[node1]
            lowest_node = node1
    del unvisited[lowest_node]
    return lowest_node


# uniform cost search function
def uniform(graph, start, end):
    path = [start]
    if start == end:
        return path
    if start not in graph:
        return None

    distance = {}
    unvisited = {}
    predecesor = {}
    path = []
    
    # initialilze the graph
    for node1 in graph:
        distance[node1] = 999
        predecesor[node1] = -1
    # distance of start from itself is 0
    distance[start] = 0

    for node1 in graph:
        unvisited[node1] = distance[node1]

    while unvisited:
        # get the shortest distance node
        lowest_node = shortest_path(unvisited)

        # enumerate all neighbor nodes, set the distance and predecesor of current node
        for node2 in graph[lowest_node]:
                new_dist = distance[lowest_node] + graph[lowest_node][node2]
                if new_dist < distance[node2]:
                    unvisited[node2] = new_dist
                    distance[node2] = new_dist
                    predecesor[node2] = lowest_node

    # get the nodes from predecesor list and store them into path
    while 1:
        if end in predecesor:
            path = path + [end]
            if end == start: break
            end = predecesor[end]
        else:
            return None
    # reverse the path to get the correct order
    path.reverse()
    return path


# best first search function
def best(graph, h_value, start, end):
    
    # the set of nodes to be evaluated
    open_queue = {}
    # the set of nodes already evaluated
    close_queue = {}
    
    # add the start node into open queue
    open_queue[start] = []
    open_queue[start].append(h_value[start])
    path = []
    path2 = [start]

    if start == end:
        return path2
    if start not in graph:
        return None

    while open_queue:
        # update the path with the lowest h_value node
        path = path + [min(open_queue.items(), key=lambda x: x[1])[0]]
        # get the lowest h_value node from open queue
        current = min(open_queue.items(), key=lambda x: x[1])[0]
        # delete the lowest h_value node from open queue
        del open_queue[current]
        close_queue[current] = []

        # if node is end, means we found the path
        if current == end:
            return path

        # enumerate all neighbor nodes
        for node in graph[current]:
            # check if the node is in the open queue
            # if yes, continue to next node
            if node in close_queue:
                    continue
            # if no, add this node and its h value into open queue
            elif node not in open_queue:
                open_queue[node] = []
                open_queue[node].append(h_value[node])


# A* search function
def astar(graph, h_value, start, end):
    # the set of nodes to be evaluated
    open_queue = {}
    # the set of nodes already evaluated
    close_queue = {}
    # store the g(n)for all nodes
    g_value = {}
    # g(n) of start node from itself is 0
    g_value[start] = 0
    open_queue[start] = []
    # store the node with its f value, f(n) = g(n) + h(n)
    open_queue[start].append(g_value[start] + h_value[start])

    path = []
    path2 = [start]
    if start == end:
        return path2
    if start not in graph:
        return None

    while open_queue:
        path = path + [min(open_queue.items(), key=lambda x: x[1])[0]]
        current = min(open_queue.items(), key=lambda x: x[1])[0]
        del open_queue[current]
        close_queue[current] = []
        
        # path found and return it
        if current == end:
            return path

        # enumerate all neighbor nodes
        for node in graph[current]:
            # calculate the g value for expanded node
            g_cost = g_value[current] + graph[current][node]
            # calculate the f value for expanded node
            f_cost = g_cost + h_value[node]

            # if the node is already expanded, continue to next node
            if node in close_queue:
                    continue
            # if the node is not expanded or the new f value is smaller
            elif node not in open_queue or f_cost < open_queue[node]:
                # add the node and its f value into open queue
                open_queue[node] = []
                open_queue[node].append(f_cost)
                # update g value of node for furture use 
                g_value[node] = g_cost


# main function, read input files and create output file
if __name__ == "__main__":

    # check there are enough arguments input
    if len(sys.argv) != 7:
	print "should be totally seven arguments"
    else:
	cost = 0
        graph_edge = {}
	graph_noedge = {}
        h_value = {}
        
        # read the graph file and create a graph with weighted edge
        with open(sys.argv[1]) as input:
            for line in input:
                (node1, node2, edge) = line.split()
                # check if the node is already in the graph
                # if yes, update its child and edge
                if node1 in graph_edge:
                    graph_edge[node1].update({node2: int(edge)})
                # if no, create a node with its child and edge
                else:
                    graph_edge[node1] = {node2: int(edge)}
                # add any remaining node into graph
                if node2 not in graph_edge:
                    graph_edge[node2] = {}
        input.close()

        # read the graph file and create a graph without weighted edge
	with open(sys.argv[1]) as input:
            for line in input:
                (node1, node2, edge) = line.split()
                if node1 not in graph_noedge:
                    graph_noedge[node1] = []
                    graph_noedge[node1] = [node2]
                else:
                    graph_noedge[node1].append(node2)

                if node2 not in graph_noedge:
                    graph_noedge[node2] = []
        input.close()
	
	# read the heuristic values in to h_value dictionary
	with open(sys.argv[2]) as input:
            for line in input:
                (node, h) = line.split()
                if node not in h_value:
                    h_value[node] = []
                    h_value[node] = int(h)
                else:
                    h_value[node].append(int(h))
        input.close()

        algorithm = sys.argv[3]
        start = sys.argv[4]
        end = sys.argv[5]
        
        # choose the right algorithm based on the user input
        if algorithm == "breadth":
            path = breadth(graph_noedge, start, end)
            
        elif algorithm == "depth":
            path = depth(graph_noedge, start, end)
    
        elif algorithm == "uniform":
	    path = uniform(graph_edge, start, end)

        elif algorithm == "best":
	    path = best(graph_noedge, h_value, start, end)

        elif algorithm == "astar":
	    path = astar(graph_edge, h_value, start, end)

	else:
	    path = None

        # open the output file and write the result  
        output = open(sys.argv[6], 'w')
        if path == None:
            output.write("false")
        else:
            for node in path:
                output.write(str(node)+ '\n')
                cost = sum(graph_edge[i][j] for i,j in zip(path,path[1::]))
            output.write(str(cost))
        output.close()



