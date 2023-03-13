from graph import Graph, graph_from_file, kruskal
from collections import deque
from time import perf_counter

# regarder si on est plus rapide en faisant les routes pour network.2.in avec ce DFS_recursif

G = graph_from_file("input/network.10.in")

def DFS_iteratif_liste(graph, v, discovered):
# Perform iterative DFS on graph starting from vertex `v`

    # create a stack used to do iterative DFS
    stack = []
 
    # push the source node into the stack
    stack.append(v)
 
    # loop till stack is empty
    while stack:
 
        # Pop a vertex from the stack
        v = stack.pop()
 
        # if the vertex is already discovered yet, ignore it
        if discovered[v]:
            continue
 
        # we will reach here if the popped vertex `v` is not discovered yet;
        # print `v` and process its undiscovered adjacent nodes into the stack
        discovered[v] = True
 
        # do for every edge (v, u)
        for adjacent in graph[v]:
            u = adjacent[0]
            if not discovered[u]:
                stack.append(u)

discovered = {i: False for i in G.nodes}

start = perf_counter()
DFS_iteratif_liste(G.graph, 1, discovered)
stop = perf_counter()
print(stop-start)

def DFS_iteratif(graph, v, discovered):
# Perform iterative DFS on graph starting from vertex `v`

    # create a stack used to do iterative DFS
    stack = deque()
 
    # push the source node into the stack
    stack.append(v)
 
    # loop till stack is empty
    while stack:
 
        # Pop a vertex from the stack
        v = stack.pop()
 
        # if the vertex is already discovered yet, ignore it
        if discovered[v]:
            continue
 
        # we will reach here if the popped vertex `v` is not discovered yet;
        # print `v` and process its undiscovered adjacent nodes into the stack
        discovered[v] = True
 
        # do for every edge (v, u)
        for adjacent in graph[v]:
            u = adjacent[0]
            if not discovered[u]:
                stack.append(u)

discovered = {i: False for i in G.nodes}

start = perf_counter()
DFS_iteratif(G.graph, 1, discovered)
stop = perf_counter()
print(stop-start)

