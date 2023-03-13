from graph import Graph, graph_from_file, kruskal
from time import perf_counter

G = graph_from_file("input/network.00.in")


def get_path_with_power_acm(G, src, dest, power):

    def DFS_chemin(G,root,arrivee):

        stack = [root]

        print(stack)
 
        while (len(stack)):
            s = stack.pop()
 
            for node in G.graph[s]:
                if (node[1] <= power) and (parent[node[0]] != None):
                    stack.append(node[0])
                    parent[node[0]] = s
                    if node[0] == arrivee:
                        return None
                    break

    parent = {i: None for i in G.nodes}              

    DFS_chemin(G, src, dest)

    if parent[dest] == None:
        return None

    chemin = [dest]

    while parent[dest] != src:
        chemin.append(parent[dest])
        dest = parent[dest]      

    chemin.append(src)

    chemin.reverse()

    return chemin

def get_path_with_power(G, src, dest, power):

    def DFS_chemin(G,root,arrivee,visited):

        stack = [root]
 
        while (len(stack)):
            s = stack.pop()
 
            if (not visited[s]):
                visited[s] = True
 
            for node in G.graph[s]:
                if (node[1] <= power) and (not visited[node[0]]):
                    stack.append(node[0])
                    parent[node[0]] = s
                    if node[0] == arrivee:
                        return None

    parent = {i: None for i in G.nodes}              
    visited_nodes = {node: False for node in G.nodes}

    DFS_chemin(G, src, dest, visited_nodes)

    if parent[dest] == None:
        return None

    chemin = [dest]

    while parent[dest] != src:
        chemin.append(parent[dest])
        dest = parent[dest]      

    chemin.append(src)

    chemin.reverse()

    return chemin

start = perf_counter()
print(get_path_with_power(G,1,4,20000))
print(get_path_with_power(G,1,4,30000))
print(get_path_with_power(G,1,4,40000))
print(get_path_with_power(G,1,4,50000))
print(get_path_with_power(G,1,4,60000))
stop = perf_counter()
print(stop - start)

start = perf_counter()
print(get_path_with_power_acm(G,1,4,20000))
print(get_path_with_power_acm(G,1,4,30000))
print(get_path_with_power_acm(G,1,4,40000))
print(get_path_with_power_acm(G,1,4,50000))
print(get_path_with_power_acm(G,1,4,60000))
stop = perf_counter()
print(stop - start)