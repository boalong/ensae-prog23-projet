import sys 
sys.path.append("delivery_network/")

from graph import Graph, graph_from_file

g = graph_from_file("input/network.00.in")
print(g)
print(g.graph[1][0][2])