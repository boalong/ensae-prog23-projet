from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "10.in"

g = graph_from_file(data_path + "network." + file_no)

g.get_kruskal()

# Chercher le sommet qui a le plus de connexions

sommet_max = 1

for i in g.nodes[1:]:
    if len(g.graph[i]) > len(g.graph[sommet_max]):
        print(len(g.graph[i]))
        sommet_max = i

sommet_min = 1

for i in g.nodes[1:]:
    if len(g.graph[i]) < len(g.graph[sommet_min]):
        print(len(g.graph[i]))
        sommet_min = i

print("Sommet le plus connecté :", sommet_max)

print("Sommet le moins connecté :", sommet_min)