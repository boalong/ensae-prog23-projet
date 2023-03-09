from time import perf_counter
from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.10.in"

g = graph_from_file(data_path + file_name)

start = perf_counter()
g.get_path_with_power(1,99999,6473280)
stop = perf_counter()
print(str(stop-start))
# 3 fois plus rapide que get_path_with_power sur acm

start = perf_counter()
nv_g = g.arbre_couvrant_min()
nv_g.get_path_with_power(1,99999,6473280)
# nv_g.min_power(1, 5)
stop = perf_counter()
print(str(stop-start))



'''
start = perf_counter()
print(g.min_power(1,4))
stop = perf_counter()
print(str(stop-start))

# on perd du temps car on construit un acm
start = perf_counter()
print(g.min_power_acm(1,4))
stop = perf_counter()
print(str(stop-start))
'''


# nickel pour graph_from_file
# connected_components : fonctionne bien pour routes.1, mais après il y a une erreur de recursion

# print(g.connected_components())
# RecursionError



