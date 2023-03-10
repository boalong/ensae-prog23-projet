from time import perf_counter
from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.10.in"

g = graph_from_file(data_path + file_name)

print(g.min_power_acm(1,4))

'''
start = perf_counter()
# g.get_path_with_power(1,99999,6473280)
g.min_power(1,99999)
stop = perf_counter()
print(str(stop-start))
# 3 fois plus rapide que get_path_with_power sur acm
'''
'''
nv_g = g.arbre_couvrant_min()
print(nv_g.nb_nodes,nv_g.nb_edges)
print(len(nv_g.connected_components()))
print(nv_g.connected_components()[1],len(nv_g.connected_components()[1]))
'''

print(g.get_path_with_power(55676, 62046,1000000))

'''
start = perf_counter()
# nv_g.get_path_with_power(1,99999,6473280)
g.min_power_acm(1,99999)
stop = perf_counter()
print(str(stop-start))
'''



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



