from graph import Graph, graph_from_file, kruskal
from time import perf_counter

g = graph_from_file("input/network.2.in")
G = kruskal(g)
G.get_dict_frozenset_connected_components_with_power()


start = perf_counter()
G.min_power_with_frozenset(1,4)
stop = perf_counter()
print(stop-start)

start = perf_counter()
G.min_power(1,4)
stop = perf_counter()
print(stop-start)