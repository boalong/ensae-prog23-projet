from time import perf_counter
from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.2.in"

g = graph_from_file(data_path + file_name)

start = perf_counter()
print(g.min_power(1,4))
stop = perf_counter()
print(str(stop-start))

# on perd du temps car on construit un acm
start = perf_counter()
print(g.min_power_acm(1,4))
stop = perf_counter()
print(str(stop-start))


# nickel pour graph_from_file
# connected_components : fonctionne bien pour routes.1, mais après il y a une erreur de recursion

# print(g.connected_components())
# RecursionError



