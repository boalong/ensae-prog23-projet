from time import perf_counter_ns
from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.10.in"

start = perf_counter_ns()
g = graph_from_file(data_path + file_name)
print(len(g.connected_components()))
stop = perf_counter_ns()

print(str(stop-start))

# nickel pour graph_from_file
# connected_components : fonctionne bien pour routes.1, mais après il y a une erreur de recursion

# print(g.connected_components())
# RecursionError



