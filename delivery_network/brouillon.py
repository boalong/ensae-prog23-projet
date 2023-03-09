from time import perf_counter
from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.1.in"

g = graph_from_file(data_path + file_name)

'''
start = perf_counter()
print(g.get_path_with_power(1,9,30))
stop = perf_counter()
print(str(stop-start))
'''

start = perf_counter()
print(g.arbre_couvrant_min())
print(g.arbre_couvrant_min().get_path_with_power(1,9,30))
stop = perf_counter()
print(str(stop-start))

# c'est systématiquement plus long sur l'arbre couvrant minimal
# c'est peut-être dû à la structure en forme d'arbre qui ralentit le DFS



# test get_path_with_power
# print(g.connected_components())
# print(g.get_path_with_power(1,7,10))


# print(g.get_path_with_power(1,4,100000))

# print(g.arbre_couvrant_de_poids_min())