from graph import Graph, graph_from_file


# data_path = "input/"
file_name = "delivery_network/test_gpwp.txt"

g = graph_from_file(file_name)
# print(g.graph)
print(g.connected_components())
print(g.get_path_with_power(1,7,10))


# print(g.get_path_with_power(1,4,100000))

# print(g.arbre_couvrant_de_poids_min())