from graph import Graph, graph_from_file


data_path = "delivery_network/"
file_name = "test_gpwp.txt"

g = graph_from_file(data_path + file_name)

print(g.get_path_with_power(1,9,0.5))


# test get_path_with_power
# print(g.connected_components())
# print(g.get_path_with_power(1,7,10))


# print(g.get_path_with_power(1,4,100000))

# print(g.arbre_couvrant_de_poids_min())