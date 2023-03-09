from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.2.in"

g = graph_from_file(data_path + file_name)
print(g.graph[1])
print(g.graph[2])



# test get_path_with_power
# print(g.connected_components())
# print(g.get_path_with_power(1,7,10))


# print(g.get_path_with_power(1,4,100000))

# print(g.arbre_couvrant_de_poids_min())