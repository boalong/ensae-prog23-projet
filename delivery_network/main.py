from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.3.in"

g = graph_from_file(data_path + file_name)

# print(g.connected_components())
print(g.get_path_with_power(1,4,100000000))
# RecursionError





