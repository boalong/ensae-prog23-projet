from graph import Graph, graph_from_file


data_path = "input/"
file_name = "network.1.in"

g = graph_from_file(data_path + file_name)
print(g.graph)
# print(g.get_path_with_power(1,4,100000))
