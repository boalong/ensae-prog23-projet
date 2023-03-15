# This will work if ran from the root folder.

from graph import graph_from_file, kruskal

g = graph_from_file("C:/Users/Letellier/Documents/ENSAE/1A/S2/Projet de programmation/ensae-prog23-private/input/network.00.in")
print(g.get_path_with_power(1, 4, 11))
print(g.get_path_with_power(1, 4, 10))