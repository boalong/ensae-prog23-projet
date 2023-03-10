import numpy as np
from time import perf_counter
from graph import Graph, graph_from_file, kruskal

data_path = "input/"
file_name = "network.05.in"

g = graph_from_file(data_path + file_name)

print(g)

print(kruskal(g).graph)


'''
print(g.nb_nodes)
print(g.nb_edges)

start = perf_counter()
# g.arbre_couvrant_min() # 3.72 secondes
# print(g.min_power_acm(1,2))
stop = perf_counter()
print(stop-start)
'''


# on peut compléter une matrice en l'initialisant avec np.zeros

'''
A = np.zeros([g.nb_nodes,g.nb_nodes])

start = perf_counter()
for i in g.nodes:
    for j in g.nodes:
        if i < j:
            A[i-1,j-1] = g.min_power(i,j)[1]
stop = perf_counter()
print(str(stop-start))
'''


'''
# B = np.zeros([g.nb_nodes,g.nb_nodes])

start = perf_counter()
cmpt = 0
for i in g.nodes:
    for j in g.nodes:
        if i < j:
            g.min_power_acm(i,j)
            cmpt += 1
            print(cmpt)
            # B[i-1,j-1] = g.min_power_acm(i,j)[1] # impossible pour les gros graphes
stop = perf_counter()
print(str(stop-start))
# ici, min_power_acm est plus court! surprise! et à chaque fois!

# print(B) # A est une matrice triangulaire avec des 0 sur la diagonale
'''


'''
data_path = "input/"
file_name = "network.1.in"

g = graph_from_file(data_path + file_name)


start = perf_counter()
print(g.get_path_with_power(1,9,30))
stop = perf_counter()
print(str(stop-start))


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

'''