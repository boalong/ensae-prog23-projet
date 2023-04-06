from graph import *

G = graph_from_file("input/network.00.in")
print(kruskal(G))
G_kruskal = kruskal_epsilon(G, 0.5) # aucune arête ne se casse
print(G_kruskal)
# print(kruskal_epsilon(G, 1))
# print(kruskal_epsilon(G, 0.5))

# print(nb_aretes_entre_deux_sommets_kruskal(G_kruskal, 2, 10))
# print(proba_trajet_kruskal(G_kruskal, 2, 10, 0.5)) # 0.5 = epsilon
# print(proba_trajet_kruskal(G_kruskal, 2, 10, 0.99)) # 0.99 = epsilon

# utilite_esperee_trajets("1", 0)
utilite_esperee_trajets("1", 0.001)
print("done")
utilite_esperee_trajets("2", 0.001)
print("done")
utilite_esperee_trajets("3", 0.001)
print("done")
utilite_esperee_trajets("4", 0.001)
print("done")