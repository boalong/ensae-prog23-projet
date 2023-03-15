from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "10.in"

g = graph_from_file(data_path + "network." + file_no)

# Calculer les min_power de toutes les routes
# pour optimiser le process, il faut stocker une liste min power pour tout le graphe
# on va associer une méthode liste power pour calculer une fois pour toute

liste_trajets = []

with open(data_path + "routes." + file_no, 'r') as f:
    n = int(f.readline())
    for _ in range(n):
        liste_trajets.append(list(map(float, f.readline().split())))

print(liste_trajets[:5])
g.get_kruskal()
start = perf_counter()
resultats = g.min_power_routes(liste_trajets, 1)
stop = perf_counter()
print("Estimation :" , (stop-start)*50)


# il faut multiplier le temps par 50 (car on ne fait que les 10000 premiers trajets ici)

'''
Résultats:

Estimation : 486.5973200000001

'''