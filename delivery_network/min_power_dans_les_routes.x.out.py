from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "4"

g = graph_from_file(data_path + "network." + file_no + ".in")
g.get_kruskal()

# Calculer les min_power de toutes les routes
# pour optimiser le process, il faut stocker une liste min power pour tout le graphe
# on va associer une méthode liste power pour calculer une fois pour toute

liste_trajets = []

with open(data_path + "routes." + file_no + ".in", 'r') as f:
    n = int(f.readline())
    for _ in range(n):
        depart, arrivee, utilite = f.readline().split()
        depart, arrivee, utilite = int(depart), int(arrivee), float(utilite)
        liste_trajets.append([depart, arrivee, utilite])
        # liste_trajets.append(list(map(float, f.readline().split())))

print(liste_trajets[:5])
start = perf_counter()
resultats = g.min_power_routes(liste_trajets, 1)
stop = perf_counter()
print("Temps d'exécution :" , stop-start)

print(len(resultats))
print(resultats[:10])

g = open("output/routes." + file_no + ".out", "w")
for min_power in resultats:
    g.write(str(min_power) + "\n")
g.close()

'''
Pour network.1.in, on obtient:
Temps d'exécution : 0.0006634000000000084

Pour network.2.in, on obtient:
Temps d'exécution : 5.1626027

Pour network.3.in, on obtient:
Temps d'exécution : 374.0334306

Pour network.4.in, on obtient:
Temps d'exécution : 645.491249

'''