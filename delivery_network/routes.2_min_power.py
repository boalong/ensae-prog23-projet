from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "2.in"

g = graph_from_file(data_path + "network." + file_no)

# Calculer les min_power de toutes les routes
# pour optimiser le process, il faut stocker une liste min power pour tout le graphe
# on va associer une méthode liste power pour calculer une fois pour toute

liste_min_power = []
liste_trajets = []
liste_temps = []

with open(data_path + "routes." + file_no, 'r') as f:
    n = int(f.readline())
    for _ in range(n):
        liste_trajets.append(list(map(float, f.readline().split())))

print(liste_trajets[:5])

a = 0
for trajet in liste_trajets:
    start = perf_counter()
    liste_min_power.append(g.min_power_acm(int(trajet[0]),int(trajet[1])))
    stop = perf_counter()
    liste_temps.append(stop-start)
    a += 1
    print(a)
    if a == 10:
        break

print(liste_temps)
print("Estimation pour network.2, routes.2 : " + str(np.mean(liste_temps)*100000))