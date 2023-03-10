from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "10.in"

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
    liste_min_power.append(g.min_power_opti_acm(int(trajet[0]),int(trajet[1])))
    stop = perf_counter()
    liste_temps.append(stop-start)
    a += 1
    print(a)
    if a == 10:
        break

print(liste_trajets[:10])
print(liste_min_power)
print(liste_temps)
print("Estimation pour network.10, routes.10 : " + str(np.mean(liste_temps)*500000))


'''
Résultats

[37.1428577, 36.155841099999996, 36.13395779999999, 43.676713899999996, 38.97880599999999, 41.764554499999974, 38.13842839999998, 41.73647549999998, 38.83312050000001, 36.53570909999996]
Estimation pour network.10, routes.10 : 19454823.22499999
'''
# Y'a du progrès !



