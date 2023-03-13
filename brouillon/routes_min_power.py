from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "10.in"

g = graph_from_file(data_path + "network." + file_no)

# Calculer les min_power de toutes les routes

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

print(liste_trajets[:10])
print(liste_min_power)
print(liste_temps)
print("Estimation pour network.10, routes.10 : " + str(np.mean(liste_temps)*500000))


'''
Résultats

[40.718369800000005, 56.199840099999996, 52.0797584, 39.3099297, 57.546092200000004, 61.17603, 60.93053209999999, 91.63223870000002, 39.7154706, 38.484807000000046]
Estimation pour network.10, routes.10 : 26889653.430000003

'''



