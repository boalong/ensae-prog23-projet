from time import perf_counter
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "2.in"

g = graph_from_file(data_path + "network." + file_no)

# Calculer les min_power de toutes les routes

start = perf_counter()

liste_min_power = []
liste_trajets = []

with open(data_path + "routes." + file_no, 'r') as f:
    n = int(f.readline())
    for _ in range(n):
        liste_trajets.append(list(map(float, f.readline().split())))

print(liste_trajets[:5])

a = 0
for trajet in liste_trajets:
    liste_min_power.append(g.min_power_acm(int(trajet[0]),int(trajet[1])))
    a += 1
    print(a)

print(liste_min_power[:5])

stop = perf_counter()

print(stop-start)




