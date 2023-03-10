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

[37.1428577, 36.155841099999996, 36.13395779999999, 43.676713899999996, 38.97880599999999, 41.764554499999974, 38.13842839999998, 41.73647549999998, 38.83312050000001, 36.53570909999996]
Estimation pour network.10, routes.10 : 19454823.22499999

[32.6108305, 36.001644399999996, 32.5533466, 32.851969199999985, 39.66903810000002, 33.66300509999999, 33.89168790000002, 35.21881229999997, 44.55355249999997, 36.78937530000002]
Estimation pour network.10, routes.10 : 17890163.095

[10.654536700000001, 7.535805400000001, 7.2846853, 7.024048399999998, 7.9240937, 7.811416099999995, 8.7937328, 8.0593218, 9.2111242, 11.466949799999995]
Estimation pour network.10, routes.10 : 4288285.709999999

[11.5852766, 7.000219100000001, 5.677677899999999, 4.995662800000002, 5.874674800000005, 5.244529200000002, 5.051318500000001, 4.963962199999997, 6.347038300000001, 8.066892899999992]
Estimation pour network.10, routes.10 : 3240362.615
'''
# Y'a du progrès !



