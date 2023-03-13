'''
from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "00.in"

g = graph_from_file(data_path + "network." + file_no)

g.get_kruskal()

print(g.min_power_routes([[1,3,1],[1,10,1],[9,6,1],[7,4,1],[4,10,1],[8,1,1]]))
'''


from time import perf_counter
import numpy as np
from graph import Graph, graph_from_file, kruskal


data_path = "input/"
file_no = "2.in"

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
start = perf_counter()
resultats = g.min_power_routes(liste_trajets)
stop = perf_counter()
print(resultats[0][:10],resultats[1][:10])
print("Estimation :" , (stop-start)*100)


# il faut multiplier le temps par 500 (car on ne fait que les 1000 premiers trajets ici)

'''
Résultas:

Estimation : 1427.30895

'''