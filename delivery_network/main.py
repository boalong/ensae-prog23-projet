from time import perf_counter
from graph import Graph, graph_from_file


## Question 10. 

data_path = "/home/onyxia/ensae-prog23-private/input/"
file_no = "7.in"

g = graph_from_file(data_path + "network." + file_no)

liste_trajets = []

with open(data_path + "routes." + file_no, 'r') as f:
    n = int(f.readline())
    for _ in range(n):
        liste_trajets.append(list(map(float, f.readline().split())))

print(liste_trajets[:5])
start = perf_counter()
for trajet in liste_trajets[:10]:
    g.min_power(trajet[0], trajet[1])
stop = perf_counter()
print("Estimation :" , ((stop-start)*50000)/60) # on divise par 60 pour avoir les minutes

# il faut multiplier le temps par 500 (car on ne fait que les 1000 premiers trajets ici)

'''
Résultats:
Estimation : 50048.314666666665

Commentaire : plus de 50000 minutes, soit 833 heures, soit 34 jours, ce qui est beaucoup trop long pour un ordinateur.
'''







