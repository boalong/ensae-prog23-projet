from time import perf_counter
from graph import Graph, graph_from_file
import numpy as np
from matplotlib import pyplot as plt

data_path = "input/"
file_name = "routes.1.in"

g = graph_from_file(data_path + file_name)

# print(g.arbre_couvrant_min())
# print(g.arbre_couvrant_min().get_path_with_power(1,2,1001))
# print(g.min_power_acm(1,2))

liste_temps = [[] for i in range(101)]


for i in g.nodes:
    for j in g.nodes:
        if i < j:
            for percentile in range(101):
                print(percentile)
                start = perf_counter()
                g.min_power_acm(i,j,percentile)
                stop = perf_counter()
                liste_temps[percentile].append(stop-start)
'''

'''
for i in g.nodes[0:2]:
    for j in g.nodes[0:3]:
        if i < j:
            for percentile in range(101):
                print(percentile)
                start = perf_counter()
                g.min_power_acm(i,j,percentile)
                stop = perf_counter()
                liste_temps[percentile].append(stop-start)

liste_moyennes = []

for i in range(0,101):
    liste_moyennes.append(np.mean(liste_temps[i]))

print(np.argmin(liste_moyennes), min(liste_moyennes))

abcisses = [i for i in range(0,101)]

plt.plot(abcisses,liste_moyennes)
plt.show()


# pour regarder lequel est le plus efficace, on fait une moyenne de ses temps sur tout le graphe
# ensuite on prend l'argmin
'''

'''
start = perf_counter()
# g.arbre_couvrant_min() # 3.72 secondes
# print(g.min_power_acm(1,2))
g.min_power_acm(1,2,50)
stop = perf_counter()
print(stop-start)
'''