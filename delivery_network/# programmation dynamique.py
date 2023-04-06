# programmation dynamique

from graph import trucks_from_file, active_trucks_from_file


def knapsack(W, wt, val, n):
    # Initialisez le tableau avec des valeurs de 0 pour tous les éléments
    K = [[0 for x in range(W + 1)] for x in range(n + 1)]
    # Construisez le tableau K de bas en haut
    for i in range(n + 1):
        for w in range(W + 1):
            if i == 0 or w == 0:
                K[i][w] = 0
            elif wt[i-1] <= w:
                K[i][w] = max(val[i-1] + K[i-1][w-wt[i-1]],  K[i-1][w])
            else:
                K[i][w] = K[i-1][w]
    return K[n][W]

def minimum_cost_for_routes(filename, trucks):
    # les fichiers routes.x.out affichent pour chaque ligne la puissance minimale requise pour le trajet
    # on lit progressivement les lignes du fichier routes.x.out
    f = open(filename, "r")
    list_costs = []
    for line in f:
        if line == "0\n":
            list_costs.append(1)
            continue
        power_min = int(line[:-3])
        for truck, char_trucks in trucks.items(): 
            if char_trucks[0] >= power_min:
                list_costs.append(char_trucks[1])
                break
            elif truck == len(trucks): 
                list_costs.append(0)
    return list_costs

def truck_minimum_cost(route, trucks_dict):
    list_costs = []
    list_power = []
    with open("/home/onyxia/ensae-prog23-private/input/" + route + ".in") as route_in:
            p= int(route_in.readline())
    
    with open("/home/onyxia/ensae-prog23-private/output/" + route + ".out") as routes_power:
        for k in range(p):
            list_power.append(int(routes_power.readline()))
    for p_min in list_power:
        costs = []
        for index in trucks_dict.keys():
            if trucks_dict[index][0] >= p_min:
                costs.append(trucks_dict[index][1])
        if costs == []:
            return None
        costs.sort()
        list_costs.append(costs[0])
    return list_costs
                



def dynamic(trucks_number, routes_number, Budget=26*10**9):
    trucks_dict = active_trucks_from_file("/home/onyxia/ensae-prog23-private/input/trucks." + str(trucks_number) + ".in")
    utilities = []
    cost = minimum_cost_for_routes("/home/onyxia/ensae-prog23-private/output/routes." + str(routes_number) + ".out", trucks_dict)
    with open("/home/onyxia/ensae-prog23-private/input/routes." + str(routes_number) + ".in", "r") as routes :
        n = int(routes.readline())
        for i in range(n):
            route_characteristics = tuple(map(int, routes.readline().split()))
            utilities.append(route_characteristics[2])
    "print(utilities)"       
    return knapsack(Budget, cost, utilities, len(cost))
    

print(truck_minimum_cost("routes.3", active_trucks_from_file("/home/onyxia/ensae-prog23-private/input/trucks.1.in")))