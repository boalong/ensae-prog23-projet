from graph import *


print(algorithme_naif("input/trucks.0.in", "input/routes.1.in", "output/routes.1.out"))


trucks = trucks_from_file("input/trucks.1.in")
active_trucks = active_trucks(trucks)
# print(trucks)
# print(active_trucks)
# on valide active_trucks pour le fichier trucks.1.in


print(trucks_for_routes("output/routes.3.out", active_trucks)[:10]) # avec le fichier routes.3.out, on obtient les 5 premiers camions utilisés
# on valide trucks_for_routes avec le fichier routes.3.out

# print(ratio_utility_cost("input/trucks.1.in", "input/routes.3.in", "output/routes.3.out")[0][:10])
# print(ratio_utility_cost("input/trucks.1.in", "input/routes.3.in", "output/routes.3.out")[1][:10])


print(routes_and_trucks_with_max_utility("input/trucks.1.in", "input/routes.3.in", "output/routes.3.out"))
# (36312, 36312, 180633148.0)

print(max_utility_from_ratio_glouton("input/trucks.1.in", "input/routes.3.in", "output/routes.3.out"))
# (53055, 53055, 467368164.0)



# print(routes_and_trucks_with_max_utility("input/trucks.0.in", "input/routes.1.in", "output/routes.1.out"))



'''
# print(max_utility_from_ratio_glouton("input/trucks.0.in", "input/routes.3.in", "output/routes.3.out"))

print(routes_and_trucks_with_max_utility("input/trucks.0.in", "input/routes.3.in", "output/routes.3.out"))
'''