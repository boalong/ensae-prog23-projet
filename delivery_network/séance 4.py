from graph import Graph, trucks_from_file, trucks_for_routes, is_list_trucks_buyable, routes_and_trucks_with_max_utility


# t0 = trucks_from_file("input/trucks.0.in")
# print(t0)


# t = trucks_from_file("input/trucks.2.in")
# print(t)

# tfr = trucks_for_routes("output/routes.3.out", t0)
# print(tfr)
# print(len(tfr))

# print(is_list_trucks_buyable(tfr, t0))

print(routes_and_trucks_with_max_utility("input/trucks.0.in", "input/routes.3.in", "output/routes.3.out"))