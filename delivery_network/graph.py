import numpy as np
from itertools import combinations
from random import random


# Séances 1 et 2

class Graph:
    """
    A class representing graphs as adjacency lists and implementing various algorithms on the graphs. Graphs in the class are not oriented. 
    Attributes: 
    -----------
    nodes: NodeType
        A list of nodes. Nodes can be of any immutable type, e.g., integer, float, or string.
        We will usually use a list of integers 1, ..., n.
    graph: dict
        A dictionnary that contains the adjacency list of each node in the form
        graph[node] = [(neighbor1, p1, d1), (neighbor1, p1, d1), ...]
        where p1 is the minimal power on the edge (node, neighbor1) and d1 is the distance on the edge
    nb_nodes: int
        The number of nodes.
    nb_edges: int
        The number of edges. 
    """

    def __init__(self, nodes=[]):
        """
        Initializes the graph with a set of nodes, and no edges. 
        Parameters: 
        -----------
        nodes: list, optional
            A list of nodes. Default is empty.
        """
        self.nodes = nodes
        self.graph = dict([(n, []) for n in nodes])
        self.nb_nodes = len(nodes)
        self.nb_edges = 0
    

    def __str__(self):
        """Prints the graph as a list of neighbors for each node (one per line)"""
        if not self.graph:
            output = "The graph is empty"            
        else:
            output = f"The graph has {self.nb_nodes} nodes and {self.nb_edges} edges.\n"
            for source, destination in self.graph.items():
                output += f"{source}-->{destination}\n"
        return output
    
# Séance 1

    def add_edge(self, node1, node2, power_min, dist=1):
        """
        Adds an edge to the graph. Graphs are not oriented, hence an edge is added to the adjacency list of both end nodes. 

        Parameters: 
        -----------
        node1: NodeType
            First end (node) of the edge
        node2: NodeType
            Second end (node) of the edge
        power_min: numeric (int or float)
            Minimum power on this edge
        dist: numeric (int or float), optional
            Distance between node1 and node2 on the edge. Default is 1.
        """
        node1 = int(node1)
        node2 = int(node2)
        if node1 not in self.graph:
            self.graph[node1] = []
            self.nb_nodes += 1
            self.nodes.append(node1)
        if node2 not in self.graph:
            self.graph[node2] = []
            self.nb_nodes += 1
            self.nodes.append(node2)

        self.graph[node1].append((node2, power_min, dist))
        self.graph[node2].append((node1, power_min, dist))
        self.nb_edges += 1

    def get_path_with_power(self, src, dest, power):
        '''
        INPUT :
            - src: int (sommet de départ)
            - dest: int (sommet d'arrivée)
            - power: float (puissance du camion)
        OUTPUT :
            - chemin: list (liste des sommets du chemin)'''

        # Faire un DFS à partir du sommet de départ. Dès qu'on tombe sur le sommet d'arrivée, arrêter le
        # DFS et reconstruire le chemin à partir des parents

        def DFS_chemin(G,root,arrivee,visited):

            # Create a stack for DFS
            stack = []
 
            # Push the current source node.
            stack.append(root)
 
            while (len(stack)):
                # Pop a vertex from stack
                s = stack.pop()
 
                # Stack may contain same vertex twice. So
                # we need to add the popped item to the list only
                # if it is not visited.
                if (not visited[s]):
                    visited[s] = True
 
                # Get all adjacent vertices of the popped vertex s
                # If a adjacent has not been visited, then push it
                # to the stack.
                # On les visitera plus tard

                # Ne pas faire une copie du graphe : seulement rajouter une condition
                for node in G.graph[s]:
                    if (node[1] <= power) and (not visited[node[0]]):
                        stack.append(node[0])
                        parent[node[0]] = s
                        if node[0] == arrivee:
                        # Dans ce cas, on peut arrêter le pgrm car on a tout ce qu'il faut pour avoir notre chemin
                            return None

        parent = {i: None for i in self.nodes} # il faut créer un dictionnaire avec le sommet précédent à chaque fois              
        visited_nodes = {node: False for node in self.nodes}

        DFS_chemin(self, src, dest, visited_nodes)

        # Récupérer le chemin à partir du dict parent

        # Si le sommet n'a pas été atteint par le parcours en profondeur
        if parent[dest] == None:
            return None

        chemin = [dest]

        # Si le sommet a été atteint
        while parent[dest] != src:
            chemin.append(parent[dest])
            dest = parent[dest]      

        chemin.append(src)

        chemin.reverse()

        return chemin
    
    # Question 5 : algorithme de Dijkstra
    def dijkstra_shortest_path_with_power(self, src, dest, power):
        '''
        INPUT : 
            - src : sommet de départ
            - dest : sommet d'arrivée
            - power : puissance maximale que l'on peut utiliser pour se déplacer
        OUTPUT :
            - chemin : liste des sommets du chemin le plus court entre src et dest avec la puissance power
        '''

        # Le début est le même que pour get_path_with_power_old

        # On crée un nouveau graphe qui ne contient que les arêtes dont la puissance est inférieure à power

        nv_graphe = Graph([])
        nv_graphe = Graph([i for i in self.nodes])
        for sommet, aretes in self.graph.items():
            for arete in aretes:
                if sommet < arete[0]:
                    if arete[1] <= power:
                        nv_graphe.add_edge(sommet, arete[0], arete[1], arete[2])

   
        # mtn on regarde s'il y a un chemin entre les deux sommets dans nv_graphe
        # pour ça, on regarde les sommets connexes de nv_graphe : si les deux sommets sont connexes, alors
        # il y a un chemin entre eux
        # pour trouver un chemin quelconque, on fait un deep search à partir de l'un des deux et on remonte à partir
        # second pour avoir un chemin

        for ssgraphe in nv_graphe.connected_components():
            if src in ssgraphe and dest not in ssgraphe:
                return None
            elif dest in ssgraphe and src not in ssgraphe:
                return None

        # On définit des fonctions intermédiaires pour l'algorithme de Dijkstra
        
        def poids_arc(s1,s2):
            if s1>s2:
                return poids_arc(s2,s1)
            for sommet in self.graph[s1]:
                if sommet[0] == s2:
                    poids = sommet[2]
            return poids
    
        def relacher(s1, s2, distance, predecesseur):
            # s1 et s2 sont adjacents
            if distance[s2] > distance[s1] + poids_arc(s1,s2):
                distance[s2] = distance[s1] + poids_arc(s1,s2)
                predecesseur[s2] = s1

        def extraire_distance_min(liste):
            d_min, ind_min = distance[liste[0]], 0 
            for i in range(1,len(liste)):
                if distance[liste[i]] < d_min:
                    d_min, ind_min = distance[liste[i]], i
            return ind_min
        
        def dijkstra(G):
            E = []
            F = nv_graphe.nodes
            while F:
                sommet = F.pop(extraire_distance_min(F))
                E.append(sommet)
                for adjacent in G[sommet]:
                    adjacent = adjacent[0]
                    relacher(sommet,adjacent,distance,predecesseur)
            
        def plus_court_chemin(G, depart, arrivee):
            dijkstra(G)
            point = arrivee
            chemin = []
            while point != depart:
                chemin.append(point)
                point = predecesseur[point]
        
            chemin.append(depart)
            chemin.reverse()
    
            return chemin
  
        # Initialisation des distances et des prédécesseurs
        distance = {}
        predecesseur = {}
        distance = {node : np.inf for node in nv_graphe.nodes}
        predecesseur = {node : None for node in nv_graphe.nodes}
        distance[src] = 0

        try:
            return plus_court_chemin(nv_graphe.graph, src, dest), distance[dest]
        except:
            return None
                                  
    def connected_components(self):
        '''
        This function returns a list of all the connected components of the graph
        OUTPUT:
            - list_components: a list of all the connected components of the graph
        '''
        list_components = [] #Initialisation of the component list
        visited_nodes = {node:False for node in self.nodes} #Initialisation of a dictionnary in order to follow  which nodes are already visited or not
        for i in self.graph.keys(): #We visit all the nodes and add an empty list to component_list for every unvisited nodes 
            if (not visited_nodes[i]):
                list_components.append([])
                #Implementation of a depth first search
                stack = [] #Initialisation of the component 
                stack.append(i) #Adding the node we are currently visiting to the component
                while (len(stack)):
                    node = stack.pop()
                    if (not visited_nodes[node]): #If we are on a node we haven't visit yet
                        list_components[-1].append(node)
                        visited_nodes[node] = True #We mark it as visited
                    for neighbour in self.graph[node]: #We visit all the neighbours of the node we are currently on
                        neighbour = neighbour[0]
                        if (not visited_nodes[neighbour]): #If we find an unvisited node, we add it to the connected component
                            stack.append(neighbour)
        return list_components
    
    def connected_components_set(self):
        '''
        This function returns a set of all the connected components of the graph
        OUTPUT:
            - set_components: a set of all the connected components of the graph
        '''
        return set(map(frozenset, self.connected_components()))
    
    def min_power(self, src, dest):
        '''
        This function returns the minimum power needed to go from src to dest
        INPUT:
            - src: the source node
            - dest: the destination node
        OUTPUT:
            - path: the path between the two nodes if it exists
            - p_min: the minimum power needed to go from src to dest
        '''
        power_list=[] #Initialisation of a list with the power of all edges of the graph
        for node in self.nodes:
            for neighbour in self.graph[node]:
                power_list.append(neighbour[1])
        power_list = list(set(power_list)) #Deleting the doubled values
        power_list.sort()
        
        a=0 #Initialisation of the parameters to realise a dichotomic search in the power list
        b=len(power_list)-1
        m=(a+b)//2
        while a<=b :
            p=power_list[m]
            p_inf=power_list[m-1]
            if self.get_path_with_power(src, dest, p)==None :
                a=m+1
            elif self.get_path_with_power(src, dest, p_inf)==None : #Check the case in which the minimum power is in the middle of the list 
                break
            else:
                b=m-1
            m=(a+b)//2
        path=self.get_path_with_power(src, dest, p)
        p_min=p
        return (path, p_min)
    
    # Proposition alternative pour la fonction min_power, fonctionnant avec une liste de puissances triée
    def get_liste_power(self):
        '''
        Renvoie une liste avec les puissances du graphe triées.
        Va permettre d'optimiser pour calc
        '''
        liste = []
        # on pourra garder une seule liste_power pour tout le graphe que l'on stockera si on fait plusieurs fois dans le même graphe

        for i in self.graph.values():
            for k in i:
                liste.append(k[1])

        liste.sort()

        # Enlever les doublons

        l_power = [liste[0]]
        for element in liste[1:]:
            if element != l_power[-1]:
                l_power.append(element)
        
        self.liste_power = l_power

    def alt_min_power(self, src, dest, pourcentile=15):
        """
        Should return path, min_power. 
        """
        try:
            type(self.liste_power)
        except:
            self.get_liste_power()

        l_power = self.liste_power

        valeur_sup = l_power[-1]
        valeur_inf = l_power[0]
        pivot = l_power[int((pourcentile/100)*len(l_power))] # regarder si ça marche mieux avec 70 comme pivot, 60 comme pivot, etc.
        # la fonction percentile ne marche pas (bcp trop long), donc on va calculer le pct 15 manuellement
        ind_pivot = l_power.index(pivot)

        # Se déplacer sur l_power et non sur l'ensemble des entiers naturels!

        # liste_power[((len(liste_power)+1)//2)-1])

        while valeur_inf != valeur_sup:
            # print(len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)]))
            if self.is_path_with_power(src, dest, pivot) == False: # veut dire que la puissance pivot est trop petite
                valeur_inf = l_power[ind_pivot + 1] # pivot + 1
                pivot = l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)][((len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                ind_pivot = l_power.index(pivot)           
            elif self.is_path_with_power(src, dest, pivot) == True:
                valeur_sup = pivot
                pivot = l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)][((len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                # pivot = (l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)],50)
                ind_pivot = l_power.index(pivot)
        return self.get_path_with_power(src, dest, pivot), pivot 

#Séance 2

    def DFS_parents(self, root=1):
        '''On fait un seul parcours en profondeur pour récupérer tous les liens de parenté à partir du sommet root
        INPUT:
            - root: le sommet de départ
        OUTPUT:
            - parent: un dictionnaire avec le sommet précédent à chaque fois'''
        # Initialisation
        parent = {i: None for i in self.nodes} # il faut créer un dictionnaire avec le sommet précédent à chaque fois              
        visited = {i: False for i in self.nodes}

        # Create a stack for DFS
        stack = []
 
        # Push the current source node.
        stack.append(root)
 
        while (len(stack)):
            # Pop a vertex from stack
            s = stack.pop()
 
            # Stack may contain same vertex twice. So
            # we need to add the popped item to the list only
            # if it is not visited.
            if (not visited[s]):
                visited[s] = True
 
            # ne pas faire une copie du graphe : seulement rajouter une condition
            for node in self.graph[s]:
                if (not visited[node[0]]):
                    stack.append(node[0])
                    parent[node[0]] = s

        return parent
 
    def get_kruskal(self):
        '''On récupère l'arbre couvrant de poids minimum'''
        self.kruskal = kruskal(self) 
    
    # Question 15
    def min_power_routes(self, liste_routes, root=1):
        '''Pour obtenir une liste des min_power pour toutes le routes demandées
        INPUT:
            - liste_routes: une liste de routes
            - root: le sommet de départ du DFS
        OUTPUT:
            - liste_trajets: une liste de trajets'''
        # liste_routes est une liste de 3 éléments : le 1er est le départ, le 2ème est l'arrivée, le 3ème est l'utilité
        parent = self.kruskal.DFS_parents(root)
        # On initialise l'output
        # liste_trajets = []
        liste_power = []
        # ct = 0
        for route in liste_routes:
            # ct += 1
            # print(ct)
            # if ct == 10001:
                # return liste_trajets, liste_power
            src = route[0]
            dest = route[1]
            chemin = []
            pwr = 0

            # Procédure pour reconstituer le trajet si src est différent de la root du DFS

            # On reconstitue les chemins jusqu'à la root pour src et dest:

            chemin.append(src)
            while src != root:
                chemin.append(parent[src])
                src = parent[src]

            chemin_dest_to_root = []
            chemin_dest_to_root.append(dest)
            while dest != root:
                chemin_dest_to_root.append(parent[dest])
                dest = parent[dest]
            # On a désormais deux listes à fusionner
            # On prend le 1er élément commun des deux listes pour faire la fusion
            # Opération coûteuse
            cond = False
            for u in chemin:
                for v in chemin_dest_to_root:
                    if u == v:
                        chemin = chemin[:chemin.index(u)+1]
                        chemin_dest_to_root = chemin_dest_to_root[:chemin_dest_to_root.index(v)]
                        chemin_dest_to_root.reverse()
                        chemin.extend(chemin_dest_to_root)
                        cond = True
                        break
                if cond == True:
                    break
            # mtn, on ne regarde que chemin (qui est mtn chemin_src_to_dest)             
            # On itère sur le chemin et on enregistre la puissance minimal requise
            for sommet in chemin[:-1]:
                for adjacent in self.graph[sommet]:
                    if adjacent[0] == chemin[chemin.index(sommet)+1]:
                    # si on tombe sur le sommet suivant du chemin
                        if adjacent[1] > pwr:
                        # on met à jour pwr
                            pwr = adjacent[1]

            # liste_trajets.append(chemin)
            liste_power.append(pwr)

        # return liste_trajets, liste_power
        return liste_power
    
def graph_from_file(filename):
    '''Crée un objet de la classe Graph à partir d'un fichier texte
    INPUT:
        - filename: le nom du fichier
    OUTPUT:
        - g: un objet de la classe Graph'''
    with open(filename, "r") as file:
        n, m = map(int, file.readline().split())
        g = Graph([i for i in range(1, n+1)])
        for _ in range(m):
            edge = list(map(float, file.readline().split()))
            if len(edge) == 3:
                node1, node2, power_min = edge
                g.add_edge(node1, node2, power_min) # will add dist=1 by default
            elif len(edge) == 4:
                node1, node2, power_min, dist = edge
                g.add_edge(node1, node2, power_min, dist)
            else:
                raise Exception("Format incorrect")
    return g







# Séance 3

# Fonctions intermédiaires pour Kruskal
def makeset(s, parent, rang):
    parent[s] = s
    rang[s] = 0

def sort_by_weight(list_edges):
    return list_edges.sort(key = lambda x: x[2])

def find(s, parent, rang):
    while s != parent[s] :
        s = parent[s]
    return s

def union(x,y, parent, rang):
    r_x = find(x, parent, rang)
    r_y = find(y, parent, rang)

    # Si x et y sont déjà dans le même set
    if r_x == r_y:
        return None
    if rang[r_x] > rang[r_y]:
        parent[r_y] = r_x
    else:
        parent[r_x] = r_y
        if rang[r_x] == rang[r_y]: 
            rang[r_y] = rang[r_y] + 1  

def kruskal_int(graphe, parent, rang):
    '''Algorithme de Kruskal intermédiaire
    INPUT:
        - G: un objet de la classe Graph
    OUTPUT:
        - X: un objet de la classe Graph : l'arbre couvrant de poids minimum de G'''
    
    for sommet in graphe.keys():
    # for sommet in self.nodes:
        makeset(sommet, parent, rang)

    # X est un graphe vide
    X = Graph([])

    # Créer une liste avec les noeuds et leur poids
    liste_aretes = []
    for sommet, arrivee in graphe.items():
        for element in arrivee:
            # On évite de mettre deux fois la même arête
            if sommet < element[0]:
                liste_aretes.append([sommet, element[0], element[1], element[2]])
    
    # Sort the edges E by weight    
    sort_by_weight(liste_aretes)
            
    for arete in liste_aretes:
        if find(arete[0], parent, rang) != find(arete[1], parent, rang):
            X.add_edge(arete[0],arete[1],arete[2],arete[3])
            union(arete[0], arete[1], parent, rang)
    
    return X

# Fonction finale pour Kruskal
def kruskal(G):
    '''Algorithme de Kruskal
    INPUT:
        - G: un objet de la classe Graph
    OUTPUT:
        - un objet de la classe Graph : l'arbre couvrant de poids minimum de G'''

    # Initialisation
    parent = {}
    rang = {}

    return kruskal_int(G.graph, parent, rang)






# Séances 4 et 5

def trucks_from_file(filename):
    '''
    Cette fonction lit un fichier texte et renvoie un dictionnaire de camions, 
    avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    INPUT:
        - filename: le nom du fichier contenant les caractéristiques des camions (trucks.x.in)
    OUTPUT:
        - trucks: un dictionnaire de camions, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    '''
    # On lit le fichier et on crée un dictionnaire de camions, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    with open(filename, "r") as file:
        n = int(file.readline()) # n est le nombre de modèles de camions
        trucks = {i: () for i in range(1,n+1)}
        for i in range(1,n+1):
            char_trucks = tuple(map(int, file.readline().split()))
            trucks[i] = char_trucks

    return trucks

def active_trucks(trucks):
    '''
    Cette fonction renvoie un dictionnaire ne contenant que les camions intéressants, c'est-à-dire
    les camions les moins chers pour une puissance donnée
    INPUT:
        - trucks: un dictionnaire de camions, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    OUTPUT:
        - trucks: un dictionnaire de camions actifs, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    '''
    # le dictionnaire de camions est déjà trié par puissance croissante
    # s'il y a un camion plus puissant et moins cher, on le remplace

    min_cost_for_power = 0

    for i in range(1, len(trucks)+1):
        if trucks[i][1] < min_cost_for_power:
            min_cost_for_power = trucks[i][1]
            for j in range(1, i):
                try:
                    if trucks[j][1] >= min_cost_for_power:
                        del trucks[j]
                except:
                    pass
        else:
            min_cost_for_power = trucks[i][1]

    return trucks

def active_trucks_from_file(filename):
    '''
    Cette fonction lit un fichier texte et renvoie un dictionnaire de camions actifs, définis comme les camions les moins chers pour une puissance donnée
    INPUT:
        - filename: le nom du fichier contenant les caractéristiques des camions (trucks.x.in)
    OUTPUT:
        - un dictionnaire de camions actifs, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)'''
    # On lit le fichier et on crée un dictionnaire de camions actifs, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    trucks = trucks_from_file(filename)
    return active_trucks(trucks)

def list_routes_from_file(filename):
    '''
    Cette fonction lit un fichier texte et renvoie une liste de trajets, avec comme élément une liste de caractéristiques (départ, arrivée, utilité)
    INPUT:
        - filename: le nom du fichier contenant les caractéristiques des trajets (routes.x.in)
    OUTPUT:
        - liste_trajets: une liste de trajets, avec comme élément une liste de caractéristiques (départ, arrivée, utilité)
    '''
    with open(filename, 'r') as f:
        n = int(f.readline())
        liste_trajets = []
        for _ in range(n):
            depart, arrivee, utilite = f.readline().split()
            depart, arrivee, utilite = int(depart), int(arrivee), float(utilite)
            liste_trajets.append([depart, arrivee, utilite])
            # liste_trajets.append(list(map(float, f.readline().split())))
    
    return liste_trajets

def trucks_for_routes(filename, trucks):
    '''
    Cette fonction lit un fichier texte de la forme routes.x.out et renvoie une liste de camions parallèle à la liste de trajets, 
    avec pour chaque trajet le camion le moins coûteux qui passe
    INPUT:
        - filename: le nom du fichier contenant les caractéristiques des trajets (routes.x.out)
        - trucks: un dictionnaire de camions actifs, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    OUTPUT:
        - list_trucks_for_routes: une liste de camions parallèle à la liste de trajets, avec pour chaque trajet le camion le moins coûteux qui passe'''
    # On crée une liste de camions parallèle à la liste de trajets, avec pour chaque trajet le camion le moins coûteux qui passe
    # il faudra d'abord supprimer les camions pour lesquels il en existe un avec une puissance supérieure et un coût inférieur
    # les fichiers routes.x.out affichent pour chaque ligne la puissance minimale requise pour le trajet
    # on lit progressivement les lignes du fichier routes.x.out
    f = open(filename, "r")
    list_trucks_for_routes = []
    for line in f:
        if line == "0\n":
            list_trucks_for_routes.append(1) # si la puissance requise pour le trajet est nulle, on prend le camion le moins coûteux
            continue
        power_min = int(line[:-3])
        # print(power_min)
        for truck, char_trucks in trucks.items(): # on itère dans l'ordre croissant des puissances des camions
            if char_trucks[0] >= power_min:
                # print(truck)
                list_trucks_for_routes.append(truck)
                break
            elif truck == len(trucks): # s'il s'agit du camion le plus puissant, et que malgré tout la puissance requise n'est pas atteinte, on entre 0
                list_trucks_for_routes.append(0)

    return list_trucks_for_routes

def is_list_trucks_buyable(list_trucks_for_routes, trucks):
    '''
    Cette fonction vérifie si le budget est suffisant pour acheter les camions nécessaires à tous les trajets de la liste entrée
    INPUT:
        - list_trucks_for_routes: une liste de camions parallèle à la liste de trajets, avec pour chaque trajet le camion le moins coûteux qui passe
        - trucks: un dictionnaire de camions actifs, avec comme clé le numéro du camion et comme valeur un tuple de caractéristiques (puissance, coût)
    OUTPUT:
        - True si le budget est suffisant, False sinon'''
    # On vérifie que le budget est suffisant pour faire toutes les routes de la liste entrée
    B = 25*10**9
    cost = 0
    for truck in list_trucks_for_routes:
        if truck != 0: # le trajet peut être effectué
            # print(truck)
            # print(trucks[truck])
            cost += trucks[truck][1]
    if B - cost >= 0:
        return True
    return False

def naive_add_routes_until_budget_exceeded(filename_in_trucks, filename_in_routes, filename_out):
    '''
    Cette fonction prend les routes dans l'ordre jusqu'à ce que le budget soit dépassé.
    Cela donne un 1er résultat que l'on pourra comparer avec les résultats de l'algorithme knapsack.
    INPUT:
        - filename_in_trucks: le nom du fichier contenant les caractéristiques des camions (trucks.x.in)
        - filename_in_routes: le nom du fichier contenant les caractéristiques des trajets (routes.x.in)
        - filename_out: le nom du fichier contenant les caractéristiques des trajets (routes.x.out)
    OUTPUT:
        - list_routes_done: une liste des trajets effectués, avec comme élément une liste de caractéristiques (départ, arrivée, utilité)
        - utility: l'utilité totale'''
    
    # On suppose que l'on a déjà trucks
    # On prend les routes dans l'ordre jusqu'à ce que le budget soit dépassé

    trucks = active_trucks_from_file(filename_in_trucks)

    list_trucks_for_routes = trucks_for_routes(filename_out, trucks)

    i=1
    list_trucks_max = list_trucks_for_routes[:i]
    while is_list_trucks_buyable(list_trucks_max, trucks):
        # print(is_list_trucks_buyable(list_routes_max, trucks))
        i+=1
        list_trucks_max = list_trucks_for_routes[:i]

    list_trucks_max.pop()

    # Afficher les routes, afficher l'utilité totale
    list_routes = list_routes_from_file(filename_in_routes)
    list_routes_done = []

    utility = 0

    for i in range(len(list_trucks_max)): # i est l'indice du camion le plus économique pour chaque trajet
        if list_trucks_max[i] != 0:
            list_routes_done.append((list_routes[i][0], list_routes[i][1]))
            utility += list_routes[i][2]

    active_index = 0
    for i in range(len(list_trucks_max)):
        if list_trucks_max[active_index] == 0:
            list_trucks_max.pop(active_index)
        else:
            active_index += 1

    # return list_routes_done, utility # le nombre de trajets effectués, l'utilité
    return len(list_routes_done), utility # le nombre de trajets effectués, l'utilité car on ne veut pas afficher les trajets effectués sinon il y a pb d'affichage
     
# Fonctions pour l'algorithme knapsack
def ratio_utility_cost(filename_in_trucks, filename_in_routes, filename_out):
    '''
    Cette fonction calcule le ratio utilité/cout pour chaque trajet
    INPUT:
        - filename_in_trucks: le nom du fichier contenant les caractéristiques des camions (trucks.x.in)
        - filename_in_routes: le nom du fichier contenant les caractéristiques des trajets (routes.x.in)
        - filename_out: le nom du fichier contenant les caractéristiques des trajets (routes.x.out)
    OUTPUT:
        - liste_ratio: une liste de tuples (ratio, indice du trajet)
        - liste_camions: une liste de camions parallèle à la liste des ratios, avec pour chaque trajet le camion le moins coûteux qui passe'''

    liste_ratio = []

    list_trucks = active_trucks_from_file(filename_in_trucks)
    list_routes = list_routes_from_file(filename_in_routes)
    list_trucks_for_routes = trucks_for_routes(filename_out, list_trucks)

    for i in range(len(list_routes)):
        if list_trucks_for_routes[i] != 0:
            liste_ratio.append((list_routes[i][2]/list_trucks[list_trucks_for_routes[i]][1], i)) # on ajoute le ratio et l'indice du trajet (attention, il s'agit de l'indice et pas du numéro du trajet: on part de 0)

    # liste_camions est la liste parallèle à liste_ratio, avec pour chaque trajet le camion le moins coûteux qui passe
    liste_camions = []

    liste_ratio.sort(reverse=True)
    
    for i in range(len(liste_ratio)):
        # avec l'indice du trajet on va récupérer le numéro du camion avec list_trucks_for_routes
        liste_camions.append(list_trucks_for_routes[liste_ratio[i][1]]) # on a une liste de camions

    return liste_ratio, liste_camions

# Algorithme glouton/knapsack
def max_utility_from_ratio_glouton(filename_in_trucks, filename_in_routes, filename_out):
    '''
    Cette fonction prend les trajets dans l'ordre décroissant du ratio utilité/cout jusqu'à ce que le budget soit dépassé.
    Elle donne une approximation de la solution optimale, mais la solution optimale n'est pas atteinte forcément, car le budget peut ne
    pas être atteint.
    INPUT:
        - filename_in_trucks: le nom du fichier contenant les caractéristiques des camions (trucks.x.in)
        - filename_in_routes: le nom du fichier contenant les caractéristiques des trajets (routes.x.in)
        - filename_out: le nom du fichier contenant les caractéristiques des trajets (routes.x.out)
    OUTPUT:
        - list_routes_done: une liste des trajets effectués, avec comme élément une liste de caractéristiques (départ, arrivée, utilité)
        - utility: l'utilité totale'''

    trucks = active_trucks_from_file(filename_in_trucks)

    liste_ratio, liste_camions = ratio_utility_cost(filename_in_trucks, filename_in_routes, filename_out)

    i=1
    list_trucks_max = liste_camions[:i]
    while is_list_trucks_buyable(list_trucks_max, trucks):
        # print(is_list_trucks_buyable(list_trucks_max, trucks))
        i+=1
        if i%1000 == 0:
            print(i)
        list_trucks_max = liste_camions[:i] # on prend les i trajets les plus rentables

    list_trucks_max.pop()

    # Afficher les routes, afficher l'utilité totale
    list_routes = list_routes_from_file(filename_in_routes)
    list_routes_done = []

    utility = 0

    for i in range(len(list_trucks_max)): # i est l'indice du camion le plus économique pour chaque trajet
        if list_trucks_max[i] != 0:
            ind_route = liste_ratio[i][1]
            list_routes_done.append((list_routes[ind_route][0], list_routes[ind_route][1]))
            utility += list_routes[ind_route][2]

    active_index = 0
    for i in range(len(list_trucks_max)):
        if list_trucks_max[active_index] == 0:
            list_trucks_max.pop(active_index)
        else:
            active_index += 1

    # return list_routes_done, utility
    return len(list_routes_done), utility # on retourne le nombre de trajets effectués et l'utilité totale car sinon on aurait un problème d'affichage

def improved_glouton(filename_in_trucks, filename_in_routes, filename_out):
    # On prend les trajets dans l'ordre décroissant du ratio utilité/cout

    trucks = active_trucks_from_file(filename_in_trucks)
    
    liste_ratio, liste_camions = ratio_utility_cost(filename_in_trucks, filename_in_routes, filename_out)
    # liste_ratio est une liste de tuples (ratio, indice du trajet)
    # liste_camions est une liste de camions, avec pour chaque trajet le camion le moins coûteux qui passe

    i=1
    list_routes_max = liste_camions[:i]
    # list_routes_max est la liste des numéros de camions qui effectuent les trajets les plus rentables
    while is_list_trucks_buyable(list_routes_max, trucks):
        # print(is_list_trucks_buyable(list_routes_max, trucks))
        i+=1
        if i%1000 == 0:
            print(i)
        list_routes_max = liste_camions[:i] # on prend les i trajets les plus rentables

    list_routes_max.pop()

    cout_actuel = 0
    for i in range(len(list_routes_max)):
        cout_actuel += trucks[list_routes_max[i]][1]

    # Il nous faut le budget restant
    budget_restant = 25*10**9 - cout_actuel

    list_routes_max_temp = list_routes_max[:]
    # On va jusqu'au bout de la list_routes_max pour voir si on peut ajouter des trajets
    for j in liste_camions[i:]:
        if trucks[j][1] <= budget_restant: # on peut ajouter le trajet
            list_routes_max_temp.append(j)
            budget_restant -= trucks[j][1]

    list_routes_max = list_routes_max_temp[:]

    # Afficher les routes, afficher l'utilité totale
    list_routes = list_routes_from_file(filename_in_routes)
    list_routes_done = []

    utility = 0

    for i in range(len(list_routes_max)): # i est l'indice du camion le plus économique pour chaque trajet
        if list_routes_max[i] != 0:
            ind_route = liste_ratio[i][1]
            list_routes_done.append((list_routes[ind_route][0], list_routes[ind_route][1]))
            utility += list_routes[ind_route][2]

    active_index = 0
    for i in range(len(list_routes_max)):
        if list_routes_max[active_index] == 0:
            list_routes_max.pop(active_index)
        else:
            active_index += 1

    return len(list_routes_max), len(list_routes_done), utility

def algorithme_naif(filename_in_trucks, filename_in_routes, filename_out):
    '''
    Cette algorithme donne une solution optimale, mais le temps de calcul est très long, car il faut tester toutes les combinaisons de trajets possibles.
    INPUT:
        - filename_in_trucks: le nom du fichier contenant les camions
        - filename_in_routes: le nom du fichier contenant les trajets
        - filename_out: le nom du fichier de sortie
    OUTPUT:
        - best_comb: la liste des trajets à effectuer
        - max_utility: l'utilité totale de la combinaison
    ''' 

    trucks = active_trucks_from_file(filename_in_trucks)
    routes = list_routes_from_file(filename_in_routes)
    list_trucks_for_routes = trucks_for_routes(filename_out, active_trucks_from_file(filename_in_trucks))

    # On regarde pour chaque combinaisons de trajets si le budget est suffisant
    # Si oui, on calcule l'utilité totale de la combinaison
    # On met à jour la combinaison avec l'utilité maximale

    # On crée une liste avec les numéros des trajets

    liste_no_routes = [i for i in range(1, len(routes)+1)]

    max_utility = 0
    best_comb = []

    for i in range(1, len(routes)+1):
        for comb in combinations(liste_no_routes, i):
            print(comb) # afficher les combinaisons pour suivre la progression de l'algorithme
            # Il faut calculer la liste des camions pour chaque trajet
            current_trucks_for_routes = []
            for route in comb:
                current_trucks_for_routes.append(list_trucks_for_routes[route-1])
            if is_list_trucks_buyable(current_trucks_for_routes, trucks):
                utility = 0
                for route in comb:
                    utility += routes[route-1][2]
                if utility > max_utility:
                    max_utility = utility
                    best_comb = comb

    return best_comb, max_utility
    # Cet algorithme n'est pas raisonnable: il faut trouver une autre méthode pour calculer toutes les combinaisons



# Question 20 (i)

# On récupère l'arbre de poids minimum, et on supprime les arêtes avec une probabilité epsilon
# Finalement, cette fonction n'est pas utilisée
def kruskal_epsilon(G, epsilon):
    '''
    Algorithme de Kruskal avec suppression des arêtes avec une probabilité epsilon
    INPUT:
        - G: un objet de la classe Graph
        - epsilon: un réel entre 0 et 1
    OUTPUT:
        - Y: un objet de la classe Graph : l'arbre couvrant de poids minimum de G avec suppression des arêtes avec une probabilité epsilon'''
    
    # Initialisation
    parent = {}
    rang = {}

    arbre_couvrant = kruskal_int(G.graph, parent, rang)
    Y = Graph([i for i in G.nodes]) # Graphe vide

    # On supprime les arêtes avec une probabilité epsilon
    for sommet, aretes in arbre_couvrant.graph.items():
        # print("Sommet:", sommet)
        # sommet est une liste de noeuds
        for i, arete in enumerate(aretes):
            # on fait attention à ne pas passer deux fois sur la même arête
            if sommet < arete[0]:
                # print("Arete:", arete)
                # tirage = random()
                # print("Tirage:", tirage)
                if random() >= epsilon:
                    # print("On conserve l'arête") # C'est plus simple de créer un nouveau graphe car on n'a pas de méthode remove_edge
                    Y.add_edge(sommet, arete[0], arete[1], arete[2])
    
    return Y

# Pour calculer l'espérance d'utilité d'un trajet, il faut calculer la probabilité qu'il ne puisse plus être accessible
# C'est-à-dire qu'une arête se soit cassée dessus
# Il faut d'abord calculer le nombre d'arêtes qu'il faut parcourir pour qu'un trajet soit effectué

def nb_aretes_entre_deux_sommets_kruskal(graphe, parent, src, dest, root=1):
    '''
    Fonction qui calcule le nombre d'arêtes entre deux sommets dans un arbre couvrant de poids minimum
    INPUT:
        - graphe: un arbre couvrant de poids minimum, objet de la classe Graph
        - parent: un dictionnaire contenant les parents des sommets
        - src: le sommet de départ
        - dest: le sommet d'arrivée
        - root: le sommet de départ du DFS
    OUTPUT:
        - nb_aretes: le nombre d'arêtes entre les deux sommets
    '''

    chemin = []

    # Procédure pour reconstituer le trajet si src est différent de la root du DFS

    # On reconstitue les chemins jusqu'à la root pour src et dest:

    chemin.append(src)
    while src != root:
        chemin.append(parent[src])
        src = parent[src]

    chemin_dest_to_root = []
    chemin_dest_to_root.append(dest)
    while dest != root:
        chemin_dest_to_root.append(parent[dest])
        dest = parent[dest]

    # On a désormais deux listes à fusionner
    # On prend le 1er élément commun des deux listes pour faire la fusion
    cond = False
    for u in chemin:
        for v in chemin_dest_to_root:
            if u == v:
                chemin = chemin[:chemin.index(u)+1]
                chemin_dest_to_root = chemin_dest_to_root[:chemin_dest_to_root.index(v)]
                chemin_dest_to_root.reverse()
                chemin.extend(chemin_dest_to_root)
                cond = True
                break
        if cond == True:
            break
    
    # print(chemin)
    return len(chemin)-1

# La probabilité de pouvoir emprunter le trajet est (1-epsilon)^nb_arêtes
# Probabilité qu'aucune arête ne se casse sur le trajet

def proba_trajet_kruskal(graphe, parent, src, dest, epsilon, root=1):
    '''
    Fonction qui calcule la probabilité qu'aucune arête ne se casse sur le trajet
    INPUT:
        - graphe: un arbre couvrant de poids minimal, objet de la classe Graph
        - parent: un dictionnaire contenant les parents des sommets
        - src: le sommet de départ
        - dest: le sommet d'arrivée
        - epsilon: un réel entre 0 et 1
        - root: le sommet de départ du DFS
    OUTPUT:
        - la probabilité qu'aucune arête ne se casse sur le trajet'''
    
    nb_aretes = nb_aretes_entre_deux_sommets_kruskal(graphe, parent, src, dest, root)
    return (1-epsilon)**nb_aretes

# L'utilité espérée est le produit de cette probabilité par l'utilité du trajet

def utilite_esperee(proba, utilite):
    '''
    Fonction qui calcule l'utilité espérée d'un trajet
    INPUT:
        - proba: la probabilité qu'aucune arête ne se casse sur le trajet
        - utilite: l'utilité du trajet
    OUTPUT:
        - l'utilité espérée du trajet'''
    
    return proba*utilite

# On crée un fichier avec pour chaque trajet l'utilité espérée
# Il faut ouvrir le fichier routes.x.in pour avoir les trajets et les utilités
def utilite_esperee_trajets(file_no, epsilon, root=1):
    '''
    Fonction qui calcule l'utilité espérée de tous les trajets
    INPUT:
        - file_no: str, le numéro du fichier 
        - epsilon: un réel entre 0 et 1
        - root: le sommet de départ du DFS
    OUTPUT:
        - un fichier avec pour chaque trajet l'utilité espérée'''

    g = graph_from_file("input/network." + file_no + ".in")
    g_kruskal = kruskal(g)
    parent = g_kruskal.DFS_parents(root)

    # Calculer les min_power de toutes les routes
    # pour optimiser le process, il faut stocker une liste min power pour tout le graphe
    # on va associer une méthode liste power pour calculer une fois pour toute

    liste_trajets = []

    # On crée une liste de trajets avec depart, arrivee, utilité espérée
    with open("input/routes." + file_no + ".in", 'r') as f:
        n = int(f.readline())
        for _ in range(n):
            depart, arrivee, utilite = f.readline().split()
            depart, arrivee, utilite = int(depart), int(arrivee), float(utilite)
            utilite_esp = utilite_esperee(proba_trajet_kruskal(g_kruskal, parent, depart, arrivee, epsilon), utilite)
            liste_trajets.append([depart, arrivee, utilite_esp])

    g = open("input/routes_esp." + file_no + ".in", "w")
    g.write(str(n) + "\n")
    for elem in liste_trajets:
        g.write(str(elem[0]) + ' ' + str(elem[1]) + ' ' + str(elem[2]) + "\n")
    g.close()

# Maintenant, il suffit d'appliquer l'algo glouton pour trouver une estimation de la solution optimale

