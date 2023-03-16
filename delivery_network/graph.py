import numpy as np

class Graph:
    def __init__(self, nodes=[]):
        self.nodes = nodes
        self.graph = dict([(n, []) for n in nodes])
        self.nb_nodes = len(nodes)
        self.nb_edges = 0
    
    def __str__(self):
        if not self.graph:
            output = "The graph is empty"            
        else:
            output = f"The graph has {self.nb_nodes} nodes and {self.nb_edges} edges.\n"
            for source, destination in self.graph.items():
                output += f"{source}-->{destination}\n"
        return output

#Séance 1    
    def add_edge(self, node1, node2, power_min, dist=1):
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
        INPUT:
        src: int (sommet de départ)
        dest: int (sommet d'arrivée)
        power: float (puissance du camion)
        OUTPUT:
        chemin: list (liste des sommets du chemin)'''

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
    
    def shortest_path_with_power(self, src, dest, power): # Question 5 : algorithme de Dijkstra
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

def kruskal(G):
    '''Algorithme de Kruskal
    INPUT:
        - G: un objet de la classe Graph
    OUTPUT:
        - X: un objet de la classe Graph : l'arbre couvrant de poids minimum de G'''

#Définition des fonctions utiles
    def find(s):
        while s != parent[s] :
            s = parent[s]
        return s

    def kruskal_int(graphe):
    # Il faut faire kruskal sur chaque composante connexe
    # Output: A minimum spanning tree defined by the edges X
        for sommet in graphe.keys():
        # for sommet in self.nodes:
            # on réalise makeset sur sommet
            parent[sommet]=sommet
            rang[sommet]=0
        
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
        liste_aretes.sort(key = lambda x: x[2])
            
        for arete in liste_aretes:
            if find(arete[0]) != find(arete[1]):
                X.add_edge(arete[0],arete[1],arete[2],arete[3])
                r_x = find(arete[0])
                r_y = find(arete[1])

        # Si x et y sont déjà dans le même set
                if r_x == r_y:
                    return None
                if rang[r_x] > rang[r_y]:
                    parent[r_y] = r_x
                else:
                    parent[r_x] = r_y
                    if rang[r_x] == rang[r_y]: 
                        rang[r_y] = rang[r_y] + 1    

    
        return X

    # Initialisation
    parent = {}
    rang = {}

    arbre_brut = kruskal_int(G.graph)

    # On trie les sommets du graphe pour passer les tests unitaires
    # arbre_brut.graph = {i: arbre_brut.graph[i] for i in range(1,arbre_brut.nb_nodes+1)}

    return arbre_brut


def trucks_from_file(filename):
    with open(filename, "r") as file:
        n = int(file.readline()) # n est le nombre de modèles de camions
        trucks = {i: () for i in range(1,n+1)}
        for i in range(1,n+1):
            char_trucks = tuple(map(int, file.readline().split()))
            trucks[i] = char_trucks
    return trucks

