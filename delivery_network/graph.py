import numpy as np

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
        
        
        # on commence par regarder si le trajet peut être couvert

        # graphe avec seulement les arêtes que peut emprunter le camion étant donnée sa puissance
        # faire attention à ne pas ajouter 2 fois la même arête
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

        '''
        for ssgraphe in nv_graphe.connected_components():
            if src in ssgraphe and dest in ssgraphe:
                ind_min = min(ssgraphe.index(src), ssgraphe.index(dest))
                ind_max = max(ssgraphe.index(src), ssgraphe.index(dest))
                chemin = [ssgraphe[i] for i in range(ind_min, ind_max+1)]
                break
            elif src in ssgraphe and dest not in ssgraphe:
                chemin = None
                break
            elif dest in ssgraphe and src not in ssgraphe:
                chemin = None
                break

        return chemin
        '''

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
                for node in G.graph[s]:
                    node = node[0]
                    if (not visited[node]):
                        stack.append(node)
                        parent[node] = s
                        if node == arrivee:
                        # Dans ce cas, on peut arrêter le pgrm car on a tout ce qu'il faut pour avoir notre chemin
                            return None

        parent = {i: None for i in nv_graphe.nodes} # il faut créer un dictionnaire avec le sommet précédent à chaque fois              
        visited_nodes = {node:False for node in nv_graphe.nodes}

        DFS_chemin(nv_graphe,src, dest, visited_nodes)

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
        
        '''
        # Bonus : algorithme de Dijkstra
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
        
        def dijkstra(G, racine):
            E = []
            F = self.nodes
            while F:
                sommet = F.pop(extraire_distance_min(F))
                E.append(sommet)
                for adjacent in G[sommet]:
                    adjacent = adjacent[0]
                    relacher(sommet,adjacent,distance,predecesseur)
            
        def plus_court_chemin(G, depart, arrivee):
            dijkstra(G, depart)
            point = arrivee
            chemin = []
            while point != depart:
                chemin.append(point)
                point = predecesseur[point]
        
            chemin.append(depart)
            chemin.reverse()
    
            return chemin
  
        # Initialisation
        distance = {node : np.inf for node in self.nodes}
        predecesseur = {node : None for node in self.nodes}
        distance[src] = 0

        try:
            return plus_court_chemin(nv_graphe.graph, src, dest), distance[dest]
        except:
            return None
        '''
    
    
    def connected_components(self):
        list_components = []

        def DFS(root,visited):

            # Create a stack for DFS
            stack = []
 
            # Push the current source node.
            stack.append(root)
 
            while (len(stack)):
                # Pop a vertex from stack and print it
                s = stack.pop()
 
                # Stack may contain same vertex twice. So
                # we need to add the popped item to the list only
                # if it is not visited.
                if (not visited[s]):
                    list_components[-1].append(s)
                    visited[s] = True
 
                # Get all adjacent vertices of the popped vertex s
                # If a adjacent has not been visited, then push it
                # to the stack.
                for node in self.graph[s]:
                    node = node[0]
                    if (not visited[node]):
                        stack.append(node)
                        
        visited_nodes = {node:False for node in self.nodes}

        for i in self.graph.keys():
            if (not visited_nodes[i]):
                list_components.append([])
                DFS(i, visited_nodes)
            
        return list_components
    

    def connected_components_set(self):
        """
        The result should be a set of frozensets (one per component), 
        For instance, for network01.in: {frozenset({1, 2, 3}), frozenset({4, 5, 6, 7})}
        """
        return set(map(frozenset, self.connected_components()))
    

    def min_power(self, src, dest):
        """
        Should return path, min_power. 
        """
        liste_power = []

        for i in self.graph.values():
            for k in i:
                liste_power.append(k[1])

        valeur_sup = max(liste_power)
        valeur_inf = min(liste_power)
        pivot = int(np.percentile(liste_power, 50)) # regarder si ça marche mieux avec 70 comme pivot, 60 comme pivot, etc.

        while valeur_sup != valeur_inf:
            if self.get_path_with_power(src, dest, pivot) == None:
                valeur_inf = pivot + 1
                pivot = (valeur_inf + valeur_sup) // 2
            elif self.get_path_with_power(src, dest, pivot) != None:
                valeur_sup = pivot
                pivot = (valeur_inf + valeur_sup) // 2
        
        return self.get_path_with_power(src, dest, pivot), pivot 


    def arbre_couvrant_min(self):

        def makeset(s):
            parent[s] = s
            rang[s] = 0

        def sort_by_weight(list_edges):
            return list_edges.sort(key = lambda x: x[2])

        def find(s):
            while s != parent[s] :
                s = parent[s]
            return s

        def union(x,y):
            r_x = find(x)
            r_y = find(y)

            # Si x et y sont déjà dans le même set
            if r_x == r_y:
                return None
            if rang[r_x] > rang[r_y]:
                parent[r_y] = r_x
            else:
                parent[r_x] = r_y
                if rang[r_x] == rang[r_y]: 
                    rang[r_y] = rang[r_y] + 1
    

        def kruskal(G):
        # Input: A connected undirected graph G = (V, E)
        # Output: A minimum spanning tree defined by the edges X
            for sommet in G.keys():
            # for sommet in self.nodes:
                makeset(sommet)
        
            # X est un graphe vide
            X = Graph()
    
            # Créer une liste avec les noeuds et leur poids
            liste_aretes = []
            for sommet, arrivee in G.items():
                for element in arrivee:
                    # On évite de mettre deux fois la même arête
                    if sommet < element[0]:
                        liste_aretes.append([sommet, element[0], element[1], element[2]])
    
            # Sort the edges E by weight    
            sort_by_weight(liste_aretes)
            
            for arete in liste_aretes:
                if find(arete[0]) != find(arete[1]):
                    X.add_edge(arete[0],arete[1],arete[2],arete[3])
                    union(arete[0], arete[1])
    
            return X
            
        # Initialisation
        parent = {}
        rang = {}

        arbre_brut = kruskal(self.graph)

        # On trie les sommets du graphe
        arbre_brut.graph = {i: arbre_brut.graph[i] for i in range(1,arbre_brut.nb_nodes+1)}

        return(arbre_brut)


    def min_power_acm(self, src, dest):
        return self.arbre_couvrant_min().min_power(src, dest)


def graph_from_file(filename):
    """
    Reads a text file and returns the graph as an object of the Graph class.

    The file should have the following format: 
        The first line of the file is 'n m'
        The next m lines have 'node1 node2 power_min dist' or 'node1 node2 power_min' (if dist is missing, it will be set to 1 by default)
        The nodes (node1, node2) should be named 1..n
        All values are integers.

    Parameters: 
    -----------
    filename: str
        The name of the file

    Outputs: 
    -----------
    g: Graph
        An object of the class Graph with the graph from file_name.
    """
    with open(filename, "r") as file:
        n, m = map(int, file.readline().split()) # pour network
        # m = int(file.readline()) # pour routes
        g = Graph([i for i in range(1, n+1)]) # pour network
        # g = Graph() # pour routes
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
