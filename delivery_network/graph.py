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
    

    def get_path_with_power_old(self, src, dest, power):
        
        
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

        for ssgraphe in nv_graphe.connected_components():
            if src in ssgraphe and dest not in ssgraphe:
                return None
            elif dest in ssgraphe and src not in ssgraphe:
                return None

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

                # ne pas faire une copie du graphe : seulement rajouter une condition
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


    def get_path_with_power(self, src, dest, power):
        '''
        INPUT:
        src: int (sommet de départ)
        dest: int (sommet d'arrivée)
        power: float (puissance du camion)
        OUTPUT:
        chemin: list (liste des sommets du chemin)'''


        # mtn on regarde s'il y a un chemin entre les deux sommets dans nv_graphe
        # pour ça, on regarde les sommets connexes de nv_graphe : si les deux sommets sont connexes, alors
        # il y a un chemin entre eux
        # pour trouver un chemin quelconque, on fait un deep search à partir de l'un des deux et on remonte à partir
        # second pour avoir un chemin

        '''
        for ssgraphe in self.connected_components():
            if src in ssgraphe and dest not in ssgraphe:
                return None
            elif dest in ssgraphe and src not in ssgraphe:
                return None
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

                # ne pas faire une copie du graphe : seulement rajouter une condition
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
        OUTPUT : 
            - list_components : liste des composantes connexes du graphe
        '''

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
    

    def min_power_old(self, src, dest, pourcentile=15):
        """
        Should return path, min_power. 
        """
        liste_power = []

        for i in self.graph.values():
            for k in i:
                liste_power.append(k[1])

        liste_power.sort()

        # Enlever les doublons

        l_power = [liste_power[0]]
        for element in liste_power[1:]:
            if element != l_power[-1]:
                l_power.append(element) 

        valeur_sup = l_power[-1]
        valeur_inf = l_power[0]
        pivot = liste_power[int((pourcentile/100)*len(liste_power))] # regarder si ça marche mieux avec 70 comme pivot, 60 comme pivot, etc.
        # la fonction percentile ne marche pas (bcp trop long), donc on va calculer le pct 15 manuellement
        ind_pivot = l_power.index(pivot)

        # Se déplacer sur l_power et non sur l'ensemble des entiers naturels!

        # liste_power[((len(liste_power)+1)//2)-1])

        while valeur_inf != valeur_sup:
            print(len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)]))
            if self.get_path_with_power(src, dest, pivot) == None: # veut dire que la puissance pivot est trop petite
                valeur_inf = l_power[ind_pivot + 1] # pivot + 1
                pivot = l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)][((len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                ind_pivot = l_power.index(pivot)           
            elif self.get_path_with_power(src, dest, pivot) != None:
                valeur_sup = pivot
                pivot = l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)][((len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                # pivot = (l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)],50)
                ind_pivot = l_power.index(pivot)
        return self.get_path_with_power(src, dest, pivot), pivot 


    def is_path_with_power(self, src, dest, power):
        # retourne True ou False
        # on regarde seulement si le trajet peut être couvert en regardant si les deux sommets sont dans la même composante connexe
        
        '''
        for cc in self.connected_components_set():
            if src in cc and dest in cc:
                return True
            elif (src not in cc and dest in cc) or (src in cc and dest not in cc):
                return False
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

                # ne pas faire une copie du graphe : seulement rajouter une condition
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
            return False
        
        return True


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


    def min_power(self, src, dest, pourcentile=15):
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

    
    def get_kruskal(self):
        self.kruskal = kruskal(self)


    def min_power_acm(self, src, dest, pourcentile=15):
        # regarder si on a déjà enregistré kruskal
        try:
            type(self.kruskal)
        except:
            self.get_kruskal()

        return self.kruskal.min_power(src, dest, pourcentile)
    

    def connected_components_with_power(self, power):

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
                    if (node[1] <= power) and (not visited[node[0]]):
                        stack.append(node[0])
                        
        visited_nodes = {node:False for node in self.nodes}

        for i in self.graph.keys():
            if (not visited_nodes[i]):
                list_components.append([])
                DFS(i, visited_nodes)
            
        return list_components


    def connected_components_set_with_power(self, power):
        """
        The result should be a set of frozensets (one per component), 
        For instance, for network01.in: {frozenset({1, 2, 3}), frozenset({4, 5, 6, 7})}
        """
        return set(map(frozenset, self.connected_components_with_power(power)))
    
    
    def get_dict_frozenset_connected_components_with_power(self):
        
        try:
            dict_power = {power: None for power in self.liste_power}
        except:
            self.get_liste_power()
            dict_power = {power: None for power in self.liste_power}
        
        for power in self.liste_power:
            print(power)
            dict_power[power] = self.connected_components_set_with_power(power)
        
        self.dict_frozenset_connected_components_with_power = dict_power


    def is_path_with_power_frozenset(self, src, dest, power):
        # on suppose que l'on a déjà enregistré dict_frozenset_connected_components_with_power
        for component in self.dict_frozenset_connected_components_with_power[power]:
            if (src in component) and (dest in component):
                return True
            elif ((src in component) and (dest not in component)) or ((src not in component) and (dest in component)):
                return False


    def min_power_with_frozenset(self, src, dest, pourcentile=15):
        """
        Should return path, min_power. 
        """
        # regarder si on a déjà enregistré dict_frozenset_connected_components_with_power
        try:
            type(self.dict_frozenset_connected_components_with_power)
        except:
            self.get_dict_frozenset_connected_components_with_power()
        
        # on a enregistré dict_frozenset_connected_components_with_power, donc on self.liste_power

        valeur_sup = self.liste_power[-1]
        valeur_inf = self.liste_power[0]
        pivot = self.liste_power[int((pourcentile/100)*len(self.liste_power))] # regarder si ça marche mieux avec 70 comme pivot, 60 comme pivot, etc.
        # la fonction percentile ne marche pas (bcp trop long), donc on va calculer le pct 15 manuellement
        ind_pivot = self.liste_power.index(pivot)

        # Se déplacer sur l_power et non sur l'ensemble des entiers naturels!

        # liste_power[((len(liste_power)+1)//2)-1])

        while valeur_inf != valeur_sup:
            # print(len(l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)]))
            if self.is_path_with_power_frozenset(src, dest, pivot) == False: # veut dire que la puissance pivot est trop petite
                valeur_inf = self.liste_power[ind_pivot + 1] # pivot + 1
                pivot = self.liste_power[self.liste_power.index(valeur_inf) : (self.liste_power.index(valeur_sup)+1)][((len(self.liste_power[self.liste_power.index(valeur_inf) : (self.liste_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                ind_pivot = self.liste_power.index(pivot)           
            elif self.is_path_with_power_frozenset(src, dest, pivot) == True:
                valeur_sup = pivot
                pivot = self.liste_power[self.liste_power.index(valeur_inf) : (self.liste_power.index(valeur_sup)+1)][((len(self.liste_power[self.liste_power.index(valeur_inf) : (self.liste_power.index(valeur_sup)+1)])+1)//2)-1] # médiane codée manuellement, au lieu de (valeur_inf + valeur_sup) // 2
                # pivot = (l_power[l_power.index(valeur_inf) : (l_power.index(valeur_sup)+1)],50)
                ind_pivot = self.liste_power.index(pivot)
        return self.get_path_with_power(src, dest, pivot), pivot         


    def min_power_kruskal(self, src, dest):
        # regarder si kruskal est déjà enregistré
        try:
            type(self.kruskal)
        except:
            self.get_kruskal()

        def DFS_min_power_kruskal(G,root,arrivee,visited,parent):
            # s'arrête dès qu'on a atteint le sommet d'arrivée

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

                # ne pas faire une copie du graphe : seulement rajouter une condition
                for node in G.graph[s]:
                    if (not visited[node[0]]):
                        stack.append(node[0])
                        parent[node[0]] = s
                        if node[0] == arrivee:
                        # Dans ce cas, on peut arrêter le pgrm car on a tout ce qu'il faut pour avoir notre chemin
                            return None

        parent = {i: None for i in self.nodes} # il faut créer un dictionnaire avec le sommet précédent à chaque fois              
        visited = {i: False for i in self.nodes}

        DFS_min_power_kruskal(self.kruskal, src, dest, visited, parent)

        # Récupérer le chemin à partir du dict parent

        '''
        # Si le sommet n'a pas été atteint par le parcours en profondeur
        if parent[dest] == None:
            return None
        '''

        chemin = []

        chemin.append(dest)

        # Si le sommet a été atteint, on reconstitue le chemin et on enregistre la puissance minimal requise
        pwr = 0
        while dest != src:
            chemin.append(parent[dest])
            # regarder si l'arête qu'on parcourt actuellement a une puissance supérieure à la puissance requise jusqu'ici
            for adjacent in self.graph[dest]:
                if adjacent[0] == parent[dest]:
                    # print(dest, adjacent[0])
                    # print(adjacent[1])
                    if adjacent[1] > pwr:
                        # print('Changement de puissance min :' + str(adjacent[1]))
                        pwr = adjacent[1]
                    break
            dest = parent[dest]

        chemin.reverse()

        return chemin, pwr
    

    def DFS_parents(self, root=1):
        '''On fait un seul parcours en profondeur pour récupérer tous les liens de parenté à partir du sommet root'''
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
    

    def min_power_routes(self, liste_routes, root=1):
        '''Pour obtenir une liste des min_power pour toutes le routes demandées'''
        # liste_routes est une liste de 3 éléments : le 1er est le départ, le 2ème est l'arrivée, le 3ème est l'utilité

        # self.get_kruskal()
        parent = self.kruskal.DFS_parents(root)

        # On initialise l'output
        liste_trajets = []
        liste_power = []

        # ct = 0
        for route in liste_routes:
            # ct += 1
            # if ct%10000 == 0:
                # print(ct)
                # return liste_trajets, liste_power
                # return liste_power
            
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
                for adjacent in self.kruskal.graph[sommet]:
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


def kruskal(G): # G est un objet de la class Graph

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

    def kruskal_int(graphe):
    # Input: A connected undirected graph G = (V, E)
    # Il faut faire kruskal sur chaque composante connexe
    # Output: A minimum spanning tree defined by the edges X
        for sommet in graphe.keys():
        # for sommet in self.nodes:
            makeset(sommet)
        
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
            if find(arete[0]) != find(arete[1]):
                X.add_edge(arete[0],arete[1],arete[2],arete[3])
                union(arete[0], arete[1])
    
        return X

    # Initialisation
    parent = {}
    rang = {}

    arbre_brut = kruskal_int(G.graph)

    # On trie les sommets du graphe pour passer les tests unitaires
    # arbre_brut.graph = {i: arbre_brut.graph[i] for i in range(1,arbre_brut.nb_nodes+1)}

    return arbre_brut