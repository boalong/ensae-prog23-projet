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
        This function returns a path between two nodes if the minimal power is below the power given in input
        INPUT:
            - src: the source node
            - dest: the destination node
            - power: the maximum power allowed
        OUTPUT:
            - path: the path between the two nodes if it exists
            - None: if the path does not exist
        '''
        visited_node={node:False  for node in self.nodes} #Initialisation of a dictionary in order to follow  which nodes are already visited or not
        def path_research(node,path): #Definition of a recursive function
            if node == dest:
                return path
            for neighbour in self.graph[node]: # We visit all nodes connected to the initial node
                neighbour,minp, dist = neighbour
                if not visited_node[neighbour] and minp<=power: # If the connected node is unvisited and the minimal power of the edge is below power_min, we visit it and continue
                    visited_node[neighbour] = True #We mark the connected node as visited
                    result = path_research(neighbour, path+[neighbour]) #We add the connected node to the path and reuse path_research 
                    if result is None:
                        return result
            return None
        return path_research(src,[src]) 
                                  
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
        self.get_kruskal()
        parent = self.kruskal.DFS_parents(root)
        # On initialise l'output
        liste_trajets = []
        liste_power = []
        ct = 0
        for route in liste_routes:
            ct += 1
            # print(ct)
            if ct == 1001:
                return liste_trajets, liste_power
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

            liste_trajets.append(chemin)
            liste_power.append(pwr)

        return liste_trajets, liste_power
    
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

