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
        # mtn on regarde s'il y a un chemin entre les deux sommets dans nv_graphe
        # pour ça, on regarde les sommets connexes de nv_graphe : si les deux sommets sont connexes, alors
        # il y a un chemin entre eux
        # pour trouver un chemin quelconque, on fait un deep search à partir de l'un des deux et on remonte à partir
        # second pour avoir un chemin
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
        return set(map(frozenset, self.connected_components()))
    
    def min_power(self, src, dest):
        #on fait une recherche dichotomique sur la puissance
        puissances=[]#initialisation de la liste des puissances
        for node in self.nodes:#Création de la liste des puissances en parcourant toutes les arrêtes du graphe
            for e in self.graph[node] :
                puissances.append(e[1])
        set(puissances)
        #print(puissances)
        puissances_unique = list(set(puissances))#on enlève les doublons
        #recherche dichotomique du chemin le plus économique en puissance
        puissances_unique.sort()#tri de la liste
        a=0
        b=len(puissances_unique)-1
        m=(a+b)//2
        while a<=b :
            p_tmp=puissances_unique[m]
            p_ant=puissances_unique[m-1]
            if self.get_path_with_power(src, dest, p_tmp)==None :
                a=m+1
            elif self.get_path_with_power(src, dest, p_ant)==None :
                break
            else:
                b=m-1
            m=(a+b)//2
        chemin_eco=self.get_path_with_power(src, dest, p_tmp)
        p_min=p_tmp
        return (chemin_eco, p_min)


def graph_from_file(filename):
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
    n=len(G.nodes)
    edges_list=[]
    for x in G.graph.keys():
        for y in G.graph[x]:
            edges_list.append((x,y[0],y[1]))
    edges = list(set(edges_list))
    A = sorted(edges, key=lambda x:[2])
    CC = [i for i in range(n)]
    T = []
    compteurT = 0
    for i in range(len(A)):
        x, y, poids = A[i]
        if CC[x-1] != CC[y-1]:
            T.append((x, y, poids))
            compteurT += 1
            auxiliaire = CC[y-1]
            for j in range(n):
                if CC[j] == auxiliaire:
                    CC[j] = CC[x-1]
        if compteurT == n - 1:
            break
    Tree = Graph(G.nodes)
    for x, y, weight in T:
        Tree.add_edge(x,y,weight)
    for key in Tree.graph.keys():
        Tree.graph[key].sort(key=lambda x: x[1])
    return Tree     

def jean_neymar(G,src,dest):
    minimum_spanning_tree=kruskal(G)
    return minimum_spanning_tree.min_power(src,dest)