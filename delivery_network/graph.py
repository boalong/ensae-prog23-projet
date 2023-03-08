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
        nv_graphe = Graph([i for i in range(1, self.nb_nodes+1)])
        for sommet, aretes in self.graph.items():
            for arete in aretes:
                if arete[1] <= power:
                    nv_graphe.add_edge(sommet, arete[0], arete[1], arete[2])
            
        # mtn on regarde s'il y a un chemin entre les deux sommets dans nv_graphe
        # pour ça, on regarde les sommets connexes de nv_graphe : si les deux sommets sont connexes, alors
        # il y a un chemin entre eux
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
    

    def connected_components(self):
        '''
        list_components=[]
        node_visited={node:False for node in self.nodes}

        def dfs(node):
            component = [node]
            for neighbour in self.graph[node]:
                neighbour=neighbour[0]
                if not node_visited[neighbour]:
                    node_visited[neighbour]=True
                    component += dfs(neighbour)
            return component
        
        for node in self.nodes:
            if not node_visited[node]:
                list_components.append(dfs(node))

        return list_components
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
        pivot = int(np.percentile(liste_power, 50))

        while valeur_sup != valeur_inf:
            if self.get_path_with_power(src, dest, pivot) == None:
                valeur_inf = pivot + 1
                pivot = (valeur_inf + valeur_sup) // 2
            elif self.get_path_with_power(src, dest, pivot) != None:
                valeur_sup = pivot
                pivot = (valeur_inf + valeur_sup) // 2
        
        return self.get_path_with_power(src, dest, pivot), pivot        


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
        g = Graph(range(1, n+1)) # pour network
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
