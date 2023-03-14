# This will work if ran from the root folder.
import sys 
sys.path.append("delivery_network/")

import unittest 
from graph import Graph, graph_from_file

class Test_GraphLoading(unittest.TestCase):
    def test_network0(self):
        g = graph_from_file("input/network.1.in")
        self.assertEqual(g.nb_nodes, 20)
        self.assertEqual(g.nb_edges, 100)
        self.assertEqual(g.graph[1][0][1], 2)
        self.assertEqual(g.graph[1][0][2], 6312)

    def test_network1(self):
        g = graph_from_file("input/network.2.in")
        self.assertEqual(g.nb_nodes, 100000)
        self.assertEqual(g.nb_edges, 100000)
        self.assertEqual(g.graph[1][0][1], 24683)
        self.assertEqual(g.graph[1][0][2], 8644) 

    def test_network4(self):
        g = graph_from_file("input/network.10.in")
        self.assertEqual(g.nb_nodes, 200000)
        self.assertEqual(g.nb_edges, 300000)
        self.assertEqual(g.graph[1][0][1], 6473280)
        self.assertEqual(g.graph[1][0][2], 0.211549803713317)

if __name__ == '__main__':
    unittest.main()
