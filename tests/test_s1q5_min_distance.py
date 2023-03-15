# Ces tests vont vérifier que la fonction dijkstra fonctionne.
import sys 
sys.path.append("delivery_network/")

import unittest 
from graph import Graph, graph_from_file

class Test_GraphLoading(unittest.TestCase):
    def test_network0(self):
        g = graph_from_file("input/network.00.in")
        self.assertEqual(g.shortest_path_with_power(1, 4, 10), None)
        self.assertEqual(g.shortest_path_with_power(1, 4, 11), ([1, 2, 3, 4], 3))

    def test_network1(self):
        g = graph_from_file("input/network.04.in")
        self.assertEqual(g.shortest_path_with_power(1, 4, 10), ([1, 2, 3, 4], 94))
        self.assertEqual(g.shortest_path_with_power(1, 4, 11), ([1, 4], 6))

if __name__ == '__main__':
    unittest.main()
