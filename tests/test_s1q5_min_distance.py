# Ces tests vont vérifier que la fonction dijkstra fonctionne.
import sys 
sys.path.append("delivery_network/")

import unittest 
from graph import Graph, graph_from_file

class Test_GraphLoading(unittest.TestCase):
    def test_network0(self):
        g = graph_from_file("input/network.1.in")


    def test_network1(self):
        g = graph_from_file("input/network.2.in")


    def test_network4(self):
        g = graph_from_file("input/network.10.in")


if __name__ == '__main__':
    unittest.main()
