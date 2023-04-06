# This will work if ran from the root folder.
import sys 
sys.path.append("delivery_network")

from graph import trucks_from_file
import unittest   # The test framework

class Test(unittest.TestCase):
    def test_trucks0(self):
        t = trucks_from_file("input/trucks.0.in")
        t_expected = {1: (2000000, 200000), 2: (6000000, 900000)}
        self.assertEqual(t, t_expected)

    def test_trucks1(self):
        t = trucks_from_file("input/trucks.1.in")
        t_expected = {1: (500000, 50000), 2: (1000000, 100000), 3: (1500000, 170000), 4: (2000000, 230000), 5: (2500000, 370000), 6: (3000000, 360000), 7: (3500000, 430000), 8: (4000000, 560000), 9: (4500000, 570000), 10: (5000000, 580000), 11: (5500000, 740000), 12: (6000000, 850000), 13: (6500000, 910000), 14: (7000000, 980000), 15: (7500000, 1200000), 16: (8000000, 1300000), 17: (8500000, 1350000), 18: (9000000, 1450000), 19: (9500000, 1500000), 20: (10000000, 2000000)}
        print(t, t_expected)
        self.assertEqual(t, t_expected)

if __name__ == '__main__':
    unittest.main()




