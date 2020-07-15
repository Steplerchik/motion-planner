import unittest

from motion_planner import *


class TestNF(unittest.TestCase):
    def test_navigation_function(self):
        end_position = np.array([5.0, 8.0, 0])
        nf = NavigationFunctionNF1(end_position, resolution=0.5)
        plot_cost_map(nf)
        cost = nf.get_cost(np.array([7.46, 8, 0]))
        print("Cost:", cost)
        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
