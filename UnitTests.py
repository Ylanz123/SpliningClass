import unittest
import math
from Path_Splining import Path_Splining


class UnitTests(unittest.TestCase):
    def test_theta_constraint(self):
        test_coordinates_pairs = [[[0, 0], [1, 1]],
                                  [[1, 1], [0, 0]],
                                  [[2, 1], [-5, 6]]]
        Spliner = Path_Splining()
        for coord_pair in test_coordinates_pairs:
            theta = Spliner.find_dual_perpendicular_angle(1, coord_pair[0], coord_pair[1])

            self.assertTrue(-math.pi <= theta <= math.pi), "Not between -2pi and 2pi" # add assertion here



if __name__ == '__main__':
    unittest.main()
