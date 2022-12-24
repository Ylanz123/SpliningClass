import unittest
import math
from SplineGenerator import SplineGenerator


class UnitTests(unittest.TestCase):
    def test_theta_constraint(self):
        test_coordinates_pairs = [[[0, 0], [1, 1]],
                                  [[1, 1], [0, 0]],
                                  [[2, 1], [-5, 6]]]
        Spliner = SplineGenerator()

    def test_spline_perpendicularity(self):
        test_waypoint_lists = [
                               [[4, 5], [7, 6], [6, 9], [4, 7], [2, 6], [0, 0]],
                               [[0, 0], [1, 5]],
                               [[5, 5.5], [10, 16], [20000, -2500]],
                               [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]],  # Straight line of waypoints doesnt work
                               [[40, 40], [40, 70], [70, 70], [70, 40]],
                               [[1.0, 1.0], [2.0, 2.0], [3.0, 3.0], [4.0, 4.0], [5.0, 5.0], [8, 5], [9, 3], [6, -4]],
                               [[-7, 0], [-5, 0], [-3, 0], [1, 2], [3, 0], [5, 2], [7, 0], [9, 2], [11, 0]],
                               [[5, 10], [9, 19], [12, 14], [8, 6], [3, -4], [-1, 0]]
                              ]
        for waypoint_list in test_waypoint_lists:
            Spliner = SplineGenerator(waypoints=waypoint_list)
            output, centres = Spliner.generate_spline(print_data=False)
            Spliner.plot_waypoints(output)
            self.assertTrue(validate_perpendicularity(output, len(waypoint_list)))



if __name__ == '__main__':
    unittest.main()
