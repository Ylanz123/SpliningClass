import unittest
import math
from Path_Splining import Path_Splining


class UnitTests(unittest.TestCase):
    def test_theta_constraint(self):
        test_coordinates_pairs = [[[0, 0], [1, 1]],
                                  [[1, 1], [0, 0]],
                                  [[2, 1], [-5, 6]]]
        Spliner = Path_Splining()

    def test_spline_perpendicularity(self):
        test_waypoint_lists = [
                               [[4, 5], [7, 6], [6, 9], [4, 7], [2, 6], [0, 0]],
                               [[0, 0], [1, 5]],
                               [[5, 5.5], [10, 16], [20000, -2500]],
                               [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]  # Straight line of waypoints doesnt work
                              ]
        for waypoint_list in test_waypoint_lists:
            Spliner = Path_Splining(waypoints=waypoint_list)
            output, centres = Spliner.improved_spline(print_data=False)
            Spliner.plot_waypoints(output)
            self.assertTrue(Spliner.validate_perpendicularity(output, len(waypoint_list)))



if __name__ == '__main__':
    unittest.main()
