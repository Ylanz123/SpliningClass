from __future__ import division
import matplotlib.pyplot as plt
import math

class Path_Splining():
    """
    Path splining class
        This class contains all the methods responsible for creating and modifying a custom
        spline between waypoints.
    """
    def __init__(self, turn_radius=3.7, boundary_points=[], waypoints=[], resolution=3, tolerance=0):
        """
        Initialiser method
            This method can be used to initialise the class with or without the parameters
            listed below.
        :param turn_radius: The minimum radius for any turn created by the spline algorithm
        in metres.
        :param boundary_points: An array of [latitude, longitude] arrays that define a
        boundary the spline path won't cross.
        :param waypoints: An array of [latitude, longitude] arrays that the spline path
        will intersect.
        :returns: Doesn't return anything.
        """
        self._turn_radius = turn_radius
        self._boundary_points = boundary_points
        self._waypoints = waypoints
        self._resolution = resolution

    def add_waypoints(self, waypoints=[]):
        """
        Add waypoint method
            This method allows the adding of waypoints to the current waypoint list. They
            will be appended after all the previous waypoints.
        :param waypoints: An array of [latitude, longitude] arrays in the order the vehicle
        has to travel. As an example index 0 will be the first waypoint to go to and index
        1 the second waypoint and so on.
        :return:
        """
        # Loop through each waypoint and append it to self._waypoints
        for waypoint in waypoints:
            self._waypoints.append(waypoint)

    def remove_waypoints(self, indices=[]):
        """
        Remove waypoints method
            This method allows users to remove waypoints by single index or multiple
            indices at a time.
        :param indices: An array of indices to remove from the waypoint list.
        [firstIndex, secondIndex, ...]
        :return:
        """
        # Sort the indices array in descending order
        indices.sort(reverse=True)
        # Pop each waypoint at each index in descending order
        for index in indices:
            self._waypoints.pop(index)

    def add_boundary(self, boundary_points=[]):
        self._boundary_points = boundary_points

    def edit_turn_radius(self, turn_radius):
        self._turn_radius = turn_radius

    def print_waypoints(self):
        print("Current Waypoints:")
        print(self._waypoints)

    def plot_waypoints(self):
        lat_vals = []
        lon_vals = []
        # Loop through to get each lat and lon value
        for waypoint in self._waypoints:
            lat_vals.append(waypoint[0])
            lon_vals.append(waypoint[1])

        plt.plot(lat_vals, lon_vals)
        plt.show()

    def vertex_angle(self, P1, P2, P3):
        P12 = math.sqrt((P1[0] - P2[0]) ** 2 + (P1[1] - P2[1]) ** 2)
        P13 = math.sqrt((P1[0] - P3[0]) ** 2 + (P1[1] - P3[1]) ** 2)
        P23 = math.sqrt((P2[0] - P3[0]) ** 2 + (P2[1] - P3[1]) ** 2)
        numerator = P12 ** 2 + P13 ** 2 - P23 ** 2
        denominator = 2 * P12 * P13
        vertex_angle = math.acos(numerator / denominator) * 180 / math.pi
        return vertex_angle

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    def find_dual_perpendicular_angle(self, radius, origin, point, n=0):
        r = radius
        xo = origin[0]
        yo = origin[1]
        xn = point[0]
        yn = point[1]

        arctan_numerator = yn - yo
        arctan_denominator = xn - xo

        arcsin_numerator = r
        arcsin_denominator = math.sqrt(xn ** 2 - 2 * xn * xo + xo ** 2 + (yn - yo) ** 2)
        arcsin_stuff = arcsin_numerator / arcsin_denominator

        numerator = - (2 * math.asin(arcsin_stuff) - 2 * math.atan2(arctan_numerator, arctan_denominator) + (self.sign(xn - xo) - 4 * n) * math.pi)
        denominator = 2
        theta = numerator / denominator

        return theta

    def mirror_across_line(self, line_point1, line_point2, point):
        xm = point[0]
        ym = point[1]
        xo = line_point1[0]
        yo = line_point1[1]
        xn = line_point2[0]
        yn = line_point2[1]

        numerator = xm * (xn - xo) ** 2 + (xn * (ym - yo) - xo * (ym - yn)) * (yn - yo)
        denominator = xn ** 2 - 2 * xn * xo + xo ** 2 + (yn - yo) ** 2
        x_intersection = numerator / denominator

        m = (yn - yo) / (xn - xo)
        y_intersection = m * (x_intersection - xo) + yo

        print(x_intersection)
        print(y_intersection)

        x_diff = xm - x_intersection
        y_diff = ym - y_intersection

        mirrored_x = xm - 2 * x_diff
        mirrored_y = ym - 2 * y_diff

        return [mirrored_x, mirrored_y]

    def constrain_pi(self, theta):
        if theta > math.pi:
            theta = theta - 2 * math.pi
        elif theta:
            theta = theta + 2 * math.pi
        return theta

    def generate_spline(self):
        """

        :return:
        """
        r = self._turn_radius
        P1 = [0, 0]
        P2 = [0, 3]
        P3 = [4, 0]
        # Get P2 Vertex Angle
        vertex_angle = self.vertex_angle(P1, P2, P3)
        print(vertex_angle)
        # The vertex angle is how much angle of the circle the waypoint COULD lie on.
        # Maybe try to for loop over x degree intervals

        origin_point = [4, 4]
        destination_point = [3.8, 0]
        theta = self.find_dual_perpendicular_angle(radius=r, origin=origin_point, point=destination_point, n=0)

        # Constrain theta between -pi and pi
        if theta > math.pi or theta < -math.pi:
            theta = self.constrain_pi(theta)

        print(theta)

        point_to_mirror = [origin_point[0] + r * math.cos(theta), origin_point[1] + r * math.sin(theta)]
        print(point_to_mirror)
        mirror_point = self.mirror_across_line(origin_point, destination_point, point_to_mirror)
        print(mirror_point)

        true_angle = math.atan2(mirror_point[1] - origin_point[1], mirror_point[0] - origin_point[0])
        print(true_angle)


if "__main__" == __name__:
    waypoints = [[5, 5], [7, 8], [10, 3], [4, 4]]
    Spliner = Path_Splining()
    Spliner.add_waypoints(waypoints)
    Spliner.print_waypoints()
    Spliner.generate_spline()