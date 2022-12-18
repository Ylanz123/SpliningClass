from __future__ import division, print_function
import matplotlib.pyplot as plt
import math

class Path_Splining():
    """
    Path splining class
        This class contains all the methods responsible for creating and modifying a custom
        spline between waypoints.
    """
    def __init__(self, turn_radius=0.8, boundary_points=[], waypoints=[], resolution=3, tolerance=0):
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

    def plot_waypoints(self, waypoints_to_plot):
        lat_vals = []
        lon_vals = []
        # Loop through to get each lat and lon value
        for waypoint in waypoints_to_plot:
            lat_vals.append(waypoint[0])
            lon_vals.append(waypoint[1])

        plt.plot(lat_vals, lon_vals, '--bo')
        plt.xlim([0, 10])
        plt.ylim([0, 10])
        plt.axis('equal')
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
        """

        :param radius:
        :param origin:
        :param point:
        :param n:
        :return:
        """
        r = radius
        origin_x = origin[0]
        origin_y = origin[1]
        next_point_x = point[0]
        next_point_y = point[1]

        arctan_numerator = next_point_y - origin_y
        arctan_denominator = next_point_x - origin_x

        arcsin_numerator = r
        arcsin_denominator = math.sqrt(next_point_x ** 2 - 2 * next_point_x * origin_x + origin_x ** 2 + (next_point_y - origin_y) ** 2)
        arcsin_stuff = arcsin_numerator / arcsin_denominator

        # Reference this desmos page: https://www.desmos.com/calculator/loniceqosa
        # This equation comes from the solve function of my calendar. I solved for when M_NI was equal to M_IN and made
        # t (theta) the subject. All this numerator denominator stuff is just to make the code more readable.
        print("Next Point:", next_point_x, next_point_y, "\nOrigin:", origin_x, origin_y)
        numerator = - (2 * math.asin(arcsin_stuff) - 2 * math.atan2(arctan_numerator, arctan_denominator) + (self.sign(next_point_x - origin_x) - 4 * n) * math.pi)
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

        x_diff = xm - x_intersection
        y_diff = ym - y_intersection

        mirrored_x = xm - 2 * x_diff
        mirrored_y = ym - 2 * y_diff

        return [mirrored_x, mirrored_y]

    def constrain_pi(self, theta):
        if theta > math.pi:
            theta = theta - 2 * math.pi
        elif theta < -math.pi:
            theta = theta + 2 * math.pi
        return theta

    def calculate_curve_exit(self, previous_waypoint, current_waypoint, next_waypoint, print_data=False):
        # Define r as the minimum turn radius to make lines a bit neater.
        r = self._turn_radius
        # Part 1: Find the centre-point of the circle the path will trace.
        # Get perpendicular gradient of current waypoint to previous waypoint.
        inverse_gradient_numerator = current_waypoint[0] - previous_waypoint[0]
        inverse_gradient_denominator = current_waypoint[1] - previous_waypoint[1]
        # Angle below in reference to unit circle.
        gradient_angle = math.atan2(inverse_gradient_numerator, inverse_gradient_denominator)
        # Get 2 possible points along that gradient from current waypoint that are of distance r.
        first_point = [current_waypoint[0] - r * math.cos(gradient_angle),
                       current_waypoint[1] + r * math.sin(gradient_angle)]
        second_point = [current_waypoint[0] - r * math.cos(gradient_angle + math.pi),
                        current_waypoint[1] + r * math.sin(gradient_angle + math.pi)]
        # Pick the point that is closer to the next waypoint.
        first_point_dist = math.sqrt((first_point[0] - next_waypoint[0]) ** 2 + (first_point[1] - next_waypoint[1]) ** 2)
        second_point_dist = math.sqrt((second_point[0] - next_waypoint[0]) ** 2 + (second_point[1] - next_waypoint[1]) ** 2)
        if first_point_dist <= second_point_dist:
            centre_point = first_point
        else:
            centre_point = second_point
        # This point is going to be the centre of the circle the plane will trace as it angles towards the
        # next waypoint.
        if print_data:
            print("\tCentre Point: ", centre_point)
        # Use the find_dual_perpendicular_angle function to calculate the angle of the point at which the plane
        # stops tracing the circle and travels to the next waypoint
        exit_angle = self.find_dual_perpendicular_angle(radius=r, origin=centre_point, point=next_waypoint, n=0)
        # Constrain theta between -pi and pi. I have a feeling this will never run if the code is written correctly.
        if exit_angle > math.pi or exit_angle < -math.pi:
            exit_angle = self.constrain_pi(exit_angle)

        # If next waypoint x is less than centre point x, add pi to angle. This is because if the angle is in the
        # or third quadrant we have to add pi to angle when calculating.
        if next_waypoint[0] < centre_point[0]:
            exit_point = [centre_point[0] - r * math.cos(exit_angle + math.pi), centre_point[1] - r * math.sin(exit_angle + math.pi)]
        else:
            exit_point = [centre_point[0] - r * math.cos(exit_angle), centre_point[1] - r * math.sin(exit_angle)]
        # There exists another point that is its mirror across the line from the circle centre to the next waypoint.
        # So we calculate that here and will have to test which one the plane will encounter first as that will
        # be the one we choose.
        exit_mirror_point = self.mirror_across_line(centre_point, next_waypoint, exit_point)
        exit_mirror_angle = math.atan2(exit_mirror_point[1] - centre_point[1], exit_mirror_point[0] - centre_point[0])
        # Store the angle from the circle centre to the current waypoint, we'll use this to find which point the
        # plane will fly over first and when we need to find which case it is. (What combination of points it is)
        current_point_angle = math.atan2(current_waypoint[1] - centre_point[1], current_waypoint[0] - centre_point[0])

        if print_data:
            print("\tExit Point:", exit_point, "\n\tExit Mirror Point:", exit_mirror_point)
            print("\tExit Angle:", exit_angle, "\n\tExit Mirror Angle:", exit_mirror_angle, "\n\tCurrent Angle:", current_point_angle)
        # Calculate the perpendicular angle from the current waypoint to the circle centre
        centre_to_current_grad_num = current_waypoint[1] - centre_point[1]
        centre_to_current_grad_den = current_waypoint[0] - centre_point[0]
        inv_centre_to_current_angle = math.atan2(-centre_to_current_grad_den, centre_to_current_grad_num)

        # Angle from the previous waypoint to the current waypoint
        previous_to_current_grad_num = current_waypoint[1] - previous_waypoint[1]
        previous_to_current_grad_den = current_waypoint[0] - previous_waypoint[0]
        previous_to_current_angle = math.atan2(previous_to_current_grad_num, previous_to_current_grad_den)

        # We can say if the inverse gradient from the centre to the current waypoint is the same as the
        # gradient from the previous waypoint to the current waypoint then we are heading in a clockwise direction
        # Think of the right hand rule from physics to help understand this.
        # Bit sketchy but it works for now. I suppose this is due to some rounding or maybe atan2
        # as the values are technically the same but there must be a deep decimal value that differs.
        clockwise = False
        if abs(inv_centre_to_current_angle - previous_to_current_angle) <= 0.0000001:
            clockwise = True

        if print_data:
            print("\tGrad Inv Angle:\t", inv_centre_to_current_angle, "\n\tComparison Angle:", previous_to_current_angle)
            print("\tClockwise:", clockwise)

        # This collection of if else statements is absolutely disgusting I know but it works.
        # In the future this will be condensed.
        if clockwise:
            if print_data:
                print("Clockwise")
            # All 5 cases where current point is positive angle
            if current_point_angle > 0:
                if print_data:
                    print("Positive angle")
                # CASE 1: Current positive, Both points negative
                if exit_angle < 0 and exit_mirror_angle < 0:
                    # CASE 1: Pick the largest value
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 2: Current positive, one point less positive, one point negative
                if (exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < 0) or (exit_mirror_angle < current_point_angle and exit_mirror_angle > 0 and exit_angle < 0):
                    # CASE 2: Pick the largest value
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 3: Current positive, both points less positive
                if exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0:
                    # CASE 3: Pick the largest value
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 4: Current positive, one point more positive, one point less positive
                if (exit_angle > current_point_angle and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_angle < current_point_angle and exit_angle > 0):
                    # CASE 4: Pick the smallest value
                    if exit_angle > exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
                # CASE 5: Current positive, both points more positive
                if exit_angle > current_point_angle and exit_mirror_angle > current_point_angle:
                    # CASE 5: Pick the largest value
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 6: Current positive, one point more positive, one point negative
                if (exit_angle > current_point_angle and exit_mirror_angle < 0) or (exit_mirror_angle > current_point_angle and exit_angle < 0):
                    # CASE 6: Pick the smallest value
                    if exit_angle > exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
            else:
                if print_data:
                    print("Negative angle")
                # CASE 7: Current negative, both points more negative
                if exit_angle < current_point_angle and exit_mirror_angle < current_point_angle:
                    # CASE 7: Pick the largest value
                    if print_data:
                        print("Case 7")
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 8: Current negative, one point more negative, one point less negative
                if (exit_angle < current_point_angle and exit_mirror_angle < 0 and exit_mirror_angle > current_point_angle) or (exit_mirror_angle < current_point_angle and exit_angle < 0 and exit_angle > current_point_angle):
                    # CASE 8: Pick the lowest value
                    if print_data:
                        print("Case 8")
                    if exit_angle > exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
                # CASE 9: Current negative, both points less negative
                if exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > current_point_angle and exit_mirror_angle < 0:
                    # CASE 9: Pick the largest value
                    if print_data:
                        print("Case 9")
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 10: Current negative, one point less negative, one point positive
                if (exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_mirror_angle < 0 and exit_angle > 0):
                    # CASE 10: Pick the largest value
                    if print_data:
                        print("Case 10")
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 11: Current negative, both points positive
                if exit_angle > 0 and exit_mirror_angle > 0:
                    # CASE 11: Pick the largest value
                    if print_data:
                        print("Case 11")
                    if exit_angle > exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 12: Current negative, one point positive, one point more negative
                if print_data:
                    print("Case 12")
                if (exit_angle > 0 and exit_mirror_angle < current_point_angle) or (
                        exit_mirror_angle > 0 and exit_angle < current_point_angle):
                    # CASE 12: Pick the smallest value
                    if exit_angle > exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
        else:
            if current_point_angle > 0:
                # CASE 1: Current positive, Both points negative
                if exit_angle < 0 and exit_mirror_angle < 0:
                    # CASE 1: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 2: Current positive, one point less positive, one point negative
                if (exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < 0) or (exit_mirror_angle < current_point_angle and exit_mirror_angle > 0 and exit_angle < 0):
                    # CASE 2: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 3: Current positive, both points less positive
                if exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0:
                    # CASE 3: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 4: Current positive, one point more positive, one point less positive
                if (exit_angle > current_point_angle and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_angle < current_point_angle and exit_angle > 0):
                    # CASE 4: Pick the smallest value
                    if exit_angle < exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
                # CASE 5: Current positive, both points more positive
                if exit_angle > current_point_angle and exit_mirror_angle > current_point_angle:
                    # CASE 5: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 6: Current positive, one point more positive, one point negative
                if (exit_angle > current_point_angle and exit_mirror_angle < 0) or (exit_mirror_angle > current_point_angle and exit_angle < 0):
                    # CASE 6: Pick the smallest value
                    if exit_angle < exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
            else:
                # CASE 7: Current negative, both points more negative
                if exit_angle < current_point_angle and exit_mirror_angle < current_point_angle:
                    # CASE 6: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 8: Current negative, one point more negative, one point less negative
                if (exit_angle < current_point_angle and exit_mirror_angle < 0 and exit_mirror_angle > current_point_angle) or (exit_mirror_angle < current_point_angle and exit_angle < 0 and exit_angle > current_point_angle):
                    # CASE 7: Pick the lowest value
                    if exit_angle < exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point
                # CASE 9: Current negative, both points less negative
                if exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > current_point_angle and exit_mirror_angle < 0:
                    # CASE 8: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 10: Current negative, one point less negative, one point positive
                if (exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_mirror_angle < 0 and exit_angle > 0):
                    # CASE 9: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 11: Current negative, both points positive
                if exit_angle > 0 and exit_mirror_angle > 0:
                    # CASE 10: Pick the largest value
                    if exit_angle < exit_mirror_angle:
                        return exit_point, centre_point
                    else:
                        return exit_mirror_point, centre_point
                # CASE 12: Current negative, one point positive, one point more negative
                if (exit_angle > 0 and exit_mirror_angle < current_point_angle) or (exit_mirror_angle > 0 and exit_angle < current_point_angle):
                    # CASE 12: Pick the smallest value
                    if exit_angle < exit_mirror_angle:
                        return exit_mirror_point, centre_point
                    else:
                        return exit_point, centre_point


        return "NO CASE FOUND: MAJOR BUG", "help"


    def improved_spline(self, print_data=False):
        output_waypoints = []
        centre_points = []
        # Alternate between line and curve until finished
        num_waypoints = len(self._waypoints)
        num_straights = num_waypoints - 1
        num_curves = num_waypoints - 2
        if print_data:
            print("Number of waypoints:", num_waypoints, "\n\tStraights:", num_straights, "\n\tCurves:", num_curves)
        # Do num_curves pairs of straight then curves
        temp_waypoint_start = self._waypoints[0]
        for index in range(num_curves):
            # Define start and end for straight line
            waypoint_start = temp_waypoint_start
            waypoint_end = self._waypoints[index + 1]
            if print_data:
                print(waypoint_start, " -> ", waypoint_end, sep="")
            # Add the two waypoints to the output list
            output_waypoints.append(waypoint_start)
            output_waypoints.append(waypoint_end)
            # From waypoint_start and waypoint_end, calculate the exit point of the curve that faces the next waypoint
            next_waypoint = self._waypoints[index + 2]
            curve_exit, centre_point = self.calculate_curve_exit(waypoint_start, waypoint_end, next_waypoint, print_data)
            output_waypoints.append(centre_point)
            if print_data:
                print("\tCurve Exit:", curve_exit)
            # Update new starting point to the curve exit
            temp_waypoint_start = curve_exit

        # Finish the path with a straight to the final waypoint
        output_waypoints.append(temp_waypoint_start)
        output_waypoints.append(self._waypoints[-1])
        return output_waypoints, centre_points

    def validate_perpendicularity(self, waypoints, initial_waypoint_count):
        check_amount = initial_waypoint_count - 2
        for index in range(check_amount):
            prev1 = waypoints[index * 3]
            curr1 = waypoints[index * 3 + 1]
            next1 = waypoints[index * 3 + 2]
            prev2 = waypoints[index * 3 + 2]
            curr2 = waypoints[index * 3 + 3]
            next2 = waypoints[index * 3 + 4]

            previous_current_length = math.hypot(curr1[0] - prev1[0], curr1[1] - prev1[1])
            current_next_length = math.hypot(next1[0] - curr1[0], next1[1] - curr1[1])
            previous_next_length = math.hypot(next1[0] - prev1[0], next1[1] - prev1[1])
            if (abs(previous_next_length ** 2) - abs(current_next_length ** 2 + previous_current_length ** 2)) > 0.0001:
                print("Validation Check Index:", index, "| Intersection: 1 | FAILED!", "\n\t", previous_next_length ** 2, current_next_length ** 2 + previous_current_length ** 2)
                print("\tWaypoints:", waypoints, initial_waypoint_count)
                return False
            print("Validation Check Index:", index, "| Intersection: 1 | OK!")
            previous_current_length = math.hypot(curr2[0] - prev2[0], curr2[1] - prev2[1])
            current_next_length = math.hypot(next2[0] - curr2[0], next2[1] - curr2[1])
            previous_next_length = math.hypot(next2[0] - prev2[0], next2[1] - prev2[1])
            if (abs(previous_next_length ** 2) - abs(current_next_length ** 2 + previous_current_length ** 2)) > 0.0001:
                print("Validation Check Index:", index, "| Intersection: 2 | FAILED!", "\n\t", previous_next_length ** 2, current_next_length ** 2 + previous_current_length ** 2)
                print("\tWaypoints:", waypoints, initial_waypoint_count)
                return False
            print("Validation Check:", index, "| Intersection: 2 | OK!")
        return True




if "__main__" == __name__:
    waypoints = [[1, 1], [2, 2], [3, 3], [4, 4], [5, 5]]
    Spliner = Path_Splining()
    Spliner.add_waypoints(waypoints)
    Spliner.plot_waypoints(waypoints)
    output, centres = Spliner.improved_spline(print_data=True)
    Spliner.plot_waypoints(output)
    print(Spliner.validate_perpendicularity(output, len(waypoints)))