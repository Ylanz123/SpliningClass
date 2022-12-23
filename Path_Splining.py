from __future__ import division, print_function
import matplotlib.pyplot as plt
import math

class Path_Splining():
    """
    This class contains all the methods responsible for creating and modifying a custom
    spline between waypoints.
    """
    def __init__(self, turn_radius=1, boundary_points=[], waypoints=[], resolution=1, tolerance=0):
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
        :param resolution: How many waypoints per metre we want. A higher value will give a higher resolution.
        :param tolerance: How many metres within the boundary the spline generator will allow.
        """
        self._turn_radius = turn_radius
        self._boundary_points = boundary_points
        self._waypoints = waypoints
        self._resolution = resolution
        self._tolerance = tolerance

    def add_waypoints(self, waypoints=[]):
        """
        This method allows the adding of waypoints to the current waypoint list. They
        will be appended after all the previous waypoints.
        :param waypoints: An array of [latitude, longitude] arrays in the order the vehicle
        has to travel. As an example index 0 will be the first waypoint to go to and index
        1 the second waypoint and so on.
        """
        # Loop through each waypoint and append it to self._waypoints
        for waypoint in waypoints:
            self._waypoints.append(waypoint)

    def remove_waypoints(self, indices=[]):
        """
        This method allows users to remove waypoints by single index or multiple
        indices at a time.
        :param indices: An array of indices to remove from the waypoint list.
        [firstIndex, secondIndex, ...]
        """
        # Sort the indices array in descending order
        indices.sort(reverse=True)
        # Pop each waypoint at each index in descending order
        for index in indices:
            self._waypoints.pop(index)

    def add_boundary(self, boundary_points=[]):
        """
        This method replaces the class instance's boundary points with the list provided by the arguments.
        :param boundary_points: A list of [lat, lon] arrays that create a closed boundary. The final point can equal
        the initial point but doesn't have to. The program will connect the ends together anyway.
        """
        self._boundary_points = boundary_points

    def edit_turn_radius(self, turn_radius):
        """
        Replaces the class instance's minimum turn radius with the argument given.
        :param turn_radius: Minimum turn radius in metres.
        """
        self._turn_radius = turn_radius

    def print_waypoints(self):
        """
        Prints the current waypoints saved to the class instance.
        """
        print("Instance Waypoints:")
        for waypoint in self._waypoints:
            print("\t", waypoint, sep="")

    def generate_spline(self, print_data=False):
        """
        Generates a spline for the class instance's waypoints.
        :param print_data: A flag that will return information about all sorts of things. This is a debugging resource.
        :return: A list of waypoints that follow a spline and a list of centre-point coordinates for each curve.
        """
        # First check that no waypoints are within the minimum turn radius of each other
        check_minimum_waypoint_radius(waypoints=self._waypoints, turn_radius=self._turn_radius, print_data=False)
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
            curve_exit, centre_point = calculate_curve_exit(waypoint_start, waypoint_end, next_waypoint, self._turn_radius)
            centre_points.append(centre_point)
            if print_data:
                print("\tCurve Exit:", curve_exit)
            # Update new starting point to the curve exit
            temp_waypoint_start = curve_exit

        # Finish the path with a straight to the final waypoint
        output_waypoints.append(temp_waypoint_start)
        output_waypoints.append(self._waypoints[-1])

        # Remove any consecutive waypoints that are the same
        index_range = range(len(output_waypoints))
        index_range.sort(reverse=True)
        for index in index_range:
            current_waypoint = output_waypoints[index]
            next_waypoint = output_waypoints[index - 1]
            if current_waypoint == next_waypoint:
                output_waypoints.pop(index)

        # Time to interpolate on the curves
        output_waypoints = interpolate_all_curves(output_waypoints, centre_points, self._turn_radius, self._resolution)
        return output_waypoints, centre_points


# GENERAL USE FUNCTIONS:
# Below are some general functions the Spline class uses and ones users can use for testing or for the passing
# of individual test cases. A lot of these functions are really just specialised math functions, that allow printing.

def check_minimum_waypoint_radius(waypoints, turn_radius, print_data=False):
    """
    Checks the distances between consecutive points and raises an error if any are within two times the given radius
    :param waypoints: A list of [lat, lon] coordinates.
    :param turn_radius: The minimum turn radius in metres.
    :param print_data: A flag that will return information about all sorts of things. This is a debugging resource.
    :return:
    """
    for index in range(len(waypoints) - 1):
        current_waypoint = waypoints[index]
        next_waypoint = waypoints[index + 1]
        distance = distance_between_two_points(current_waypoint, next_waypoint)
        if print_data:
            print("Index:", index, "| Distance:", distance / 2)
        if distance < turn_radius:
            raise ValueError("Waypoints too close together.")

def distance_between_two_points(point_one, point_two):
    """
    Gets the distance between two points.
    :param point_one: [x1, y1]
    :param point_two: [x2, y2]
    :return: Distance between two points.
    """
    distance = math.sqrt((point_one[1] - point_two[1]) ** 2 + (point_one[0] - point_two[0]) ** 2)
    return distance

def get_maximum_turn_radius(waypoints):
    """
    Finds the maximum turn radius for a given list of points.
    :param waypoints: A list of [lat, lon] coordinates.
    :return: Maximum possible turn radius.
    """
    # Check that there are at least two waypoints
    if len(waypoints) < 2:
        return None
    # Define first distance and divide by two
    maximum_turn_radius = distance_between_two_points(waypoints[0], waypoints[1]) / 2
    for index in range(1, len(waypoints) - 1):
        current_waypoint = waypoints[index]
        next_waypoint = waypoints[index + 1]
        distance = distance_between_two_points(current_waypoint, next_waypoint)
        if distance / 2 < maximum_turn_radius:
            maximum_turn_radius = distance / 2
    return maximum_turn_radius

def constrain_pi(theta):
    """
    Will constrain a given angle to between pi and -pi.
    :param theta: Angle in radians to constrain.
    :return: The constrained angle in radians.
    """
    while theta < - math.pi or theta > math.pi:
        if theta < - math.pi:
            theta = theta + 2 * math.pi
        if theta > math.pi:
            theta = theta - 2 * math.pi
    return theta

def sign(x):
    """
    If x is above zero, return 1.
    If x is zero, return 0.
    If x is below zero, return -1.
    :param x: A number.
    :return: The sign of x.
    """
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

def vertex_angle(point_one, point_two, point_three):
    """
    Finds the angle between three points.
    :param point_one: [x, y]
    :param point_two: [x, y]
    :param point_three: [x, y]
    :return: An angle in radians.
    """
    math.hypot()
    dist_one_two = distance_between_two_points(point_one, point_two)
    dist_one_three = distance_between_two_points(point_one, point_three)
    dist_two_three = distance_between_two_points(point_two, point_three)
    numerator = dist_one_two ** 2 + dist_one_three ** 2 - dist_two_three ** 2
    denominator = 2 * dist_one_two * dist_one_three
    angle = math.acos(numerator / denominator) * 180 / math.pi
    return angle

def get_angle_range(first_angle, second_angle, is_clockwise):
    """
    Returns negative angle if clockwise and positive if counter-clockwise in coordination with the unit circle
    :param first_angle:
    :param second_angle:
    :param is_clockwise:
    :return:
    """
    if first_angle > 0:
        if second_angle > first_angle:
            if is_clockwise:
                angle_range = second_angle - first_angle
            else:
                angle_range = second_angle - first_angle
        else:
            if is_clockwise:
                angle_range = first_angle - second_angle
            else:
                angle_range = 2 * math.pi - first_angle + second_angle
    else:
        if second_angle > first_angle:
            if is_clockwise:
                angle_range = 2 * math.pi + first_angle - second_angle
            else:
                angle_range = second_angle - first_angle
        else:
            if is_clockwise:
                angle_range = first_angle - second_angle
            else:
                angle_range = 2 * math.pi - first_angle + second_angle
    if is_clockwise:
        angle_range = - angle_range
    return angle_range

def get_arc_length(angle_range, radius):
    circumference = radius * 2 * math.pi
    angle_range_fraction = angle_range / (2 * math.pi)
    arc_length = circumference * angle_range_fraction
    return abs(arc_length)

def get_angle_interval(angle_range, arc_length, resolution):
    points_count = math.floor(arc_length * resolution)
    if points_count != 0:
        angle_interval = angle_range / points_count
        return angle_interval
    else:
        return angle_range

def get_circle_direction(previous_waypoint, current_waypoint, centre_point, error, print_data=False):
    if print_data:
        print("Previous:", previous_waypoint)
        print("Current:", current_waypoint)
        print("Centre Point:", centre_point)
    # Calculate the perpendicular angle from the current waypoint to the circle centre
    centre_to_current_grad_num = current_waypoint[1] - centre_point[1]
    centre_to_current_grad_den = current_waypoint[0] - centre_point[0]
    inv_centre_to_current_angle = math.atan2(-centre_to_current_grad_den, centre_to_current_grad_num)

    # Angle from the previous waypoint to the current waypoint
    previous_to_current_grad_num = current_waypoint[1] - previous_waypoint[1]
    previous_to_current_grad_den = current_waypoint[0] - previous_waypoint[0]
    previous_to_current_angle = math.atan2(previous_to_current_grad_num, previous_to_current_grad_den)

    if print_data:
        print("\tGrad Inv Angle:\t", inv_centre_to_current_angle, "\n\tComparison Angle:", previous_to_current_angle)
        print("\tClockwise:", abs(inv_centre_to_current_angle - previous_to_current_angle) <= error)

    # We can say if the inverse gradient from the centre to the current waypoint is the same as the
    # gradient from the previous waypoint to the current waypoint then we are heading in a clockwise direction
    # Think of the right hand rule from physics to help understand this.
    # Bit sketchy but it works for now. I suppose this is due to some rounding or maybe atan2
    # as the values are technically the same but there must be a deep decimal value that differs.
    if abs(inv_centre_to_current_angle - previous_to_current_angle) <= error:
        return True
    return False

def mirror_across_line(line_point1, line_point2, point):
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

def find_dual_perpendicular_angle(radius, origin, point, n=0):
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

    arctan_numerator = (next_point_y - origin_y)
    arctan_denominator = (next_point_x - origin_x)

    arcsin_numerator = r
    arcsin_denominator = math.sqrt(next_point_x ** 2 - 2 * next_point_x * origin_x + next_point_y ** 2 - 2 * next_point_y * origin_y + origin_x ** 2 + origin_y ** 2)
    arcsin_stuff = arcsin_numerator / arcsin_denominator

    # Reference this desmos page: https://www.desmos.com/calculator/loniceqosa
    # This equation comes from the solve function of my calendar. I solved for when M_NI was equal to M_IN and made
    # t (theta) the subject. All this numerator denominator stuff is just to make the code more readable.
    # print("Next Point:", next_point_x, next_point_y, "\nOrigin:", origin_x, origin_y)
    numerator = - (2 * math.asin(arcsin_stuff) - 2 * math.atan2(arctan_numerator, arctan_denominator) + (sign(next_point_x - origin_x) - 2 * (2 * n + 1)) * math.pi)
    denominator = 2
    theta = numerator / denominator

    return theta

def get_closest_centre_point(previous_waypoint, current_waypoint, next_waypoint, r):
    # Find the centre-point of the circle the path will trace.
    # Get perpendicular gradient of current waypoint to previous waypoint.
    inverse_gradient_numerator = current_waypoint[0] - previous_waypoint[0]
    inverse_gradient_denominator = current_waypoint[1] - previous_waypoint[1]
    # Angle below in reference to unit circle.
    gradient_angle = math.atan2(-inverse_gradient_numerator, inverse_gradient_denominator)
    # Get 2 possible points along that gradient from current waypoint that are of distance r.
    first_point = [current_waypoint[0] + r * math.cos(gradient_angle),
                   current_waypoint[1] + r * math.sin(gradient_angle)]
    second_point = [current_waypoint[0] + r * math.cos(gradient_angle + math.pi),
                    current_waypoint[1] + r * math.sin(gradient_angle + math.pi)]
    # Pick the point that is closer to the next waypoint.
    first_point_dist = math.sqrt(
        (first_point[0] - next_waypoint[0]) ** 2 + (first_point[1] - next_waypoint[1]) ** 2)
    second_point_dist = math.sqrt(
        (second_point[0] - next_waypoint[0]) ** 2 + (second_point[1] - next_waypoint[1]) ** 2)
    if first_point_dist <= second_point_dist:
        return first_point
    else:
        return second_point

def validate_perpendicularity(waypoints, initial_waypoint_count):
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

def calculate_curve_exit(previous_waypoint, current_waypoint, next_waypoint, radius, print_data=False):
    # Define r as the minimum turn radius to make lines a bit neater.
    r = radius
    centre_point = get_closest_centre_point(previous_waypoint, current_waypoint, next_waypoint, r)
    # This point is going to be the centre of the circle the plane will trace as it angles towards the
    # next waypoint.
    if print_data:
        print("\tCentre Point: ", centre_point)
    # Use the find_dual_perpendicular_angle function to calculate the angle of the point at which the plane
    # stops tracing the circle and travels to the next waypoint
    exit_angle = find_dual_perpendicular_angle(radius=r, origin=centre_point, point=next_waypoint, n=0)

    # If next waypoint x is less than centre point x, add pi to angle. This is because if the angle is in the
    # or third quadrant we have to add pi to angle when calculating.
    if next_waypoint[0] < centre_point[0]:
        exit_angle = exit_angle + math.pi
        exit_angle = constrain_pi(exit_angle)

    # Constrain theta between -pi and pi.
    if exit_angle > math.pi or exit_angle < -math.pi:
        raise ValueError("Constrain to PI error.")


    exit_point = [centre_point[0] + r * math.cos(exit_angle), centre_point[1] + r * math.sin(exit_angle)]
    # There exists another point that is its mirror across the line from the circle centre to the next waypoint.
    # So we calculate that here and will have to test which one the plane will encounter first as that will
    # be the one we choose.
    exit_mirror_point = mirror_across_line(centre_point, next_waypoint, exit_point)
    exit_mirror_angle = math.atan2(exit_mirror_point[1] - centre_point[1], exit_mirror_point[0] - centre_point[0])
    # Store the angle from the circle centre to the current waypoint, we'll use this to find which point the
    # plane will fly over first and when we need to find which case it is. (What combination of points it is)
    current_point_angle = math.atan2(current_waypoint[1] - centre_point[1], current_waypoint[0] - centre_point[0])

    if print_data:
        print("\tExit Point:", exit_point, "\n\tExit Mirror Point:", exit_mirror_point)
        print("\tExit Angle:", exit_angle, "\n\tExit Mirror Angle:", exit_mirror_angle, "\n\tCurrent Angle:", current_point_angle)

    clockwise = get_circle_direction(previous_waypoint, current_waypoint, centre_point, 1e-5, print_data=print_data)

    # Round all the angles for comparison. Don't think this level of accuracy will be a problem
    exit_angle = round(exit_angle, 8)
    exit_mirror_angle = round(exit_mirror_angle, 8)
    current_point_angle = round(current_point_angle, 8)
    # Round lat lon values to nearest 1.1 millimeter (7th decimal point)
    exit_point = [round(exit_point[0], 8), round(exit_point[1], 8)]
    exit_mirror_point = [round(exit_mirror_point[0], 8), round(exit_mirror_point[1], 8)]

    if print_data:
        print("Exit Point rounded:", exit_point)
    # Check if the current angle equals either of the exit angles. If it does, return that one
    if current_point_angle == exit_angle:
        return exit_point, ["straight", centre_point]
    if current_point_angle == exit_mirror_angle:
        return exit_mirror_point, ["straight", centre_point]

    # This collection of if else statements is absolutely disgusting, but it works.
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
                if print_data:
                    print("Case 1")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 2: Current positive, one point less positive, one point negative
            if (exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < 0) or (exit_mirror_angle < current_point_angle and exit_mirror_angle > 0 and exit_angle < 0):
                # CASE 2: Pick the largest value
                if print_data:
                    print("Case 2")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 3: Current positive, both points less positive
            if exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0:
                # CASE 3: Pick the largest value
                if print_data:
                    print("Case 3")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 4: Current positive, one point more positive, one point less positive
            if (exit_angle > current_point_angle and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_angle < current_point_angle and exit_angle > 0):
                # CASE 4: Pick the smallest value
                if print_data:
                    print("Case 4")
                if exit_angle > exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
            # CASE 5: Current positive, both points more positive
            if exit_angle > current_point_angle and exit_mirror_angle > current_point_angle:
                # CASE 5: Pick the largest value
                if print_data:
                    print("Case 5")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 6: Current positive, one point more positive, one point negative
            if (exit_angle > current_point_angle and exit_mirror_angle < 0) or (exit_mirror_angle > current_point_angle and exit_angle < 0):
                # CASE 6: Pick the smallest value
                if print_data:
                    print("Case 6")
                if exit_angle > exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
        else:
            if print_data:
                print("Negative angle")
            # CASE 7: Current negative, both points more negative
            if exit_angle < current_point_angle and exit_mirror_angle < current_point_angle:
                # CASE 7: Pick the largest value
                if print_data:
                    print("Case 7")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 8: Current negative, one point more negative, one point less negative
            if (exit_angle < current_point_angle and exit_mirror_angle < 0 and exit_mirror_angle > current_point_angle) or (exit_mirror_angle < current_point_angle and exit_angle < 0 and exit_angle > current_point_angle):
                # CASE 8: Pick the lowest value
                if print_data:
                    print("Case 8")
                if exit_angle > exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
            # CASE 9: Current negative, both points less negative
            if exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > current_point_angle and exit_mirror_angle < 0:
                # CASE 9: Pick the largest value
                if print_data:
                    print("Case 9")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 10: Current negative, one point less negative, one point positive
            if (exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_mirror_angle < 0 and exit_angle > 0):
                # CASE 10: Pick the largest value
                if print_data:
                    print("Case 10")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 11: Current negative, both points positive
            if exit_angle > 0 and exit_mirror_angle > 0:
                # CASE 11: Pick the largest value
                if print_data:
                    print("Case 11")
                if exit_angle > exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 12: Current negative, one point positive, one point more negative
            if print_data:
                print("Case 12")
            if (exit_angle > 0 and exit_mirror_angle < current_point_angle) or (
                    exit_mirror_angle > 0 and exit_angle < current_point_angle):
                # CASE 12: Pick the smallest value
                if exit_angle > exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
    else:
        if current_point_angle > 0:
            # CASE 1: Current positive, Both points negative
            if exit_angle < 0 and exit_mirror_angle < 0:
                # CASE 1: Pick the largest value
                if print_data:
                    print("Case 1")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 2: Current positive, one point less positive, one point negative
            if (exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < 0) or (exit_mirror_angle < current_point_angle and exit_mirror_angle > 0 and exit_angle < 0):
                # CASE 2: Pick the largest value
                if print_data:
                    print("Case 2")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 3: Current positive, both points less positive
            if exit_angle < current_point_angle and exit_angle > 0 and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0:
                # CASE 3: Pick the largest value
                if print_data:
                    print("Case 3")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 4: Current positive, one point more positive, one point less positive
            if (exit_angle > current_point_angle and exit_mirror_angle < current_point_angle and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_angle < current_point_angle and exit_angle > 0):
                # CASE 4: Pick the smallest value
                if print_data:
                    print("Case 4")
                if exit_angle < exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
            # CASE 5: Current positive, both points more positive
            if exit_angle > current_point_angle and exit_mirror_angle > current_point_angle:
                # CASE 5: Pick the largest value
                if print_data:
                    print("Case 5")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 6: Current positive, one point more positive, one point negative
            if (exit_angle > current_point_angle and exit_mirror_angle < 0) or (exit_mirror_angle > current_point_angle and exit_angle < 0):
                # CASE 6: Pick the smallest value
                if print_data:
                    print("Case 6")
                if exit_angle < exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
        else:
            # CASE 7: Current negative, both points more negative
            if exit_angle < current_point_angle and exit_mirror_angle < current_point_angle:
                # CASE 7: Pick the largest value
                if print_data:
                    print("Case 7")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 8: Current negative, one point more negative, one point less negative
            if (exit_angle < current_point_angle and exit_mirror_angle < 0 and exit_mirror_angle > current_point_angle) or (exit_mirror_angle < current_point_angle and exit_angle < 0 and exit_angle > current_point_angle):
                # CASE 8: Pick the lowest value
                if print_data:
                    print("Case 8")
                if exit_angle < exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]
            # CASE 9: Current negative, both points less negative
            if exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > current_point_angle and exit_mirror_angle < 0:
                # CASE 9: Pick the largest value
                if print_data:
                    print("Case 9")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 10: Current negative, one point less negative, one point positive
            if (exit_angle > current_point_angle and exit_angle < 0 and exit_mirror_angle > 0) or (exit_mirror_angle > current_point_angle and exit_mirror_angle < 0 and exit_angle > 0):
                # CASE 10: Pick the largest value
                if print_data:
                    print("Case 10")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 11: Current negative, both points positive
            if exit_angle > 0 and exit_mirror_angle > 0:
                # CASE 11: Pick the largest value
                if print_data:
                    print("Case 11")
                if exit_angle < exit_mirror_angle:
                    return exit_point, ["curve", centre_point]
                else:
                    return exit_mirror_point, ["curve", centre_point]
            # CASE 12: Current negative, one point positive, one point more negative
            if (exit_angle > 0 and exit_mirror_angle < current_point_angle) or (exit_mirror_angle > 0 and exit_angle < current_point_angle):
                # CASE 12: Pick the smallest value
                if print_data:
                    print("Case 12")
                if exit_angle < exit_mirror_angle:
                    return exit_mirror_point, ["curve", centre_point]
                else:
                    return exit_point, ["curve", centre_point]

    return "NO CASE FOUND: MAJOR BUG", "help"

def interpolate_all_curves(waypoints, centre_points, turn_radius, resolution, print_data=False):
    r = turn_radius
    # Get index of curve points
    curve_indices = []
    for index in range(len(centre_points)):
        if centre_points[index][0] == "curve":
            curve_indices.append(index + 1)
    waypoint_index_position = 1
    output = waypoints[:]
    output_injection_matrix = []
    for index in range(len(centre_points)):
        centre_point_data = centre_points[index]
        if centre_point_data[0] == "curve":
            corresponding_points = [waypoints[waypoint_index_position], waypoints[waypoint_index_position + 1]]
            centre_point = centre_point_data[1]
            # Calculate the arc length using the angles
            first_angle = math.atan2(corresponding_points[0][1] - centre_point[1], corresponding_points[0][0] - centre_point[0])
            second_angle = math.atan2(corresponding_points[1][1] - centre_point[1], corresponding_points[1][0] - centre_point[0])
            # To figure out which direction to go around the circle to find the arc length
            is_clockwise = get_circle_direction(previous_waypoint=waypoints[waypoint_index_position - 1], current_waypoint=corresponding_points[0], centre_point=centre_point, error=1e-5)
            angle_range = get_angle_range(first_angle, second_angle, is_clockwise)
            arc_length = get_arc_length(angle_range, r)
            if print_data:
                print("Waypoint:", corresponding_points[0], corresponding_points[1])
                print("First angle:", first_angle, "Second angle:", second_angle)
                print("Angle range:", angle_range, angle_range * 180 / math.pi)
                print("Arc length:", arc_length)
            # Find the angle interval using the arc length and resolution
            angle_interval = get_angle_interval(angle_range, arc_length, resolution)
            # Using the resolution sample a bunch of points along the curve
            current_additional_angle = angle_interval
            injection_section = []
            while abs(current_additional_angle) < abs(angle_range):
                sample_point = [centre_point[0] + r * math.cos(first_angle + current_additional_angle), centre_point[1] + r * math.sin(first_angle + current_additional_angle)]
                injection_point_data = [waypoint_index_position + 1, sample_point]
                injection_section.insert(0, injection_point_data)
                current_additional_angle += angle_interval
            output_injection_matrix.append(injection_section)
            waypoint_index_position += 1
        waypoint_index_position += 1
    # Inject new sampled points into output along with original waypoints
    output_injection_matrix = output_injection_matrix[::-1]  # Reverse it so adding points doesn't mess with indexing
    for injection_section in output_injection_matrix:
        for sample_point in injection_section:
            output.insert(sample_point[0], sample_point[1])
    return output

def plot_waypoints(waypoints_to_plot, centre_points=[]):
    if waypoints_to_plot == None:
        return False
    lat_vals = []
    lon_vals = []
    # Loop through to get each lat and lon value
    for waypoint in waypoints_to_plot:
        lat_vals.append(waypoint[0])
        lon_vals.append(waypoint[1])
    centre_x = []
    centre_y = []
    for centre_point in centre_points:
        if centre_point[0] == "curve":
            centre_x.append(centre_point[1][0])
            centre_y.append(centre_point[1][1])
    plt.plot(lat_vals, lon_vals, '--bo')
    plt.scatter(centre_x, centre_y)
    plt.xlim([0, 10])
    plt.ylim([0, 10])
    plt.axis('equal')

    plt.show()


if "__main__" == __name__:
    #waypoints = [[4.0, 5.0], [7.0, 6.0], [6.0, 9.0], [4.0, 7.0], [2.0, 6], [1, 3], [-3, 0], [-4, 5]]
    #waypoints = [[40, 40], [40, 70], [70, 70], [70, 40]]
    #waypoints = [[1.0, 1.0], [2.0, 2.0], [3.0, 3.0], [4.0, 4.0], [5.0, 5.0], [8, 5], [9, 3], [6, -4]]
    #waypoints = [[-10, 0], [-7, 0], [-5, 0], [-3, 0], [1, 2], [5, 4], [3, 0], [5, 2], [7, 0], [9, 2], [11, 0]]
    waypoints = [[5, 10], [9, 19], [12, 14], [11, 5], [3, -4], [-4, 2]]
    Spline_Generator = Path_Splining(waypoints=[], turn_radius=0, resolution=0, tolerance=0, boundary_points=[])
    Spline_Generator.print_waypoints()
    output, centres = Spline_Generator.generate_spline()
    plot_waypoints(output, centres)

