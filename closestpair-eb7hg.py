# Elijah Bosley (eb7hg@virginia.edu)
# Collaborated with Simmy Bhatia
import math
from functools import total_ordering
from typing import List


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return "{},{}".format(self.x, self.y)

    def __str__(self):
        return "{},{}".format(self.x, self.y)


@total_ordering
class PointsDistancePair:
    def __init__(self, points, dist):
        self.points = points
        self.distance = dist

    def __eq__(self, other):
        return self.distance == other.distance

    def __lt__(self, other):
        return self.distance < other.distance

    def __repr__(self):
        return "{}: {}".format(self.points, self.distance)

    def __str__(self):
        return "{}: {}".format(self.points, self.distance)


def get_x(point: Point) -> float:
    return point.x


def get_y(point: Point) -> float:
    return point.y


def merge_sort(points):
    if len(points) <= 1:
        return points
    median = int(len(points) / 2)  # Center divider
    left = points[:median]
    right = points[median:]
    left = merge_sort(left)
    right = merge_sort(right)

    return merge(left, right)


def merge(left: List[Point], right: List[Point]):
    sorted_points = []
    while len(left) > 0 and len(right) > 0:
        if left[0].y < right[0].y:
            sorted_points.append(left[0])
            left.remove(left[0])
        else:
            sorted_points.append(right[0])
            right.remove(right[0])
    if len(left) is 0:
        sorted_points += right
    else:
        sorted_points += left
    return sorted_points


def square(a: float) -> float:
    return a * a


def distance(a: Point, b: Point) -> float:  # calculate the distance between a and b given a point cloud
    return float(math.sqrt(square(b.x - a.x) + square(b.y - a.y)))


def find_points_on_runway(runway_width: float, median: float, points: List[Point]) -> List[Point]:
    max_x = median + runway_width
    min_x = median - runway_width

    points_on_runway = [point for point in points if min_x < point.x < max_x]
    # points_on_runway.sort(key=get_y)  # TODO use merge sort

    points_on_runway = merge_sort(points_on_runway)

    return points_on_runway


def compare_runway_points(points: list) -> PointsDistancePair:
    # starting from the bottom point recurse up the list of points by checking 15 above the current item

    best_distance_points = []
    best_distance_val = float("inf")
    for i in range(len(points)):  # iterate through every point
        max_idx = min(15, len(points))  # find the maximum value to run until
        num_comparisons = 0
        for j in range(i + 1, len(points)):  # iterate through the 15 points above
            if num_comparisons < max_idx:
                curr_distance = distance(points[i], points[j])
                if curr_distance < best_distance_val:
                    best_distance_val = curr_distance
                    best_distance_points = [points[i], points[i + 1]]
                num_comparisons += 1
            else:
                break

    return PointsDistancePair(best_distance_points, best_distance_val)


def closest_pair(points: list) -> PointsDistancePair:  # returns 2 points that are closest to one another

    median = int(len(points) / 2)  # Center divider

    left = points[0:median]
    right = points[median:]
    # print(str(len(left)) + " : " + str(len(right)))
    left_points_distance_pair = PointsDistancePair([], float("inf"))
    right_points_distance_pair = PointsDistancePair([], float("inf"))

    # Left side
    if len(left) < 2:
        left_dist = float("inf")
    elif len(left) == 2:
        left_dist = distance(left[0], left[1])
        left_points_distance_pair = PointsDistancePair([left[0], left[1]], left_dist)
    else:
        left_points_distance_pair = closest_pair(left)  # do some recursion
        left_dist = left_points_distance_pair.distance

    # Right Side
    if len(right) < 2:
        right_dist = float("inf")
    elif len(right) == 2:
        right_dist = distance(right[0], right[1])
        right_points_distance_pair = PointsDistancePair([right[0], right[1]], right_dist)
    else:
        right_points_distance_pair = closest_pair(right)
        right_dist = right_points_distance_pair.distance

    # Combine Step
    runway_width = min(left_dist, right_dist)  # this is delta

    runway_points = find_points_on_runway(runway_width, points[median].x, points)  # find all points within 2 delta
    runway_pair = compare_runway_points(runway_points)
    return min(left_points_distance_pair, right_points_distance_pair, runway_pair)


def main():
    f = open("garden.txt", "r")
    points = []
    plants = f.readline().strip()
    for line in f:
        x, y = line.split()
        points.append(Point(float(x), float(y)))

    # print("There are {} plants".format(plants))
    points.sort(key=get_x)  # Start by sorting the points by x
    pair = closest_pair(points)
    print(pair.distance)


main()
