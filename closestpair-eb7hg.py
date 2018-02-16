# Elijah Bosley (eb7hg@virginia.edu)
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


def shortest_path(point_list: List[Point],
                  shortest_left: List[PointsDistancePair],
                  shortest_right: List[PointsDistancePair]) -> PointsDistancePair:
    def square(a: float) -> float:
        return a * a

    def distance(a: Point, b: Point) -> float:  # calculate the distance between a and b given a point cloud
        return float(math.sqrt(square(b.x - a.x) + square(b.y - a.y)))

    def find_points_on_runway(runway_width: float, median: float, points: List[Point]) -> List[Point]:
        max_x = median + runway_width
        min_x = median - runway_width

        points_on_runway = [point for point in points if min_x < point.x < max_x]
        points_on_runway.sort(key=get_y)  # TODO use merge sort
        return points_on_runway

    def conquer(points: list) -> PointsDistancePair:
        # starting from the bottom point recurse up the list of points by checking 15 above the current item
        max_idx = min(15, len(points))  # find the maximum value to run until
        best_distance_points = []
        best_distance_val = float("inf")
        for i in range(max_idx - 1):
            curr_distance = distance(points[i], points[i + 1])
            if curr_distance < best_distance_val:
                best_distance_val = curr_distance
                best_distance_points = [points[i], points[i + 1]]

        return PointsDistancePair(best_distance_points, best_distance_val)

    def divide(points: list) -> PointsDistancePair:  # returns 2 points that are closest to one another

        if len(points) < 2:
            return PointsDistancePair([], float("inf"))

        elif len(points) is 2:
            return PointsDistancePair([points[0], points[1]], distance(points[0], points[1]))
        else:

            median = int(len(points) / 2)  # Center divider

            left = points[0:median]
            right = points[median:]

            shortest_left.append(divide(left))
            shortest_right.append(divide(right))

            min_left = min(shortest_left)
            min_right = min(shortest_right)
            runway_width = min(min_left, min_right)

            shortest_runway = []
            runway_points = find_points_on_runway(runway_width.distance, points[median].x,
                                                  points)  # find all points within 2 delta
            shortest_runway.append(conquer(runway_points))
            min_runway = min(shortest_runway)

            return min(min_left, min_right, min_runway)

    return divide(point_list)


def main():
    f = open("garden_10k.txt", "r")
    points = []
    plants = f.readline().strip()
    for line in f:
        x, y = line.split()
        points.append(Point(float(x), float(y)))

    print("There are {} plants".format(plants))
    points.sort(key=get_x)  # Start by sorting the points by x
    pair = shortest_path(points, [], [])
    print(pair)


main()
