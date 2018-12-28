""" A test of making a new simulation that is much more efficient.

"""

import random

from car import Car
from gui import ZipperView
from new_intersection import Intersection, distance
from rail import LeftRail, RightRail, StraightRail


cars = []
rails = []

# create all the rails with no cars on them yet
for rotation in range(0, 4):
    leftrail = LeftRail(rotation)
    rightrail = RightRail(rotation)
    straightrail = StraightRail(rotation)
    rails.extend([leftrail, rightrail, straightrail])


intersection = Intersection(cars, rails)

view = ZipperView(intersection=intersection,
                  window_size=(800, 600),
                  x_lanes=2,
                  y_lanes=2)

j = 0
while not view.quitting:
    view.tick()

    # if a car is done its path, remove it from list of cars in intersection
    for i, car in enumerate(intersection.cars):
        if car.get_pos(view.time) > car.rail.total_distance:
            intersection.cars.pop(i)

    # add another car to the intersection
    if random.randint(0, 100) < 9:
        randRail = random.randint(0, 11)
        car = Car(1, intersection.rails[randRail], "CAR" + str(j), start_time=view.time)

        # don't create car if another car in initial point

        # TODO: this should just see where the last car on that rail is
        origin_occupied = False
        for c2 in intersection.cars:
            if distance(c2.get_location(view.time), car.get_location(view.time)) < car.radius * 2:
                origin_occupied = True
            if c2.rail == car.rail:
                origin_occupied = True
        if origin_occupied:
            continue
        j += 1

        intersection.add_car(car)

        intersection.update()
