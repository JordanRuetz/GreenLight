from typing import Callable
import math


class Rail:
    def __init__(self, fun: Callable, max_position: int):
        self.fun = fun
        self.max_position = max_position

    def get(self, position: float):
        return self.fun(position)


class LeftRail(Rail):
    def __init__(self, transform):
        self.outside_len = 100
        self.inside_width = 100
        self.curve_length = (2 * math.pi * 75) / 4
        self.curve_scalar_prop = self.curve_length / (self.outside_len * 2 + self.curve_length)
        self.cutoffs = [(1 - self.curve_scalar_prop) * 1000 / 2, 1000 - (1 - self.curve_scalar_prop) * 1000 / 2]
        self.inner_start = (-25, -50)
        self.transform = transform

    def applyTransform(self, x, y, transform):
        for i in range(transform):
            x, y = -1 * y, x
        return x, y

    def get(self, scalar):
        if scalar < self.cutoffs[0]:
            len_along_entry = scalar * self.outside_len / self.cutoffs[0]
            x_cord = 0 - (self.outside_len - len_along_entry + self.inside_width / 2)
            y_cord = -25
            return self.applyTransform(x_cord, y_cord, self.transform)
        elif scalar > self.cutoffs[1]:
            len_along_entry = (1000 - scalar) * self.outside_len / (1000 - self.cutoffs[1])
            y_cord = self.outside_len - len_along_entry + self.inside_width / 2
            x_cord = 25
            return self.applyTransform(x_cord, y_cord, self.transform)
        else:
            inner_scalar = self.cutoffs[1] - self.cutoffs[0]
            prop_scalar = (scalar - self.cutoffs[0]) / inner_scalar
            angle = 90 * prop_scalar
            x_cord = 0 - self.inside_width / 2 + 75 * math.sin(angle)
            y_cord = self.inside_width / 2 - 75 * math.cos(angle)
            return self.applyTransform(x_cord, y_cord, self.transform)


class RightRail(Rail):
    def __init__(self, transform):
        self.outside_len = 100
        self.inside_width = 100
        self.curve_length = (2 * math.pi * 25) / 4
        self.curve_scalar_prop = self.curve_length / (self.outside_len * 2 + self.curve_length)
        self.cutoffs = [(1 - self.curve_scalar_prop) * 1000 / 2, 1000 - (1 - self.curve_scalar_prop) * 1000 / 2]
        self.inner_start = (-25, -50)
        self.transform = transform

    def applyTransform(self, x, y, transform):
        for i in range(transform + 1):
            x, y = -1 * y, x
        return x, y

    def get(self, scalar):
        scalar = 1000 - scalar
        if scalar < self.cutoffs[0]:
            len_along_entry = scalar * self.outside_len / self.cutoffs[0]
            x_cord = 0 - (self.outside_len - len_along_entry + self.inside_width / 2)
            y_cord = 25
            return self.applyTransform(x_cord, y_cord, self.transform)
        elif scalar > self.cutoffs[1]:
            len_along_entry = (1000 - scalar) * self.outside_len / (1000 - self.cutoffs[1])
            y_cord = self.outside_len - len_along_entry + self.inside_width / 2
            x_cord = -25
            return self.applyTransform(x_cord, y_cord, self.transform)
        else:
            inner_scalar = self.cutoffs[1] - self.cutoffs[0]
            prop_scalar = (scalar - self.cutoffs[0]) / inner_scalar
            angle = 90 * prop_scalar
            x_cord = 0 - self.inside_width / 2 + 25 * math.sin(angle)
            y_cord = self.inside_width / 2 - 25 * math.cos(angle)
            return self.applyTransform(x_cord, y_cord, self.transform)


class StraightRail(Rail):
    def __init__(self, transform):
        self.outside_len = 100
        self.inside_width = 100
        self.total_len = self.inside_width + self.outside_len
        self.transform = transform

    def applyTransform(self, x, y, transform):
        for i in range(transform):
            x, y = -1 * y, x
        return x, y

    def get(self, scalar):
        scalar_prop = scalar / 1000
        x_cord = self.total_len * scalar_prop - self.total_len / 2
        y_cord = -25
        return self.applyTransform(x_cord, y_cord, self.transform)

if __name__ == '__main__':
    rail_1 = StraightRail(0)
    rail_2 = StraightRail(1)
    for step_1 in range(rail_1.max_position):
        for step_2 in range(rail_2.max_position):
            if rail_1.get(step_1) == rail_2.get(step_2):
                print(rail_1.get(float(step_1)), rail_2.get(float(step_2)))
