# Author: https://github.com/Weilory
# Script: Compute the Convex Hull of a set of points using the Radar Search (Plotting each step!)

from math import *


class Vertex:
    """
    Vertex Object
    an array of two dimensional dots
    Attributes
        dots: [[x, y], [x, y] ...]
        x: [x, x ...]
        y: [y, y ...]
        min_x: minimum x value
        max_x: maximum x value
        min_y: minimum y value
        max_y: maximum y value
    Methods
        __init__: either inited by dots or x and y values
        concat: return a Vertex
        __str__: return string narrates dots, x and y
    """

    def __init__(
            self,
            dots=None,
            x=None,
            y=None
    ):
        if (dots is not None) and (x is None) and (y is None):
            # init by dots
            self.dots = dots
            self.x = []
            self.y = []
            for dt in self.dots:
                self.x.append(dt[0])
                self.y.append(dt[1])
        elif (dots is None) and (x is not None) and (y is not None):
            # init by x y
            self.x = x
            self.y = y
            self.dots = []
            for ii, xx in enumerate(self.x):
                self.dots.append([xx, y[ii]])
        else:
            raise TypeError('Vertex should be inited with either dots or x y values')

        # bounding rect
        self.max_x = self.x[0]
        self.max_y = self.y[0]
        self.min_x = self.x[0]
        self.min_y = self.y[0]
        for ii in range(len(self.x)):
            if self.max_x < self.x[ii]:
                self.max_x = self.x[ii]
            elif self.min_x > self.x[ii]:
                self.min_x = self.x[ii]
            if self.max_y < self.y[ii]:
                self.max_y = self.y[ii]
            elif self.min_y > self.y[ii]:
                self.min_y = self.y[ii]

    def __str__(self):
        return f'dots: {self.dots}\nx: {self.x}\ny:{self.y}\n'

    def concat(self):
        nv = Vertex(
            dots=None,
            x=[xx for xx in self.x],
            y=[yy for yy in self.y],
        )
        return nv


class ConvexHull(Vertex):
    """
        ConvexHull Object
            initialized by a set of dots

        Attributes
            get_dots: vertex
            get_hull: vertex
            route_bottom_left: [[x, y], [x, y] ...]
            route_top_right: [[x, y], [x, y] ...]

        Methods
            (All non-static methods should not be called since they are designed to execute in initializer)
            convex_hull: run two routes and assign variables
        """

    # pass in two dots [x, y]
    # return the radians which describes where the second one is refer to the first
    # note that
    #   atan(dy / 0) = atan(infinity) = 0.5 pi
    @staticmethod
    def theta(v1, v2):
        dy = v2[1] - v1[1]
        dx = v2[0] - v1[0]
        if dx == 0:
            if dy == 0:
                # v1 == v2
                return None
            # v1.x == v2.x
            return 0.5 * pi
        return abs(atan(dy / dx))

    '''
    start at x positive axis, rotate anti-clockwise until reaches a full circle (360)
    pass in two dots in format of [x, y]
    return a radians in domain of [0, 2pi] that indicates where the second one is refer to the first
    '''

    @staticmethod
    def x_pos_up(v1, v2):
        the = ConvexHull.theta(v1, v2)
        if v1[0] < v2[0] and v1[1] <= v2[1]:
            pass
        elif v1[0] >= v2[0] and v1[1] < v2[1]:
            the = pi - the
        elif v1[0] > v2[0] and v1[1] >= v2[1]:
            the = pi + the
        elif v1[0] <= v2[0] and v1[1] > v2[1]:
            the = 2 * pi - the
        else:
            # v1 == v2
            the = 2 * pi
        return the

    '''
    start at x negative axis, rotate anti-clockwise until reaches a full circle (360)
    pass in two dots in format of [x, y]
    return a radians in domain of [0, 2pi] that indicates where the second one is refer to the first
    '''

    @staticmethod
    def x_neg_down(v1, v2):
        the = ConvexHull.theta(v1, v2)
        if v1[0] > v2[0] and v1[1] >= v2[1]:
            pass
        elif v1[0] <= v2[0] and v1[1] > v2[1]:
            the = pi - the
        elif v1[0] < v2[0] and v1[1] <= v2[1]:
            the = pi + the
        elif v1[0] >= v2[0] and v1[1] < v2[1]:
            the = 2 * pi - the
        else:
            # v1 == v2
            return 2 * pi
        return the

    # pass in [[x, y], [x, y] ...]
    # return the dot at bottom left corner
    # bottom take priority than left
    @staticmethod
    def find_bottom_left(dts):
        bottoms = [dts[0]]
        for dt in dts:
            if dt[1] < bottoms[0][1]:
                bottoms = [dt]
            elif dt[1] == bottoms[0][1]:
                bottoms.append(dt)
        bottom_left = bottoms[0]
        for dt in bottoms:
            if dt[0] < bottom_left[0]:
                bottom_left = dt
        return bottom_left

    # pass in [[x, y], [x, y] ...]
    # return the dot at top right corner
    # top take priority that right
    @staticmethod
    def find_top_right(dts):
        tops = [dts[0]]
        for dt in dts:
            if dt[1] > tops[0][1]:
                tops = [dt]
            elif dt[1] == tops[0][1]:
                tops.append(dt)
        top_right = tops[0]
        for dt in tops:
            if dt[0] > top_right[0]:
                top_right = dt
        return top_right

    # pass in [1, 2, 0, 4]
    # return 2
    @staticmethod
    def min_at(arr):
        return arr.index(min(arr))

    """
    route1
        start at bottom left, use x_pos_up to find which dot has smallest radians refer to bottom left dot
        delete the found dot from array, append the found dot to hull

        then declare a target dot, assign to found dot, use x_pos_up to find which dot has smallest radians 
        refer to target
        delete the found dot from array, append the found dot to hull, assign target to found dot
        loop this part (where target is used) until target is updated to top right 

    route2
        start at top right, use x_neg_down to find which dot has smallest radians refer to top right dot
        delete the found dot from array, append the found dot to hull

        then declare a target dot, assign to found dot, use x_neg_down to find which dot has smallest radians 
        refer to target
        delete the found dot from array, append the found dot to hull, assign target to found dot
        loop this part (where target is used) until target is updated to bottom left

    you may wonder why not using a route3 for the left, a route4 for the right, so that it will be 2 times faster?
    we can only apply two side limitations, since the convex hull can be a triangle. 
    """

    @staticmethod
    def route(
            dts,  # dots
            refer,  # static dot as initial reference (where finding start)
            target,  # condition to reach for terminating the loop (return when found = target)
            func,  # function to work out radians between two vertexes
    ):
        rou = [refer]
        nds = []
        for dt in dts:
            nds.append(func(refer, dt))
        ind = ConvexHull.min_at(nds)
        tar = dts[ind]
        rou.append(tar)
        del dts[ind]

        while tar != target:
            nds = []
            for dt in dts:
                nds.append(func(tar, dt))
            ind = ConvexHull.min_at(nds)
            tar = dts[ind]
            rou.append(tar)
            del dts[ind]

        return rou

    def __init__(self, dots=None, x=None, y=None):
        super().__init__(dots, x, y)
        self.get_dots = self.concat()
        self.route_bottom_left = []
        self.route_top_right = []
        # routes are initialized in self.convex_hull
        self.get_hull = Vertex(self.convex_hull())

    def convex_hull(self):
        # dot at bottom left
        bottom_left = ConvexHull.find_bottom_left(self.dots)
        # dot at top right
        top_right = ConvexHull.find_top_right(self.dots)
        # assign variable to hulls
        self.route_bottom_left = ConvexHull.route(self.dots, bottom_left, top_right, ConvexHull.x_pos_up)
        self.route_top_right = ConvexHull.route(self.dots, top_right, bottom_left, ConvexHull.x_neg_down)
        # return a convex hull ch
        # which ch[0] = ch[-1]
        return self.route_bottom_left[:-1] + self.route_top_right


if __name__ == '__main__':
    from random import *
    from matplotlib import pyplot as plt
    from matplotlib.animation import FuncAnimation

    # animation: a dot extends a line
    def from_to_animate(
            fig_,
            dots,  # [[x, y], [x, y] ...]
            dense,  # unit length per segment,
            # for example, if dense=2,
            # then [[0, 0], [6, 0]] will be parsed into [[0, 0], [2, 0], [4, 0], [6, 0]]
            color,
            interval,  # time in milliseconds between two dots
            line=True,  # show line
            dot=True  # show dot
    ):
        def update(frame):
            if line:
                if frame != 0:
                    plt.plot([x_[frame - 1], x_[frame]], [y_[frame - 1], y_[frame]], color=color)
            p.set_data(x_[frame], y_[frame])
            return p,

        def init():
            p.set_data(x_[0], y_[0])
            return p,

        x_ = []
        y_ = []
        for ii in range(len(dots) - 1):
            from_ = dots[ii]
            to_ = dots[ii + 1]
            x_start, y_start = from_
            x_end, y_end = to_
            dis = ((x_end - x_start) ** 2 + (y_end - y_start) ** 2) ** 0.5
            seg = round(dis / dense)  # restricted to [1, infinity]
            if seg <= 1:
                seg = 1
                dse = 2
            else:
                dse = seg - 1
            x_step = (x_end - x_start) / dse
            y_step = (y_end - y_start) / dse
            for jj in range(seg):
                x_.append(x_start + jj * x_step)
                y_.append(y_start + jj * y_step)

        if dot:
            p, = plt.plot(x_[0], y_[0], "o", color=color)
        else:
            p, = plt.plot(x_[0], y_[0], color=color)

        return FuncAnimation(fig=fig_, func=update, init_func=init, frames=[ii for ii in range(len(x_))],
                             interval=interval, repeat=False)

    # generates random x y coordinates
    def random_xy(x_min, x_max, y_min, y_max, amount):
        xst = []
        yst = []
        for ii in range(amount):
            xst.append(randint(x_min, x_max))
            yst.append(randint(y_min, y_max))
        return xst, yst


    xs, ys = random_xy(
        0,  # x min
        100,  # x max
        0,  # y min
        100,  # y max
        50  # amount of sample dots
    )

    # calculation are performed in initializing
    ch = ConvexHull(
        x=xs,
        y=ys,
    )

    print('Random Generated Dots:')
    print(ch.get_dots.dots)

    print()

    print('Obtained Convex Hull:')
    print(ch.get_hull.dots)

    fig = plt.figure()
    plt.axis('off')
    plt.xlim(-10, 110)
    plt.ylim(-10, 110)
    plt.scatter(xs, ys, color='red')
    va_bottom_left = from_to_animate(fig, ch.route_bottom_left, 2, 'blue', 10)
    va_top_right = from_to_animate(fig, ch.route_top_right, 2, 'blue', 10)
    plt.show()



