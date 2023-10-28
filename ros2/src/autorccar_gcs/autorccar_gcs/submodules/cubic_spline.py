import math
import numpy as np
import bisect

class CubicSpline1D:
    def __init__(self, x, y):

        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.CalculateAMatrix(h)
        B = self.CalculateBMatrix(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def CalculatePosition(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.FindIndex(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def CalculateFirstDerivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.FindIndex(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def CalculateSecondDerivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.FindIndex(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def FindIndex(self, x):
        return bisect.bisect(self.x, x) - 1

    def CalculateAMatrix(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def CalculateBMatrix(self, h, a):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    def __init__(self, x, y):
        self.s = self.CalculateDistanceParameter(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def CalculateDistanceParameter(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def CalculatePosition(self, s):
        x = self.sx.CalculatePosition(s)
        y = self.sy.CalculatePosition(s)

        return x, y

    def CalculateCurvature(self, s):
        dx = self.sx.CalculateFirstDerivative(s)
        ddx = self.sx.CalculateSecondDerivative(s)
        dy = self.sy.CalculateFirstDerivative(s)
        ddy = self.sy.CalculateSecondDerivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def CalculateHeading(self, s):
        dx = self.sx.CalculateFirstDerivative(s)
        dy = self.sy.CalculateFirstDerivative(s)
        heading = math.atan2(dy, dx)
        return heading


def CalculateCubicSplinePath(x_points, y_points, ds=0.1):
    csp = CubicSpline2D(x_points, y_points)
    s = list(np.arange(0, csp.s[-1], ds))

    ref_x, ref_y, ref_heading, ref_k = [], [], [], []
    for i_s in s:
        ix, iy = csp.CalculatePosition(i_s)
        ref_x.append(ix)
        ref_y.append(iy)
        ref_heading.append(csp.CalculateHeading(i_s))
        ref_k.append(csp.CalculateCurvature(i_s))

    return ref_x, ref_y, ref_heading, ref_k, s
