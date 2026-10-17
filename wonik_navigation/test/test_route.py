#!/usr/bin/env python3
"""Unit tests of wonik_navigation.route (the route supervisor's avoidance trigger and rejoin point)."""
import unittest

from wonik_navigation.route import LETHAL, GridView, blocked_indices, last_in


def grid_with_box(x0, y0, x1, y1, res=0.05, size=200, origin=(-2.0, -5.0)):
    """size x size grid of free cells with a lethal box [x0, x1] x [y0, y1]."""
    data = [0] * (size * size)
    for j in range(size):
        for i in range(size):
            x = origin[0] + (i + 0.5) * res
            y = origin[1] + (j + 0.5) * res
            if x0 <= x <= x1 and y0 <= y <= y1:
                data[j * size + i] = LETHAL
    return GridView(res, origin[0], origin[1], size, size, data)


ROUTE = [(0.05 * k, 0.0) for k in range(161)]  # 0 .. 8 m along x
ALL = range(len(ROUTE))


class BlockedTest(unittest.TestCase):
    def test_points_near_the_obstacle(self):
        g = grid_with_box(3.7, -0.3, 4.3, 0.3)  # a 0.6 m box on the route at x = 4
        blocked = blocked_indices(ROUTE, g, ALL, 0.6)
        self.assertAlmostEqual(ROUTE[blocked[0]][0], 3.7 - 0.6, delta=0.06)
        self.assertAlmostEqual(ROUTE[blocked[-1]][0], 4.3 + 0.6, delta=0.06)

    def test_clear_route(self):
        g = grid_with_box(3.7, 1.0, 4.3, 1.6)  # beside the route, more than 0.6 m off it
        self.assertEqual(blocked_indices(ROUTE, g, ALL, 0.6), [])

    def test_only_the_given_points(self):
        g = grid_with_box(6.7, -0.3, 7.3, 0.3)
        self.assertEqual(blocked_indices(ROUTE, g, range(0, 101), 0.6), [])  # up to x = 5 m
        self.assertNotEqual(blocked_indices(ROUTE, g, ALL, 0.6), [])

    def test_inflation_is_not_an_obstacle(self):
        g = grid_with_box(3.7, -0.3, 4.3, 0.3)
        g.data = [99 if v == LETHAL else v for v in g.data]  # inscribed inflation, not lethal
        self.assertEqual(blocked_indices(ROUTE, g, ALL, 0.6), [])

    def test_last_in(self):
        self.assertEqual(last_in([3, 7, 12], 0, 10), 7)
        self.assertEqual(last_in([3, 7, 12], 8, 11), None)
        self.assertEqual(last_in([], 0, 10), None)
        self.assertEqual(last_in([3, 7, 12], 12, 20), 12)


if __name__ == "__main__":
    unittest.main()
