"""Route helpers for the route supervisor (ROS-free, so the tests exercise the same code as the node)."""
import math

LETHAL = 100  # nav_msgs/OccupancyGrid value of a lethal costmap cell (costmap_2d publishes 254 as 100)


class GridView:
    """Read access to a nav_msgs/OccupancyGrid-like grid: resolution, origin (x, y), width, height and
    row-major data (unrotated origin)."""

    def __init__(self, resolution, origin_x, origin_y, width, height, data):
        self.res = resolution
        self.ox, self.oy = origin_x, origin_y
        self.width, self.height = width, height
        self.data = data

    @classmethod
    def from_msg(cls, msg):
        i = msg.info
        return cls(i.resolution, i.origin.position.x, i.origin.position.y, i.width, i.height, msg.data)

    def lethal_within(self, x, y, radius):
        """An obstacle cell within radius [m] of (x, y)."""
        n = int(math.ceil(radius / self.res))
        cx = int(math.floor((x - self.ox) / self.res))
        cy = int(math.floor((y - self.oy) / self.res))
        for j in range(max(cy - n, 0), min(cy + n, self.height - 1) + 1):
            for i in range(max(cx - n, 0), min(cx + n, self.width - 1) + 1):
                if self.data[j * self.width + i] != LETHAL:
                    continue
                dx = self.ox + (i + 0.5) * self.res - x
                dy = self.oy + (j + 0.5) * self.res - y
                if dx * dx + dy * dy <= (radius + 0.5 * self.res) ** 2:
                    return True
        return False


def blocked_indices(points, grid, indices, radius):
    """Sorted indices (of those given) of the route points with an obstacle within radius on the grid."""
    return [k for k in sorted(indices) if grid.lethal_within(points[k][0], points[k][1], radius)]


def last_in(blocked, start, end):
    """Largest of the sorted indices in [start, end], or None."""
    found = None
    for k in blocked:
        if k > end:
            break
        if k >= start:
            found = k
    return found
