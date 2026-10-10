"""Grid navigation independent of camera and motor hardware."""
from collections import deque


def shortest_path(start, goal, grid=4):
    if not all(isinstance(n, int) and 1 <= n <= grid * grid for n in (start, goal)):
        raise ValueError('Node outside grid')
    pending = deque([(start, [start])])
    seen = {start}
    while pending:
        node, route = pending.popleft()
        if node == goal:
            return route
        row, col = divmod(node - 1, grid)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            r, c = row + dr, col + dc
            nxt = r * grid + c + 1
            if 0 <= r < grid and 0 <= c < grid and nxt not in seen:
                seen.add(nxt)
                pending.append((nxt, route + [nxt]))
    raise ValueError('No route')


def heading(current, following, grid=4):
    r1, c1 = divmod(current - 1, grid)
    r2, c2 = divmod(following - 1, grid)
    return {(-1, 0): 180.0, (1, 0): 0.0, (0, -1): 270.0, (0, 1): 90.0}[(r2-r1, c2-c1)]


def angle_error(target, actual):
    return (target - actual + 180.0) % 360.0 - 180.0
