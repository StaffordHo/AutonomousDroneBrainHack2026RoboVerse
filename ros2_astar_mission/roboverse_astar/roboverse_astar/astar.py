import heapq
import math
from typing import Dict, Iterable, List, Optional, Tuple


GridCell = Tuple[int, int]


def heuristic(a: GridCell, b: GridCell) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def neighbors_8(cell: GridCell) -> Iterable[Tuple[GridCell, float]]:
    x, y = cell
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield (x + dx, y + dy), math.sqrt(dx * dx + dy * dy)


def astar_grid(
    occupancy,
    width: int,
    height: int,
    start: GridCell,
    goal: GridCell,
    occupied_threshold: int = 60,
    unknown_cost: float = 4.0,
    free_cost: float = 1.0,
) -> Optional[List[GridCell]]:
    """
    A* over nav_msgs OccupancyGrid-style data.

    Unknown cells are allowed with a penalty. This is intentional for exploration:
    the planner can route toward frontiers while still avoiding known occupied
    cells. Inflated occupied cells should already be encoded as >= threshold.
    """
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        return None
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        return None

    if start == goal:
        return [start]

    def occ(cell: GridCell) -> int:
        return int(occupancy[cell[1] * width + cell[0]])

    if occ(goal) >= occupied_threshold:
        return None

    frontier = []
    heapq.heappush(frontier, (0.0, start))

    came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
    cost_so_far: Dict[GridCell, float] = {start: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        for nxt, step_cost in neighbors_8(current):
            x, y = nxt
            if not (0 <= x < width and 0 <= y < height):
                continue

            value = occ(nxt)
            if nxt == start:
                value = 0
            if value >= occupied_threshold:
                continue

            cell_cost = unknown_cost if value < 0 else free_cost + value / 100.0
            new_cost = cost_so_far[current] + step_cost * cell_cost

            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        return None

    path = []
    current = goal
    while current is not None:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def simplify_path(path: List[GridCell], stride: int = 3) -> List[GridCell]:
    if len(path) <= 2:
        return path

    stride = max(1, stride)
    reduced = [path[0]]
    reduced.extend(path[i] for i in range(stride, len(path) - 1, stride))
    reduced.append(path[-1])
    return reduced
