import math


def normalize_angle_deg(angle):
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle


def angle_diff_deg(a, b):
    return abs(normalize_angle_deg(a - b))


class ExplorationMemory:
    """
    Lightweight local exploration memory.

    This does not use GNSS. It uses local NED position only.

    Safer behaviour:
    - Path memory does NOT fully control navigation.
    - It only nudges the preferred mission heading.
    - It is prevented from selecting headings too far away from the intended heading.
    - Backward-compatible with older competition_mission.py that may still pass
      preferred_heading_penalty.
    """

    def __init__(
        self,
        cell_size_m=1.0,
        lookahead_m=4.0,
        novelty_weight=2.5,
        revisit_penalty=1.2,
        blocked_penalty=80.0,
        turn_penalty=0.04,
        preferred_heading_penalty=0.10,
        max_heading_deviation_deg=35.0,
    ):
        self.cell_size_m = cell_size_m
        self.lookahead_m = lookahead_m

        self.novelty_weight = novelty_weight
        self.revisit_penalty = revisit_penalty
        self.blocked_penalty = blocked_penalty
        self.turn_penalty = turn_penalty
        self.preferred_heading_penalty = preferred_heading_penalty
        self.max_heading_deviation_deg = max_heading_deviation_deg

        self.start_n = None
        self.start_e = None

        self.visited_cells = {}
        self.blocked_cells = set()
        self.path_history = []

        self.blocked_cells_by_height = {}
        self.height_cell_size_m = 0.5   

        self.obstacle_cells = {}


    def initialize(self, start_n, start_e):
        self.start_n = float(start_n)
        self.start_e = float(start_e)

    def _cell(self, n, e):
        if self.start_n is None or self.start_e is None:
            return (0, 0)

        rel_n = n - self.start_n
        rel_e = e - self.start_e

        return (
            int(round(rel_n / self.cell_size_m)),
            int(round(rel_e / self.cell_size_m)),
        )

    def _point_from(self, n, e, yaw_deg, distance_m):
        yaw_rad = math.radians(yaw_deg)

        return (
            n + distance_m * math.cos(yaw_rad),
            e + distance_m * math.sin(yaw_rad),
        )

    def mark_visited(self, n, e):
        cell = self._cell(n, e)
        self.visited_cells[cell] = self.visited_cells.get(cell, 0) + 1
        self.path_history.append((float(n), float(e)))

    def mark_blocked_point(self, n, e):
        self.blocked_cells.add(self._cell(n, e))

    def mark_blocked_ray(self, n, e, yaw_deg, distance_m):
        distance_m = max(0.5, min(distance_m, self.lookahead_m))
        blocked_n, blocked_e = self._point_from(n, e, yaw_deg, distance_m)
        self.mark_blocked_point(blocked_n, blocked_e)

    # ----------------- Start of Height ------------
    def mark_unblocked_point(self, n, e):
        cell = self._cell(n, e)
        self.blocked_cells.discard(cell)

    def mark_unblocked_ray(self, n, e, yaw_deg, distance_m):
        distance_m = max(0.5, min(distance_m, self.lookahead_m))
        steps = int(distance_m / self.cell_size_m)
        for i in range(1, steps + 1):
            check_n, check_e = self._point_from(n, e, yaw_deg, i * self.cell_size_m)
            self.mark_unblocked_point(check_n, check_e)

    def mark_blocked_ray_at_height(self, n, e, yaw_deg, distance_m, down):
        distance_m = max(0.5, min(distance_m, self.lookahead_m))
        blocked_n, blocked_e = self._point_from(n, e, yaw_deg, distance_m)
        cell = self._cell(blocked_n, blocked_e)
        if cell not in self.blocked_cells_by_height:
            self.blocked_cells_by_height[cell] = set()
        height_key = round(down / self.height_cell_size_m)
        self.blocked_cells_by_height[cell].add(height_key)
        self.blocked_cells.add(cell)

    def refresh_blocked_cells_for_height(self, down):
        height_key = round(down / self.height_cell_size_m)
        to_unblock = set()
        for cell, heights in self.blocked_cells_by_height.items():
            if height_key not in heights:
                to_unblock.add(cell)
        self.blocked_cells -= to_unblock
    # ------------------- End of Height -----------------


    # Marking of obstacle
    def mark_obstacle_at(self, n, e, observed_distance):
        """Store obstacle with the distance it was observed from."""
        cell = self._cell(n, e)
        # Keep the closest observed distance (most conservative)
        existing = self.obstacle_cells.get(cell, float('inf'))
        self.obstacle_cells[cell] = min(existing, observed_distance)

    def get_clearance_to_obstacles(self, n, e, radius_m=3.0):
        """
        Return the minimum distance from point (n,e) to any known obstacle
        within radius_m. Returns radius_m if no obstacles found nearby.
        """
        query_cell = self._cell(n, e)
        radius_cells = int(math.ceil(radius_m / self.cell_size_m))
        min_dist = radius_m

        for dn in range(-radius_cells, radius_cells + 1):
            for de in range(-radius_cells, radius_cells + 1):
                neighbor = (query_cell[0] + dn, query_cell[1] + de)
                if neighbor in self.obstacle_cells:
                    actual_dist = math.sqrt(
                        (dn * self.cell_size_m) ** 2 +
                        (de * self.cell_size_m) ** 2
                    )
                    min_dist = min(min_dist, actual_dist)

        return min_dist

    def heading_score(
        self,
        current_n,
        current_e,
        candidate_heading_deg,
        current_yaw_deg,
        preferred_heading_deg,
        clearances=None,
    ):
        score = 0.0

        deviation = angle_diff_deg(candidate_heading_deg, preferred_heading_deg)

        # Hard clamp: memory cannot hijack the mission direction.
        if deviation > self.max_heading_deviation_deg:
            return -1e9

        # Penalise unnecessary turning.
        score -= self.turn_penalty * angle_diff_deg(candidate_heading_deg, current_yaw_deg)

        # Prefer staying near the intended mission heading.
        score -= self.preferred_heading_penalty * deviation

        # Add live obstacle-clearance preference.
        if clearances is not None:
            rel = normalize_angle_deg(candidate_heading_deg - current_yaw_deg)

            if abs(rel) <= 25:
                score += min(clearances.get("center", 0.0), 8.0) * 1.4
            elif rel > 0:
                score += min(clearances.get("right", 0.0), 8.0) * 0.9
            else:
                score += min(clearances.get("left", 0.0), 8.0) * 0.9

        steps = int(self.lookahead_m / self.cell_size_m)

        for i in range(1, steps + 1):
            distance = i * self.cell_size_m

            check_n, check_e = self._point_from(
                current_n,
                current_e,
                candidate_heading_deg,
                distance,
            )

            cell = self._cell(check_n, check_e)

            if cell in self.blocked_cells:
                score -= self.blocked_penalty / i
                break

            # NEW: penalise proximity to known obstacles along this heading
            clearance_to_obs = self.get_clearance_to_obstacles(check_n, check_e, radius_m=2.0)
            if clearance_to_obs < 1.5:
                score -= self.blocked_penalty * 0.5 / i
                break
            elif clearance_to_obs < 2.5:
                score -= 10.0 * (2.5 - clearance_to_obs)

            visit_count = self.visited_cells.get(cell, 0)

            if visit_count == 0:
                score += self.novelty_weight
            else:
                score -= self.revisit_penalty * visit_count

        return score

    def choose_heading(
        self,
        current_n,
        current_e,
        current_yaw_deg,
        preferred_heading_deg,
        clearances=None,
    ):
        """
        Choose a safe heading near the preferred mission heading.

        It only tries small offsets around the preferred heading.
        """
        preferred_heading_deg = normalize_angle_deg(preferred_heading_deg)

        offsets = [0, 15, -15, 25, -25, 35, -35]

        best_heading = preferred_heading_deg
        best_score = -1e9

        for offset in offsets:
            heading = normalize_angle_deg(preferred_heading_deg + offset)

            score = self.heading_score(
                current_n=current_n,
                current_e=current_e,
                candidate_heading_deg=heading,
                current_yaw_deg=current_yaw_deg,
                preferred_heading_deg=preferred_heading_deg,
                clearances=clearances,
            )

            if score > best_score:
                best_score = score
                best_heading = heading

        return best_heading, best_score

    def debug_summary(self):
        return {
            "visited_cells": len(self.visited_cells),
            "blocked_cells": len(self.blocked_cells),
            "path_points": len(self.path_history),
        }
    
    def print_map(self, title="Exploration Map"):
        """Print an ASCII map of visited and blocked cells to the terminal."""
        if not self.visited_cells and not self.blocked_cells and not self.obstacle_cells:
            print(f"\n[{title}] No data recorded.")
            return

        all_cells = (
            set(self.visited_cells.keys())
            | self.blocked_cells
            | set(self.obstacle_cells.keys())
        )

        if not all_cells:
            return

        ns = [c[0] for c in all_cells]
        es = [c[1] for c in all_cells]
        n_min, n_max = min(ns), max(ns)
        e_min, e_max = min(es), max(es)

        # Pad by 1 cell on all sides
        n_min -= 1; n_max += 1
        e_min -= 1; e_max += 1

        LEGEND = {
            "visited_once":  "·",   # visited 1x
            "visited_multi": "o",   # visited 2x
            "visited_heavy": "O",   # visited 3+x
            "blocked":       "█",   # hard blocked
            "obstacle":      "X",   # obstacle (softer)
            "origin":        "@",   # start position
            "empty":         " ",
        }

        print(f"\n{'='*60}")
        print(f"  {title}")
        print(f"  Cells — visited: {len(self.visited_cells)}  "
            f"blocked: {len(self.blocked_cells)}  "
            f"obstacles: {len(self.obstacle_cells)}")
        print(f"  North range: {n_min}..{n_max}  East range: {e_min}..{e_max}")
        print(f"{'='*60}")

        # North increases upward, so iterate n_max → n_min
        for n in range(n_max, n_min - 1, -1):
            row = []
            for e in range(e_min, e_max + 1):
                cell = (n, e)
                if cell == (0, 0):
                    row.append(LEGEND["origin"])
                elif cell in self.blocked_cells:
                    row.append(LEGEND["blocked"])
                elif cell in self.obstacle_cells:
                    row.append(LEGEND["obstacle"])
                elif cell in self.visited_cells:
                    count = self.visited_cells[cell]
                    if count >= 3:
                        row.append(LEGEND["visited_heavy"])
                    elif count == 2:
                        row.append(LEGEND["visited_multi"])
                    else:
                        row.append(LEGEND["visited_once"])
                else:
                    row.append(LEGEND["empty"])
            print("  " + " ".join(row))

        print(f"{'='*60}")
        print(f"  Legend:  @ origin   · visited   o visited×2   O visited×3+")
        print(f"           X obstacle  █ blocked   (space) unexplored")
        print(f"{'='*60}\n")
