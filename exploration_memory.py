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

            visit_count = self.visited_cells.get(cell, 0)

            if visit_count == 0:
                score += self.novelty_weight
            else:
                score -= self.revisit_penalty * visit_count

        return score

    def projected_path_score(
        self,
        current_n,
        current_e,
        candidate_heading_deg,
        step_m=1.0,
        steps=4,
    ):
        """
        Lightweight lookahead score for frontier ranking.

        It rewards cells that have not been visited yet, penalizes repeated cells,
        and applies a strong penalty if the path projects into known blocked cells.
        """
        score = 0.0
        new_cells = 0
        revisits = 0
        blocked = 0

        for i in range(1, max(1, steps) + 1):
            check_n, check_e = self._point_from(
                current_n,
                current_e,
                candidate_heading_deg,
                step_m * i,
            )
            cell = self._cell(check_n, check_e)

            if cell in self.blocked_cells:
                blocked += 1
                score -= self.blocked_penalty / i
                break

            visit_count = self.visited_cells.get(cell, 0)

            if visit_count == 0:
                new_cells += 1
                score += self.novelty_weight * (1.0 + 0.15 * i)
            else:
                revisits += 1
                score -= self.revisit_penalty * visit_count / i

        return score, {
            "new_cells": new_cells,
            "revisits": revisits,
            "blocked": blocked,
        }

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
