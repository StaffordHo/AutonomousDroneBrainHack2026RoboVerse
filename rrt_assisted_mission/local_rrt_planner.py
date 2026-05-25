import math
import random


def normalize_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def heading_between_deg(start, end):
    return normalize_angle_deg(
        math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
    )


def distance_2d(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class LocalRRTPlanner:
    """
    Small local RRT helper for N/E local-position navigation.

    This is intentionally conservative: it plans only a short local path through
    remembered blocked cells and uses live depth clearance for the first segment.
    The main frontier planner remains in charge of mission intent.
    """

    def __init__(
        self,
        max_range_m=7.0,
        step_m=0.8,
        max_iterations=140,
        goal_sample_rate=0.28,
        goal_tolerance_m=1.0,
        obstacle_radius_m=0.85,
        first_segment_buffer_m=0.35,
        sample_spacing_m=0.25,
        seed=7,
    ):
        self.max_range_m = float(max_range_m)
        self.step_m = float(step_m)
        self.max_iterations = int(max_iterations)
        self.goal_sample_rate = float(goal_sample_rate)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self.obstacle_radius_m = float(obstacle_radius_m)
        self.first_segment_buffer_m = float(first_segment_buffer_m)
        self.sample_spacing_m = float(sample_spacing_m)
        self.rng = random.Random(seed)

    def plan(
        self,
        start,
        goal,
        blocked_points=None,
        clearance_provider=None,
        score_point=None,
        allow_partial=True,
    ):
        start = (float(start[0]), float(start[1]))
        goal = self._clamp_to_range(start, (float(goal[0]), float(goal[1])))
        blocked_points = list(blocked_points or [])

        if distance_2d(start, goal) < 0.2:
            return [start, goal]

        nodes = [{"point": start, "parent": None, "score": 0.0}]
        best_index = 0
        best_score = self._progress_score(start, goal, score_point)

        for _ in range(self.max_iterations):
            sample = goal if self.rng.random() < self.goal_sample_rate else self._sample(start)
            nearest_index = self._nearest_index(nodes, sample)
            nearest = nodes[nearest_index]["point"]
            new_point = self._steer(nearest, sample)

            if not self._inside_range(start, new_point):
                continue

            first_segment = nearest_index == 0
            if not self._segment_is_free(
                nearest,
                new_point,
                blocked_points,
                clearance_provider if first_segment else None,
            ):
                continue

            node_score = self._progress_score(new_point, goal, score_point)
            nodes.append({"point": new_point, "parent": nearest_index, "score": node_score})
            new_index = len(nodes) - 1

            if node_score > best_score:
                best_score = node_score
                best_index = new_index

            if distance_2d(new_point, goal) <= self.goal_tolerance_m and self._segment_is_free(
                new_point,
                goal,
                blocked_points,
                None,
            ):
                nodes.append({"point": goal, "parent": new_index, "score": node_score})
                return self._reconstruct(nodes, len(nodes) - 1)

        if allow_partial and best_index != 0:
            return self._reconstruct(nodes, best_index)

        return None

    def _progress_score(self, point, goal, score_point):
        score = -distance_2d(point, goal)
        if score_point is not None:
            score += float(score_point(point[0], point[1]))
        return score

    def _sample(self, start):
        radius = self.max_range_m * math.sqrt(self.rng.random())
        angle = self.rng.uniform(-math.pi, math.pi)
        return (
            start[0] + radius * math.cos(angle),
            start[1] + radius * math.sin(angle),
        )

    def _clamp_to_range(self, start, goal):
        distance = distance_2d(start, goal)
        if distance <= self.max_range_m:
            return goal

        ratio = self.max_range_m / max(distance, 1e-6)
        return (
            start[0] + (goal[0] - start[0]) * ratio,
            start[1] + (goal[1] - start[1]) * ratio,
        )

    def _inside_range(self, start, point):
        return distance_2d(start, point) <= self.max_range_m + 0.05

    def _nearest_index(self, nodes, sample):
        best_index = 0
        best_distance = float("inf")

        for index, node in enumerate(nodes):
            current_distance = distance_2d(node["point"], sample)
            if current_distance < best_distance:
                best_distance = current_distance
                best_index = index

        return best_index

    def _steer(self, start, target):
        distance = distance_2d(start, target)
        if distance <= self.step_m:
            return target

        ratio = self.step_m / max(distance, 1e-6)
        return (
            start[0] + (target[0] - start[0]) * ratio,
            start[1] + (target[1] - start[1]) * ratio,
        )

    def _segment_is_free(self, start, end, blocked_points, clearance_provider):
        distance = distance_2d(start, end)
        if distance <= 0.01:
            return True

        if clearance_provider is not None:
            heading = heading_between_deg(start, end)
            live_clearance = clearance_provider(heading)
            if live_clearance < distance + self.first_segment_buffer_m:
                return False

        steps = max(1, int(math.ceil(distance / self.sample_spacing_m)))
        for step in range(steps + 1):
            t = step / steps
            point = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
            )
            for blocked in blocked_points:
                if distance_2d(point, blocked) <= self.obstacle_radius_m:
                    return False

        return True

    def _reconstruct(self, nodes, index):
        path = []
        while index is not None:
            path.append(nodes[index]["point"])
            index = nodes[index]["parent"]
        path.reverse()
        return path
