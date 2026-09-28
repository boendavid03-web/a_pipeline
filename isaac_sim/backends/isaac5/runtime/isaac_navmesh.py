#!/usr/bin/env python3
"""Fail-closed Isaac NavMesh queries for crowd route and velocity validation."""

from __future__ import annotations

import asyncio
import math


class IsaacNavMeshGuard:
    def __init__(
        self,
        navigation,
        navmesh,
        carb_module,
        radius_m: float = 0.25,
        audit_radius_m: float | None = None,
    ):
        self.navigation = navigation
        self.navmesh = navmesh
        self.carb = carb_module
        self.radius_m = float(radius_m)
        self.audit_radius_m = float(
            radius_m if audit_radius_m is None else audit_radius_m
        )
        self.path_query_count = 0
        self.rejected_segment_count = 0
        self.rejected_segment_samples = []
        self.projected_step_count = 0
        self.projected_step_samples = []
        self.projected_step_failure_count = 0
        self.projected_step_failure_samples = []
        self.replan_failure_count = 0
        self.maximum_goal_projection_m = 0.0
        self.maximum_start_projection_m = 0.0

    def _reject_projected_step(self, start, end, goal, reason: str, **details):
        self.projected_step_failure_count += 1
        if len(self.projected_step_failure_samples) < 50:
            self.projected_step_failure_samples.append(
                {
                    "start": start,
                    "requested_end": end,
                    "goal": goal,
                    "reason": reason,
                    **details,
                }
            )
        return None

    @classmethod
    async def bake(
        cls,
        radius_m: float = 0.25,
        audit_radius_m: float | None = None,
        timeout_sec: float = 30.0,
    ):
        import carb
        import omni.anim.navigation.core as nav

        navigation = nav.acquire_interface()
        stream = navigation.get_navmesh_event_stream()
        loop = asyncio.get_running_loop()
        future = loop.create_future()

        def on_event(event):
            if event.type == nav.EVENT_TYPE_NAVMESH_UPDATED and not future.done():
                future.set_result(True)

        subscription = stream.create_subscription_to_pop(on_event, name="a_pipeline_hunav_bake")
        if not navigation.start_navmesh_baking():
            raise RuntimeError("Isaac NavMesh bake refused to start")
        await asyncio.wait_for(future, timeout=timeout_sec)
        del subscription
        navmesh = navigation.get_navmesh()
        if navmesh is None:
            raise RuntimeError("Isaac NavMesh bake completed without a NavMesh")
        return cls(navigation, navmesh, carb, radius_m, audit_radius_m)

    def closest(self, point, tolerance_m: float = 0.10) -> tuple[float, float, float]:
        query = self.carb.Float3(float(point[0]), float(point[1]), float(point[2]))
        result = self.navmesh.query_closest_point(query, agent_radius=self.radius_m)
        if not result or result[0] is None:
            raise RuntimeError(f"no NavMesh point for {tuple(point)}")
        value = result[0]
        projected = (float(value[0]), float(value[1]), float(value[2]))
        projection = math.dist(tuple(float(x) for x in point), projected)
        if projection > tolerance_m:
            raise RuntimeError(f"Arena point is not on NavMesh: requested={tuple(point)} projected={projected}")
        return projected

    def path(self, start, end, end_tolerance_m: float = 0.10) -> tuple[tuple[float, float, float], ...]:
        self.path_query_count += 1
        start_point = self.closest(start)
        end_point = self.closest(end, tolerance_m=end_tolerance_m)
        result = self.navmesh.query_shortest_path(
            self.carb.Float3(*start_point), self.carb.Float3(*end_point), agent_radius=self.radius_m
        )
        if result is None or not result.get_points():
            raise RuntimeError(f"no NavMesh path from {start_point} to {end_point}")
        return tuple((float(p[0]), float(p[1]), float(p[2])) for p in result.get_points())

    def resolve_route(self, route) -> tuple[tuple[float, float, float], ...]:
        resolved = []
        current = self.closest(route[0], tolerance_m=0.50)
        self.maximum_start_projection_m = max(
            self.maximum_start_projection_m, math.dist(tuple(route[0]), current)
        )
        for end in route[1:]:
            leg = self.path(current, end, end_tolerance_m=0.50)
            projected_end = self.closest(end, tolerance_m=0.50)
            self.maximum_goal_projection_m = max(
                self.maximum_goal_projection_m, math.dist(tuple(end), projected_end)
            )
            resolved.extend(leg if not resolved else leg[1:])
            if math.dist(resolved[-1], projected_end) > 1.0e-4:
                resolved.append(projected_end)
            current = projected_end
        return tuple(resolved)

    def segment_is_safe(self, start_xy, end_xy) -> bool:
        start = (float(start_xy[0]), float(start_xy[1]), 0.0)
        end = (float(end_xy[0]), float(end_xy[1]), 0.0)
        reason = None
        path_end = None
        try:
            # Runtime steps are only a few centimetres and start from the
            # previously accepted point.  Endpoint containment is sufficient
            # here; full shortest-path queries remain in route preflight and
            # recovery replanning where connectivity actually changes.
            path_end = self.closest(end)
            safe = math.dist(path_end[:2], end[:2]) <= 0.05
            if not safe:
                reason = "path_endpoint_mismatch"
        except RuntimeError as exc:
            safe = False
            reason = str(exc)
        if not safe:
            self.rejected_segment_count += 1
            if len(self.rejected_segment_samples) < 50:
                self.rejected_segment_samples.append(
                    {
                        "start": start,
                        "end": end,
                        "path_end": path_end,
                        "reason": reason,
                    }
                )
        return safe

    def project_step(self, start_xy, end_xy, goal_xy):
        """Keep a short requested step on the NavMesh without freezing at corners.

        HuNav returns the next integrated position, not a NavMesh path.  When
        that point cuts the inside of a corner, follow the current validated
        local route corner using the same step length.  ``SemanticRouteRuntime``
        already stores radius-aware shortest-path corners, so recomputing a
        full shortest path for every projected control sample is redundant and
        can make dense runtime recovery unboundedly expensive.
        """
        start = (float(start_xy[0]), float(start_xy[1]), 0.0)
        end = (float(end_xy[0]), float(end_xy[1]), 0.0)
        goal = (float(goal_xy[0]), float(goal_xy[1]), 0.0)
        requested_step = math.dist(start[:2], end[:2])
        if requested_step <= 1.0e-9:
            return end[:2]

        try:
            # The lookup tolerance is only a recovery search radius.  The
            # returned pose is still constrained to the NavMesh.  One capsule
            # radius lets a previously integrated corner cut recover instead
            # of becoming permanently fail-closed just beyond 0.10 m.
            projected_end = self.closest(end, tolerance_m=self.radius_m)
            if math.dist(projected_end[:2], end[:2]) <= 1.0e-3:
                return end[:2]

            projected_start = self.closest(start, tolerance_m=self.radius_m)
            start_error = math.dist(projected_start[:2], start[:2])
            target = (
                projected_start
                if start_error > 1.0e-3
                else self.closest(goal, tolerance_m=0.50)
            )
            if math.dist(start[:2], target[:2]) <= 1.0e-4:
                target = projected_end

            distance = math.dist(start[:2], target[:2])
            if distance <= 1.0e-9:
                return self._reject_projected_step(
                    start, end, goal, "path_target_has_zero_distance"
                )
            scale = min(1.0, requested_step / distance)
            constrained = (
                start[0] + (target[0] - start[0]) * scale,
                start[1] + (target[1] - start[1]) * scale,
            )

            # query_closest_point is a nearest-point query, not an inside test;
            # at triangle/edge boundaries it can jump to another nearby edge
            # even for a point interpolated along a valid shortest-path
            # segment.  The radius-aware shortest path is therefore the
            # normal-motion authority.  If another constraint left the start
            # away from the returned mesh point, explicitly walk back toward
            # that point and require strict progress on every control step.
            recovery_error = math.dist(constrained, projected_start[:2])
            if start_error > 1.0e-3 and recovery_error >= start_error - 1.0e-5:
                return self._reject_projected_step(
                    start,
                    end,
                    goal,
                    "corrective_step_did_not_reduce_navmesh_error",
                    projected_start=projected_start,
                    path_target=target,
                    constrained_end=constrained,
                    start_error_m=start_error,
                    constrained_error_m=recovery_error,
                )

            self.projected_step_count += 1
            if len(self.projected_step_samples) < 50:
                self.projected_step_samples.append(
                    {
                        "start": start,
                        "requested_end": end,
                        "projected_end": projected_end,
                        "goal": goal,
                        "path_target": target,
                        "constrained_end": constrained,
                    }
                )
            return constrained
        except RuntimeError as exc:
            return self._reject_projected_step(start, end, goal, str(exc))

    def point_is_safe(self, point_xy, tolerance_m: float = 0.05) -> bool:
        """Check an actually committed center point without changing counters."""
        return self.point_projection_error_m(point_xy) <= tolerance_m

    def point_projection_error_m(self, point_xy) -> float:
        """Measure planar distance from a committed centre to the radius-aware mesh."""
        try:
            point = (float(point_xy[0]), float(point_xy[1]), 0.0)
            query = self.carb.Float3(*point)
            result = self.navmesh.query_closest_point(
                query, agent_radius=self.audit_radius_m
            )
            if not result or result[0] is None:
                return math.inf
            projected = result[0]
            return math.dist(point[:2], (float(projected[0]), float(projected[1])))
        except Exception:
            return math.inf

    def replan_path(self, start_xy, goal) -> tuple[tuple[float, float, float], ...] | None:
        """Return replacement corners without allowing a local failure to stop Isaac."""
        try:
            return self.path(
                (float(start_xy[0]), float(start_xy[1]), 0.0),
                goal,
                end_tolerance_m=0.50,
            )
        except RuntimeError:
            self.replan_failure_count += 1
            return None

    def replan_is_available(self, start_xy, goal) -> bool:
        """Compatibility helper for older probes; runtime should use replan_path."""
        return self.replan_path(start_xy, goal) is not None
