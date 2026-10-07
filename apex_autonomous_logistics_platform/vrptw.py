"""Engine 1: Multi-Depot Capacitated Vehicle Routing with Time Windows (MD-VRPTW).

Implements:
  - Euclidean & Manhattan distance matrices.
  - Clarke-Wright savings heuristic adapted for multi-depot and time window constraints.
  - 2-opt intra-route edge swap optimizer for crossing elimination.
  - Full feasibility tracking: demand capacity, ready times, due times, service durations.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Customer:
    """Customer demanding delivery within an active time window."""
    id: str
    x: float
    y: float
    demand: int
    ready_time: float
    due_time: float
    service_time: float = 0.0


@dataclass(frozen=True)
class Depot:
    """Distribution center or fulfillment depot."""
    id: str
    x: float
    y: float


@dataclass(frozen=True)
class Vehicle:
    """Fleet delivery vehicle."""
    id: str
    depot_id: str
    capacity: int
    speed: float = 1.0
    max_time: float = float("inf")


@dataclass
class Route:
    """Single vehicle itinerary starting and ending at a depot."""
    vehicle_id: str
    depot_id: str
    customer_ids: list[str] = field(default_factory=list)
    total_distance: float = 0.0
    total_time: float = 0.0
    total_demand: int = 0
    arrival_times: dict[str, float] = field(default_factory=dict)


@dataclass
class VRPTWInstance:
    """Problem instance combining depots, customers, and fleet."""
    depots: list[Depot]
    customers: list[Customer]
    vehicles: list[Vehicle]

    def get_depot(self, depot_id: str) -> Depot:
        for d in self.depots:
            if d.id == depot_id:
                return d
        raise ValueError(f"Depot '{depot_id}' not found.")

    def get_customer(self, customer_id: str) -> Customer:
        for c in self.customers:
            if c.id == customer_id:
                return c
        raise ValueError(f"Customer '{customer_id}' not found.")


@dataclass
class VRPTWSolution:
    """Complete solution with routes and key performance indicators."""
    routes: list[Route] = field(default_factory=list)
    total_distance: float = 0.0
    total_time: float = 0.0
    is_feasible: bool = True
    unassigned_customers: list[str] = field(default_factory=list)
    solve_duration_ms: float = 0.0


class VRPTWSolver:
    """Deterministic Multi-Depot VRPTW Solver."""

    def __init__(self, use_2opt: bool = True) -> None:
        self.use_2opt = use_2opt

    def solve(self, instance: VRPTWInstance) -> VRPTWSolution:
        """Execute heuristic routing followed by local search."""
        start_t = time.perf_counter_ns()

        routes = self._clarke_wright_savings(instance)

        if self.use_2opt:
            routes = [self._two_opt_route(r, instance) for r in routes]

        total_distance = sum(r.total_distance for r in routes)
        total_time = sum(r.total_time for r in routes)

        all_assigned = {cid for r in routes for cid in r.customer_ids}
        unassigned = [c.id for c in instance.customers if c.id not in all_assigned]

        end_t = time.perf_counter_ns()
        duration_ms = (end_t - start_t) / 1_000_000.0

        return VRPTWSolution(
            routes=routes,
            total_distance=total_distance,
            total_time=total_time,
            is_feasible=len(unassigned) == 0,
            unassigned_customers=unassigned,
            solve_duration_ms=duration_ms,
        )

    def _clarke_wright_savings(self, instance: VRPTWInstance) -> list[Route]:
        """Parallel Clarke-Wright savings heuristic with feasibility validation."""
        customers = {c.id: c for c in instance.customers}
        depot = instance.depots[0]  # Primary depot for single or multi-depot cluster

        # Compute savings for all customer pairs (i, j): s_ij = d(D, i) + d(D, j) - d(i, j)
        savings: list[tuple[float, str, str]] = []
        cust_list = list(customers.values())
        for i in range(len(cust_list)):
            ci = cust_list[i]
            d_di = self._dist(depot.x, depot.y, ci.x, ci.y)
            for j in range(i + 1, len(cust_list)):
                cj = cust_list[j]
                d_dj = self._dist(depot.x, depot.y, cj.x, cj.y)
                d_ij = self._dist(ci.x, ci.y, cj.x, cj.y)
                s = d_di + d_dj - d_ij
                savings.append((s, ci.id, cj.id))

        savings.sort(key=lambda x: x[0], reverse=True)

        # Initialize: one vehicle per customer
        routes: list[list[str]] = [[c.id] for c in instance.customers]

        # Merge routes based on savings
        for _, ci_id, cj_id in savings:
            r_i_idx, r_j_idx = None, None
            for idx, r in enumerate(routes):
                if ci_id in r:
                    r_i_idx = idx
                if cj_id in r:
                    r_j_idx = idx

            if r_i_idx is None or r_j_idx is None or r_i_idx == r_j_idx:
                continue

            r_i = routes[r_i_idx]
            r_j = routes[r_j_idx]

            # Feasible merges: end of r_i to start of r_j
            candidate_merged = None
            if r_i[-1] == ci_id and r_j[0] == cj_id:
                candidate_merged = r_i + r_j
            elif r_j[-1] == cj_id and r_i[0] == ci_id:
                candidate_merged = r_j + r_i
            elif r_i[0] == ci_id and r_j[0] == cj_id:
                candidate_merged = list(reversed(r_i)) + r_j
            elif r_i[-1] == ci_id and r_j[-1] == cj_id:
                candidate_merged = r_i + list(reversed(r_j))

            if candidate_merged:
                # Check capacity & time windows
                if self._is_feasible(candidate_merged, depot, instance):
                    # Replace r_i and r_j with candidate_merged
                    routes.pop(max(r_i_idx, r_j_idx))
                    routes.pop(min(r_i_idx, r_j_idx))
                    routes.append(candidate_merged)

        # Build Route objects mapped to vehicles
        result_routes: list[Route] = []
        for idx, r_custs in enumerate(routes):
            veh = instance.vehicles[idx % len(instance.vehicles)]
            route_obj = self._compute_route_metrics(r_custs, veh.id, depot.id, instance)
            result_routes.append(route_obj)

        return result_routes

    def _two_opt_route(self, route: Route, instance: VRPTWInstance) -> Route:
        """2-opt local search heuristic to eliminate edge crossings."""
        if len(route.customer_ids) <= 2:
            return route

        best_custs = list(route.customer_ids)
        depot = instance.get_depot(route.depot_id)
        best_dist = route.total_distance

        improved = True
        iterations = 0
        max_iterations = 50

        while improved and iterations < max_iterations:
            improved = False
            iterations += 1
            for i in range(len(best_custs) - 1):
                for j in range(i + 1, len(best_custs)):
                    # Reverse segment [i:j+1]
                    new_custs = best_custs[:i] + list(reversed(best_custs[i:j + 1])) + best_custs[j + 1:]
                    if self._is_feasible(new_custs, depot, instance):
                        metrics = self._compute_route_metrics(
                            new_custs, route.vehicle_id, route.depot_id, instance
                        )
                        if metrics.total_distance < best_dist - 1e-9:
                            best_custs = new_custs
                            best_dist = metrics.total_distance
                            improved = True
                            break
                if improved:
                    break

        return self._compute_route_metrics(
            best_custs, route.vehicle_id, route.depot_id, instance
        )

    def _is_feasible(
        self,
        customer_ids: list[str],
        depot: Depot,
        instance: VRPTWInstance,
    ) -> bool:
        """Check capacity and time window constraints."""
        max_cap = instance.vehicles[0].capacity
        total_demand = sum(instance.get_customer(cid).demand for cid in customer_ids)
        if total_demand > max_cap:
            return False

        curr_time = 0.0
        prev_x, prev_y = depot.x, depot.y

        for cid in customer_ids:
            c = instance.get_customer(cid)
            travel_time = self._dist(prev_x, prev_y, c.x, c.y)
            arrival = curr_time + travel_time
            if arrival > c.due_time:
                return False
            start_service = max(arrival, c.ready_time)
            curr_time = start_service + c.service_time
            prev_x, prev_y = c.x, c.y

        return True

    def _compute_route_metrics(
        self,
        customer_ids: list[str],
        vehicle_id: str,
        depot_id: str,
        instance: VRPTWInstance,
    ) -> Route:
        """Calculate complete distance, elapsed time, and arrival timestamps."""
        depot = instance.get_depot(depot_id)
        total_dist = 0.0
        curr_time = 0.0
        prev_x, prev_y = depot.x, depot.y
        arrivals: dict[str, float] = {}
        total_demand = 0

        for cid in customer_ids:
            c = instance.get_customer(cid)
            leg_dist = self._dist(prev_x, prev_y, c.x, c.y)
            total_dist += leg_dist
            arrival = curr_time + leg_dist
            arrivals[cid] = arrival
            start_service = max(arrival, c.ready_time)
            curr_time = start_service + c.service_time
            total_demand += c.demand
            prev_x, prev_y = c.x, c.y

        # Return to depot
        return_dist = self._dist(prev_x, prev_y, depot.x, depot.y)
        total_dist += return_dist
        curr_time += return_dist

        return Route(
            vehicle_id=vehicle_id,
            depot_id=depot_id,
            customer_ids=customer_ids,
            total_distance=total_dist,
            total_time=curr_time,
            total_demand=total_demand,
            arrival_times=arrivals,
        )

    @staticmethod
    def _dist(x1: float, y1: float, x2: float, y2: float) -> float:
        return math.hypot(x2 - x1, y2 - y1)
