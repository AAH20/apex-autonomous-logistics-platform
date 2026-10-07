"""Engine 5: Dynamic Cross-Docking Terminal Assignment & Flow Shop Scheduler.

Implements:
  - Bilateral Cross-Docking Terminal Assignment Problem (CDTAP).
  - Quadratic assignment heuristic matching high-volume pallet flows to opposite doors.
  - Automated Guided Vehicle (AGV) flow shop makespan scheduler.
  - Outbound linehaul trailer departure deadline verification.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class Door:
    """Dock door on the cross-dock terminal perimeter."""
    id: str
    door_type: str  # "INBOUND" or "OUTBOUND"
    x: float
    y: float


@dataclass(frozen=True)
class InboundTrailer:
    """Arriving trailer carrying palletized freight destined for multiple linehauls."""
    id: str
    arrival_time: float
    pallet_destinations: dict[str, int]  # outbound_trailer_id -> pallet_count

    @property
    def total_pallets(self) -> int:
        return sum(self.pallet_destinations.values())


@dataclass(frozen=True)
class OutboundTrailer:
    """Departing linehaul trailer waiting for pallets at an outbound door."""
    id: str
    departure_deadline: float
    capacity_pallets: int


@dataclass(frozen=True)
class CrossDockTerminal:
    """Physical layout and robotic material handling fleet."""
    id: str
    inbound_doors: list[Door]
    outbound_doors: list[Door]
    num_agvs: int = 6
    agv_speed_m_per_s: float = 2.0


@dataclass
class CrossDockPlan:
    """Optimized dock assignment and material handling execution plan."""
    inbound_assignments: dict[str, str]  # inbound_trailer_id -> door_id
    outbound_assignments: dict[str, str]  # outbound_trailer_id -> door_id
    total_pallet_distance_m: float
    outbound_ready_times: dict[str, float]  # outbound_trailer_id -> finish_timestamp
    agv_utilization_pct: float
    is_feasible: bool


class CrossDockScheduler:
    """Optimizes trailer door assignments and robotic pallet transfers."""

    def optimize_schedule(
        self,
        terminal: CrossDockTerminal,
        inbound_trailers: list[InboundTrailer],
        outbound_trailers: list[OutboundTrailer],
    ) -> CrossDockPlan:
        """Find door assignment that minimizes total transfer distance and execute flow shop."""
        # 1. Solve assignment: Match inbound trailers to inbound doors, outbound trailers to outbound doors
        in_assign, out_assign = self._solve_quadratic_assignment(
            terminal, inbound_trailers, outbound_trailers
        )

        # 2. Compute total pallet travel distance
        door_map = {d.id: d for d in terminal.inbound_doors + terminal.outbound_doors}
        total_pallet_dist = 0.0

        transfer_tasks: list[dict[str, Any]] = []

        for in_tr in inbound_trailers:
            in_door = door_map[in_assign[in_tr.id]]
            for out_tr_id, qty in in_tr.pallet_destinations.items():
                if qty <= 0:
                    continue
                out_door = door_map[out_assign[out_tr_id]]
                dist = math.hypot(in_door.x - out_door.x, in_door.y - out_door.y)
                total_pallet_dist += dist * qty

                # Break into unit transfers for AGV simulation
                transfer_tasks.append({
                    "inbound_trailer": in_tr.id,
                    "outbound_trailer": out_tr_id,
                    "in_door": in_door,
                    "out_door": out_door,
                    "distance_m": dist,
                    "quantity": qty,
                    "available_time": in_tr.arrival_time,
                })

        # 3. Simulate AGV Flow Shop execution
        ready_times, agv_util = self._simulate_agv_flow_shop(
            terminal, transfer_tasks, outbound_trailers
        )

        is_feasible = all(
            ready_times[ot.id] <= ot.departure_deadline
            for ot in outbound_trailers
        )

        return CrossDockPlan(
            inbound_assignments=in_assign,
            outbound_assignments=out_assign,
            total_pallet_distance_m=total_pallet_dist,
            outbound_ready_times=ready_times,
            agv_utilization_pct=agv_util,
            is_feasible=is_feasible,
        )

    def evaluate_naive_distance(
        self,
        terminal: CrossDockTerminal,
        inbound_trailers: list[InboundTrailer],
        outbound_trailers: list[OutboundTrailer],
    ) -> float:
        """Compute worst-case reverse-ordered assignment distance."""
        in_doors_rev = list(reversed(terminal.inbound_doors))
        out_doors = terminal.outbound_doors

        door_map = {d.id: d for d in terminal.inbound_doors + terminal.outbound_doors}

        in_assign = {tr.id: in_doors_rev[i % len(in_doors_rev)].id for i, tr in enumerate(inbound_trailers)}
        out_assign = {tr.id: out_doors[i % len(out_doors)].id for i, tr in enumerate(outbound_trailers)}

        total_dist = 0.0
        for in_tr in inbound_trailers:
            in_door = door_map[in_assign[in_tr.id]]
            for out_tr_id, qty in in_tr.pallet_destinations.items():
                out_door = door_map[out_assign[out_tr_id]]
                dist = math.hypot(in_door.x - out_door.x, in_door.y - out_door.y)
                total_dist += dist * qty
        return total_dist

    def _solve_quadratic_assignment(
        self,
        terminal: CrossDockTerminal,
        inbound_trailers: list[InboundTrailer],
        outbound_trailers: list[OutboundTrailer],
    ) -> tuple[dict[str, str], dict[str, str]]:
        """Greedy bilateral alignment: align high-traffic trailer pairs opposite each other."""
        n_in = min(len(inbound_trailers), len(terminal.inbound_doors))
        n_out = min(len(outbound_trailers), len(terminal.outbound_doors))

        # Sum flow between every (inbound_trailer, outbound_trailer) pair
        flow_pairs: list[tuple[int, str, str]] = []
        for in_tr in inbound_trailers:
            for out_tr in outbound_trailers:
                qty = in_tr.pallet_destinations.get(out_tr.id, 0)
                flow_pairs.append((qty, in_tr.id, out_tr.id))

        flow_pairs.sort(key=lambda x: x[0], reverse=True)

        assigned_in: dict[str, str] = {}
        assigned_out: dict[str, str] = {}
        used_in_doors: set[str] = set()
        used_out_doors: set[str] = set()

        # Find closest opposite door pairs
        door_pairs: list[tuple[float, Door, Door]] = []
        for ind in terminal.inbound_doors:
            for outd in terminal.outbound_doors:
                dist = math.hypot(ind.x - outd.x, ind.y - outd.y)
                door_pairs.append((dist, ind, outd))
        door_pairs.sort(key=lambda x: x[0])

        for qty, in_id, out_id in flow_pairs:
            if qty <= 0:
                continue
            if in_id not in assigned_in and out_id not in assigned_out:
                # Find best available close door pair
                for dist, ind, outd in door_pairs:
                    if ind.id not in used_in_doors and outd.id not in used_out_doors:
                        assigned_in[in_id] = ind.id
                        assigned_out[out_id] = outd.id
                        used_in_doors.add(ind.id)
                        used_out_doors.add(outd.id)
                        break

        # Assign remaining
        for in_tr in inbound_trailers:
            if in_tr.id not in assigned_in:
                avail = next(d.id for d in terminal.inbound_doors if d.id not in used_in_doors)
                assigned_in[in_tr.id] = avail
                used_in_doors.add(avail)

        for out_tr in outbound_trailers:
            if out_tr.id not in assigned_out:
                avail = next(d.id for d in terminal.outbound_doors if d.id not in used_out_doors)
                assigned_out[out_tr.id] = avail
                used_out_doors.add(avail)

        return assigned_in, assigned_out

    def _simulate_agv_flow_shop(
        self,
        terminal: CrossDockTerminal,
        tasks: list[dict[str, Any]],
        outbound_trailers: list[OutboundTrailer],
    ) -> tuple[dict[str, float], float]:
        """Simulate parallel AGV fleet conveying pallets across the facility floor."""
        # Min-heap of AGV available times: (available_time_s, agv_index)
        agv_heap: list[tuple[float, int]] = [(0.0, i) for i in range(terminal.num_agvs)]
        heapq.heapify(agv_heap)

        outbound_finish_times: dict[str, float] = {ot.id: 0.0 for ot in outbound_trailers}
        total_agv_busy_time = 0.0

        for task in tasks:
            qty = task["quantity"]
            dist = task["distance_m"]
            avail_t = task["available_time"]
            out_tr_id = task["outbound_trailer"]

            # Travel time per pallet roundtrip: travel forward with load + return empty
            trip_time = (dist / terminal.agv_speed_m_per_s) * 1.5 + 0.1  # 0.1s handling

            for _ in range(qty):
                earliest_agv_t, agv_idx = heapq.heappop(agv_heap)
                start_t = max(earliest_agv_t, avail_t)
                finish_t = start_t + trip_time

                total_agv_busy_time += trip_time
                outbound_finish_times[out_tr_id] = max(outbound_finish_times[out_tr_id], finish_t)
                heapq.heappush(agv_heap, (finish_t, agv_idx))

        makespan = max(t for t, _ in agv_heap)
        total_capacity = makespan * terminal.num_agvs if makespan > 0 else 1.0
        utilization = min(100.0, (total_agv_busy_time / total_capacity) * 100.0)

        return outbound_finish_times, utilization
