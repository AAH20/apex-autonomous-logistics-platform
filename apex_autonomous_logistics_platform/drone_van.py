"""Engine 7: Collaborative Air-Ground Drone-Van Multi-Modal Dispatch (FSTSP).

Implements:
  - Flying Sidekick Traveling Salesperson Problem (FSTSP).
  - Triplet search <launch i, drone-delivery j, rendezvous k> with battery feasibility.
  - Parallel vehicle-drone makespan synchronization: max(t_van, t_drone).
  - Comparative speedup benchmarking against traditional pure-van route.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class DeliveryStop:
    """Ground stop for van and/or autonomous aerial drone."""
    id: str
    x: float
    y: float
    weight_kg: float
    is_drone_eligible: bool = False


@dataclass(frozen=True)
class VanConfig:
    """Ground delivery vehicle specifications."""
    id: str
    speed: float = 10.0  # m/s (~36 km/h urban speed)
    capacity_kg: float = 1000.0


@dataclass(frozen=True)
class DroneConfig:
    """Autonomous quadcopter/eVTOL specs."""
    id: str
    speed: float = 20.0  # m/s (~72 km/h cruise speed)
    max_payload_kg: float = 3.5
    battery_endurance_seconds: float = 1200.0  # 20 minutes
    launch_time_seconds: float = 30.0
    recovery_time_seconds: float = 30.0


@dataclass
class DroneSortie:
    """Aerial delivery sortie executed by the drone while the van proceeds."""
    drone_id: str
    launch_stop_id: str
    customer_stop_id: str
    rendezvous_stop_id: str
    package_weight_kg: float
    flight_duration_seconds: float
    launch_time_seconds: float
    rendezvous_time_seconds: float
    van_rendezvous_stop_id: str


@dataclass
class FSTSPMission:
    """Mission definition for collaborative air-ground parcel delivery."""
    depot: DeliveryStop
    customers: list[DeliveryStop]
    van: VanConfig
    drone: DroneConfig

    def get_stop(self, stop_id: str) -> DeliveryStop:
        if self.depot.id == stop_id:
            return self.depot
        for c in self.customers:
            if c.id == stop_id:
                return c
        raise ValueError(f"Stop '{stop_id}' not found.")


@dataclass
class FSTSPSolution:
    """Synchronized ground route and aerial sorties."""
    van_route: list[str]
    drone_sorties: list[DroneSortie]
    mission_makespan_seconds: float
    pure_van_baseline_seconds: float
    speedup_factor: float
    is_feasible: bool


class DroneVanDispatcher:
    """Solves the Flying Sidekick Traveling Salesperson Problem."""

    def solve(self, mission: FSTSPMission) -> FSTSPSolution:
        """Construct synchronized van route with parallel drone sorties."""
        # 1. Separate drone-eligible off-route stops from primary van stops
        drone_eligible = [
            c for c in mission.customers
            if c.is_drone_eligible and c.weight_kg <= mission.drone.max_payload_kg
        ]

        # Initial van stops: depot -> heavy/ground stops -> depot
        van_stops = [mission.depot] + [c for c in mission.customers if c not in drone_eligible] + [mission.depot]

        drone_sorties: list[DroneSortie] = []
        assigned_drone_stops: set[str] = set()

        # Forward timetable simulation for van stops: departure times by stop index
        van_dep_times: list[float] = [0.0] * len(van_stops)
        curr_t = 0.0
        for i in range(1, len(van_stops)):
            prev = van_stops[i - 1]
            curr = van_stops[i]
            dist = math.hypot(curr.x - prev.x, curr.y - prev.y)
            curr_t += dist / mission.van.speed
            van_dep_times[i] = curr_t + 30.0
            curr_t = van_dep_times[i]

        last_rendezvous_idx = 0
        for cand in drone_eligible:
            best_sortie = None
            best_idx_pair = None

            for i in range(last_rendezvous_idx, len(van_stops) - 2):
                stop_i = van_stops[i]
                for k in range(i + 1, min(i + 3, len(van_stops) - 1)):
                    stop_k = van_stops[k]

                    d_i_cand = math.hypot(cand.x - stop_i.x, cand.y - stop_i.y)
                    d_cand_k = math.hypot(stop_k.x - cand.x, stop_k.y - cand.y)
                    flight_dist = d_i_cand + d_cand_k

                    flight_time = (
                        mission.drone.launch_time_seconds +
                        (flight_dist / mission.drone.speed) +
                        mission.drone.recovery_time_seconds
                    )

                    if flight_time > mission.drone.battery_endurance_seconds:
                        continue

                    launch_time = van_dep_times[i]
                    drone_arrival_k = launch_time + flight_time
                    rendezvous_t = max(van_dep_times[k] - 30.0, drone_arrival_k)

                    sortie = DroneSortie(
                        drone_id=mission.drone.id,
                        launch_stop_id=stop_i.id,
                        customer_stop_id=cand.id,
                        rendezvous_stop_id=stop_k.id,
                        package_weight_kg=cand.weight_kg,
                        flight_duration_seconds=flight_time,
                        launch_time_seconds=launch_time,
                        rendezvous_time_seconds=rendezvous_t,
                        van_rendezvous_stop_id=stop_k.id,
                    )
                    best_sortie = sortie
                    best_idx_pair = (i, k)
                    break
                if best_sortie:
                    break

            if best_sortie and best_idx_pair:
                drone_sorties.append(best_sortie)
                assigned_drone_stops.add(cand.id)
                last_rendezvous_idx = best_idx_pair[1]

        # Any unassigned drone stops are inserted into van route
        final_van_stops = list(van_stops[:-1])
        for cand in drone_eligible:
            if cand.id not in assigned_drone_stops:
                final_van_stops.append(cand)
        final_van_stops.append(mission.depot)

        # Compute synchronized timeline where van waits if drone arrives later than van
        curr_t = 0.0
        for i in range(1, len(final_van_stops)):
            prev = final_van_stops[i - 1]
            curr = final_van_stops[i]
            dist = math.hypot(curr.x - prev.x, curr.y - prev.y)
            van_arrival = curr_t + dist / mission.van.speed

            drone_arrival = 0.0
            for s in drone_sorties:
                if s.rendezvous_stop_id == curr.id:
                    drone_arrival = max(drone_arrival, s.launch_time_seconds + s.flight_duration_seconds)

            effective_arrival = max(van_arrival, drone_arrival)
            curr_t = effective_arrival + (30.0 if i < len(final_van_stops) - 1 else 0.0)

        makespan = curr_t

        pure_van_baseline = self.evaluate_pure_van_baseline(mission)
        speedup = pure_van_baseline / makespan if makespan > 0 else 1.0

        return FSTSPSolution(
            van_route=[s.id for s in final_van_stops],
            drone_sorties=drone_sorties,
            mission_makespan_seconds=makespan,
            pure_van_baseline_seconds=pure_van_baseline,
            speedup_factor=speedup,
            is_feasible=True,
        )

    def evaluate_pure_van_baseline(self, mission: FSTSPMission) -> float:
        """Compute total travel and service time when the delivery van alone visits all customers."""
        sorted_custs = sorted(mission.customers, key=lambda c: (c.x, c.y))
        all_stops = [mission.depot] + sorted_custs + [mission.depot]

        total_dist = 0.0
        for i in range(len(all_stops) - 1):
            s1, s2 = all_stops[i], all_stops[i + 1]
            total_dist += math.hypot(s2.x - s1.x, s2.y - s1.y)

        # Driving time + 30s handling per customer stop
        return (total_dist / mission.van.speed) + (len(mission.customers) * 30.0)

    def _compute_van_timeline(
        self,
        stops: list[DeliveryStop],
        van_speed: float,
    ) -> dict[str, dict[str, float]]:
        """Compute arrival and departure times for each stop along the van route."""
        timeline: dict[str, dict[str, float]] = {}
        curr_t = 0.0

        for i, stop in enumerate(stops):
            if i == 0:
                timeline[stop.id] = {"arrival": 0.0, "departure": 0.0}
            else:
                prev = stops[i - 1]
                dist = math.hypot(stop.x - prev.x, stop.y - prev.y)
                travel_t = dist / van_speed
                arrival = curr_t + travel_t
                departure = arrival + 30.0  # 30s handling
                timeline[stop.id] = {"arrival": arrival, "departure": departure}
                curr_t = departure

        return timeline
