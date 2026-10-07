"""Test suite for Engine 7: Collaborative Drone-Van Multi-Modal Dispatch (FSTSP)."""

import unittest
from apex_autonomous_logistics_platform.drone_van import (
    DeliveryStop,
    DroneConfig,
    VanConfig,
    FSTSPMission,
    DroneVanDispatcher,
)


class TestDroneVanDispatcher(unittest.TestCase):
    """Unit tests for Flying Sidekick Traveling Salesperson Problem (FSTSP)."""

    def setUp(self):
        self.van = VanConfig(id="VAN_01", speed=10.0, capacity_kg=1000.0)
        self.drone = DroneConfig(
            id="UAV_01",
            speed=20.0,
            max_payload_kg=3.5,
            battery_endurance_seconds=1200.0,  # 20 minutes
            launch_time_seconds=30.0,
            recovery_time_seconds=30.0,
        )

        # 6 delivery stops: Depot at (0, 0), and 5 customer stops
        # Some stops have light packages eligible for drone delivery
        self.stops = [
            DeliveryStop(id="DEPOT", x=0.0, y=0.0, weight_kg=0.0, is_drone_eligible=False),
            DeliveryStop(id="STOP_1", x=1000.0, y=0.0, weight_kg=15.0, is_drone_eligible=False),
            DeliveryStop(id="STOP_2_DRONE", x=1500.0, y=1000.0, weight_kg=2.0, is_drone_eligible=True),  # Isolated off-route
            DeliveryStop(id="STOP_3", x=2000.0, y=0.0, weight_kg=25.0, is_drone_eligible=False),
            DeliveryStop(id="STOP_4_DRONE", x=2500.0, y=800.0, weight_kg=1.5, is_drone_eligible=True),   # Isolated off-route
            DeliveryStop(id="STOP_5", x=3000.0, y=0.0, weight_kg=10.0, is_drone_eligible=False),
        ]

        self.mission = FSTSPMission(
            depot=self.stops[0],
            customers=self.stops[1:],
            van=self.van,
            drone=self.drone,
        )

    def test_drone_van_dispatch_cuts_makespan_vs_pure_van(self):
        dispatcher = DroneVanDispatcher()
        solution = dispatcher.solve(self.mission)

        self.assertTrue(solution.is_feasible)
        self.assertGreater(len(solution.drone_sorties), 0)

        # Pure van baseline (van visits every stop serially)
        pure_van_time = dispatcher.evaluate_pure_van_baseline(self.mission)

        # Collaborative makespan must be strictly faster
        self.assertLess(solution.mission_makespan_seconds, pure_van_time)
        speedup = pure_van_time / solution.mission_makespan_seconds
        self.assertGreater(speedup, 1.15)  # At least 15% faster

    def test_drone_battery_endurance_not_exceeded(self):
        dispatcher = DroneVanDispatcher()
        solution = dispatcher.solve(self.mission)

        for sortie in solution.drone_sorties:
            self.assertLessEqual(sortie.flight_duration_seconds, self.drone.battery_endurance_seconds)
            self.assertLessEqual(sortie.package_weight_kg, self.drone.max_payload_kg)

    def test_rendezvous_synchronization(self):
        dispatcher = DroneVanDispatcher()
        solution = dispatcher.solve(self.mission)

        for sortie in solution.drone_sorties:
            # Drone rendezvous point must match van arrival
            self.assertEqual(sortie.rendezvous_stop_id, sortie.van_rendezvous_stop_id)
            self.assertGreaterEqual(sortie.rendezvous_time_seconds, sortie.launch_time_seconds)


if __name__ == "__main__":
    unittest.main()
