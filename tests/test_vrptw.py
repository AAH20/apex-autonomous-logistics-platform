"""Test suite for Engine 1: Multi-Depot VRPTW Solver."""

import unittest
from apex_autonomous_logistics_platform.vrptw import (
    Customer,
    Depot,
    Vehicle,
    VRPTWInstance,
    VRPTWSolver,
)


class TestVRPTWSolver(unittest.TestCase):
    """Unit tests for Multi-Depot Capacitated Vehicle Routing with Time Windows."""

    def setUp(self):
        self.depot = Depot(id="DEPOT_1", x=0.0, y=0.0)
        self.customers = [
            Customer(id="C1", x=2.0, y=3.0, demand=15, ready_time=5.0, due_time=40.0, service_time=5.0),
            Customer(id="C2", x=-3.0, y=1.0, demand=20, ready_time=10.0, due_time=50.0, service_time=5.0),
            Customer(id="C3", x=4.0, y=-2.0, demand=25, ready_time=15.0, due_time=60.0, service_time=5.0),
            Customer(id="C4", x=-1.0, y=-4.0, demand=10, ready_time=20.0, due_time=70.0, service_time=5.0),
            Customer(id="C5", x=5.0, y=5.0, demand=30, ready_time=25.0, due_time=80.0, service_time=5.0),
        ]
        self.vehicles = [
            Vehicle(id="V1", depot_id="DEPOT_1", capacity=60, speed=1.0),
            Vehicle(id="V2", depot_id="DEPOT_1", capacity=60, speed=1.0),
        ]
        self.instance = VRPTWInstance(
            depots=[self.depot],
            customers=self.customers,
            vehicles=self.vehicles,
        )

    def test_solve_feasible_instance(self):
        solver = VRPTWSolver()
        solution = solver.solve(self.instance)

        self.assertTrue(solution.is_feasible)
        self.assertGreater(len(solution.routes), 0)
        self.assertLessEqual(len(solution.routes), len(self.vehicles))

        # All customers visited exactly once
        visited = []
        for route in solution.routes:
            visited.extend(route.customer_ids)
        self.assertEqual(sorted(visited), sorted([c.id for c in self.customers]))

    def test_capacity_constraint_respected(self):
        solver = VRPTWSolver()
        solution = solver.solve(self.instance)

        for route in solution.routes:
            total_demand = sum(
                next(c.demand for c in self.customers if c.id == cid)
                for cid in route.customer_ids
            )
            self.assertLessEqual(total_demand, 60)

    def test_time_window_arrival_and_service(self):
        solver = VRPTWSolver()
        solution = solver.solve(self.instance)

        for route in solution.routes:
            current_time = 0.0
            prev_x, prev_y = self.depot.x, self.depot.y
            for cid in route.customer_ids:
                cust = next(c for c in self.customers if c.id == cid)
                travel_dist = ((cust.x - prev_x)**2 + (cust.y - prev_y)**2)**0.5
                arrival = current_time + travel_dist
                start_service = max(arrival, cust.ready_time)
                self.assertLessEqual(start_service, cust.due_time)
                current_time = start_service + cust.service_time
                prev_x, prev_y = cust.x, cust.y

    def test_2opt_route_improvement(self):
        solver = VRPTWSolver()
        raw_routes = solver._clarke_wright_savings(self.instance)
        raw_distance = sum(r.total_distance for r in raw_routes)

        improved_routes = [solver._two_opt_route(r, self.instance) for r in raw_routes]
        improved_distance = sum(r.total_distance for r in improved_routes)

        self.assertLessEqual(improved_distance, raw_distance + 1e-9)


if __name__ == "__main__":
    unittest.main()
