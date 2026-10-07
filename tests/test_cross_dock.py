"""Test suite for Engine 5: Dynamic Cross-Docking Terminal Assignment & Flow Shop."""

import unittest
from apex_autonomous_logistics_platform.cross_dock import (
    CrossDockTerminal,
    Door,
    InboundTrailer,
    OutboundTrailer,
    CrossDockScheduler,
)


class TestCrossDockScheduler(unittest.TestCase):
    """Unit tests for Cross-Docking Door Assignment and AGV Pallet Flow Scheduling."""

    def setUp(self):
        # I-shaped cross-dock terminal with 4 inbound doors (North side) and 4 outbound doors (South side)
        inbound_doors = [
            Door(id="IN_1", door_type="INBOUND", x=0.0, y=10.0),
            Door(id="IN_2", door_type="INBOUND", x=20.0, y=10.0),
            Door(id="IN_3", door_type="INBOUND", x=40.0, y=10.0),
            Door(id="IN_4", door_type="INBOUND", x=60.0, y=10.0),
        ]
        outbound_doors = [
            Door(id="OUT_1", door_type="OUTBOUND", x=0.0, y=0.0),
            Door(id="OUT_2", door_type="OUTBOUND", x=20.0, y=0.0),
            Door(id="OUT_3", door_type="OUTBOUND", x=40.0, y=0.0),
            Door(id="OUT_4", door_type="OUTBOUND", x=60.0, y=0.0),
        ]
        self.terminal = CrossDockTerminal(
            id="CHICAGO_CROSSDOCK_01",
            inbound_doors=inbound_doors,
            outbound_doors=outbound_doors,
            num_agvs=6,
            agv_speed_m_per_s=2.0,
        )

        # 4 Inbound trailers with manifest distributions targeting outbound destinations
        self.inbound_trailers = [
            InboundTrailer(id="TR_IN_A", arrival_time=0.0, pallet_destinations={"OUT_TR_1": 40, "OUT_TR_2": 10}),
            InboundTrailer(id="TR_IN_B", arrival_time=0.0, pallet_destinations={"OUT_TR_2": 35, "OUT_TR_3": 15}),
            InboundTrailer(id="TR_IN_C", arrival_time=10.0, pallet_destinations={"OUT_TR_3": 50, "OUT_TR_4": 5}),
            InboundTrailer(id="TR_IN_D", arrival_time=15.0, pallet_destinations={"OUT_TR_4": 45, "OUT_TR_1": 5}),
        ]
        self.outbound_trailers = [
            OutboundTrailer(id="OUT_TR_1", departure_deadline=600.0, capacity_pallets=50),
            OutboundTrailer(id="OUT_TR_2", departure_deadline=600.0, capacity_pallets=50),
            OutboundTrailer(id="OUT_TR_3", departure_deadline=600.0, capacity_pallets=65),
            OutboundTrailer(id="OUT_TR_4", departure_deadline=600.0, capacity_pallets=60),
        ]

    def test_door_assignment_minimizes_travel_distance(self):
        scheduler = CrossDockScheduler()
        plan = scheduler.optimize_schedule(
            self.terminal,
            self.inbound_trailers,
            self.outbound_trailers,
        )

        self.assertTrue(plan.is_feasible)
        self.assertEqual(len(plan.inbound_assignments), 4)
        self.assertEqual(len(plan.outbound_assignments), 4)

        # Total pallet-meters should be significantly less than naive worst-case assignment
        naive_distance = scheduler.evaluate_naive_distance(
            self.terminal,
            self.inbound_trailers,
            self.outbound_trailers,
        )
        self.assertLess(plan.total_pallet_distance_m, naive_distance * 0.75)

    def test_agv_flow_makespan_respects_outbound_deadlines(self):
        scheduler = CrossDockScheduler()
        plan = scheduler.optimize_schedule(
            self.terminal,
            self.inbound_trailers,
            self.outbound_trailers,
        )

        # All pallets must be transferred before outbound departure deadlines
        for out_id, departure_time in plan.outbound_ready_times.items():
            trailer = next(t for t in self.outbound_trailers if t.id == out_id)
            self.assertLessEqual(departure_time, trailer.departure_deadline)

        self.assertGreater(plan.agv_utilization_pct, 50.0)


if __name__ == "__main__":
    unittest.main()
