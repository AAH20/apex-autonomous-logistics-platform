"""Test suite for Engine 3: Multi-Echelon Bullwhip Suppressor & Kalman Demand Sensing."""

import unittest
from apex_autonomous_logistics_platform.inventory import (
    DemandObservation,
    EchelonNode,
    MultiEchelonNetwork,
    KalmanDemandFilter,
    BullwhipSuppressor,
)


class TestInventorySuppression(unittest.TestCase):
    """Unit tests for Multi-Echelon Inventory Optimization and Bullwhip Suppression."""

    def setUp(self):
        # 3-tier supply chain: Retailer -> Regional DC -> Central Factory
        self.retailer = EchelonNode(id="RETAIL_1", tier=1, lead_time_days=2.0, holding_cost_per_unit=1.0, service_level=0.999)
        self.rdc = EchelonNode(id="RDC_EAST", tier=2, lead_time_days=5.0, holding_cost_per_unit=0.5, service_level=0.999)
        self.factory = EchelonNode(id="FACTORY_CENTRAL", tier=3, lead_time_days=10.0, holding_cost_per_unit=0.2, service_level=0.999)

        self.network = MultiEchelonNetwork(
            nodes=[self.retailer, self.rdc, self.factory],
            upstream_map={
                "RETAIL_1": "RDC_EAST",
                "RDC_EAST": "FACTORY_CENTRAL",
            }
        )

    def test_kalman_filter_denoises_phantom_demand(self):
        kf = KalmanDemandFilter(process_variance=4.0, measurement_variance=16.0, initial_demand=100.0)

        # True demand is ~100 with noise and one outlier spike (phantom order of 250)
        observations = [102.0, 98.0, 105.0, 97.0, 250.0, 101.0, 99.0]
        estimates = [kf.update(z).estimated_demand for z in observations]

        # The outlier of 250 should be smoothed out and not cause massive jump
        self.assertLess(estimates[4], 160.0)
        # Final estimate should stabilize near true mean
        self.assertAlmostEqual(estimates[-1], 100.0, delta=10.0)

    def test_bullwhip_ratio_suppression(self):
        suppressor = BullwhipSuppressor(self.network)

        # Simulate 30 days of noisy retail demand
        import random
        random.seed(42)
        raw_demands = [100.0 + random.gauss(0, 15.0) for _ in range(30)]

        simulation_result = suppressor.simulate_replenishment(raw_demands)

        # Classical unmitigated bullwhip has Var(Factory Orders) >> Var(Retail Demand)
        # Suppressed ratio Var(Upstream) / Var(Downstream) must be <= 1.25
        self.assertLessEqual(simulation_result.bullwhip_ratio_factory, 1.25)
        self.assertEqual(len(simulation_result.stockout_events), 0)

    def test_guaranteed_service_model_safety_stocks(self):
        suppressor = BullwhipSuppressor(self.network)
        safety_stocks = suppressor.compute_optimal_safety_stocks(mean_daily_demand=100.0, std_daily_demand=15.0)

        # Upstream tiers with longer lead times require proportionally sized safety stocks
        self.assertGreater(safety_stocks["FACTORY_CENTRAL"], safety_stocks["RETAIL_1"])
        self.assertGreater(safety_stocks["RDC_EAST"], safety_stocks["RETAIL_1"])


if __name__ == "__main__":
    unittest.main()
