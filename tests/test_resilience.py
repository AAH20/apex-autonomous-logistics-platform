"""Test suite for Engine 8: Supply Chain Disruption Resilience & Multi-Commodity Min-Cost Flow."""

import unittest
from apex_autonomous_logistics_platform.resilience import (
    SupplyNode,
    TransportEdge,
    CommodityDemand,
    SupplyChainGraph,
    DisruptionResilienceEngine,
)


class TestDisruptionResilience(unittest.TestCase):
    """Unit tests for Choke-Point Analysis, Min-Cost Flow, and Dynamic Rerouting."""

    def setUp(self):
        # Global Trade Network: Asia Factory -> (Suez Canal / Cape of Good Hope) -> Europe Market
        # Alternate Route via Pacific / West Coast rail
        self.nodes = [
            SupplyNode(id="ASIA_HUB", node_type="PRODUCER", supply_capacity=1000.0),
            SupplyNode(id="SUEZ_CHOKE", node_type="INTERMODAL", supply_capacity=0.0),
            SupplyNode(id="CAPE_ROUTE", node_type="INTERMODAL", supply_capacity=0.0),
            SupplyNode(id="EU_PORT", node_type="INTERMODAL", supply_capacity=0.0),
            SupplyNode(id="EU_CONSUMER", node_type="CONSUMER", supply_capacity=-1000.0),
        ]

        self.edges = [
            # Primary path via Suez: Cheap and fast
            TransportEdge(source="ASIA_HUB", target="SUEZ_CHOKE", capacity=800.0, unit_cost=50.0, transit_days=10.0),
            TransportEdge(source="SUEZ_CHOKE", target="EU_PORT", capacity=800.0, unit_cost=40.0, transit_days=8.0),
            # Secondary backup path via Cape of Good Hope: More expensive, higher capacity
            TransportEdge(source="ASIA_HUB", target="CAPE_ROUTE", capacity=1000.0, unit_cost=90.0, transit_days=22.0),
            TransportEdge(source="CAPE_ROUTE", target="EU_PORT", capacity=1000.0, unit_cost=70.0, transit_days=14.0),
            # Last mile to consumer
            TransportEdge(source="EU_PORT", target="EU_CONSUMER", capacity=1200.0, unit_cost=10.0, transit_days=2.0),
        ]

        self.graph = SupplyChainGraph(nodes=self.nodes, edges=self.edges)
        self.demand = CommodityDemand(
            commodity_id="CONSUMER_ELECTRONICS",
            source_node="ASIA_HUB",
            sink_node="EU_CONSUMER",
            quantity_required=800.0,
        )

    def test_nominal_min_cost_flow_routes_via_cheapest_choke(self):
        engine = DisruptionResilienceEngine()
        flow_result = engine.solve_min_cost_flow(self.graph, [self.demand])

        self.assertTrue(flow_result.is_satisfied)
        self.assertEqual(flow_result.fulfilled_quantity, 800.0)

        # In nominal state, all 800 units should flow through cheap Suez path
        suez_flow = flow_result.edge_flows.get(("ASIA_HUB", "SUEZ_CHOKE"), 0.0)
        self.assertEqual(suez_flow, 800.0)
        cape_flow = flow_result.edge_flows.get(("ASIA_HUB", "CAPE_ROUTE"), 0.0)
        self.assertEqual(cape_flow, 0.0)

    def test_choke_point_disruption_triggers_autonomous_rerouting(self):
        engine = DisruptionResilienceEngine()

        # Simulate Suez Canal complete blockage (capacity -> 0.0)
        disrupted_graph = self.graph.simulate_disruption(failed_edge=("ASIA_HUB", "SUEZ_CHOKE"))
        reroute_result = engine.solve_min_cost_flow(disrupted_graph, [self.demand])

        # Flow must be rerouted via Cape of Good Hope with zero unfulfilled demand
        self.assertTrue(reroute_result.is_satisfied)
        self.assertEqual(reroute_result.fulfilled_quantity, 800.0)
        cape_flow = reroute_result.edge_flows.get(("ASIA_HUB", "CAPE_ROUTE"), 0.0)
        self.assertEqual(cape_flow, 800.0)

        # Cost increases gracefully due to longer transit
        nominal_cost = engine.solve_min_cost_flow(self.graph, [self.demand]).total_cost
        self.assertGreater(reroute_result.total_cost, nominal_cost)

    def test_choke_point_vulnerability_index(self):
        engine = DisruptionResilienceEngine()
        vulnerability_report = engine.assess_network_vulnerability(self.graph, [self.demand])

        # Single bridge with no alternate path loses 100% flow
        top_critical = vulnerability_report.critical_choke_points[0]
        self.assertEqual(top_critical.flow_lost_pct, 100.0)

        # Suez edges have highest cost escalation among alternative-routed links
        suez_choke = next(c for c in vulnerability_report.critical_choke_points if "SUEZ" in c.edge_id)
        self.assertGreater(suez_choke.cost_escalation_pct, 50.0)
        self.assertGreaterEqual(vulnerability_report.network_resilience_index, 0.70)


if __name__ == "__main__":
    unittest.main()
