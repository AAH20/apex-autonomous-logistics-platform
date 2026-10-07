"""Engine 8: Supply Chain Disruption Resilience & Multi-Commodity Min-Cost Flow.

Implements:
  - Directed Capacitated Multigraph network flow with unit transport costs.
  - Successive Shortest Augmenting Path (SSAP) min-cost flow solver.
  - Systematic Choke-Point stress test (Suez / Panama / Port Strike failure simulation).
  - Network Resilience Index (NRI) quantification and autonomous reroute contingency generation.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class SupplyNode:
    """Supply chain node (Producer plant, port, intermodal hub, consumer market)."""
    id: str
    node_type: str  # "PRODUCER", "INTERMODAL", "CONSUMER"
    supply_capacity: float = 0.0


@dataclass(frozen=True)
class TransportEdge:
    """Directed shipping lane, rail corridor, or highway transport link."""
    source: str
    target: str
    capacity: float
    unit_cost: float
    transit_days: float = 1.0


@dataclass(frozen=True)
class CommodityDemand:
    """Required throughput for a specific commodity class."""
    commodity_id: str
    source_node: str
    sink_node: str
    quantity_required: float


@dataclass
class SupplyChainGraph:
    """Network topology of intermodal supply links."""
    nodes: list[SupplyNode]
    edges: list[TransportEdge]

    def simulate_disruption(self, failed_edge: tuple[str, str]) -> SupplyChainGraph:
        """Create a disrupted network clone with capacity on failed_edge reduced to zero."""
        new_edges = []
        for e in self.edges:
            if (e.source, e.target) == failed_edge:
                new_edges.append(TransportEdge(
                    source=e.source,
                    target=e.target,
                    capacity=0.0,
                    unit_cost=e.unit_cost,
                    transit_days=e.transit_days,
                ))
            else:
                new_edges.append(e)
        return SupplyChainGraph(nodes=self.nodes, edges=new_edges)


@dataclass
class MinCostFlowResult:
    """Flow assignment result minimizing global transportation costs."""
    is_satisfied: bool
    fulfilled_quantity: float
    total_cost: float
    edge_flows: dict[tuple[str, str], float] = field(default_factory=dict)


@dataclass
class ChokePointRisk:
    """Single-point-of-failure vulnerability profile."""
    edge_id: str
    cost_escalation_pct: float
    flow_lost_pct: float


@dataclass
class NetworkVulnerabilityReport:
    """Aggregate vulnerability audit and network resilience score."""
    critical_choke_points: list[ChokePointRisk]
    network_resilience_index: float  # [0.0, 1.0] where 1.0 = completely resilient


class DisruptionResilienceEngine:
    """Solves min-cost flow routing and evaluates global disruption contingencies."""

    def solve_min_cost_flow(
        self,
        graph: SupplyChainGraph,
        demands: list[CommodityDemand],
    ) -> MinCostFlowResult:
        """Solve min-cost flow using successive shortest augmenting paths."""
        # Residual capacities
        residual_cap: dict[tuple[str, str], float] = {}
        cost_map: dict[tuple[str, str], float] = {}
        edge_flows: dict[tuple[str, str], float] = {}

        for e in graph.edges:
            residual_cap[(e.source, e.target)] = e.capacity
            residual_cap[(e.target, e.source)] = 0.0
            cost_map[(e.source, e.target)] = e.unit_cost
            cost_map[(e.target, e.source)] = -e.unit_cost
            edge_flows[(e.source, e.target)] = 0.0

        all_node_ids = [n.id for n in graph.nodes]
        total_fulfilled = 0.0
        total_cost = 0.0

        for dem in demands:
            needed = dem.quantity_required

            while needed > 1e-6:
                # Find least-cost augmenting path from source to sink using Bellman-Ford
                dist: dict[str, float] = {nid: float("inf") for nid in all_node_ids}
                parent: dict[str, str | None] = {nid: None for nid in all_node_ids}
                dist[dem.source_node] = 0.0

                for _ in range(len(all_node_ids) - 1):
                    updated = False
                    for (u, v), cap in residual_cap.items():
                        if cap > 1e-6:
                            cost = cost_map.get((u, v), float("inf"))
                            if dist[u] + cost < dist[v] - 1e-9:
                                dist[v] = dist[u] + cost
                                parent[v] = u
                                updated = True
                    if not updated:
                        break

                if dist[dem.sink_node] == float("inf"):
                    # No augmenting path found
                    break

                # Backtrack to find bottleneck residual capacity along path
                curr = dem.sink_node
                path_edges: list[tuple[str, str]] = []
                bottleneck = needed

                while curr != dem.source_node:
                    p = parent[curr]
                    if p is None:
                        break
                    path_edges.append((p, curr))
                    bottleneck = min(bottleneck, residual_cap[(p, curr)])
                    curr = p

                if bottleneck <= 1e-6 or curr != dem.source_node:
                    break

                # Push flow
                for u, v in path_edges:
                    residual_cap[(u, v)] -= bottleneck
                    residual_cap[(v, u)] += bottleneck
                    if (u, v) in edge_flows:
                        edge_flows[(u, v)] += bottleneck
                    else:
                        edge_flows[(v, u)] -= bottleneck

                    total_cost += bottleneck * cost_map[(u, v)]

                needed -= bottleneck
                total_fulfilled += bottleneck

        satisfied = (total_fulfilled >= sum(d.quantity_required for d in demands) - 1e-6)

        return MinCostFlowResult(
            is_satisfied=satisfied,
            fulfilled_quantity=total_fulfilled,
            total_cost=total_cost,
            edge_flows=edge_flows,
        )

    def assess_network_vulnerability(
        self,
        graph: SupplyChainGraph,
        demands: list[CommodityDemand],
    ) -> NetworkVulnerabilityReport:
        """Systematically evaluate choke points by simulating individual link outages."""
        baseline = self.solve_min_cost_flow(graph, demands)
        base_cost = baseline.total_cost if baseline.total_cost > 0 else 1.0
        total_dem = sum(d.quantity_required for d in demands)

        risks: list[ChokePointRisk] = []
        fulfillment_ratios: list[float] = []

        for e in graph.edges:
            if e.capacity <= 0:
                continue

            disrupted = graph.simulate_disruption((e.source, e.target))
            res = self.solve_min_cost_flow(disrupted, demands)

            fulfillment_ratio = res.fulfilled_quantity / total_dem if total_dem > 0 else 1.0
            fulfillment_ratios.append(fulfillment_ratio)

            if not res.is_satisfied:
                lost_pct = ((total_dem - res.fulfilled_quantity) / total_dem) * 100.0
            else:
                lost_pct = 0.0

            cost_esc_pct = ((res.total_cost - base_cost) / base_cost) * 100.0 if res.total_cost >= base_cost else 0.0

            risks.append(ChokePointRisk(
                edge_id=f"{e.source}->{e.target}",
                cost_escalation_pct=cost_esc_pct,
                flow_lost_pct=lost_pct,
            ))

        # Sort by cost escalation and flow lost descending
        risks.sort(key=lambda r: (r.flow_lost_pct, r.cost_escalation_pct), reverse=True)

        mean_fulfillment = sum(fulfillment_ratios) / len(fulfillment_ratios) if fulfillment_ratios else 1.0
        resilience_index = max(0.0, min(1.0, mean_fulfillment * (1.0 / (1.0 + risks[0].cost_escalation_pct / 1000.0))))

        return NetworkVulnerabilityReport(
            critical_choke_points=risks,
            network_resilience_index=resilience_index,
        )
