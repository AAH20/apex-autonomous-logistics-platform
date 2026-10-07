"""Engine 3: Multi-Echelon Bullwhip Suppressor & Kalman Demand Sensing.

Implements:
  - Recursive 1D Kalman Filter for demand denoising and phantom spike filtering.
  - Guaranteed Service Model (GSM) multi-echelon safety stock allocation.
  - Variance-smoothing base-stock replenishment to suppress upstream Bullwhip amplification.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class EchelonNode:
    """A node in the multi-echelon supply network (Retailer, DC, Factory)."""
    id: str
    tier: int  # 1 = Retail, 2 = Regional DC, 3 = Factory
    lead_time_days: float
    holding_cost_per_unit: float
    service_level: float = 0.95


@dataclass
class DemandObservation:
    """State update from the Kalman demand sensor."""
    raw_demand: float
    estimated_demand: float
    variance: float
    kalman_gain: float


@dataclass
class MultiEchelonNetwork:
    """Topology of multi-tier supply chain network."""
    nodes: list[EchelonNode]
    upstream_map: dict[str, str] = field(default_factory=dict)

    def get_node(self, node_id: str) -> EchelonNode:
        for n in self.nodes:
            if n.id == node_id:
                return n
        raise ValueError(f"EchelonNode '{node_id}' not found.")


@dataclass
class MultiEchelonSimulationResult:
    """Result of multi-echelon supply replenishment simulation."""
    bullwhip_ratio_factory: float
    stockout_events: list[dict[str, Any]] = field(default_factory=list)
    total_holding_cost: float = 0.0
    daily_orders: dict[str, list[float]] = field(default_factory=dict)


class KalmanDemandFilter:
    """1D Kalman Filter tracking latent consumer demand rate."""

    def __init__(
        self,
        process_variance: float = 4.0,
        measurement_variance: float = 16.0,
        initial_demand: float = 100.0,
        initial_covariance: float = 10.0,
    ) -> None:
        self.q = process_variance
        self.r = measurement_variance
        self.x = initial_demand
        self.p = initial_covariance

    def update(self, measurement: float) -> DemandObservation:
        """Predict and update step with incoming demand observation."""
        # Predict step
        x_pred = self.x
        p_pred = self.p + self.q

        # Outlier attenuation: if measurement deviates > 3 std devs, dampen innovation
        residual = measurement - x_pred
        std_dev = math.sqrt(p_pred + self.r)
        if abs(residual) > 2.5 * std_dev:
            residual = math.copysign(2.5 * std_dev, residual)

        # Update step
        kalman_gain = p_pred / (p_pred + self.r)
        self.x = x_pred + kalman_gain * residual
        self.p = (1.0 - kalman_gain) * p_pred

        return DemandObservation(
            raw_demand=measurement,
            estimated_demand=self.x,
            variance=self.p,
            kalman_gain=kalman_gain,
        )


def _norm_inv_cdf(p: float) -> float:
    """High-precision rational approximation of standard normal quantile (Acklam's formula)."""
    if p <= 0.0 or p >= 1.0:
        raise ValueError("Probability must be in (0, 1)")

    # Coefficients in rational approximations
    a = [
        -3.969683028665376e+01,  2.209460984245205e+02,
        -2.759285104469687e+02,  1.383577518672690e+02,
        -3.066479806614716e+01,  2.506628277459239e+00,
    ]
    b = [
        -5.447609879822406e+01,  1.615858368580409e+02,
        -1.556989798598866e+02,  6.680131188771972e+01,
        -1.328068155288572e+01,
    ]
    c = [
        -7.784894002430293e-03, -3.223964580411365e-01,
        -2.400758277161838e+00, -2.549732539343734e+00,
        4.374664141464968e+00,  2.938163982698783e+00,
    ]
    d = [
        7.784695709041462e-03,  3.224671290700398e-01,
        2.445134137142996e+00,  3.754408661907416e+00,
    ]

    p_low = 0.02425
    p_high = 1.0 - p_low

    if p < p_low:
        q = math.sqrt(-2.0 * math.log(p))
        return (
            ((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]
        ) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    elif p > p_high:
        q = math.sqrt(-2.0 * math.log(1.0 - p))
        return -(
            ((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]
        ) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1.0)
    else:
        q = p - 0.5
        r = q * q
        return (
            (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q
        ) / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1.0)


class BullwhipSuppressor:
    """Multi-echelon inventory balancer and bullwhip effect suppressor."""

    def __init__(self, network: MultiEchelonNetwork) -> None:
        self.network = network

    def compute_optimal_safety_stocks(
        self,
        mean_daily_demand: float,
        std_daily_demand: float,
    ) -> dict[str, float]:
        """Compute safety stock targets under Guaranteed Service Model (GSM)."""
        safety_stocks: dict[str, float] = {}
        for node in self.network.nodes:
            z_score = _norm_inv_cdf(node.service_level)
            # Guaranteed Service safety stock equation: SS = z * sigma * sqrt(LeadTime)
            ss = z_score * std_daily_demand * math.sqrt(node.lead_time_days)
            safety_stocks[node.id] = ss
        return safety_stocks

    def simulate_replenishment(self, demand_series: list[float]) -> MultiEchelonSimulationResult:
        """Simulate replenishment policies and compute Bullwhip amplification factor."""
        if not demand_series:
            return MultiEchelonSimulationResult(bullwhip_ratio_factory=1.0)

        # Baseline statistics
        mean_demand = sum(demand_series) / len(demand_series)
        var_demand = sum((d - mean_demand) ** 2 for d in demand_series) / max(len(demand_series) - 1, 1)

        kf = KalmanDemandFilter(initial_demand=mean_demand)
        safety_stocks = self.compute_optimal_safety_stocks(
            mean_daily_demand=mean_demand,
            std_daily_demand=math.sqrt(var_demand) if var_demand > 0 else 1.0,
        )

        orders: dict[str, list[float]] = {n.id: [] for n in self.network.nodes}
        on_hand: dict[str, float] = {n.id: safety_stocks[n.id] + mean_demand for n in self.network.nodes}
        # Pre-seed pipeline with steady-state orders arriving from day 1 to lead_time
        pipeline: dict[str, list[tuple[int, float]]] = {}
        for n in self.network.nodes:
            lt = int(math.ceil(n.lead_time_days))
            pipeline[n.id] = [(d + 1, mean_demand) for d in range(lt)]
        stockouts: list[dict[str, Any]] = []

        # Find topological order: Tier 1 to Tier N
        sorted_nodes = sorted(self.network.nodes, key=lambda n: n.tier)

        for day, raw_d in enumerate(demand_series):
            # 1. Morning: Inbound shipments arrive
            for node in sorted_nodes:
                arrived = [qty for arr_day, qty in pipeline[node.id] if arr_day <= day]
                pipeline[node.id] = [(arr_day, qty) for arr_day, qty in pipeline[node.id] if arr_day > day]
                on_hand[node.id] += sum(arrived)

            kf_est = kf.update(raw_d).estimated_demand

            # 2. Daytime: Tier 1 Retailer satisfies customer demand
            retail_node = sorted_nodes[0]
            if on_hand[retail_node.id] < raw_d:
                stockouts.append({"day": day, "node": retail_node.id, "shortage": raw_d - on_hand[retail_node.id]})
                on_hand[retail_node.id] = 0.0
            else:
                on_hand[retail_node.id] -= raw_d

            # 3. Evening: Each node orders smoothed replenishment based on Kalman estimate
            for idx, node in enumerate(sorted_nodes):
                target_base_stock = safety_stocks[node.id] + kf_est * node.lead_time_days
                current_inventory_pos = on_hand[node.id] + sum(qty for _, qty in pipeline[node.id])

                # Smooth replenishment order sizing
                order_qty = max(0.0, target_base_stock - current_inventory_pos)
                order_qty = 0.7 * order_qty + 0.3 * kf_est

                orders[node.id].append(order_qty)

                # Push order into upstream pipeline
                arrival_day = day + int(math.ceil(node.lead_time_days))
                pipeline[node.id].append((arrival_day, order_qty))

        # Compute variance of factory orders vs variance of raw retail demand
        factory_node = sorted_nodes[-1]
        factory_orders = orders[factory_node.id]
        mean_factory = sum(factory_orders) / len(factory_orders)
        var_factory = sum((o - mean_factory) ** 2 for o in factory_orders) / max(len(factory_orders) - 1, 1)

        bullwhip_ratio = var_factory / var_demand if var_demand > 0 else 1.0

        return MultiEchelonSimulationResult(
            bullwhip_ratio_factory=bullwhip_ratio,
            stockout_events=stockouts,
            daily_orders=orders,
        )
