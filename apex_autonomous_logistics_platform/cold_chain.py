"""Engine 6: Cold-Chain Arrhenius Degradation & Thermal Excursion Governor.

Implements:
  - Arrhenius chemical kinetic degradation rate calculation.
  - United States Pharmacopeia (USP <1079>) Mean Kinetic Temperature (MKT).
  - Continuous shelf-life depreciation under real-time sensor telemetry.
  - Automated thermal excursion triage (NOMINAL, WARNING, CRITICAL, SPOILED).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

# Universal gas constant R in J / (mol * K)
R_GAS_CONSTANT = 8.31446261815324


@dataclass(frozen=True)
class ColdChainProduct:
    """Biopharmaceutical or perishable food specification."""
    id: str
    target_temp_c: float
    max_critical_temp_c: float
    activation_energy_j_mol: float = 83144.0  # ~83.14 kJ/mol typical for biologics
    baseline_shelf_life_days: float = 180.0


@dataclass(frozen=True)
class TemperatureLog:
    """Discrete temperature measurement at a given mission hour."""
    timestamp_hours: float
    temp_c: float


@dataclass(frozen=True)
class ReeferContainer:
    """Refrigerated container (reefer) actively carrying a product payload."""
    id: str
    product: ColdChainProduct
    initial_shelf_life_remaining_days: float = 180.0


@dataclass
class ColdChainStatus:
    """Thermal telemetry evaluation and shelf-life disposition."""
    is_viable: bool
    excursion_severity: str  # "NOMINAL", "WARNING", "CRITICAL", "SPOILED"
    mean_kinetic_temp_c: float
    shelf_life_lost_days: float
    remaining_shelf_life_days: float
    recommended_action: str


class ColdChainGovernor:
    """Monitors real-time thermal integrity and calculates Arrhenius degradation."""

    def evaluate_shipment(
        self,
        reefer: ReeferContainer,
        logs: list[TemperatureLog],
    ) -> ColdChainStatus:
        """Evaluate cold-chain telemetry logs and compute shelf-life depreciation."""
        if not logs:
            return ColdChainStatus(
                is_viable=True,
                excursion_severity="NOMINAL",
                mean_kinetic_temp_c=reefer.product.target_temp_c,
                shelf_life_lost_days=0.0,
                remaining_shelf_life_days=reefer.initial_shelf_life_remaining_days,
                recommended_action="CONTINUE_NORMAL_TRANSIT",
            )

        prod = reefer.product
        t_target_k = prod.target_temp_c + 273.15
        ea_over_r = prod.activation_energy_j_mol / R_GAS_CONSTANT

        # Cumulative Arrhenius degradation integral
        equivalent_hours_lost = 0.0
        max_temp_seen = -273.15
        temp_readings: list[float] = []

        for log in logs:
            temp_readings.append(log.temp_c)
            max_temp_seen = max(max_temp_seen, log.temp_c)

            t_k = log.temp_c + 273.15
            # Arrhenius acceleration factor: alpha(T) = exp( (Ea/R) * (1/T_target - 1/T_actual) )
            if t_k > 0:
                acceleration = math.exp(ea_over_r * (1.0 / t_target_k - 1.0 / t_k))
            else:
                acceleration = 1.0

            equivalent_hours_lost += acceleration * 1.0  # assuming 1-hour intervals

        shelf_life_lost_days = equivalent_hours_lost / 24.0
        remaining_days = max(0.0, reefer.initial_shelf_life_remaining_days - shelf_life_lost_days)

        # Compute MKT
        mkt_c = self.calculate_mkt(temp_readings, prod.activation_energy_j_mol)

        # Severity determination
        is_viable = remaining_days > 0.0
        if not is_viable:
            severity = "SPOILED"
            action = "CONDEMN_AND_DISPOSE"
        elif max_temp_seen > prod.max_critical_temp_c + 2.0:
            severity = "CRITICAL"
            action = "EMERGENCY_DIVERT_TO_NEAREST_COLD_HUB"
        elif max_temp_seen > prod.max_critical_temp_c:
            severity = "WARNING"
            action = "EXPEDITE_DELIVERY_AND_REINSPECT"
        else:
            severity = "NOMINAL"
            action = "CONTINUE_NORMAL_TRANSIT"

        return ColdChainStatus(
            is_viable=is_viable,
            excursion_severity=severity,
            mean_kinetic_temp_c=mkt_c,
            shelf_life_lost_days=shelf_life_lost_days,
            remaining_shelf_life_days=remaining_days,
            recommended_action=action,
        )

    def calculate_mkt(self, temperatures_c: list[float], delta_h_j_mol: float = 83144.0) -> float:
        """Calculate USP <1079> Mean Kinetic Temperature (MKT) in Celsius.

        Formula:
            T_MKT = (-dH / R) / ln( (1/N) * sum(exp(-dH / (R * T_k))) ) - 273.15
        """
        if not temperatures_c:
            return 0.0

        n = len(temperatures_c)
        dh_over_r = delta_h_j_mol / R_GAS_CONSTANT

        sum_exp = 0.0
        for tc in temperatures_c:
            tk = tc + 273.15
            if tk <= 0:
                tk = 1.0
            sum_exp += math.exp(-dh_over_r / tk)

        mean_exp = sum_exp / n
        if mean_exp <= 0:
            return sum(temperatures_c) / n

        t_mkt_k = (-dh_over_r) / math.log(mean_exp)
        return t_mkt_k - 273.15
