"""Test suite for Engine 6: Cold-Chain Arrhenius Degradation & Thermal Excursion Governor."""

import unittest
from apex_autonomous_logistics_platform.cold_chain import (
    ColdChainProduct,
    TemperatureLog,
    ReeferContainer,
    ColdChainGovernor,
)


class TestColdChainGovernor(unittest.TestCase):
    """Unit tests for Arrhenius Degradation and Mean Kinetic Temperature (MKT) Tracking."""

    def setUp(self):
        # mRNA Vaccine (Pfizer/Moderna analog: target -20C, critical limit 5C, Ea = 85 kJ/mol)
        self.vaccine = ColdChainProduct(
            id="VACCINE_MRNA_LOT42",
            target_temp_c=-20.0,
            max_critical_temp_c=5.0,
            activation_energy_j_mol=85000.0,
            baseline_shelf_life_days=180.0,
        )

        self.reefer = ReeferContainer(
            id="REEFER_TX_770",
            product=self.vaccine,
            initial_shelf_life_remaining_days=180.0,
        )

    def test_arrhenius_nominal_storage_minimal_degradation(self):
        governor = ColdChainGovernor()
        # 10 days stored nominally at -20°C
        logs = [TemperatureLog(timestamp_hours=h, temp_c=-20.0) for h in range(1, 241)]
        status = governor.evaluate_shipment(self.reefer, logs)

        self.assertTrue(status.is_viable)
        self.assertEqual(status.excursion_severity, "NOMINAL")
        self.assertAlmostEqual(status.shelf_life_lost_days, 10.0, delta=1.5)

    def test_thermal_excursion_accelerates_degradation(self):
        governor = ColdChainGovernor()
        # Nominal for 48 hours (-20C), then compressor fails: temp rises to +8°C for 12 hours
        logs = [TemperatureLog(timestamp_hours=h, temp_c=-20.0) for h in range(1, 49)]
        logs += [TemperatureLog(timestamp_hours=48 + h, temp_c=8.0) for h in range(1, 13)]

        status = governor.evaluate_shipment(self.reefer, logs)

        # Excursion breached max_critical_temp_c (5.0C)
        self.assertIn(status.excursion_severity, ["WARNING", "CRITICAL"])
        # Shelf life lost should be significantly higher due to exponential Arrhenius rate
        self.assertGreater(status.shelf_life_lost_days, 15.0)

    def test_mkt_calculation_matches_usp_standard(self):
        governor = ColdChainGovernor()
        # Cyclic temperature readings: 2C, 4C, 8C, 2C
        temps = [2.0, 4.0, 8.0, 2.0]
        mkt = governor.calculate_mkt(temps, delta_h_j_mol=83140.0)

        # MKT is weighted towards higher temperatures, so it must be strictly greater than arithmetic mean
        arithmetic_mean = sum(temps) / len(temps)  # 4.0
        self.assertGreater(mkt, arithmetic_mean)
        self.assertLess(mkt, 8.0)


if __name__ == "__main__":
    unittest.main()
