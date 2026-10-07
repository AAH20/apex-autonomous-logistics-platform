"""Test suite for Engine 4: Autonomous Vessel Weather & Ocean Current Navigation."""

import unittest
from apex_autonomous_logistics_platform.maritime import (
    GeoPoint,
    OceanCurrentField,
    VesselConfig,
    MaritimeWeatherRouter,
)


class TestMaritimeRouter(unittest.TestCase):
    """Unit tests for Autonomous Vessel Zermelo Navigation and Fuel Optimization."""

    def setUp(self):
        # 15,000 TEU Container Ship
        self.vessel = VesselConfig(
            mmsi="VESSEL_9981",
            deadweight_tonnage=150000.0,
            design_speed_knots=18.0,
            base_fuel_burn_tons_per_day=45.0,
            max_wave_height_meters=6.0,
        )

        # 10x10 degree oceanic grid with a favorable eastward current (Gulf Stream analog)
        # and a severe storm center
        self.current_field = OceanCurrentField(
            grid_resolution_deg=1.0,
            min_lon=0.0, max_lon=10.0,
            min_lat=0.0, max_lat=10.0,
        )
        # Add eastward current along latitude 5.0 (speed 4.0 knots eastward)
        self.current_field.add_jet_stream(lat=5.0, u_knots=4.0, v_knots=0.0)
        # Add storm with wave height 8.0m at (lon=5.0, lat=7.0)
        self.current_field.add_storm_center(lon=5.0, lat=7.0, radius_deg=1.5, wave_height_m=8.5)

    def test_zermelo_routing_leverages_currents_and_avoids_storm(self):
        router = MaritimeWeatherRouter(self.current_field, self.vessel)
        start = GeoPoint(lon=1.0, lat=5.0)
        destination = GeoPoint(lon=9.0, lat=5.0)

        route = router.compute_optimal_voyage(start, destination)

        self.assertTrue(route.is_feasible)
        self.assertGreater(len(route.waypoints), 2)
        # Max wave encountered along path must not breach vessel safety limits
        self.assertLessEqual(route.max_wave_encountered_m, self.vessel.max_wave_height_meters)

        # Favorable current should reduce fuel burn compared to still-water transit
        still_water_fuel = router.compute_still_water_fuel(start, destination)
        self.assertLess(route.total_fuel_burn_tons, still_water_fuel)

    def test_cii_carbon_intensity_rating(self):
        router = MaritimeWeatherRouter(self.current_field, self.vessel)
        start = GeoPoint(lon=0.0, lat=5.0)
        destination = GeoPoint(lon=8.0, lat=5.0)

        route = router.compute_optimal_voyage(start, destination)
        cii_rating = router.evaluate_cii_rating(route)

        # Favorable trajectory should achieve IMO Grade A or B
        self.assertIn(cii_rating.grade, ["A", "B"])
        self.assertGreater(cii_rating.co2_reduction_pct, 5.0)


if __name__ == "__main__":
    unittest.main()
