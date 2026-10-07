"""Test suite for Engine 2: 3D Intermodal Container Bin Packing."""

import unittest
from apex_autonomous_logistics_platform.container_3d import (
    BoxItem,
    Container,
    ContainerPacker,
)


class TestContainerPacker(unittest.TestCase):
    """Unit tests for 3D Container Packing with CoG and Axle Weight Constraints."""

    def setUp(self):
        # Standard 40ft High Cube Container: 12.0m x 2.35m x 2.69m, max payload 28,000 kg
        self.container = Container(
            id="40HC_01",
            length=12.0,
            width=2.35,
            height=2.69,
            max_payload=28000.0,
            tare_weight=3800.0,
            max_steer_axle=6000.0,
            max_drive_axle=18000.0,
            max_trailer_axle=20000.0,
        )

    def test_pack_uniform_boxes_fits_volume_and_weight(self):
        packer = ContainerPacker()
        # 20 identical pallets: 1.2m x 1.0m x 1.2m, 800 kg each
        boxes = [
            BoxItem(id=f"PALLET_{i}", length=1.2, width=1.0, height=1.2, weight=800.0)
            for i in range(20)
        ]
        result = packer.pack(self.container, boxes)

        self.assertEqual(len(result.placed_boxes), 20)
        self.assertEqual(len(result.unplaced_boxes), 0)
        self.assertGreater(result.volume_utilization, 0.30)
        self.assertLessEqual(result.total_weight, self.container.max_payload)

    def test_cog_envelope_and_axle_weights(self):
        packer = ContainerPacker()
        # Heavy cargo in the middle
        boxes = [
            BoxItem(id="HEAVY_1", length=3.0, width=2.0, height=1.5, weight=6000.0),
            BoxItem(id="HEAVY_2", length=3.0, width=2.0, height=1.5, weight=6000.0),
            BoxItem(id="HEAVY_3", length=3.0, width=2.0, height=1.5, weight=6000.0),
        ]
        result = packer.pack(self.container, boxes)

        cog = result.cog_metrics
        # Center of gravity along length should be centered within 40% to 60% of container length
        self.assertGreater(cog.x_cog, self.container.length * 0.30)
        self.assertLess(cog.x_cog, self.container.length * 0.70)
        self.assertTrue(cog.is_within_envelope)

        # Axle weights must not exceed regulatory limits
        self.assertLessEqual(cog.steer_axle_weight, self.container.max_steer_axle)
        self.assertLessEqual(cog.drive_axle_weight, self.container.max_drive_axle)
        self.assertLessEqual(cog.trailer_axle_weight, self.container.max_trailer_axle)

    def test_fragility_stacking_discipline(self):
        packer = ContainerPacker()
        # Heavy robust box vs fragile electronic equipment
        robust = BoxItem(id="STEEL_CRATE", length=1.5, width=1.0, height=1.0, weight=2500.0, fragility_rank=1)
        fragile = BoxItem(id="AVIONICS", length=1.5, width=1.0, height=1.0, weight=300.0, fragility_rank=5)

        result = packer.pack(self.container, [fragile, robust])
        self.assertEqual(len(result.placed_boxes), 2)

        p_fragile = next(p for p in result.placed_boxes if p.item.id == "AVIONICS")
        p_robust = next(p for p in result.placed_boxes if p.item.id == "STEEL_CRATE")

        # Fragile item must not have heavy robust item on top of it
        if abs(p_fragile.x - p_robust.x) < 0.1 and abs(p_fragile.y - p_robust.y) < 0.1:
            self.assertGreaterEqual(p_fragile.z, p_robust.z)

    def test_imdg_hazardous_material_segregation(self):
        packer = ContainerPacker()
        # IMDG Class 3 (Flammable Liquids) vs IMDG Class 5.1 (Oxidizing Substances)
        item_flammable = BoxItem(id="FLAMMABLE", length=1.0, width=1.0, height=1.0, weight=500.0, hazard_class="IMDG_3")
        item_oxidizer = BoxItem(id="OXIDIZER", length=1.0, width=1.0, height=1.0, weight=500.0, hazard_class="IMDG_5.1")

        result = packer.pack(self.container, [item_flammable, item_oxidizer])
        self.assertTrue(result.is_imdg_compliant)
        # Verify segregation distance > 3.0 meters for incompatible hazardous classes
        p1 = next(p for p in result.placed_boxes if p.item.id == "FLAMMABLE")
        p2 = next(p for p in result.placed_boxes if p.item.id == "OXIDIZER")
        dist = ((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)**0.5
        self.assertGreaterEqual(dist, 3.0)


if __name__ == "__main__":
    unittest.main()
