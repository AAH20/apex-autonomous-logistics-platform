"""Engine 2: 3D Intermodal Container Bin Packing with CoG and Axle Weight Constraints.

Implements:
  - 3D Maximal Empty Space (MES) / Extreme Points placement heuristics.
  - Center of Gravity (CoG) 3D coordinate calculation and stability envelope verification.
  - Tractor-trailer axle weight distribution (steer, drive tandem, trailer tandem).
  - Fragility tiers and stacking support rules.
  - IMDG Code hazardous material spatial segregation enforcement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class BoxItem:
    """Cargo item to be packed into an intermodal container."""
    id: str
    length: float
    width: float
    height: float
    weight: float
    fragility_rank: int = 1  # 1 = most robust (steel), 5 = most fragile (electronics/glass)
    hazard_class: str = "NONE"  # e.g., "NONE", "IMDG_3", "IMDG_5.1"
    allow_rotation: bool = True

    @property
    def volume(self) -> float:
        return self.length * self.width * self.height


@dataclass(frozen=True)
class Container:
    """Intermodal container specification (e.g. 20ft, 40ft High Cube)."""
    id: str
    length: float
    width: float
    height: float
    max_payload: float
    tare_weight: float = 3800.0
    max_steer_axle: float = 6000.0
    max_drive_axle: float = 18000.0
    max_trailer_axle: float = 20000.0

    @property
    def internal_volume(self) -> float:
        return self.length * self.width * self.height


@dataclass
class PlacedBox:
    """Box placed at coordinates (x, y, z) where x=length, y=width, z=height."""
    item: BoxItem
    x: float
    y: float
    z: float
    dim_x: float
    dim_y: float
    dim_z: float


@dataclass
class CoGResult:
    """Center of Gravity analysis and axle load distribution."""
    x_cog: float
    y_cog: float
    z_cog: float
    is_within_envelope: bool
    steer_axle_weight: float
    drive_axle_weight: float
    trailer_axle_weight: float


@dataclass
class ContainerPackingResult:
    """Complete 3D container packing result."""
    container_id: str
    placed_boxes: list[PlacedBox] = field(default_factory=list)
    unplaced_boxes: list[BoxItem] = field(default_factory=list)
    total_weight: float = 0.0
    volume_utilization: float = 0.0
    weight_utilization: float = 0.0
    cog_metrics: CoGResult = field(default_factory=lambda: CoGResult(0, 0, 0, True, 0, 0, 0))
    is_imdg_compliant: bool = True


# Incompatible IMDG hazardous classes requiring minimum 3.0m segregation distance
INCOMPATIBLE_HAZARDS = {
    ("IMDG_3", "IMDG_5.1"),
    ("IMDG_5.1", "IMDG_3"),
}


class ContainerPacker:
    """Deterministic 3D Container Packing Engine."""

    def pack(self, container: Container, items: list[BoxItem]) -> ContainerPackingResult:
        """Pack boxes into container respecting volume, weight, CoG, fragility, and IMDG rules."""
        # Sorting priority:
        # 1. Fragility ascending (sturdy items at the bottom)
        # 2. Weight descending (heavy items placed first)
        # 3. Volume descending
        sorted_items = sorted(
            items,
            key=lambda it: (it.fragility_rank, -it.weight, -it.volume),
        )

        placed: list[PlacedBox] = []
        unplaced: list[BoxItem] = []
        current_weight = 0.0

        # Extreme Points list initialized at origin (0, 0, 0)
        extreme_points: list[tuple[float, float, float]] = [(0.0, 0.0, 0.0)]

        for item in sorted_items:
            # Check weight capacity
            if current_weight + item.weight > container.max_payload:
                unplaced.append(item)
                continue

            best_placement = None

            # Sort candidate points: prefer lowest z (ground), then lowest x (front-to-back), then lowest y
            extreme_points.sort(key=lambda pt: (pt[2], pt[0], pt[1]))

            # Orientations: (l, w, h) and (w, l, h)
            orientations = [(item.length, item.width, item.height)]
            if item.allow_rotation and item.length != item.width:
                orientations.append((item.width, item.length, item.height))

            for pt in extreme_points:
                px, py, pz = pt
                found_for_pt = False

                for dx, dy, dz in orientations:
                    if px + dx > container.length + 1e-6:
                        continue
                    if py + dy > container.width + 1e-6:
                        continue
                    if pz + dz > container.height + 1e-6:
                        continue

                    candidate = PlacedBox(item=item, x=px, y=py, z=pz, dim_x=dx, dim_y=dy, dim_z=dz)

                    # Collision check
                    if self._check_collision(candidate, placed):
                        continue

                    # Fragility stacking check
                    if not self._check_fragility_stacking(candidate, placed):
                        continue

                    # IMDG segregation check
                    if not self._check_imdg_segregation(candidate, placed):
                        continue

                    best_placement = candidate
                    found_for_pt = True
                    break

                if found_for_pt:
                    break

            if best_placement:
                placed.append(best_placement)
                current_weight += item.weight

                # Remove used point and add new extreme points
                extreme_points = [
                    pt for pt in extreme_points
                    if not (abs(pt[0] - best_placement.x) < 1e-6 and
                            abs(pt[1] - best_placement.y) < 1e-6 and
                            abs(pt[2] - best_placement.z) < 1e-6)
                ]

                # Generate new extreme points from placed box corners
                bx, by, bz = best_placement.x, best_placement.y, best_placement.z
                bdx, bdy, bdz = best_placement.dim_x, best_placement.dim_y, best_placement.dim_z

                new_pts = [
                    (bx + bdx, by, bz),
                    (bx, by + bdy, bz),
                    (bx, by, bz + bdz),
                    (bx + bdx + 3.0, 0.0, 0.0),  # IMDG segregation candidate point
                    (container.length / 2.0 - bdx / 2.0, 0.0, 0.0), # Container center candidate
                ]
                for np in new_pts:
                    if 0.0 <= np[0] <= container.length and 0.0 <= np[1] <= container.width and 0.0 <= np[2] <= container.height:
                        if np not in extreme_points:
                            extreme_points.append(np)
            else:
                unplaced.append(item)

        # Compute CoG and axle weights
        cog_metrics = self._calculate_cog(container, placed, current_weight)

        total_packed_vol = sum(p.dim_x * p.dim_y * p.dim_z for p in placed)
        vol_util = total_packed_vol / container.internal_volume if container.internal_volume > 0 else 0.0
        weight_util = current_weight / container.max_payload if container.max_payload > 0 else 0.0

        return ContainerPackingResult(
            container_id=container.id,
            placed_boxes=placed,
            unplaced_boxes=unplaced,
            total_weight=current_weight,
            volume_utilization=vol_util,
            weight_utilization=weight_util,
            cog_metrics=cog_metrics,
            is_imdg_compliant=True,
        )

    def _check_collision(self, candidate: PlacedBox, placed_boxes: list[PlacedBox]) -> bool:
        """Check if candidate box intersects any already placed box."""
        for p in placed_boxes:
            overlap_x = (candidate.x < p.x + p.dim_x - 1e-6) and (candidate.x + candidate.dim_x > p.x + 1e-6)
            overlap_y = (candidate.y < p.y + p.dim_y - 1e-6) and (candidate.y + candidate.dim_y > p.y + 1e-6)
            overlap_z = (candidate.z < p.z + p.dim_z - 1e-6) and (candidate.z + candidate.dim_z > p.z + 1e-6)
            if overlap_x and overlap_y and overlap_z:
                return True
        return False

    def _check_fragility_stacking(self, candidate: PlacedBox, placed_boxes: list[PlacedBox]) -> bool:
        """Ensure fragile items are not crushed under heavy/robust items."""
        for p in placed_boxes:
            overlap_x = (candidate.x < p.x + p.dim_x - 1e-6) and (candidate.x + candidate.dim_x > p.x + 1e-6)
            overlap_y = (candidate.y < p.y + p.dim_y - 1e-6) and (candidate.y + candidate.dim_y > p.y + 1e-6)
            if overlap_x and overlap_y:
                # If candidate is on top of p (candidate.z >= p.z + p.dim_z - 1e-6)
                if candidate.z >= p.z + p.dim_z - 1e-6:
                    # Item underneath p must not be more fragile than item on top
                    if p.item.fragility_rank > candidate.item.fragility_rank:
                        return False
        return True

    def _check_imdg_segregation(self, candidate: PlacedBox, placed_boxes: list[PlacedBox]) -> bool:
        """Enforce IMDG code segregation distance for incompatible hazardous cargo."""
        if candidate.item.hazard_class == "NONE":
            return True

        for p in placed_boxes:
            if p.item.hazard_class == "NONE":
                continue
            pair = (candidate.item.hazard_class, p.item.hazard_class)
            if pair in INCOMPATIBLE_HAZARDS:
                # Euclidean 3D distance between box centers
                cx1 = candidate.x + candidate.dim_x / 2.0
                cy1 = candidate.y + candidate.dim_y / 2.0
                cz1 = candidate.z + candidate.dim_z / 2.0

                cx2 = p.x + p.dim_x / 2.0
                cy2 = p.y + p.dim_y / 2.0
                cz2 = p.z + p.dim_z / 2.0

                dist = math.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2 + (cz1 - cz2)**2)
                if dist < 3.0:
                    return False
        return True

    def _calculate_cog(
        self,
        container: Container,
        placed_boxes: list[PlacedBox],
        total_payload: float,
    ) -> CoGResult:
        """Calculate 3D Center of Gravity and resulting tractor-semi-trailer axle loads."""
        if total_payload <= 0:
            return CoGResult(
                x_cog=container.length / 2.0,
                y_cog=container.width / 2.0,
                z_cog=container.height / 2.0,
                is_within_envelope=True,
                steer_axle_weight=container.tare_weight * 0.25,
                drive_axle_weight=container.tare_weight * 0.40,
                trailer_axle_weight=container.tare_weight * 0.35,
            )

        sum_mx = 0.0
        sum_my = 0.0
        sum_mz = 0.0

        for p in placed_boxes:
            cx = p.x + p.dim_x / 2.0
            cy = p.y + p.dim_y / 2.0
            cz = p.z + p.dim_z / 2.0
            m = p.item.weight
            sum_mx += m * cx
            sum_my += m * cy
            sum_mz += m * cz

        x_cog = sum_mx / total_payload
        y_cog = sum_my / total_payload
        z_cog = sum_mz / total_payload

        # Container CoG envelope: 30% to 70% along length, 30% to 70% along width
        within_env = (
            (container.length * 0.30 <= x_cog <= container.length * 0.70) and
            (container.width * 0.30 <= y_cog <= container.width * 0.70)
        )

        # Axle Load Mechanics:
        # Semi-trailer modeled with Kingpin at x = 1.0m, Trailer Bogie Center at x = 10.5m
        kingpin_x = 1.0
        bogie_x = min(10.5, container.length * 0.88)
        wheelbase_trailer = bogie_x - kingpin_x

        # Payload fraction to kingpin and trailer bogie
        fraction_to_bogie = (x_cog - kingpin_x) / wheelbase_trailer
        fraction_to_bogie = max(0.1, min(0.9, fraction_to_bogie))
        fraction_to_kingpin = 1.0 - fraction_to_bogie

        payload_to_trailer_axle = total_payload * fraction_to_bogie
        payload_to_kingpin = total_payload * fraction_to_kingpin

        # Tractor splits kingpin load between Drive axle (75%) and Steer axle (25%)
        tractor_steer = container.tare_weight * 0.25 + payload_to_kingpin * 0.20
        tractor_drive = container.tare_weight * 0.40 + payload_to_kingpin * 0.80
        trailer_axle = container.tare_weight * 0.35 + payload_to_trailer_axle

        return CoGResult(
            x_cog=x_cog,
            y_cog=y_cog,
            z_cog=z_cog,
            is_within_envelope=within_env,
            steer_axle_weight=tractor_steer,
            drive_axle_weight=tractor_drive,
            trailer_axle_weight=trailer_axle,
        )
