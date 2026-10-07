"""Apex Autonomous Logistics Platform.

Autonomous Global Supply Chain, Maritime Freight & Multi-Modal Logistics Operating System:
  - Engine 1: Multi-Depot Capacitated Vehicle Routing with Time Windows (MD-VRPTW)
  - Engine 2: 3D Intermodal Container Bin Packing with CoG & Axle Weight Constraints
  - Engine 3: Multi-Echelon Bullwhip Suppressor & Kalman Demand Sensing
  - Engine 4: Autonomous Vessel Weather & Ocean Current Navigation (Zermelo Navigation)
  - Engine 5: Dynamic Cross-Docking Terminal Assignment & Flow Shop Scheduler
  - Engine 6: Cold-Chain Arrhenius Degradation & Thermal Excursion Governor
  - Engine 7: Collaborative Air-Ground Drone-Van Multi-Modal Dispatch (FSTSP)
  - Engine 8: Supply Chain Disruption Resilience & Multi-Commodity Min-Cost Flow

Zero external dependencies. Pure Python 3.10+ standard library.
"""

__version__ = "0.1.0"

from .vrptw import Customer, Depot, Vehicle, VRPTWInstance, VRPTWSolver, VRPTWSolution, Route
from .container_3d import BoxItem, Container, PlacedBox, CoGResult, ContainerPackingResult, ContainerPacker
from .inventory import (
    EchelonNode,
    DemandObservation,
    MultiEchelonNetwork,
    MultiEchelonSimulationResult,
    KalmanDemandFilter,
    BullwhipSuppressor,
)
from .maritime import GeoPoint, VesselConfig, OceanCurrentField, VoyageRoute, CIIRating, MaritimeWeatherRouter
from .cross_dock import Door, InboundTrailer, OutboundTrailer, CrossDockTerminal, CrossDockPlan, CrossDockScheduler
from .cold_chain import ColdChainProduct, TemperatureLog, ReeferContainer, ColdChainStatus, ColdChainGovernor
from .drone_van import DeliveryStop, VanConfig, DroneConfig, DroneSortie, FSTSPMission, FSTSPSolution, DroneVanDispatcher
from .resilience import (
    SupplyNode,
    TransportEdge,
    CommodityDemand,
    SupplyChainGraph,
    MinCostFlowResult,
    ChokePointRisk,
    NetworkVulnerabilityReport,
    DisruptionResilienceEngine,
)
from .benchmark import run_all_benchmarks, print_benchmark_report, EngineBenchmarkResult

__all__ = [
    # Engine 1
    "Customer",
    "Depot",
    "Vehicle",
    "VRPTWInstance",
    "VRPTWSolver",
    "VRPTWSolution",
    "Route",
    # Engine 2
    "BoxItem",
    "Container",
    "PlacedBox",
    "CoGResult",
    "ContainerPackingResult",
    "ContainerPacker",
    # Engine 3
    "EchelonNode",
    "DemandObservation",
    "MultiEchelonNetwork",
    "MultiEchelonSimulationResult",
    "KalmanDemandFilter",
    "BullwhipSuppressor",
    # Engine 4
    "GeoPoint",
    "VesselConfig",
    "OceanCurrentField",
    "VoyageRoute",
    "CIIRating",
    "MaritimeWeatherRouter",
    # Engine 5
    "Door",
    "InboundTrailer",
    "OutboundTrailer",
    "CrossDockTerminal",
    "CrossDockPlan",
    "CrossDockScheduler",
    # Engine 6
    "ColdChainProduct",
    "TemperatureLog",
    "ReeferContainer",
    "ColdChainStatus",
    "ColdChainGovernor",
    # Engine 7
    "DeliveryStop",
    "VanConfig",
    "DroneConfig",
    "DroneSortie",
    "FSTSPMission",
    "FSTSPSolution",
    "DroneVanDispatcher",
    # Engine 8
    "SupplyNode",
    "TransportEdge",
    "CommodityDemand",
    "SupplyChainGraph",
    "MinCostFlowResult",
    "ChokePointRisk",
    "NetworkVulnerabilityReport",
    "DisruptionResilienceEngine",
    # Benchmark
    "run_all_benchmarks",
    "print_benchmark_report",
    "EngineBenchmarkResult",
]
