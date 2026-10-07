"""CLI for Apex Autonomous Logistics Platform."""

from __future__ import annotations

import argparse
import json
import sys

from .benchmark import run_all_benchmarks, print_benchmark_report
from .vrptw import Customer, Depot, Vehicle, VRPTWInstance, VRPTWSolver
from .container_3d import BoxItem, Container, ContainerPacker
from .inventory import EchelonNode, MultiEchelonNetwork, BullwhipSuppressor
from .maritime import GeoPoint, OceanCurrentField, VesselConfig, MaritimeWeatherRouter
from .cross_dock import CrossDockTerminal, Door, InboundTrailer, OutboundTrailer, CrossDockScheduler
from .cold_chain import ColdChainProduct, TemperatureLog, ReeferContainer, ColdChainGovernor
from .drone_van import DeliveryStop, DroneConfig, VanConfig, FSTSPMission, DroneVanDispatcher
from .resilience import SupplyNode, TransportEdge, CommodityDemand, SupplyChainGraph, DisruptionResilienceEngine


def cmd_benchmark(args: argparse.Namespace) -> None:
    """Run benchmark telemetry across all 8 logistics engines."""
    results = run_all_benchmarks(iterations=args.iterations)
    print_benchmark_report(results)


def cmd_demo(args: argparse.Namespace) -> None:
    """Run an end-to-end multi-modal logistics scenario demonstrating all engines."""
    print("Executing Apex Autonomous Logistics Platform End-to-End Orchestration...")
    print("-" * 75)

    # 1. Maritime shipping
    field = OceanCurrentField()
    field.add_jet_stream(lat=5.0, u_knots=4.0, v_knots=0.0)
    vessel = VesselConfig("GLOBAL_CARRIER_01", 140000.0)
    router = MaritimeWeatherRouter(field, vessel)
    voyage = router.compute_optimal_voyage(GeoPoint(1.0, 5.0), GeoPoint(9.0, 5.0))
    cii = router.evaluate_cii_rating(voyage)
    print(f"1. [MARITIME] Voyage {voyage.total_distance_nm:.0f} nm | Fuel: {voyage.total_fuel_burn_tons:.1f}t | IMO CII: Grade {cii.grade} (-{cii.co2_reduction_pct:.1f}% CO2)")

    # 2. Container packing
    container = Container("40HC_DEMO", length=12.0, width=2.35, height=2.69, max_payload=28000.0)
    boxes = [BoxItem(f"PALLET_{i}", 1.2, 1.0, 1.2, 750.0) for i in range(24)]
    packer = ContainerPacker()
    pack_res = packer.pack(container, boxes)
    print(f"2. [CONTAINER 3D] Packed {len(pack_res.placed_boxes)}/24 | Payload: {pack_res.total_weight:.0f} kg | CoG: ({pack_res.cog_metrics.x_cog:.2f}m, {pack_res.cog_metrics.y_cog:.2f}m) within envelope: {pack_res.cog_metrics.is_within_envelope}")

    # 3. Cross-docking
    in_doors = [Door(f"IN_{i}", "INBOUND", float(i * 20), 10.0) for i in range(3)]
    out_doors = [Door(f"OUT_{i}", "OUTBOUND", float(i * 20), 0.0) for i in range(3)]
    terminal = CrossDockTerminal("INTERMODAL_PORT", in_doors, out_doors, num_agvs=6)
    in_trs = [InboundTrailer(f"IT_{i}", 0.0, {f"OT_{j}": (i + j + 1) * 10 for j in range(3)}) for i in range(3)]
    out_trs = [OutboundTrailer(f"OT_{i}", 600.0, 100) for i in range(3)]
    cd_sched = CrossDockScheduler()
    cd_plan = cd_sched.optimize_schedule(terminal, in_trs, out_trs)
    print(f"3. [CROSS-DOCK] AGV Pallet Transfers: {cd_plan.total_pallet_distance_m:.0f} pallet-meters | AGV Util: {cd_plan.agv_utilization_pct:.1f}% | Feasible: {cd_plan.is_feasible}")

    # 4. Drone-van last-mile
    van = VanConfig("EV_VAN", 10.0)
    drone = DroneConfig("OCTOCOPTER", 20.0, 3.5, 1200.0)
    stops = [
        DeliveryStop("DEPOT", 0.0, 0.0, 0.0, False),
        DeliveryStop("URBAN_1", 1000.0, 0.0, 15.0, False),
        DeliveryStop("HILLTOP_ROOFTOP", 1500.0, 1000.0, 2.0, True),
        DeliveryStop("URBAN_2", 2000.0, 0.0, 20.0, False),
    ]
    fstsp = DroneVanDispatcher().solve(FSTSPMission(stops[0], stops[1:], van, drone))
    print(f"4. [LAST-MILE FSTSP] Makespan: {fstsp.mission_makespan_seconds:.1f}s | Speedup vs Pure Van: {fstsp.speedup_factor:.2f}x | Aerial Sorties: {len(fstsp.drone_sorties)}")

    # 5. Cold-chain
    prod = ColdChainProduct("BIOLOGIC_SERUM", -20.0, 5.0)
    reefer = ReeferContainer("REEFER_01", prod)
    logs = [TemperatureLog(float(h), -20.0) for h in range(72)]
    status = ColdChainGovernor().evaluate_shipment(reefer, logs)
    print(f"5. [COLD-CHAIN] Biologic Serum Telemetry: MKT {status.mean_kinetic_temp_c:.1f}°C | Status: {status.excursion_severity} | Action: {status.recommended_action}")

    # 6. Disruption resilience
    nodes = [SupplyNode("ORIGIN", "PRODUCER"), SupplyNode("CHOKE_A", "INTERMODAL"), SupplyNode("CHOKE_B", "INTERMODAL"), SupplyNode("DEST", "CONSUMER")]
    edges = [
        TransportEdge("ORIGIN", "CHOKE_A", 500.0, 30.0), TransportEdge("CHOKE_A", "DEST", 500.0, 20.0),
        TransportEdge("ORIGIN", "CHOKE_B", 800.0, 60.0), TransportEdge("CHOKE_B", "DEST", 800.0, 50.0),
    ]
    res_engine = DisruptionResilienceEngine()
    report = res_engine.assess_network_vulnerability(SupplyChainGraph(nodes, edges), [CommodityDemand("FREIGHT", "ORIGIN", "DEST", 500.0)])
    print(f"6. [NETWORK RESILIENCE] Choke-Point Audited: NRI={report.network_resilience_index:.2f} | Critical link: {report.critical_choke_points[0].edge_id} (cost esc: {report.critical_choke_points[0].cost_escalation_pct:.1f}%)")
    print("-" * 75)
    print("All 8 autonomous logistics engines verified operational.")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="apex-logistics",
        description="Autonomous Global Supply Chain & Multi-Modal Logistics Operating System",
    )
    sub = parser.add_subparsers(dest="command")

    # benchmark
    bp = sub.add_parser("benchmark", help="Run benchmark telemetry suite")
    bp.add_argument("--iterations", "-n", type=int, default=15, help="Number of test iterations")

    # demo
    sub.add_parser("demo", help="Run multi-modal end-to-end logistics demonstration")

    args = parser.parse_args()

    if args.command == "benchmark":
        cmd_benchmark(args)
    elif args.command == "demo":
        cmd_demo(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
