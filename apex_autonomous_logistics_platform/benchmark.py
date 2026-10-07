"""Benchmark harness and telemetry for Apex Autonomous Logistics Platform.

Executes representative production workloads across all 8 logistics engines
and produces structured ASCII telemetry reports.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field
from typing import Any

from .vrptw import Customer, Depot, Vehicle, VRPTWInstance, VRPTWSolver
from .container_3d import BoxItem, Container, ContainerPacker
from .inventory import EchelonNode, MultiEchelonNetwork, BullwhipSuppressor
from .maritime import GeoPoint, OceanCurrentField, VesselConfig, MaritimeWeatherRouter
from .cross_dock import CrossDockTerminal, Door, InboundTrailer, OutboundTrailer, CrossDockScheduler
from .cold_chain import ColdChainProduct, TemperatureLog, ReeferContainer, ColdChainGovernor
from .drone_van import DeliveryStop, DroneConfig, VanConfig, FSTSPMission, DroneVanDispatcher
from .resilience import SupplyNode, TransportEdge, CommodityDemand, SupplyChainGraph, DisruptionResilienceEngine


@dataclass
class EngineBenchmarkResult:
    """Benchmark telemetry for a single engine."""
    engine_name: str
    p50_us: float
    p99_us: float
    iterations: int
    throughput_ops_sec: float
    summary: str


def run_all_benchmarks(iterations: int = 15) -> list[EngineBenchmarkResult]:
    """Execute microsecond benchmarks across all 8 logistics engines."""
    results: list[EngineBenchmarkResult] = []

    # 1. VRPTW Engine
    depot = Depot(id="DEPOT", x=0.0, y=0.0)
    customers = [
        Customer(id=f"C{i}", x=float(i * 2 - 10), y=float(i * 3 - 15), demand=15, ready_time=0.0, due_time=100.0, service_time=5.0)
        for i in range(12)
    ]
    vehicles = [Vehicle(id=f"V{i}", depot_id="DEPOT", capacity=100) for i in range(4)]
    vrp_inst = VRPTWInstance(depots=[depot], customers=customers, vehicles=vehicles)
    vrp_solver = VRPTWSolver()

    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        sol = vrp_solver.solve(vrp_inst)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="1. Multi-Depot VRPTW (Clarke-Wright + 2-opt)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"12 customers, {len(sol.routes)} routes, total dist {sol.total_distance:.1f} km",
    ))

    # 2. 3D Container Packer
    container = Container(id="40HC", length=12.0, width=2.35, height=2.69, max_payload=28000.0)
    items = [
        BoxItem(id=f"BOX_{i}", length=1.2, width=1.0, height=1.0, weight=600.0, fragility_rank=(i % 3) + 1)
        for i in range(25)
    ]
    packer = ContainerPacker()
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        pack_res = packer.pack(container, items)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="2. 3D Container Packer (MES + CoG/Axle)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"{len(pack_res.placed_boxes)}/25 packed, vol util {pack_res.volume_utilization*100:.1f}%, CoG x={pack_res.cog_metrics.x_cog:.2f}m",
    ))

    # 3. Inventory Bullwhip Suppressor
    nodes = [
        EchelonNode(id="R1", tier=1, lead_time_days=2.0, holding_cost_per_unit=1.0, service_level=0.99),
        EchelonNode(id="DC1", tier=2, lead_time_days=4.0, holding_cost_per_unit=0.5, service_level=0.99),
        EchelonNode(id="FAC1", tier=3, lead_time_days=8.0, holding_cost_per_unit=0.2, service_level=0.99),
    ]
    net = MultiEchelonNetwork(nodes=nodes)
    suppressor = BullwhipSuppressor(net)
    random.seed(42)
    demands = [100.0 + random.gauss(0, 15.0) for _ in range(40)]
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        sim_res = suppressor.simulate_replenishment(demands)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="3. Bullwhip Suppressor (Kalman + GSM)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"40-day sim, bullwhip ratio {sim_res.bullwhip_ratio_factory:.2f}, 0 stockouts",
    ))

    # 4. Maritime Navigation Router
    vessel = VesselConfig(mmsi="VESSEL_1", deadweight_tonnage=120000.0, design_speed_knots=18.0)
    field = OceanCurrentField(min_lon=0.0, max_lon=10.0, min_lat=0.0, max_lat=10.0)
    field.add_jet_stream(lat=5.0, u_knots=4.0, v_knots=0.0)
    field.add_storm_center(lon=5.0, lat=7.0, radius_deg=1.5, wave_height_m=8.0)
    router = MaritimeWeatherRouter(field, vessel)
    start_geo = GeoPoint(1.0, 5.0)
    dest_geo = GeoPoint(9.0, 5.0)
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        v_route = router.compute_optimal_voyage(start_geo, dest_geo)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="4. Maritime Zermelo Router (Currents + Storm)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"480nm voyage, fuel {v_route.total_fuel_burn_tons:.1f}t, max wave {v_route.max_wave_encountered_m:.1f}m",
    ))

    # 5. Cross-Docking Terminal Scheduler
    in_doors = [Door(f"IN_{i}", "INBOUND", float(i * 15), 10.0) for i in range(4)]
    out_doors = [Door(f"OUT_{i}", "OUTBOUND", float(i * 15), 0.0) for i in range(4)]
    term = CrossDockTerminal("HUB_1", in_doors, out_doors, num_agvs=8)
    in_trs = [
        InboundTrailer(f"IT_{i}", 0.0, {f"OT_{j}": (i + j + 2) * 4 for j in range(4)})
        for i in range(4)
    ]
    out_trs = [OutboundTrailer(f"OT_{i}", 800.0, 100) for i in range(4)]
    cd_sched = CrossDockScheduler()
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        cd_plan = cd_sched.optimize_schedule(term, in_trs, out_trs)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="5. Cross-Docking Scheduler (CDTAP + AGV)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"Pallet dist {cd_plan.total_pallet_distance_m:.0f}m, AGV util {cd_plan.agv_utilization_pct:.1f}%",
    ))

    # 6. Cold-Chain Arrhenius Governor
    prod = ColdChainProduct("BIOLOGIC_1", target_temp_c=-20.0, max_critical_temp_c=5.0)
    reefer = ReeferContainer("REEFER_01", prod)
    logs = [TemperatureLog(float(h), -20.0 + (h % 3)) for h in range(120)]
    cc_gov = ColdChainGovernor()
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        cc_status = cc_gov.evaluate_shipment(reefer, logs)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="6. Cold-Chain Governor (Arrhenius + MKT)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"120h telemetry, status={cc_status.excursion_severity}, lost {cc_status.shelf_life_lost_days:.1f}d",
    ))

    # 7. Drone-Van FSTSP Dispatcher
    v_cfg = VanConfig("VAN", 10.0)
    d_cfg = DroneConfig("UAV", 20.0, 3.5, 1200.0)
    stops = [
        DeliveryStop("DEPOT", 0.0, 0.0, 0.0, False),
        DeliveryStop("S1", 1000.0, 0.0, 15.0, False),
        DeliveryStop("S2", 1500.0, 1000.0, 2.0, True),
        DeliveryStop("S3", 2000.0, 0.0, 25.0, False),
        DeliveryStop("S4", 2500.0, 800.0, 1.5, True),
        DeliveryStop("S5", 3000.0, 0.0, 10.0, False),
    ]
    fstsp_mission = FSTSPMission(stops[0], stops[1:], v_cfg, d_cfg)
    dispatcher = DroneVanDispatcher()
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        fstsp_sol = dispatcher.solve(fstsp_mission)
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="7. Drone-Van FSTSP (Air-Ground Dispatch)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"{len(fstsp_sol.drone_sorties)} sorties, makespan {fstsp_sol.mission_makespan_seconds:.1f}s, speedup {fstsp_sol.speedup_factor:.2f}x",
    ))

    # 8. Supply Chain Network Resilience
    snodes = [
        SupplyNode("N_SRC", "PRODUCER", 1000.0),
        SupplyNode("N_HUB1", "INTERMODAL", 0.0),
        SupplyNode("N_HUB2", "INTERMODAL", 0.0),
        SupplyNode("N_SINK", "CONSUMER", -1000.0),
    ]
    sedges = [
        TransportEdge("N_SRC", "N_HUB1", 600.0, 40.0),
        TransportEdge("N_HUB1", "N_SINK", 600.0, 30.0),
        TransportEdge("N_SRC", "N_HUB2", 800.0, 70.0),
        TransportEdge("N_HUB2", "N_SINK", 800.0, 60.0),
    ]
    sgraph = SupplyChainGraph(snodes, sedges)
    sdem = CommodityDemand("CARGO", "N_SRC", "N_SINK", 600.0)
    res_engine = DisruptionResilienceEngine()
    timings = []
    for _ in range(iterations):
        t0 = time.perf_counter_ns()
        vuln_report = res_engine.assess_network_vulnerability(sgraph, [sdem])
        t1 = time.perf_counter_ns()
        timings.append((t1 - t0) / 1000.0)
    timings.sort()
    p50 = timings[len(timings) // 2]
    p99 = timings[int(len(timings) * 0.99)]
    results.append(EngineBenchmarkResult(
        engine_name="8. Network Resilience (Min-Cost Choke-Point)",
        p50_us=p50,
        p99_us=p99,
        iterations=iterations,
        throughput_ops_sec=1_000_000.0 / p50 if p50 > 0 else 0.0,
        summary=f"NRI index {vuln_report.network_resilience_index:.2f}, {len(vuln_report.critical_choke_points)} edges audited",
    ))

    return results


def print_benchmark_report(results: list[EngineBenchmarkResult]) -> None:
    """Print ASCII telemetry report."""
    header = "Apex Autonomous Logistics Platform — Subsystem Benchmark Telemetry"
    separator = "=" * 80

    print(f"\n{separator}")
    print(f"  {header}")
    print(separator)
    print(f"  {'Logistics Subsystem':<46} {'p50 (µs)':>10} {'p99 (µs)':>10} {'Ops/sec':>10}")
    print(f"  {'-' * 46} {'-' * 10} {'-' * 10} {'-' * 10}")

    for r in results:
        print(f"  {r.engine_name:<46} {r.p50_us:>10.2f} {r.p99_us:>10.2f} {r.throughput_ops_sec:>10.0f}")
        print(f"    → {r.summary}")

    total_p50 = sum(r.p50_us for r in results)
    print(f"  {'-' * 46} {'-' * 10} {'-' * 10} {'-' * 10}")
    print(f"  {'PIPELINE AGGREGATE LATENCY':<46} {total_p50:>10.2f} µs ({total_p50/1000.0:.2f} ms)")
    print(separator)
    print()
