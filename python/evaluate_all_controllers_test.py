import os
import traci
import numpy as np
import pandas as pd
from fuzzy_parameterized import create_fuzzy_controller

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
NETWORK_FILE = os.path.join(_PROJECT_ROOT, "sumo", "intersection.net.xml")
TEST_SCENARIOS = {
    "Unseen_Asymmetric": os.path.join(_PROJECT_ROOT, "sumo", "test_scenarios", "test_unseen_asymmetric.rou.xml"),
    "Unseen_Stress":     os.path.join(_PROJECT_ROOT, "sumo", "test_scenarios", "test_unseen_stress.rou.xml")
}
RESULTS_DIR = os.path.join(_PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

TLS_ID = "A0"
EMERGENCY_VEHICLE_ID = "emergency1"
GREEN_PHASES = [0, 2]
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

# 4 CONTROLLERS
CONTROLLERS = {
    "Baseline_Fuzzy": [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2],
    "Old_PSO_Synthetic": [9.8354, 2.9543, 17.78, 8, 22.3155, 7.6659, 1.6186, 0.5, 4.2632, 0.5, 7.165, 4],
    "Single_Scenario_PSO": [10.0, 4.8843, 22.0119, 4.6249, 22.4976, 1.0, 1.4879, 4.0, 6.7085, 3.2259, 7.3769, 0.5283],
    "Robust_Multi_PSO": [2.0796, 2.9734, 15.5670, 5.8443, 26.4280, 3.6692, 0.7954, 2.4311, 5.9661, 3.1725, 9.0442, 3.3297]
}

def evaluate_run(parameters, route_file, seed=42):
    controller = create_fuzzy_controller(parameters)

    sumo_cmd = [
        "sumo",
        "-n", NETWORK_FILE,
        "-r", route_file,
        "--no-step-log", "true",
        "--seed", str(seed)
    ]

    traci.start(sumo_cmd)
    previous_phase = -1
    emergency_seen = False
    emergency_start_time = None
    emergency_arrival_time = None

    ARRIVAL_INTERVAL = 10
    seen_incoming_vehicles = set()
    arrival_count = 0
    arrival_rate = 0
    last_arrival_measurement = 0

    total_delay = 0.0
    total_stops = 0
    completed_vehicles = 0
    total_queue = 0
    queue_measurements = 0
    maximum_queue = 0
    teleports = 0

    previous_time_loss = {}
    was_moving = {}

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

            # Teleport check
            teleports += traci.simulation.getStartingTeleportNumber()

            if EMERGENCY_VEHICLE_ID in traci.vehicle.getIDList():
                if not emergency_seen:
                    emergency_seen = True
                    emergency_start_time = current_time
            else:
                if emergency_seen and emergency_arrival_time is None:
                    emergency_arrival_time = current_time

            completed_vehicles += traci.simulation.getArrivedNumber()

            for v_id in traci.vehicle.getIDList():
                curr_tl = traci.vehicle.getTimeLoss(v_id)
                prev_tl = previous_time_loss.get(v_id, 0.0)
                total_delay += max(0.0, curr_tl - prev_tl)
                previous_time_loss[v_id] = curr_tl

                speed = traci.vehicle.getSpeed(v_id)
                moving_now = speed > 0.1
                if v_id not in was_moving:
                    was_moving[v_id] = moving_now
                else:
                    if was_moving[v_id] and not moving_now:
                        total_stops += 1
                    was_moving[v_id] = moving_now

            # Arrival rate
            current_incoming = set()
            for e_id in INCOMING_EDGES:
                current_incoming.update(traci.edge.getLastStepVehicleIDs(e_id))
            new_arrivals = current_incoming - seen_incoming_vehicles
            arrival_count += len(new_arrivals)
            seen_incoming_vehicles.update(new_arrivals)

            if current_time - last_arrival_measurement >= ARRIVAL_INTERVAL:
                arrival_rate = arrival_count
                arrival_count = 0
                last_arrival_measurement = current_time

            # Queue
            queue_length = 0
            for e_id in INCOMING_EDGES:
                lanes = traci.edge.getLaneNumber(e_id)
                for l_idx in range(lanes):
                    l_id = f"{e_id}_{l_idx}"
                    for v_id in traci.lane.getLastStepVehicleIDs(l_id):
                        if traci.vehicle.getSpeed(v_id) < 0.1:
                            queue_length += 1
            total_queue += queue_length
            queue_measurements += 1
            maximum_queue = max(maximum_queue, queue_length)

            # Signal control
            current_phase = traci.trafficlight.getPhase(TLS_ID)
            if current_phase != previous_phase:
                previous_phase = current_phase

                if EMERGENCY_VEHICLE_ID in traci.vehicle.getIDList():
                    em_road = traci.vehicle.getRoadID(EMERGENCY_VEHICLE_ID)
                    if em_road in INCOMING_EDGES:
                        traci.trafficlight.setPhase(TLS_ID, 0)
                        traci.trafficlight.setPhaseDuration(TLS_ID, 30)
                        continue

                if current_phase in GREEN_PHASES:
                    controller.input["queue"] = queue_length
                    controller.input["arrival"] = arrival_rate
                    controller.compute()
                    gc = controller.output["green_change"]
                    dur = max(20, min(60, 42 + gc))
                    traci.trafficlight.setPhaseDuration(TLS_ID, dur)

        mean_tl = total_delay / completed_vehicles if completed_vehicles > 0 else float("inf")
        mean_st = total_stops / completed_vehicles if completed_vehicles > 0 else 0
        mean_q = total_queue / queue_measurements if queue_measurements > 0 else 0
        em_time = (emergency_arrival_time - emergency_start_time) if (emergency_start_time and emergency_arrival_time) else -1

        ALPHA, BETA, GAMMA, DELTA = 0.50, 0.15, 0.20, 0.15
        em_pen = max(0.0, em_time - 90.0) / 90.0 if em_time > 0 else 0.0
        fitness = ALPHA * mean_tl + BETA * mean_st + GAMMA * mean_q + DELTA * em_pen * mean_tl

        return {
            "fitness": fitness,
            "delay": mean_tl,
            "stops": mean_st,
            "mean_queue": mean_q,
            "max_queue": maximum_queue,
            "emergency_time": em_time,
            "teleports": teleports,
            "throughput": completed_vehicles
        }
    finally:
        traci.close()

if __name__ == "__main__":
    seeds = [42, 101, 202, 303, 404]
    raw_records = []

    print("==========================================================================")
    print("  COMPREHENSIVE GENERALIZATION BENCHMARK ON UNSEEN TEST SCENARIOS")
    print("==========================================================================")

    for sc_name, sc_file in TEST_SCENARIOS.items():
        print(f"\n--- Testing Scenario: {sc_name} ---")
        for ctrl_name, params in CONTROLLERS.items():
            for s in seeds:
                res = evaluate_run(params, sc_file, seed=s)
                rec = {
                    "scenario": sc_name,
                    "controller": ctrl_name,
                    "seed": s,
                    **res
                }
                raw_records.append(rec)
                print(f"  [{sc_name}] {ctrl_name:<20} Seed {s:>3}: Delay={res['delay']:>6.2f}s | Stops={res['stops']:>5.2f} | MaxQ={res['max_queue']:>2d} | EmTime={res['emergency_time']:>5.1f}s | Teleports={res['teleports']}")

    df_raw = pd.DataFrame(raw_records)
    out_csv = os.path.join(RESULTS_DIR, "unseen_test_generalization_benchmark.csv")
    df_raw.to_csv(out_csv, index=False)
    print(f"\nRaw results saved to: {out_csv}")

    # Aggregated Summary Table
    print("\n" + "="*110)
    print(f"{'Scenario':<20} | {'Controller':<22} | {'Delay (s)':<18} | {'Stops':<14} | {'Mean Queue':<14} | {'Max Queue':<12} | {'Teleports'}")
    print("="*110)

    for (sc, ctrl), grp in df_raw.groupby(["scenario", "controller"]):
        d_mean, d_std = grp["delay"].mean(), grp["delay"].std()
        s_mean, s_std = grp["stops"].mean(), grp["stops"].std()
        q_mean, q_std = grp["mean_queue"].mean(), grp["mean_queue"].std()
        mq_mean       = grp["max_queue"].mean()
        tps_mean      = grp["teleports"].mean()

        print(f"{sc:<20} | {ctrl:<22} | {d_mean:>6.2f} ± {d_std:<7.2f} | {s_mean:>4.2f} ± {s_std:<5.2f} | {q_mean:>5.2f} ± {q_std:<6.2f} | {mq_mean:>5.1f}        | {tps_mean:>4.1f}")
    print("="*110)
