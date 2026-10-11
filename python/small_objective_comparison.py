import os
import random
import csv
import time
import numpy as np
import traci

from fuzzy_parameterized import create_fuzzy_controller

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
NETWORK_FILE = os.path.join(_PROJECT_ROOT, "sumo", "intersection.net.xml")
TRAIN_DIR    = os.path.join(_PROJECT_ROOT, "sumo", "training_scenarios")
RESULTS_DIR  = os.path.join(_PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

TRAIN_SCENARIOS = {
    "balanced": os.path.join(TRAIN_DIR, "train_balanced.rou.xml"),
    "ns_heavy": os.path.join(TRAIN_DIR, "train_ns_heavy.rou.xml"),
    "ew_heavy": os.path.join(TRAIN_DIR, "train_ew_heavy.rou.xml"),
}

# Exact baseline fitness values for normalization (measured on Seed 42)
J_BASELINE = {
    "balanced": 736.75,
    "ns_heavy": 34.85,
    "ew_heavy": 1415.91
}

TLS_ID = "A0"
EMERGENCY_VEHICLE_ID = "emergency1"
GREEN_PHASES = [0, 2]
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

ROBUST_BOUNDS = [
    (0.0,  5.0),   # Queue Low center
    (2.0,  6.0),   # Queue Low spread
    (10.0, 18.0),  # Queue Med center
    (2.0,  6.0),   # Queue Med spread
    (22.0, 30.0),  # Queue High center
    (2.0,  6.0),   # Queue High spread

    (0.0,  3.0),   # Arrival Low center
    (1.0,  3.5),   # Arrival Low spread
    (4.0,  7.0),   # Arrival Med center
    (1.0,  3.5),   # Arrival Med spread
    (8.0, 10.0),   # Arrival High center
    (1.0,  3.5),   # Arrival High spread
]

def enforce_semantic_spacing(parameters):
    p = list(parameters)
    p[0] = max(0.0, min(5.0, p[0]))
    p[2] = max(p[0] + 4.0, min(18.0, p[2]))
    p[4] = max(p[2] + 4.0, min(30.0, p[4]))
    p[1] = max(2.0, min(6.0, p[1]))
    p[3] = max(2.0, min(6.0, p[3]))
    p[5] = max(2.0, min(6.0, p[5]))

    p[6] = max(0.0, min(3.0, p[6]))
    p[8] = max(p[6] + 2.0, min(7.0, p[8]))
    p[10] = max(p[8] + 2.0, min(10.0, p[10]))
    p[7] = max(1.0, min(3.5, p[7]))
    p[9] = max(1.0, min(3.5, p[9]))
    p[11] = max(1.0, min(3.5, p[11]))
    return p

def evaluate_single_run(parameters, route_file, seed=42):
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

        if completed_vehicles == 0:
            return float("inf"), float("inf"), 0, 0, 0, 0, 0

        mean_tl = total_delay / completed_vehicles
        mean_st = total_stops / completed_vehicles
        mean_q = total_queue / queue_measurements if queue_measurements > 0 else 0
        em_time = (emergency_arrival_time - emergency_start_time) if (emergency_start_time and emergency_arrival_time) else -1

        ALPHA, BETA, GAMMA, DELTA = 0.50, 0.15, 0.20, 0.15
        em_pen = max(0.0, em_time - 90.0) / 90.0 if em_time > 0 else 0.0
        fitness = ALPHA * mean_tl + BETA * mean_st + GAMMA * mean_q + DELTA * em_pen * mean_tl

        return fitness, mean_tl, mean_st, mean_q, maximum_queue, em_time, teleports
    finally:
        traci.close()

def evaluate_all_scenarios(parameters, seed=42):
    """Evaluate candidate parameters on all 3 training scenarios."""
    res = {}
    for sc_name, sc_path in TRAIN_SCENARIOS.items():
        f, d, s, mq, maxq, emt, tp = evaluate_single_run(parameters, sc_path, seed=seed)
        res[sc_name] = {
            "fitness": f,
            "delay": d,
            "stops": s,
            "mean_queue": mq,
            "max_queue": maxq,
            "emergency_time": emt,
            "teleports": tp
        }
    return res

def compute_objective_value(sc_results, formulation):
    """
    Formulations:
    A: J_raw           = mean(J_k) + 0.5 * max(J_k)
    B: J_norm          = mean(R_k) + 0.5 * max(R_k)
    C: J_norm25        = mean(R_k) + 0.25 * max(R_k)
    D: J_norm_weighted = 0.7 * mean(R_k) + 0.3 * max(R_k)
    """
    j_vals = [sc_results["balanced"]["fitness"], sc_results["ns_heavy"]["fitness"], sc_results["ew_heavy"]["fitness"]]
    r_vals = [
        sc_results["balanced"]["fitness"] / J_BASELINE["balanced"],
        sc_results["ns_heavy"]["fitness"] / J_BASELINE["ns_heavy"],
        sc_results["ew_heavy"]["fitness"] / J_BASELINE["ew_heavy"]
    ]

    mean_j = float(np.mean(j_vals))
    max_j  = float(np.max(j_vals))

    mean_r = float(np.mean(r_vals))
    max_r  = float(np.max(r_vals))

    if formulation == "A":
        return mean_j + 0.5 * max_j, mean_j, max_j, mean_r, max_r
    elif formulation == "B":
        return mean_r + 0.5 * max_r, mean_j, max_j, mean_r, max_r
    elif formulation == "C":
        return mean_r + 0.25 * max_r, mean_j, max_j, mean_r, max_r
    elif formulation == "D":
        return 0.7 * mean_r + 0.3 * max_r, mean_j, max_j, mean_r, max_r
    else:
        raise ValueError(f"Unknown formulation {formulation}")

def run_diagnostic_pso(formulation_id, form_name):
    random.seed(42)
    np.random.seed(42)

    PARTICLES = 10
    ITERATIONS = 10
    W_MAX = 0.85
    W_MIN = 0.40
    C1 = 1.5
    C2 = 1.5

    print("\n" + "=" * 70)
    print(f"  RUNNING FORMULATION {formulation_id}: {form_name}")
    print("=" * 70)

    particles = []
    seed_0 = [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2] # baseline
    seed_1 = [2.0, 4.0, 14.0, 4.0, 26.0, 4.0, 1.0, 2.0, 5.0, 2.0, 9.0, 2.0]

    for i in range(PARTICLES):
        if i == 0:
            pos = enforce_semantic_spacing(seed_0)
        elif i == 1:
            pos = enforce_semantic_spacing(seed_1)
        else:
            pos = [random.uniform(b[0], b[1]) for b in ROBUST_BOUNDS]
            pos = enforce_semantic_spacing(pos)

        vel = [random.uniform(-0.5, 0.5) for _ in range(12)]
        sc_res = evaluate_all_scenarios(pos, seed=42)
        obj_val, mean_j, max_j, mean_r, max_r = compute_objective_value(sc_res, formulation_id)

        print(f"  Init P{i+1:>2}: Obj={obj_val:.4f} (Mean_R={mean_r:.3f}, Max_R={max_r:.3f})")

        particles.append({
            "position": pos,
            "velocity": vel,
            "best_position": list(pos),
            "best_objective": obj_val,
            "sc_results": sc_res
        })

    global_best = min(particles, key=lambda p: p["best_objective"])
    gbest_pos = list(global_best["best_position"])
    gbest_obj = global_best["best_objective"]
    gbest_sc  = global_best["sc_results"]

    print(f"Initial Global Best Objective: {gbest_obj:.4f}")

    conv = [gbest_obj]

    for it in range(1, ITERATIONS + 1):
        w = W_MAX - (W_MAX - W_MIN) * (it / ITERATIONS)
        t0 = time.time()

        for idx, p in enumerate(particles):
            for d in range(12):
                r1, r2 = random.random(), random.random()
                cog = C1 * r1 * (p["best_position"][d] - p["position"][d])
                soc = C2 * r2 * (gbest_pos[d] - p["position"][d])
                p["velocity"][d] = w * p["velocity"][d] + cog + soc
                p["velocity"][d] = max(-1.0, min(1.0, p["velocity"][d]))
                p["position"][d] += p["velocity"][d]

            p["position"] = enforce_semantic_spacing(p["position"])
            sc_res = evaluate_all_scenarios(p["position"], seed=42)
            obj_val, _, _, _, _ = compute_objective_value(sc_res, formulation_id)

            if obj_val < p["best_objective"]:
                p["best_objective"] = obj_val
                p["best_position"] = list(p["position"])
                p["sc_results"] = sc_res

            if obj_val < gbest_obj:
                gbest_obj = obj_val
                gbest_pos = list(p["position"])
                gbest_sc  = sc_res
                print(f"    * Iter {it} P{idx+1}: New Best Obj = {gbest_obj:.4f}")

        elapsed = time.time() - t0
        conv.append(gbest_obj)
        print(f"  Iter {it:>2}/{ITERATIONS} (W={w:.3f}): Best Obj = {gbest_obj:.4f} ({elapsed:.1f}s)")

    print(f"\nFinal Best Parameters for {formulation_id}:")
    print([round(x, 4) for x in gbest_pos])

    return {
        "formulation_id": formulation_id,
        "formulation_name": form_name,
        "best_objective": gbest_obj,
        "best_parameters": gbest_pos,
        "sc_results": gbest_sc,
        "convergence": conv
    }

if __name__ == "__main__":
    t_start = time.time()

    formulations = [
        ("A", "Current Raw (J_raw = Mean + 0.5 * Max)"),
        ("B", "Baseline-Normalized (J_norm = Mean(R) + 0.5 * Max(R))"),
        ("C", "Normalized Lower Penalty (J_norm25 = Mean(R) + 0.25 * Max(R))"),
        ("D", "Normalized Weighted (J_weighted = 0.7*Mean(R) + 0.3*Max(R))")
    ]

    all_results = []
    for f_id, f_name in formulations:
        res = run_diagnostic_pso(f_id, f_name)
        all_results.append(res)

    total_time = time.time() - t_start
    print(f"\nDiagnostic experiment completed in {total_time:.1f} seconds ({total_time/60:.1f} minutes).")

    # Save detailed summary to CSV
    summary_rows = []
    for r in all_results:
        f_id = r["formulation_id"]
        sc = r["sc_results"]
        p = r["best_parameters"]

        r_bal = sc["balanced"]["fitness"] / J_BASELINE["balanced"]
        r_ns  = sc["ns_heavy"]["fitness"] / J_BASELINE["ns_heavy"]
        r_ew  = sc["ew_heavy"]["fitness"] / J_BASELINE["ew_heavy"]
        mean_r = np.mean([r_bal, r_ns, r_ew])
        max_r  = np.max([r_bal, r_ns, r_ew])

        summary_rows.append({
            "Formulation": f_id,
            "Name": r["formulation_name"],
            "Objective_Val": r["best_objective"],
            "Mean_Norm_R": mean_r,
            "Worst_Norm_R": max_r,

            "Bal_Fitness": sc["balanced"]["fitness"],
            "Bal_Delay": sc["balanced"]["delay"],
            "Bal_Stops": sc["balanced"]["stops"],
            "Bal_MeanQ": sc["balanced"]["mean_queue"],
            "Bal_MaxQ": sc["balanced"]["max_queue"],
            "Bal_EmTime": sc["balanced"]["emergency_time"],
            "Bal_Teleports": sc["balanced"]["teleports"],

            "NS_Fitness": sc["ns_heavy"]["fitness"],
            "NS_Delay": sc["ns_heavy"]["delay"],
            "NS_Stops": sc["ns_heavy"]["stops"],
            "NS_MeanQ": sc["ns_heavy"]["mean_queue"],
            "NS_MaxQ": sc["ns_heavy"]["max_queue"],
            "NS_EmTime": sc["ns_heavy"]["emergency_time"],
            "NS_Teleports": sc["ns_heavy"]["teleports"],

            "EW_Fitness": sc["ew_heavy"]["fitness"],
            "EW_Delay": sc["ew_heavy"]["delay"],
            "EW_Stops": sc["ew_heavy"]["stops"],
            "EW_MeanQ": sc["ew_heavy"]["mean_queue"],
            "EW_MaxQ": sc["ew_heavy"]["max_queue"],
            "EW_EmTime": sc["ew_heavy"]["emergency_time"],
            "EW_Teleports": sc["ew_heavy"]["teleports"],

            "Parameters": str([round(x, 4) for x in p])
        })

    out_csv = os.path.join(RESULTS_DIR, "objective_formulation_comparison.csv")
    keys = summary_rows[0].keys()
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(summary_rows)
    print(f"Saved diagnostic summary to {out_csv}")
