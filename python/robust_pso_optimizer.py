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

TRAIN_SCENARIOS = [
    os.path.join(TRAIN_DIR, "train_balanced.rou.xml"),
    os.path.join(TRAIN_DIR, "train_ns_heavy.rou.xml"),
    os.path.join(TRAIN_DIR, "train_ew_heavy.rou.xml"),
]

TLS_ID = "A0"
EMERGENCY_VEHICLE_ID = "emergency1"
GREEN_PHASES = [0, 2]
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

# Regularized parameter bounds preventing linguistic collapse
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
    """Enforce strict linguistic order and separation between centers."""
    p = list(parameters)

    # Queue centers
    p[0] = max(0.0, min(5.0, p[0]))
    p[2] = max(p[0] + 4.0, min(18.0, p[2]))
    p[4] = max(p[2] + 4.0, min(30.0, p[4]))

    # Queue spreads
    p[1] = max(2.0, min(6.0, p[1]))
    p[3] = max(2.0, min(6.0, p[3]))
    p[5] = max(2.0, min(6.0, p[5]))

    # Arrival centers
    p[6] = max(0.0, min(3.0, p[6]))
    p[8] = max(p[6] + 2.0, min(7.0, p[8]))
    p[10] = max(p[8] + 2.0, min(10.0, p[10]))

    # Arrival spreads
    p[7] = max(1.0, min(3.5, p[7]))
    p[9] = max(1.0, min(3.5, p[9]))
    p[11] = max(1.0, min(3.5, p[11]))

    return p

def evaluate_single_run(parameters, route_file, seed=42):
    """Run one microscopic SUMO simulation and compute 4-term fitness."""
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

    previous_time_loss = {}
    was_moving = {}

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

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
            return float("inf"), float("inf"), 0, 0, 0, 0

        mean_tl = total_delay / completed_vehicles
        mean_st = total_stops / completed_vehicles
        mean_q = total_queue / queue_measurements if queue_measurements > 0 else 0
        em_time = (emergency_arrival_time - emergency_start_time) if (emergency_start_time and emergency_arrival_time) else -1

        ALPHA, BETA, GAMMA, DELTA = 0.50, 0.15, 0.20, 0.15
        em_pen = max(0.0, em_time - 90.0) / 90.0 if em_time > 0 else 0.0
        fitness = ALPHA * mean_tl + BETA * mean_st + GAMMA * mean_q + DELTA * em_pen * mean_tl

        return fitness, mean_tl, mean_st, mean_q, maximum_queue, em_time
    finally:
        traci.close()

def evaluate_multi_scenario_robust(parameters, lambda_worst=0.50):
    """
    Evaluate candidate parameters across all training scenarios.
    Objective = Mean(Fitness) + lambda * Max(Fitness)
    """
    scenario_fitnesses = []
    for sc in TRAIN_SCENARIOS:
        f, _, _, _, _, _ = evaluate_single_run(parameters, sc, seed=42)
        scenario_fitnesses.append(f)

    mean_f  = np.mean(scenario_fitnesses)
    worst_f = np.max(scenario_fitnesses)
    robust_f = mean_f + lambda_worst * worst_f
    return robust_f, mean_f, worst_f, scenario_fitnesses

def run_robust_pso():
    random.seed(42)
    np.random.seed(42)

    PARTICLES  = 16
    ITERATIONS = 12
    W_MAX = 0.85
    W_MIN = 0.40
    C1 = 1.5
    C2 = 1.5
    LAMBDA_WORST = 0.50

    print("=" * 65)
    print("  ROBUST MULTI-SCENARIO FUZZY-PSO OPTIMIZER")
    print("=" * 65)
    print(f"Particles: {PARTICLES} | Iterations: {ITERATIONS}")
    print(f"Training Scenarios: {len(TRAIN_SCENARIOS)} (Balanced, NS-Heavy, EW-Heavy)")
    print(f"Robust Objective: J_robust = Mean(J) + {LAMBDA_WORST} * Max(J)\n")

    # Particle 0 = Hand-tuned Baseline
    # Particle 1 = Data-driven regularized seed
    # Particles 2..M = Uniform within bounded semantic regions
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
        rob_f, mean_f, worst_f, sc_f = evaluate_multi_scenario_robust(pos, LAMBDA_WORST)

        print(f"Init Particle {i+1:>2}/{PARTICLES}: J_robust={rob_f:.4f} (Mean={mean_f:.4f}, Worst={worst_f:.4f})")

        particles.append({
            "position": pos,
            "velocity": vel,
            "best_position": list(pos),
            "best_fitness": rob_f,
            "mean_fitness": mean_f,
            "worst_fitness": worst_f
        })

    # Global Best
    global_best = min(particles, key=lambda p: p["best_fitness"])
    gbest_pos = list(global_best["best_position"])
    gbest_fit = global_best["best_fitness"]

    print(f"\nInitial Global Best Robust Fitness: {gbest_fit:.4f}\n")

    conv_log = []
    conv_log.append({
        "iteration": 0,
        "best_robust_fitness": gbest_fit,
        "mean_fitness": global_best["mean_fitness"],
        "worst_fitness": global_best["worst_fitness"],
        "inertia": W_MAX
    })

    # Main PSO Loop
    for it in range(1, ITERATIONS + 1):
        w = W_MAX - (W_MAX - W_MIN) * (it / ITERATIONS)
        t0 = time.time()
        print(f"--- Iteration {it}/{ITERATIONS} (W = {w:.3f}) ---")

        for idx, p in enumerate(particles):
            # Update velocity
            for d in range(12):
                r1, r2 = random.random(), random.random()
                cog = C1 * r1 * (p["best_position"][d] - p["position"][d])
                soc = C2 * r2 * (gbest_pos[d] - p["position"][d])
                p["velocity"][d] = w * p["velocity"][d] + cog + soc
                # Clip velocity
                p["velocity"][d] = max(-1.0, min(1.0, p["velocity"][d]))
                p["position"][d] += p["velocity"][d]

            # Enforce bounds & semantic spacing
            p["position"] = enforce_semantic_spacing(p["position"])

            # Evaluate
            rob_f, mean_f, worst_f, _ = evaluate_multi_scenario_robust(p["position"], LAMBDA_WORST)

            if rob_f < p["best_fitness"]:
                p["best_fitness"] = rob_f
                p["best_position"] = list(p["position"])
                p["mean_fitness"] = mean_f
                p["worst_fitness"] = worst_f

            if rob_f < gbest_fit:
                gbest_fit = rob_f
                gbest_pos = list(p["position"])
                print(f"  * NEW GLOBAL BEST at Particle {idx+1}: J_robust={gbest_fit:.4f} (Mean={mean_f:.4f}, Worst={worst_f:.4f})")

        elapsed = time.time() - t0
        print(f"End Iteration {it}: Best J_robust = {gbest_fit:.4f} ({elapsed:.1f}s)\n")

        conv_log.append({
            "iteration": it,
            "best_robust_fitness": gbest_fit,
            "inertia": round(w, 4)
        })

    print("=" * 65)
    print("ROBUST OPTIMIZATION COMPLETE!")
    print(f"Optimal Robust Fitness: {gbest_fit:.4f}")
    print("Optimal Parameter Vector:")
    print([round(x, 4) for x in gbest_pos])
    print("=" * 65)

    # Save convergence CSV
    conv_file = os.path.join(RESULTS_DIR, "robust_pso_convergence.csv")
    with open(conv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["iteration", "best_robust_fitness", "inertia"])
        writer.writeheader()
        writer.writerows(conv_log)
    print(f"Saved convergence log to {conv_file}")

    # Save optimal parameters python snippet
    param_file = os.path.join(_PROJECT_ROOT, "python", "robust_optimal_parameters.py")
    with open(param_file, "w") as f:
        f.write(f"# Robust Multi-Scenario PSO Optimal Parameters\n")
        f.write(f"ROBUST_PSO_PARAMETERS = {[round(x, 4) for x in gbest_pos]}\n")
        f.write(f"ROBUST_FITNESS = {round(gbest_fit, 4)}\n")
    print(f"Saved parameters to {param_file}")

if __name__ == "__main__":
    run_robust_pso()
