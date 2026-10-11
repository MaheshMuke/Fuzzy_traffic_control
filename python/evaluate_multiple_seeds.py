import os
import traci
import numpy as np
from fuzzy_parameterized import create_fuzzy_controller

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
NETWORK_FILE = os.path.join(_PROJECT_ROOT, "sumo", "intersection.net.xml")
ROUTE_FILE   = os.path.join(_PROJECT_ROOT, "sumo", "traffic.rou.xml")

TLS_ID = "A0"
EMERGENCY_VEHICLE_ID = "emergency1"
GREEN_PHASES = [0, 2]
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

NEW_PSO_PARAMETERS = [
    10.0000, 4.8843,
    22.0119, 4.6249,
    22.4976, 1.0000,
    1.4879, 4.0000,
    6.7085, 3.2259,
    7.3769, 0.5283
]

BASELINE_PARAMETERS = [
    0, 5,
    15, 5,
    30, 5,
    0, 2,
    5, 2,
    10, 2
]

def evaluate_seed(parameters, seed, route_file=ROUTE_FILE):
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

            vehicle_ids = traci.vehicle.getIDList()
            for v_id in vehicle_ids:
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
            return None

        mean_tl = total_delay / completed_vehicles
        mean_st = total_stops / completed_vehicles
        mean_q = total_queue / queue_measurements if queue_measurements > 0 else 0
        em_time = (emergency_arrival_time - emergency_start_time) if (emergency_start_time and emergency_arrival_time) else -1

        ALPHA, BETA, GAMMA, DELTA = 0.50, 0.15, 0.20, 0.15
        em_pen = max(0.0, em_time - 90.0) / 90.0 if em_time > 0 else 0.0
        fitness = ALPHA * mean_tl + BETA * mean_st + GAMMA * mean_q + DELTA * em_pen * mean_tl

        return {
            "seed": seed,
            "fitness": fitness,
            "time_loss": mean_tl,
            "stops": mean_st,
            "mean_queue": mean_q,
            "max_queue": maximum_queue,
            "emergency_time": em_time,
            "throughput": completed_vehicles
        }
    finally:
        traci.close()

if __name__ == "__main__":
    seeds = [42, 101, 202, 303, 404, 505, 606, 707, 808, 909]
    print(f"Evaluating across {len(seeds)} random seeds: {seeds}\n")

    pso_results = []
    base_results = []

    print("Evaluating New PSO Controller across seeds...")
    for s in seeds:
        res = evaluate_seed(NEW_PSO_PARAMETERS, s)
        pso_results.append(res)
        print(f"  Seed {s:>4}: Delay={res['time_loss']:.4f}s, Stops={res['stops']:.4f}, MeanQueue={res['mean_queue']:.4f}, Fitness={res['fitness']:.4f}")

    print("\nEvaluating Baseline Controller across seeds...")
    for s in seeds:
        res = evaluate_seed(BASELINE_PARAMETERS, s)
        base_results.append(res)
        print(f"  Seed {s:>4}: Delay={res['time_loss']:.4f}s, Stops={res['stops']:.4f}, MeanQueue={res['mean_queue']:.4f}, Fitness={res['fitness']:.4f}")

    # Compute statistics
    print("\n" + "="*85)
    print(f"{'Seed':<6} | {'PSO Delay (s)':<14} | {'PSO Stops':<11} | {'PSO Fitness':<12} | {'Base Delay (s)':<14} | {'Base Fitness':<12}")
    print("="*85)
    for p, b in zip(pso_results, base_results):
        print(f"{p['seed']:<6} | {p['time_loss']:<14.4f} | {p['stops']:<11.4f} | {p['fitness']:<12.4f} | {b['time_loss']:<14.4f} | {b['fitness']:<12.4f}")
    print("="*85)

    p_delays = [r['time_loss'] for r in pso_results]
    p_stops = [r['stops'] for r in pso_results]
    p_fits = [r['fitness'] for r in pso_results]

    b_delays = [r['time_loss'] for r in base_results]
    b_stops = [r['stops'] for r in base_results]
    b_fits = [r['fitness'] for r in base_results]

    print("\nSUMMARY STATISTICS:")
    print(f"New PSO Delay:   Mean = {np.mean(p_delays):.4f}s, Std = {np.std(p_delays):.4f}s (Min: {np.min(p_delays):.4f}, Max: {np.max(p_delays):.4f})")
    print(f"New PSO Stops:   Mean = {np.mean(p_stops):.4f},   Std = {np.std(p_stops):.4f}   (Min: {np.min(p_stops):.4f}, Max: {np.max(p_stops):.4f})")
    print(f"New PSO Fitness: Mean = {np.mean(p_fits):.4f},   Std = {np.std(p_fits):.4f}")
    print()
    print(f"Baseline Delay:  Mean = {np.mean(b_delays):.4f}s, Std = {np.std(b_delays):.4f}s")
    print(f"Baseline Stops:  Mean = {np.mean(b_stops):.4f},   Std = {np.std(b_stops):.4f}")
    print(f"Baseline Fitness:Mean = {np.mean(b_fits):.4f},   Std = {np.std(b_fits):.4f}")
