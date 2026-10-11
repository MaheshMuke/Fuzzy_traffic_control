import os
import traci
from fuzzy_parameterized import create_fuzzy_controller

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
NETWORK_FILE = os.path.join(_PROJECT_ROOT, "sumo", "intersection.net.xml")
TEST_ROUTE_FILE = os.path.join(_PROJECT_ROOT, "sumo", "test_traffic.rou.xml")

TLS_ID = "A0"
EMERGENCY_VEHICLE_ID = "emergency1"
GREEN_PHASES = [0, 2]
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

# 1. Baseline parameters
BASELINE_PARAMETERS = [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2]

# 2. Old PSO parameters (trained on synthetic flat 60 veh/direction)
OLD_PSO_PARAMETERS = [9.8354, 2.9543, 17.78, 8, 22.3155, 7.6659, 1.6186, 0.5, 4.2632, 0.5, 7.165, 4]

# 3. New PSO parameters (trained on training scenario)
NEW_PSO_PARAMETERS = [10.0, 4.8843, 22.0119, 4.6249, 22.4976, 1.0, 1.4879, 4.0, 6.7085, 3.2259, 7.3769, 0.5283]

def generate_unseen_test_route_file(output_path=TEST_ROUTE_FILE):
    """
    Build an UNSEEN test scenario:
    - Extended 700s simulation (vs 600s train)
    - Asymmetric demand (higher south-north tidal flow)
    - Different surge timing (heavy peak from 150s to 380s)
    - Emergency vehicle arrives at t=260s (vs 325s in train)
    - Total ~275 vehicles (vs 219 in train)
    """
    xml = """<routes>
    <vType id="car" accel="2.6" decel="4.5" length="5" maxSpeed="13.9"/>
    <vType id="emergency" accel="3.0" decel="5.0" length="5" maxSpeed="15.0" color="1,0,0"/>

    <route id="leftRight" edges="left0A0 A0A1 A1B1 B1right1"/>
    <route id="rightLeft" edges="right1B1 B1A1 A1A0 A0left0"/>
    <route id="bottomTop" edges="bottom0A0 A0B0 B0B1 B1top1"/>
    <route id="topBottom" edges="top1B1 B1B0 B0A0 A0bottom0"/>

    <!-- UNSEEN TEST SCENARIO: 700 seconds -->
    <!-- Window 1: 0 - 150s (Moderate start) -->
    <flow id="test_flow_0" type="car" route="leftRight" begin="0" end="150" number="18"/>
    <flow id="test_flow_1" type="car" route="rightLeft" begin="0" end="150" number="18"/>
    <flow id="test_flow_2" type="car" route="bottomTop" begin="0" end="150" number="22"/>
    <flow id="test_flow_3" type="car" route="topBottom" begin="0" end="150" number="20"/>

    <!-- Window 2: 150 - 380s (Heavy Congestion Surge + Early Emergency Vehicle at 260s) -->
    <flow id="test_flow_4" type="car" route="leftRight" begin="150" end="380" number="25"/>
    <flow id="test_flow_5" type="car" route="rightLeft" begin="150" end="380" number="25"/>
    <flow id="test_flow_6" type="car" route="bottomTop" begin="150" end="380" number="32"/>
    <flow id="test_flow_7" type="car" route="topBottom" begin="150" end="380" number="28"/>
    <vehicle id="emergency1" type="emergency" route="leftRight" depart="260"/>

    <!-- Window 3: 380 - 550s (Tapering Moderate Peak) -->
    <flow id="test_flow_8" type="car" route="leftRight" begin="380" end="550" number="16"/>
    <flow id="test_flow_9" type="car" route="rightLeft" begin="380" end="550" number="16"/>
    <flow id="test_flow_10" type="car" route="bottomTop" begin="380" end="550" number="18"/>
    <flow id="test_flow_11" type="car" route="topBottom" begin="380" end="550" number="18"/>

    <!-- Window 4: 550 - 700s (Free-flow tail) -->
    <flow id="test_flow_12" type="car" route="leftRight" begin="550" end="700" number="10"/>
    <flow id="test_flow_13" type="car" route="rightLeft" begin="550" end="700" number="10"/>
    <flow id="test_flow_14" type="car" route="bottomTop" begin="550" end="700" number="12"/>
    <flow id="test_flow_15" type="car" route="topBottom" begin="550" end="700" number="12"/>
</routes>
"""
    with open(output_path, "w") as f:
        f.write(xml)
    print(f"Generated unseen test route file: {output_path}")

def evaluate_on_route(parameters, route_file, seed=42):
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
            "throughput": completed_vehicles
        }
    finally:
        traci.close()

if __name__ == "__main__":
    generate_unseen_test_route_file()

    print("\nEVALUATING ON UNSEEN TEST SCENARIO...")
    res_base = evaluate_on_route(BASELINE_PARAMETERS, TEST_ROUTE_FILE, seed=42)
    print("Baseline evaluated.")
    res_old  = evaluate_on_route(OLD_PSO_PARAMETERS, TEST_ROUTE_FILE, seed=42)
    print("Old PSO evaluated.")
    res_new  = evaluate_on_route(NEW_PSO_PARAMETERS, TEST_ROUTE_FILE, seed=42)
    print("New PSO evaluated.")

    header = f"{'Metric':<25} | {'Baseline':<12} | {'Old PSO':<12} | {'New PSO (Trained)':<18} | {'New vs Base (%)':<16}"
    sep = "=" * len(header)
    print("\n" + sep)
    print("      UNSEEN TEST SCENARIO BENCHMARK RESULTS (GENERALIZATION AUDIT)")
    print(sep)
    print(header)
    print(sep)

    metrics = [
        ("Fitness (Test Objective)", "fitness"),
        ("Mean Delay (s/veh)", "delay"),
        ("Mean Stops (stops/veh)", "stops"),
        ("Throughput (completed)", "throughput"),
        ("Mean Queue (veh)", "mean_queue"),
        ("Max Queue (veh)", "max_queue"),
        ("Emergency Time (s)", "emergency_time")
    ]

    for label, key in metrics:
        b = res_base[key]
        o = res_old[key]
        n = res_new[key]
        b_str = f"{b:.4f}" if isinstance(b, float) else f"{b}"
        o_str = f"{o:.4f}" if isinstance(o, float) else f"{o}"
        n_str = f"{n:.4f}" if isinstance(n, float) else f"{n}"
        imp = ((b - n) / b * 100) if (isinstance(b, (int, float)) and b != 0 and key != "throughput") else 0
        sign = "+" if imp > 0 else ""
        print(f"{label:<25} | {b_str:<12} | {o_str:<12} | {n_str:<18} | {sign}{imp:.2f}%")
    print(sep)
