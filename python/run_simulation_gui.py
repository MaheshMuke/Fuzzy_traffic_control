import os
import sys
import time
import traci
from fuzzy_parameterized import create_fuzzy_controller

# ============================================================
# CONTROLLER PARAMETER SETS
# ============================================================

# 1. Robust Multi-Scenario PSO Parameters (Generalizes across all suites)
ROBUST_PSO_PARAMETERS = [
    2.0796, 2.9734,
    15.5670, 5.8443,
    26.4280, 3.6692,
    0.7954, 2.4311,
    5.9661, 3.1725,
    9.0442, 3.3297
]

# 2. Single-Scenario PSO Parameters (Overfitted on symmetric training)
SINGLE_PSO_PARAMETERS = [
    10.0000, 4.8843,
    22.0119, 4.6249,
    22.4976, 1.0000,
    1.4879, 4.0000,
    6.7085, 3.2259,
    7.3769, 0.5283
]

# 3. Baseline Hand-Tuned Parameters
BASELINE_PARAMETERS = [
    0, 5,
    15, 5,
    30, 5,
    0, 2,
    5, 2,
    10, 2
]

# ============================================================
# PATHS
# ============================================================
PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
NETWORK_FILE = os.path.join(PROJECT_ROOT, "sumo", "intersection.net.xml")

SCENARIOS = {
    "train": os.path.join(PROJECT_ROOT, "sumo", "traffic.rou.xml"),
    "unseen_asymmetric": os.path.join(PROJECT_ROOT, "sumo", "test_scenarios", "test_unseen_asymmetric.rou.xml"),
    "unseen_stress": os.path.join(PROJECT_ROOT, "sumo", "test_scenarios", "test_unseen_stress.rou.xml"),
}

TLS_ID = "A0"
GREEN_PHASES = [0, 2]
EMERGENCY_VEHICLE_ID = "emergency1"
INCOMING_EDGES = ["A1A0", "B0A0", "bottom0A0", "left0A0"]

def run_gui_simulation(mode="robust", scenario_key="train", step_delay_ms=50):
    """
    Launch SUMO GUI and run the adaptive traffic controller visually.
    
    Parameters:
        mode: 'robust' (default), 'single', or 'baseline'
        scenario_key: 'train', 'unseen_asymmetric', or 'unseen_stress'
        step_delay_ms: delay in milliseconds per simulation step for comfortable viewing
    """
    if mode == "robust":
        params = ROBUST_PSO_PARAMETERS
        controller_name = "Robust Multi-Scenario PSO Fuzzy (Final Regularized)"
    elif mode == "single":
        params = SINGLE_PSO_PARAMETERS
        controller_name = "Single-Scenario PSO Fuzzy (Overfitted)"
    else:
        params = BASELINE_PARAMETERS
        controller_name = "Baseline Fuzzy (Hand-Tuned Uniform)"

    route_file = SCENARIOS.get(scenario_key, SCENARIOS["train"])

    print("\n" + "=" * 65)
    print(f"  LAUNCHING SUMO-GUI: {controller_name.upper()}")
    print("=" * 65)
    print(f"Network:      {NETWORK_FILE}")
    print(f"Routes:       {route_file}")
    print(f"Step delay:   {step_delay_ms} ms (adjust live using the slider in SUMO-GUI)")
    print("=" * 65 + "\n")

    controller = create_fuzzy_controller(params)

    sumo_cmd = [
        "sumo-gui",
        "-n", NETWORK_FILE,
        "-r", route_file,
        "--start",
        "--delay", str(step_delay_ms),
        "--seed", "42"
    ]

    try:
        traci.start(sumo_cmd)
    except Exception as e:
        print(f"Error starting sumo-gui: {e}")
        print("Please ensure sumo-gui is installed and accessible in your PATH.")
        return

    previous_phase = -1
    last_arrival_measurement = 0
    arrival_count = 0
    arrival_rate = 0
    seen_incoming_vehicles = set()
    emergency_handled = False

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

            # Track Arrivals
            current_incoming = set()
            for edge in INCOMING_EDGES:
                current_incoming.update(traci.edge.getLastStepVehicleIDs(edge))
            
            new_arrivals = current_incoming - seen_incoming_vehicles
            arrival_count += len(new_arrivals)
            seen_incoming_vehicles.update(new_arrivals)

            if current_time - last_arrival_measurement >= 10:
                arrival_rate = arrival_count
                arrival_count = 0
                last_arrival_measurement = current_time

            # Measure Queue
            queue_length = 0
            for edge in INCOMING_EDGES:
                lanes = traci.edge.getLaneNumber(edge)
                for l_idx in range(lanes):
                    lane_id = f"{edge}_{l_idx}"
                    for v_id in traci.lane.getLastStepVehicleIDs(lane_id):
                        if traci.vehicle.getSpeed(v_id) < 0.1:
                            queue_length += 1

            # Traffic Light Phase Control
            current_phase = traci.trafficlight.getPhase(TLS_ID)

            if current_phase != previous_phase:
                previous_phase = current_phase

                # Emergency vehicle override check
                if EMERGENCY_VEHICLE_ID in traci.vehicle.getIDList():
                    em_road = traci.vehicle.getRoadID(EMERGENCY_VEHICLE_ID)
                    if em_road in INCOMING_EDGES:
                        traci.trafficlight.setPhase(TLS_ID, 0)
                        traci.trafficlight.setPhaseDuration(TLS_ID, 30)
                        if not emergency_handled:
                            print(f"[{current_time:6.1f}s] *** EMERGENCY VEHICLE DETECTED ON {em_road} -> 30s PRIORITY GREEN GRANTED ***")
                            emergency_handled = True
                        continue

                # Fuzzy adaptive green duration
                if current_phase in GREEN_PHASES:
                    controller.input["queue"] = queue_length
                    controller.input["arrival"] = arrival_rate
                    controller.compute()

                    green_change = controller.output["green_change"]
                    base_green = 42
                    new_duration = max(20, min(60, base_green + green_change))

                    traci.trafficlight.setPhaseDuration(TLS_ID, new_duration)

                    print(f"[{current_time:6.1f}s] Phase: {current_phase} | Queue: {queue_length:2d} veh | Arrival Rate: {arrival_rate:2d} | "
                          f"Fuzzy ΔG: {green_change:+5.1f}s -> Green Duration: {new_duration:4.1f}s")

    except traci.exceptions.FatalTraCIError:
        print("\nSimulation GUI window was closed by user.")
    finally:
        try:
            traci.close()
        except Exception:
            pass
        print("\nSimulation finished cleanly!")

if __name__ == "__main__":
    mode = "robust"
    scenario = "train"

    if len(sys.argv) > 1:
        arg1 = sys.argv[1].lower()
        if arg1 in ["baseline", "base"]:
            mode = "baseline"
        elif arg1 in ["single", "overfitted"]:
            mode = "single"
        elif arg1 in ["robust", "final"]:
            mode = "robust"

    if len(sys.argv) > 2:
        arg2 = sys.argv[2].lower()
        if arg2 in ["asymmetric", "unseen_asymmetric"]:
            scenario = "unseen_asymmetric"
        elif arg2 in ["stress", "unseen_stress"]:
            scenario = "unseen_stress"
        elif arg2 in ["train", "balanced"]:
            scenario = "train"

    run_gui_simulation(mode=mode, scenario_key=scenario, step_delay_ms=40)
