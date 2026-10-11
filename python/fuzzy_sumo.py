import traci
from fuzzy_controller import get_green_change

print("Starting SUMO...", flush=True)

import os
_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

sumo_cmd = [
    "sumo-gui",
    "-n", os.path.join(_ROOT, "sumo", "intersection.net.xml"),
    "-r", os.path.join(_ROOT, "sumo", "traffic.rou.xml"),
    "--start"
]

traci.start(sumo_cmd)

print("SUMO connected!", flush=True)

# Traffic light we are controlling
TLS_ID = "A0"

# Green phases for A0
GREEN_PHASES = [0, 2]

previous_phase = -1

for step in range(300):

    traci.simulationStep()

    # --------------------------------
    # Get current traffic information
    # --------------------------------
    
    # Count stopped vehicles approaching A0 only
    incoming_edges = [
    	"A1A0",
    	"B0A0",
    	"bottom0A0",
    	"left0A0"
    ]

    queue_length = 0

    for edge_id in incoming_edges:

    	lane_count = traci.edge.getLaneNumber(edge_id)

    	for lane_index in range(lane_count):

        	lane_id = f"{edge_id}_{lane_index}"

        	vehicle_ids = traci.lane.getLastStepVehicleIDs(lane_id)

        	for vehicle_id in vehicle_ids:

            		speed = traci.vehicle.getSpeed(vehicle_id)

            		if speed < 0.1:
                		queue_length += 1
    
    
    arrival_rate = traci.simulation.getDepartedNumber()

    # --------------------------------
    # Get current traffic-light phase
    # --------------------------------

    current_phase = traci.trafficlight.getPhase(TLS_ID)

    # Only make a decision when a new phase starts
    if current_phase != previous_phase:

        previous_phase = current_phase

        print(
            "\nNew phase:",
            current_phase,
            "| Queue:",
            queue_length,
            "| Arrivals:",
            arrival_rate,
            flush=True
        )

        # --------------------------------
        # Apply fuzzy control to green phase
        # --------------------------------

        if current_phase in GREEN_PHASES:

            green_change = get_green_change(
                queue_length,
                arrival_rate
            )

            # Original green duration = 42 seconds
            new_duration = 42 + green_change

            # Keep green time between 20 and 60 seconds
            new_duration = max(20, min(60, new_duration))

            traci.trafficlight.setPhaseDuration(
                TLS_ID,
                new_duration
            )

            print(
                "Fuzzy adjustment:",
                round(green_change, 2),
                "seconds",
                "| New green duration:",
                round(new_duration, 2),
                "seconds",
                flush=True
            )

# Close SUMO
traci.close()

print("Simulation finished!", flush=True)