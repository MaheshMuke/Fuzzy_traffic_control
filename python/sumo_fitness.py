# import traci

# from fuzzy_parameterized import create_fuzzy_controller


# # ============================================================
# # SUMO SETTINGS
# # ============================================================

# SUMO_BINARY = "sumo"

# NETWORK_FILE = r"C:\FuzzyPSO\sumo\intersection.net.xml"
# ROUTE_FILE = r"C:\FuzzyPSO\sumo\traffic.rou.xml"

# TLS_ID = "A0"

# GREEN_PHASES = [0, 2]

# INCOMING_EDGES = [
#     "A1A0",
#     "B0A0",
#     "bottom0A0",
#     "left0A0"
# ]


# # ============================================================
# # EVALUATE ONE FUZZY PARAMETER SET
# # ============================================================

# def evaluate_parameters(parameters):

#     # Create fuzzy controller
#     controller = create_fuzzy_controller(parameters)

#     sumo_cmd = [
#         SUMO_BINARY,
#         "-n", NETWORK_FILE,
#         "-r", ROUTE_FILE,
#         "--no-step-log",
#         "true",
#         "--seed",
#         "42"
#     ]

#     traci.start(sumo_cmd)

#     previous_phase = -1

#     # ========================================================
#     # ARRIVAL RATE VARIABLES
#     # ========================================================

#     ARRIVAL_INTERVAL = 10

#     seen_incoming_vehicles = set()

#     arrival_count = 0

#     arrival_rate = 0

#     last_arrival_measurement = 0


#     # ========================================================
#     # PERFORMANCE VARIABLES
#     # ========================================================

#     total_delay = 0.0

#     total_stops = 0

#     completed_vehicles = 0

#     previous_time_loss = {}

#     was_moving = {}


#     try:

#         while traci.simulation.getMinExpectedNumber() > 0:

#             traci.simulationStep()


#             # =================================================
#             # COMPLETED VEHICLES
#             # =================================================

#             completed_vehicles += (
#                 traci.simulation.getArrivedNumber()
#             )


#             # =================================================
#             # TIME LOSS
#             # =================================================

#             vehicle_ids = traci.vehicle.getIDList()

#             for vehicle_id in vehicle_ids:

#                 current_time_loss = (
#                     traci.vehicle.getTimeLoss(
#                         vehicle_id
#                     )
#                 )

#                 previous_loss = (
#                     previous_time_loss.get(
#                         vehicle_id,
#                         0.0
#                     )
#                 )

#                 additional_loss = (
#                     current_time_loss
#                     - previous_loss
#                 )

#                 total_delay += max(
#                     0.0,
#                     additional_loss
#                 )

#                 previous_time_loss[vehicle_id] = (
#                     current_time_loss
#                 )


#                 # =============================================
#                 # STOPS
#                 # =============================================

#                 speed = traci.vehicle.getSpeed(
#                     vehicle_id
#                 )

#                 moving_now = speed > 0.1

#                 if vehicle_id not in was_moving:

#                     was_moving[vehicle_id] = (
#                         moving_now
#                     )

#                 else:

#                     if (
#                         was_moving[vehicle_id]
#                         and not moving_now
#                     ):

#                         total_stops += 1

#                     was_moving[vehicle_id] = (
#                         moving_now
#                     )


#             # =================================================
#             # ARRIVAL RATE
#             # =================================================

#             current_time = traci.simulation.getTime()

#             current_incoming_vehicles = set()

#             for edge_id in INCOMING_EDGES:

#                 vehicle_ids = (
#                     traci.edge.getLastStepVehicleIDs(
#                         edge_id
#                     )
#                 )

#                 for vehicle_id in vehicle_ids:

#                     current_incoming_vehicles.add(
#                         vehicle_id
#                     )


#             # Find vehicles that were not seen before
#             new_arrivals = (
#                 current_incoming_vehicles
#                 - seen_incoming_vehicles
#             )

#             arrival_count += len(new_arrivals)

#             seen_incoming_vehicles.update(
#                 new_arrivals
#             )


#             # Every 10 seconds update arrival rate
#             if (
#                 current_time
#                 - last_arrival_measurement
#                 >= ARRIVAL_INTERVAL
#             ):

#                 arrival_rate = arrival_count

#                 arrival_count = 0

#                 last_arrival_measurement = (
#                     current_time
#                 )


#             # =================================================
#             # FUZZY TRAFFIC SIGNAL CONTROL
#             # =================================================

#             current_phase = (
#                 traci.trafficlight.getPhase(
#                     TLS_ID
#                 )
#             )

#             if current_phase != previous_phase:

#                 previous_phase = current_phase

#                 if current_phase in GREEN_PHASES:

#                     # =========================================
#                     # QUEUE LENGTH
#                     # =========================================

#                     queue_length = 0

#                     for edge_id in INCOMING_EDGES:

#                         lane_count = (
#                             traci.edge.getLaneNumber(
#                                 edge_id
#                             )
#                         )

#                         for lane_index in range(
#                             lane_count
#                         ):

#                             lane_id = (
#                                 f"{edge_id}_{lane_index}"
#                             )

#                             vehicle_ids = (
#                                 traci.lane
#                                 .getLastStepVehicleIDs(
#                                     lane_id
#                                 )
#                             )

#                             for vehicle_id in vehicle_ids:

#                                 speed = (
#                                     traci.vehicle
#                                     .getSpeed(
#                                         vehicle_id
#                                     )
#                                 )

#                                 if speed < 0.1:

#                                     queue_length += 1


#                     # =========================================
#                     # FUZZY INPUTS
#                     # =========================================

#                     controller.input["queue"] = (
#                         queue_length
#                     )

#                     controller.input["arrival"] = (
#                         arrival_rate
#                     )

#                     controller.compute()


#                     # =========================================
#                     # FUZZY OUTPUT
#                     # =========================================

#                     green_change = (
#                         controller.output[
#                             "green_change"
#                         ]
#                     )


#                     # =========================================
#                     # NEW GREEN DURATION
#                     # =========================================

#                     base_green = 42

#                     new_duration = (
#                         base_green
#                         + green_change
#                     )

#                     new_duration = max(
#                         20,
#                         min(
#                             60,
#                             new_duration
#                         )
#                     )

#                     traci.trafficlight.setPhaseDuration(
#                         TLS_ID,
#                         new_duration
#                     )


#         # =====================================================
#         # FINAL METRICS
#         # =====================================================

#         if completed_vehicles == 0:

#             return float("inf"), 0, 0, 0


#         mean_time_loss = (
#             total_delay
#             / completed_vehicles
#         )

#         mean_stops = (
#             total_stops
#             / completed_vehicles
#         )


#         # =====================================================
#         # TEMPORARY FITNESS
#         # =====================================================

#         alpha = 0.8

#         beta = 0.2

#         fitness = (
#             alpha * mean_time_loss
#             + beta * mean_stops
#         )


#         return fitness, mean_time_loss, mean_stops, completed_vehicles


#     finally:

#         traci.close()


# # ============================================================
# # TEST
# # ============================================================

# if __name__ == "__main__":

#     test_parameters = [

#         0, 5,
#         15, 5,
#         30, 5,

#         0, 2,
#         5, 2,
#         10, 2

#     ]

#     fitness, mean_time_loss, mean_stops = (
#         evaluate_parameters(
#             test_parameters
#         )
#     )

#     print()
#     print("==============================")
#     print("SUMO FITNESS TEST")
#     print("==============================")

#     print(
#         "Fitness:",
#         round(fitness, 4)
#     )

#     print(
#         "Mean time loss:",
#         round(mean_time_loss, 4),
#         "seconds/vehicle"
#     )

#     print(
#         "Mean stops:",
#         round(mean_stops, 4),
#         "stops/vehicle"
#     )



import os
import traci

from fuzzy_parameterized import create_fuzzy_controller


# ==========================================
# SUMO CONFIGURATION
# ==========================================

SUMO_BINARY = "sumo"

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

NETWORK_FILE = os.path.join(_PROJECT_ROOT, "sumo", "intersection.net.xml")
ROUTE_FILE   = os.path.join(_PROJECT_ROOT, "sumo", "traffic.rou.xml")

TLS_ID = "A0"

EMERGENCY_VEHICLE_ID = "emergency1"

GREEN_PHASES = [0, 2]

INCOMING_EDGES = [
    "A1A0",
    "B0A0",
    "bottom0A0",
    "left0A0"
]


# ==========================================
# FITNESS EVALUATION
# ==========================================

def evaluate_parameters(parameters):

    controller = create_fuzzy_controller(parameters)

    sumo_cmd = [
        SUMO_BINARY,
        "-n", NETWORK_FILE,
        "-r", ROUTE_FILE,
        "--no-step-log", "true",
        "--seed", "42"
    ]

    traci.start(sumo_cmd)

    previous_phase = -1
    
    emergency_seen = False
    emergency_start_time = None
    emergency_arrival_time = None

    # ------------------------------------------
    # Arrival-rate measurement
    # ------------------------------------------

    ARRIVAL_INTERVAL = 10

    seen_incoming_vehicles = set()

    arrival_count = 0
    arrival_rate = 0

    last_arrival_measurement = 0


    # ------------------------------------------
    # Performance measurements
    # ------------------------------------------

    total_delay = 0.0

    total_stops = 0

    completed_vehicles = 0

    total_queue = 0

    queue_measurements = 0

    maximum_queue = 0


    # ------------------------------------------
    # Vehicle tracking
    # ------------------------------------------

    previous_time_loss = {}

    was_moving = {}


    try:

        while traci.simulation.getMinExpectedNumber() > 0:

            traci.simulationStep()
            # ==========================================
            # EMERGENCY VEHICLE TRACKING
            # ==========================================

            if EMERGENCY_VEHICLE_ID in traci.vehicle.getIDList():

                if not emergency_seen:
                    emergency_seen = True
                    emergency_start_time = traci.simulation.getTime()

            else:

                if emergency_seen and emergency_arrival_time is None:
                    emergency_arrival_time = traci.simulation.getTime()


            # ======================================
            # COMPLETED VEHICLES
            # ======================================

            completed_vehicles += traci.simulation.getArrivedNumber()


            # ======================================
            # TIME LOSS AND STOPS
            # ======================================

            vehicle_ids = traci.vehicle.getIDList()

            for vehicle_id in vehicle_ids:

                current_time_loss = (
                    traci.vehicle.getTimeLoss(vehicle_id)
                )

                previous_loss = previous_time_loss.get(
                    vehicle_id,
                    0.0
                )

                additional_loss = (
                    current_time_loss - previous_loss
                )

                total_delay += max(
                    0.0,
                    additional_loss
                )

                previous_time_loss[
                    vehicle_id
                ] = current_time_loss


                # ------------------------------
                # Stop detection
                # ------------------------------

                speed = traci.vehicle.getSpeed(
                    vehicle_id
                )

                moving_now = speed > 0.1

                if vehicle_id not in was_moving:

                    was_moving[
                        vehicle_id
                    ] = moving_now

                else:

                    if (
                        was_moving[vehicle_id]
                        and not moving_now
                    ):

                        total_stops += 1

                    was_moving[
                        vehicle_id
                    ] = moving_now


            # ======================================
            # ARRIVAL RATE
            # ======================================

            current_time = traci.simulation.getTime()

            current_incoming_vehicles = set()

            for edge_id in INCOMING_EDGES:

                vehicle_ids = (
                    traci.edge.getLastStepVehicleIDs(
                        edge_id
                    )
                )

                for vehicle_id in vehicle_ids:

                    current_incoming_vehicles.add(
                        vehicle_id
                    )


            new_arrivals = (
                current_incoming_vehicles
                - seen_incoming_vehicles
            )

            arrival_count += len(new_arrivals)

            seen_incoming_vehicles.update(
                new_arrivals
            )


            if (
                current_time
                - last_arrival_measurement
                >= ARRIVAL_INTERVAL
            ):

                arrival_rate = arrival_count

                arrival_count = 0

                last_arrival_measurement = current_time


            # ======================================
            # QUEUE MEASUREMENT
            # ======================================

            queue_length = 0

            for edge_id in INCOMING_EDGES:

                lane_count = (
                    traci.edge.getLaneNumber(
                        edge_id
                    )
                )

                for lane_index in range(lane_count):

                    lane_id = (
                        f"{edge_id}_{lane_index}"
                    )

                    vehicle_ids = (
                        traci.lane.getLastStepVehicleIDs(
                            lane_id
                        )
                    )

                    for vehicle_id in vehicle_ids:

                        speed = traci.vehicle.getSpeed(
                            vehicle_id
                        )

                        if speed < 0.1:

                            queue_length += 1


            # Record queue statistics

            total_queue += queue_length

            queue_measurements += 1

            maximum_queue = max(
                maximum_queue,
                queue_length
            )


            # ======================================
            # FUZZY SIGNAL CONTROL
            # ======================================

            current_phase = (
                traci.trafficlight.getPhase(
                    TLS_ID
                )
            )


            if current_phase != previous_phase:

                previous_phase = current_phase
                
                # ======================================
                # EMERGENCY VEHICLE OVERRIDE
                # ======================================

                emergency_present = (
                    EMERGENCY_VEHICLE_ID
                    in traci.vehicle.getIDList()
                )

                if emergency_present:

                    emergency_road = traci.vehicle.getRoadID(
                        EMERGENCY_VEHICLE_ID
                    )

                    # Give priority when the emergency vehicle
                    # is approaching the controlled intersection.
                    if emergency_road in INCOMING_EDGES:

                        traci.trafficlight.setPhase(
                            TLS_ID,
                            0
                        )

                        traci.trafficlight.setPhaseDuration(
                            TLS_ID,
                            30
                        )

                        continue


                if current_phase in GREEN_PHASES:

                    controller.input[
                        "queue"
                    ] = queue_length

                    controller.input[
                        "arrival"
                    ] = arrival_rate

                    controller.compute()


                    green_change = (
                        controller.output[
                            "green_change"
                        ]
                    )


                    base_green = 42

                    new_duration = (
                        base_green
                        + green_change
                    )


                    # Keep green time within limits

                    new_duration = max(
                        20,
                        min(
                            60,
                            new_duration
                        )
                    )


                    traci.trafficlight.setPhaseDuration(
                        TLS_ID,
                        new_duration
                    )


        # ==========================================
        # FINAL STATISTICS
        # ==========================================

        if completed_vehicles == 0:

            return (
                float("inf"),
                0,
                0,
                0,
                0,
                0
            )
            
        
        # ==========================================
        # EMERGENCY RESPONSE TIME
        # ==========================================

        if (
            emergency_start_time is not None
            and emergency_arrival_time is not None
        ):

            emergency_response_time = (
                emergency_arrival_time
                - emergency_start_time
            )

        else:

            emergency_response_time = -1


        mean_time_loss = (
            total_delay
            / completed_vehicles
        )


        mean_stops = (
            total_stops
            / completed_vehicles
        )


        if queue_measurements > 0:

            mean_queue = (
                total_queue
                / queue_measurements
            )

        else:

            mean_queue = 0


        # ==========================================
        # IMPROVED FITNESS FUNCTION  (4 terms)
        # ==========================================
        # Previously only 2 terms were used even though
        # mean_queue and emergency_response_time were
        # already being measured.  All 4 are now included.

        ALPHA = 0.50   # mean time loss per vehicle (primary)
        BETA  = 0.15   # mean stops per vehicle
        GAMMA = 0.20   # mean queue length
        DELTA = 0.15   # emergency vehicle response penalty

        # Emergency penalty: normalise response time against
        # an ideal baseline of 90 seconds free travel.
        EMERGENCY_BASELINE = 90.0

        if (
            emergency_response_time is not None
            and emergency_response_time > 0
        ):
            emergency_penalty = max(
                0.0,
                emergency_response_time - EMERGENCY_BASELINE
            ) / EMERGENCY_BASELINE
        else:
            emergency_penalty = 0.0

        fitness = (
            ALPHA * mean_time_loss
            + BETA  * mean_stops
            + GAMMA * mean_queue
            + DELTA * emergency_penalty * mean_time_loss
        )


        # ==========================================
        # RETURN RESULTS
        # ==========================================

        return (
            fitness,
            mean_time_loss,
            mean_stops,
            completed_vehicles,
            mean_queue,
            maximum_queue,
            emergency_response_time
        )

    finally:

        traci.close()


# ==========================================
# TEST
# ==========================================

if __name__ == "__main__":

    test_parameters = [
        0, 5,
        15, 5,
        30, 5,
        0, 2,
        5, 2,
        10, 2
    ]


    (
        fitness,
        mean_time_loss,
        mean_stops,
        throughput,
        mean_queue,
        maximum_queue,
        emergency_response_time
    ) = evaluate_parameters(
        test_parameters
    )
    
    print(
        "Emergency response time:",
        emergency_response_time,
        "seconds"
    )


    print("==============================")
    print("SUMO FITNESS TEST")
    print("==============================")

    print(
        "Fitness:",
        round(fitness, 4)
    )

    print(
        "Mean time loss:",
        round(mean_time_loss, 4),
        "seconds/vehicle"
    )

    print(
        "Mean stops:",
        round(mean_stops, 4),
        "stops/vehicle"
    )

    print(
        "Throughput:",
        throughput,
        "vehicles"
    )

    print(
        "Mean queue:",
        round(mean_queue, 4),
        "vehicles"
    )

    print(
        "Maximum queue:",
        maximum_queue,
        "vehicles"
    )