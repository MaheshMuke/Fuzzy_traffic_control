"""
pso_optimizer.py
----------------
Particle Swarm Optimization that tunes the fuzzy controller's
Gaussian membership-function parameters.

Improvements over the original version
---------------------------------------
1. Dataset-driven route file
   - Before PSO starts, the real-world traffic dataset is loaded
     and a time-varying SUMO route file is written.  The simulation
     now covers light / moderate / heavy traffic in one run instead
     of a flat uniform flow.

2. More particles and iterations
   - PARTICLES: 10  ->  20
   - ITERATIONS: 10  ->  20
   Total evaluations: 100  ->  400

3. Adaptive inertia weight
   - W starts at W_MAX (0.9) for broad exploration and decays
     linearly to W_MIN (0.4) by the final iteration, preventing
     premature convergence.

4. Data-driven initial membership-function centres
   - The first PSO particle is seeded with centres derived from
     the 10th / 50th / 90th percentiles of the real dataset,
     rather than the hand-picked constants from the original.
"""

import os
import random

from sumo_fitness    import evaluate_parameters
from dataset_loader  import load_dataset, get_fuzzy_input_bounds
from generate_routes import generate_route_file


# ============================================================
# REPRODUCIBILITY
# ============================================================

random.seed(42)

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

RESULTS_FILE = os.path.join(_PROJECT_ROOT, "results", "pso_convergence.csv")


# ============================================================
# STEP 1 — LOAD DATASET & GENERATE REALISTIC ROUTE FILE
# ============================================================

print()
print("=" * 45)
print("LOADING REAL-WORLD TRAFFIC DATASET")
print("=" * 45)

dataset = load_dataset()

print(f"Dataset rows loaded: {len(dataset)}")
print()
print("Generating time-varying SUMO route file from dataset...")

generate_route_file()   # overwrites sumo/traffic.rou.xml

print("Route file ready.")


# ============================================================
# STEP 2 — DATA-DRIVEN FUZZY INITIAL PARAMETERS
# ============================================================

bounds_from_data = get_fuzzy_input_bounds(dataset)

print()
print("Data-driven fuzzy input centres:")
print(
    f"  Queue  low/med/high : "
    f"{bounds_from_data['queue_low_center']} / "
    f"{bounds_from_data['queue_med_center']} / "
    f"{bounds_from_data['queue_high_center']}"
)
print(
    f"  Arrival low/med/high : "
    f"{bounds_from_data['arrival_low_center']} / "
    f"{bounds_from_data['arrival_med_center']} / "
    f"{bounds_from_data['arrival_high_center']}"
)

# Original hand-picked baseline
BASELINE_PARAMETERS = [
    # Queue  (centre, sigma)
    0,  5,
    15, 5,
    30, 5,
    # Arrival (centre, sigma)
    0,  2,
    5,  2,
    10, 2,
]

# Data-driven seed (particle 0)
DATA_DRIVEN_PARAMETERS = [
    # Queue
    bounds_from_data["queue_low_center"],   5,
    bounds_from_data["queue_med_center"],   5,
    bounds_from_data["queue_high_center"],  5,
    # Arrival
    bounds_from_data["arrival_low_center"], 2,
    bounds_from_data["arrival_med_center"], 2,
    bounds_from_data["arrival_high_center"],2,
]


# ============================================================
# PSO SETTINGS
# ============================================================

PARTICLES  = 20     # was 10
ITERATIONS = 20     # was 10

W_MAX = 0.90        # inertia at iteration 0  (exploration)
W_MIN = 0.40        # inertia at final iter   (exploitation)

C1 = 1.5            # cognitive coefficient
C2 = 1.5            # social coefficient


# ============================================================
# PARAMETER BOUNDS
# ============================================================

BOUNDS = [
    (0,  10),    # Queue Low center
    (1,   8),    # Queue Low spread
    (5,  25),    # Queue Medium center
    (1,   8),    # Queue Medium spread
    (20, 30),    # Queue High center
    (1,   8),    # Queue High spread
    (0,   3),    # Arrival Low center
    (0.5, 4),    # Arrival Low spread
    (2,   8),    # Arrival Medium center
    (0.5, 4),    # Arrival Medium spread
    (7,  10),    # Arrival High center
    (0.5, 4),    # Arrival High spread
]


# ============================================================
# HELPERS
# ============================================================

def enforce_parameter_order(parameters):
    """Keep membership-function centres in Low -> Med -> High order."""
    parameters = parameters.copy()

    queue_centers = [parameters[0], parameters[2], parameters[4]]
    queue_centers.sort()
    parameters[0], parameters[2], parameters[4] = queue_centers

    arrival_centers = [parameters[6], parameters[8], parameters[10]]
    arrival_centers.sort()
    parameters[6], parameters[8], parameters[10] = arrival_centers

    return parameters


def run_fitness(parameters):
    """Evaluate parameters; return scalar fitness."""
    fitness, _, _, _, _, _, _ = evaluate_parameters(parameters)
    return fitness


def adaptive_inertia(iteration):
    """Linearly decay inertia weight from W_MAX to W_MIN."""
    return W_MAX - (W_MAX - W_MIN) * iteration / max(1, ITERATIONS - 1)


# ============================================================
# STEP 3 — INITIALISE PARTICLES
# ============================================================

particles = []

print()
print("=" * 45)
print("CREATING INITIAL PSO POPULATION")
print("=" * 45)


for i in range(PARTICLES):

    print()
    print(f"Testing initial particle {i + 1}/{PARTICLES}")

    if i == 0:
        # Particle 0 = data-driven centres
        position = DATA_DRIVEN_PARAMETERS.copy()
        print("  (data-driven seed from real dataset)")

    elif i == 1:
        # Particle 1 = original hand-picked baseline
        position = BASELINE_PARAMETERS.copy()
        print("  (original hand-picked baseline)")

    else:
        # Remaining particles = random within bounds
        position = [
            random.uniform(BOUNDS[d][0], BOUNDS[d][1])
            for d in range(len(BOUNDS))
        ]
        position = enforce_parameter_order(position)

    velocity = [
        random.uniform(-1, 1)
        for _ in range(len(BOUNDS))
    ]

    print(
        "  Parameters:",
        [round(x, 3) for x in position]
    )

    fitness = run_fitness(position)

    print(f"  Fitness: {round(fitness, 4)}")

    particles.append({
        "position":      position,
        "velocity":      velocity,
        "best_position": position.copy(),
        "best_fitness":  fitness,
    })


# ============================================================
# INITIAL GLOBAL BEST
# ============================================================

best_particle = min(particles, key=lambda p: p["best_fitness"])

global_best_position = best_particle["best_position"].copy()
global_best_fitness  = best_particle["best_fitness"]

print()
print(
    "Initial global best fitness:",
    round(global_best_fitness, 4)
)


# ============================================================
# CONVERGENCE FILE
# ============================================================

os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)

with open(RESULTS_FILE, "w") as f:
    f.write("iteration,best_fitness,inertia_weight\n")
    f.write(f"0,{global_best_fitness},{W_MAX}\n")


# ============================================================
# STEP 4 — PSO MAIN LOOP  (with adaptive inertia)
# ============================================================

for iteration in range(ITERATIONS):

    W = adaptive_inertia(iteration)

    print()
    print("=" * 45)
    print(f"PSO ITERATION {iteration + 1}/{ITERATIONS}  |  W = {round(W, 3)}")
    print("=" * 45)

    for particle_number, particle in enumerate(particles, start=1):

        print()
        print(f"  Particle {particle_number}/{PARTICLES}")

        # ------------------------------------------------
        # Update velocity and position
        # ------------------------------------------------

        for d in range(len(BOUNDS)):

            r1 = random.random()
            r2 = random.random()

            cognitive = (
                C1 * r1
                * (particle["best_position"][d] - particle["position"][d])
            )

            social = (
                C2 * r2
                * (global_best_position[d] - particle["position"][d])
            )

            particle["velocity"][d] = (
                W * particle["velocity"][d]
                + cognitive
                + social
            )

            particle["position"][d] = (
                particle["position"][d]
                + particle["velocity"][d]
            )

            # Clamp to bounds
            particle["position"][d] = max(
                BOUNDS[d][0],
                min(BOUNDS[d][1], particle["position"][d])
            )

        particle["position"] = enforce_parameter_order(
            particle["position"]
        )

        print(
            "  Parameters:",
            [round(x, 3) for x in particle["position"]]
        )

        # ------------------------------------------------
        # Evaluate
        # ------------------------------------------------

        current_fitness = run_fitness(particle["position"])

        print(f"  Fitness: {round(current_fitness, 4)}")

        # ------------------------------------------------
        # Update personal best
        # ------------------------------------------------

        if current_fitness < particle["best_fitness"]:
            particle["best_fitness"]  = current_fitness
            particle["best_position"] = particle["position"].copy()
            print("  >> New personal best!")

        # ------------------------------------------------
        # Update global best
        # ------------------------------------------------

        if current_fitness < global_best_fitness:
            global_best_fitness  = current_fitness
            global_best_position = particle["position"].copy()
            print("  >>> NEW GLOBAL BEST <<<")

    # --------------------------------------------------------
    # Log iteration result
    # --------------------------------------------------------

    print()
    print(
        f"Best fitness after iteration {iteration + 1}: "
        f"{round(global_best_fitness, 4)}"
    )

    with open(RESULTS_FILE, "a") as f:
        f.write(f"{iteration + 1},{global_best_fitness},{round(W, 4)}\n")


# ============================================================
# FINAL RESULT
# ============================================================

print()
print("=" * 45)
print("FUZZY-PSO OPTIMISATION COMPLETE")
print("=" * 45)
print(f"Best fitness   : {round(global_best_fitness, 4)}")
print("Best parameters:")
print([round(x, 4) for x in global_best_position])
print()
print("Convergence log saved to:")
print(RESULTS_FILE)