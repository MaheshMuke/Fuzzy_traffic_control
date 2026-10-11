from sumo_fitness import evaluate_parameters

# ==========================================
# 1. BASELINE PARAMETERS
# ==========================================
baseline_parameters = [
    0, 5,       # Queue: low (centre, width)
    15, 5,      # Queue: medium
    30, 5,      # Queue: high
    0, 2,       # Arrival: low
    5, 2,       # Arrival: medium
    10, 2       # Arrival: high
]

# ==========================================
# 2. OLD PSO PARAMETERS (Synthetic uniform traffic)
# ==========================================
old_pso_parameters = [
    9.8354, 2.9543,
    17.7800, 8.0000,
    22.3155, 7.6659,
    1.6186, 0.5000,
    4.2632, 0.5000,
    7.1650, 4.0000
]

# ==========================================
# 3. NEW PSO PARAMETERS (Real Kaggle VANET dataset)
# ==========================================
new_pso_parameters = [
    10.0000, 4.8843,
    22.0119, 4.6249,
    22.4976, 1.0000,
    1.4879, 4.0000,
    6.7085, 3.2259,
    7.3769, 0.5283
]

def run_evaluation(name, params):
    print(f"\nEvaluating {name}...")
    fitness, delay, stops, throughput, mean_queue, max_queue, em_time = evaluate_parameters(params)
    return {
        "name": name,
        "fitness": fitness,
        "delay": delay,
        "stops": stops,
        "throughput": throughput,
        "mean_queue": mean_queue,
        "max_queue": max_queue,
        "emergency_time": em_time
    }

def print_summary_table(results):
    base = results[0]
    old = results[1]
    new = results[2]

    header = f"{'Metric':<25} | {'Baseline':<12} | {'Old PSO (Synth)':<16} | {'New PSO (VANET)':<16} | {'New vs Base (%)':<16} | {'New vs Old (%)':<16}"
    sep = "-" * len(header)
    print("\n" + sep)
    print("                 TRAFFIC SIGNAL CONTROLLER PERFORMANCE COMPARISON")
    print(sep)
    print(header)
    print(sep)

    metrics = [
        ("Fitness (Overall Cost)", "fitness", False),
        ("Mean Time Loss (s/veh)", "delay", False),
        ("Mean Stops (stops/veh)", "stops", False),
        ("Throughput (veh completed)", "throughput", True),
        ("Mean Queue (veh)", "mean_queue", False),
        ("Maximum Queue (veh)", "max_queue", False),
        ("Emergency Time (s)", "emergency_time", False),
    ]

    for label, key, higher_is_better in metrics:
        b_val = base[key]
        o_val = old[key]
        n_val = new[key]

        if higher_is_better:
            imp_base = ((n_val - b_val) / b_val * 100) if b_val != 0 else 0
            imp_old = ((n_val - o_val) / o_val * 100) if o_val != 0 else 0
        else:
            imp_base = ((b_val - n_val) / b_val * 100) if b_val != 0 else 0
            imp_old = ((o_val - n_val) / o_val * 100) if o_val != 0 else 0

        b_str = f"{b_val:.2f}" if isinstance(b_val, float) else f"{b_val}"
        o_str = f"{o_val:.2f}" if isinstance(o_val, float) else f"{o_val}"
        n_str = f"{n_val:.2f}" if isinstance(n_val, float) else f"{n_val}"
        
        sign_base = "+" if imp_base > 0 else ""
        sign_old = "+" if imp_old > 0 else ""
        
        print(f"{label:<25} | {b_str:<12} | {o_str:<16} | {n_str:<16} | {sign_base}{imp_base:.2f}%{' ':>9} | {sign_old}{imp_old:.2f}%{' ':>9}")

    print(sep)

if __name__ == "__main__":
    r_base = run_evaluation("Baseline Fuzzy", baseline_parameters)
    r_old = run_evaluation("Old PSO (Synthetic)", old_pso_parameters)
    r_new = run_evaluation("New PSO (Real VANET)", new_pso_parameters)
    print_summary_table([r_base, r_old, r_new])