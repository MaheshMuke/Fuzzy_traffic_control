import numpy as np

bounds = [
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

names = [
    "Queue Low Center", "Queue Low Spread",
    "Queue Med Center", "Queue Med Spread",
    "Queue High Center", "Queue High Spread",
    "Arrival Low Center", "Arrival Low Spread",
    "Arrival Med Center", "Arrival Med Spread",
    "Arrival High Center", "Arrival High Spread"
]

single_pso = [10.0, 4.8843, 22.0119, 4.6249, 22.4976, 1.0, 1.4879, 4.0, 6.7085, 3.2259, 7.3769, 0.5283]
baseline   = [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2]

print("PARAMETER BOUNDARY AUDIT:")
print(f"{'Parameter':<22} | {'Baseline':<8} | {'Single PSO':<10} | {'Lower':<6} | {'Upper':<6} | {'Boundary Status'}")
print("-" * 75)
for n, b_val, s_val, (lo, hi) in zip(names, baseline, single_pso, bounds):
    at_bound = "PINNED AT UPPER BOUND!" if abs(s_val - hi) < 1e-3 else ("PINNED AT LOWER BOUND!" if abs(s_val - lo) < 1e-3 else "Internal")
    print(f"{n:<22} | {b_val:<8.1f} | {s_val:<10.4f} | {lo:<6.1f} | {hi:<6.1f} | {at_bound}")
