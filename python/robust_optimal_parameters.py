# ============================================================
# ROBUST MULTI-SCENARIO FUZZY-PSO OPTIMAL PARAMETERS
# ============================================================
# Trained across 3 diverse multi-directional traffic regimes:
# 1. Balanced multi-phase dynamic
# 2. North-South heavy tidal corridor
# 3. East-West heavy arterial corridor
# With worst-case regularization: J_robust = Mean(J) + 0.5 * Max(J)
# ============================================================

ROBUST_PSO_PARAMETERS = [
    2.0796, 2.9734,   # Queue Low  (center, spread)
    15.5670, 5.8443,  # Queue Med  (center, spread)
    26.4280, 3.6692,  # Queue High (center, spread)
    0.7954, 2.4311,   # Arrival Low  (center, spread)
    5.9661, 3.1725,   # Arrival Med  (center, spread)
    9.0442, 3.3297    # Arrival High (center, spread)
]

ROBUST_FITNESS = 761.6949
