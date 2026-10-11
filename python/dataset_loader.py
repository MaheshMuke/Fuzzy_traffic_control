"""
dataset_loader.py
-----------------
Loads and processes the REAL VANET Traffic Congestion Dataset
(vanet_traffic_data.csv) from Kaggle.

Dataset columns used
---------------------
  queue_length_veh    -> fuzzy input: queue length  (0-30 vehicles)
  flow_veh_per_hr     -> fuzzy input: arrival rate  (converted to veh/10s)
  avg_speed_kmph      -> supplementary (speed check)
  avg_wait_time_s     -> used in optimal green time estimate
  density_veh_per_km  -> used in scenario classification
  label               -> Free-flow / Moderate / Heavy / Gridlock

Dataset has 195,714 rows covering all 4 traffic states.

Public API
----------
  load_dataset()              -> list of cleaned row dicts
  get_scenario_flows(ds, sc)  -> avg flow metrics for a scenario
  get_composite_flow_plan(ds) -> 5-window plan for SUMO 600s simulation
  get_fuzzy_input_bounds(ds)  -> data-driven percentile centres for PSO
  print_summary(ds)           -> quick console report
"""

import csv
import os
import statistics
import math

# ============================================================
# PATH
# ============================================================

DATASET_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..",
    "data",
    "vanet_traffic_data.csv"
)

# ============================================================
# CONSTANTS  (derived from dataset exploration)
# ============================================================

# queue_length_veh range in dataset: -1.27 .. 72.99
# We clamp negatives to 0 and cap at 30 (fuzzy universe max)
QUEUE_MAX = 30.0

# flow_veh_per_hr range: 41 .. 2153
# Convert to veh per 10-second interval:  flow/hr ÷ 360
# SUMO arrival_rate universe: 0-10 veh/10s  → cap at 10
FLOW_TO_10S = 1.0 / 360.0
ARRIVAL_MAX = 10.0

# Label → scenario mapping
LABEL_TO_SCENARIO = {
    "Free-flow": "light",
    "Moderate":  "moderate",
    "Heavy":     "heavy",
    "Gridlock":  "heavy",    # treat gridlock as heavy traffic
}

# ============================================================
# LOAD
# ============================================================

def load_dataset(path=DATASET_PATH, max_rows=None):
    """
    Load the VANET CSV and return a list of cleaned row dicts.

    Parameters
    ----------
    path     : str   path to vanet_traffic_data.csv
    max_rows : int   optional row limit (useful for quick tests)

    Returns
    -------
    list of dicts with keys:
        queue_length, arrival_rate, avg_speed_kmph,
        avg_wait_time_s, density_veh_per_km,
        scenario, label
    """
    rows = []

    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for i, raw in enumerate(reader):

            if max_rows and i >= max_rows:
                break

            # --------------------------------------------------
            # Queue length  (clamp negatives, cap at universe max)
            # --------------------------------------------------
            queue = float(raw["queue_length_veh"])
            queue = max(0.0, min(QUEUE_MAX, queue))

            # --------------------------------------------------
            # Arrival rate  (flow/hr -> veh per 10-second window)
            # --------------------------------------------------
            flow_hr = float(raw["flow_veh_per_hr"])
            arrival = min(ARRIVAL_MAX, flow_hr * FLOW_TO_10S)

            # --------------------------------------------------
            # Other columns
            # --------------------------------------------------
            label    = raw["label"].strip()
            scenario = LABEL_TO_SCENARIO.get(label, "moderate")

            rows.append({
                "queue_length":       round(queue,   2),
                "arrival_rate":       round(arrival, 3),
                "avg_speed_kmph":     float(raw["avg_speed_kmph"]),
                "avg_wait_time_s":    max(0.0, float(raw["avg_wait_time_s"])),
                "density_veh_per_km": float(raw["density_veh_per_km"]),
                "flow_veh_per_hr":    flow_hr,
                "scenario":           scenario,
                "label":              label,
            })

    return rows


# ============================================================
# SCENARIO FLOWS
# ============================================================

def get_scenario_flows(dataset, scenario="moderate"):
    """
    Return average traffic metrics for a named scenario,
    computed from the real VANET dataset rows.

    Scenarios
    ---------
    light    : Free-flow rows
    moderate : Moderate rows
    heavy    : Heavy + Gridlock rows
    """
    if scenario not in ("light", "moderate", "heavy"):
        raise ValueError(
            f"Unknown scenario '{scenario}'. "
            "Choose from: light, moderate, heavy"
        )

    subset = [r for r in dataset if r["scenario"] == scenario]

    if not subset:
        raise ValueError(f"No rows found for scenario '{scenario}'")

    def mean(key):
        return statistics.mean(r[key] for r in subset)

    avg_flow_hr  = mean("flow_veh_per_hr")
    avg_queue    = mean("queue_length")
    avg_speed    = mean("avg_speed_kmph")
    avg_wait     = mean("avg_wait_time_s")

    # Convert hourly flow to per-minute for SUMO <flow number=>
    # Split equally across 4 directions
    flow_per_min_per_dir = avg_flow_hr / 60.0 / 4.0

    return {
        "scenario":              scenario,
        "avg_queue":             round(avg_queue, 2),
        "avg_arrival_rate":      round(mean("arrival_rate"), 3),
        "avg_speed_kmph":        round(avg_speed, 2),
        "avg_wait_time_s":       round(avg_wait, 2),
        "flow_per_min_per_dir":  round(flow_per_min_per_dir, 2),
        "row_count":             len(subset),
    }


# ============================================================
# COMPOSITE FLOW PLAN  (for a single 600-second SUMO run)
# ============================================================

def get_composite_flow_plan(dataset):
    """
    Build a time-varying traffic schedule derived from the real dataset
    that covers all three traffic intensities within one 600-second run.

    Windows
    -------
    0 – 100 s  : light    (Free-flow rows from VANET dataset)
    100 – 250 s: moderate (Moderate rows)
    250 – 400 s: heavy    (Heavy + Gridlock rows)
    400 – 520 s: moderate (tapering back)
    520 – 600 s: light    (back to free-flow)

    Returns list of dicts:
        begin, end, vehicles_LR, vehicles_RL,
        vehicles_BT, vehicles_TB, include_emergency
    """
    windows = [
        ("light",    0,   100),
        ("moderate", 100, 250),
        ("heavy",    250, 400),
        ("moderate", 400, 520),
        ("light",    520, 600),
    ]

    plan = []

    for scenario, begin, end in windows:

        flows = get_scenario_flows(dataset, scenario)
        duration_min = (end - begin) / 60.0

        # Vehicles per direction in this window
        n = max(1, round(flows["flow_per_min_per_dir"] * duration_min))

        plan.append({
            "begin":             begin,
            "end":               end,
            "scenario":          scenario,
            "vehicles_LR":       n,
            "vehicles_RL":       n,
            "vehicles_BT":       max(1, round(n * 0.85)),  # cross-traffic slightly less
            "vehicles_TB":       max(1, round(n * 0.85)),
            "avg_queue":         flows["avg_queue"],
            "avg_wait_time_s":   flows["avg_wait_time_s"],
            "include_emergency": (scenario == "heavy"),
        })

    return plan


# ============================================================
# FUZZY INPUT BOUNDS  (data-driven percentiles → PSO seeds)
# ============================================================

def get_fuzzy_input_bounds(dataset):
    """
    Compute percentile-based membership-function centres from
    the real VANET dataset.

    Returns dict with centres for queue (Low/Med/High) and
    arrival (Low/Med/High) derived from the 10th, 50th, and
    90th percentiles of the real data.
    """
    queues   = sorted(r["queue_length"]  for r in dataset)
    arrivals = sorted(r["arrival_rate"]  for r in dataset)

    def pct(data, p):
        idx = (p / 100.0) * (len(data) - 1)
        lo  = int(idx)
        hi  = min(lo + 1, len(data) - 1)
        return data[lo] + (idx - lo) * (data[hi] - data[lo])

    return {
        "queue_low_center":    round(min(QUEUE_MAX,    max(0, pct(queues,   10))), 2),
        "queue_med_center":    round(min(QUEUE_MAX,    max(0, pct(queues,   50))), 2),
        "queue_high_center":   round(min(QUEUE_MAX,    max(0, pct(queues,   90))), 2),
        "arrival_low_center":  round(min(ARRIVAL_MAX,  max(0, pct(arrivals, 10))), 3),
        "arrival_med_center":  round(min(ARRIVAL_MAX,  max(0, pct(arrivals, 50))), 3),
        "arrival_high_center": round(min(ARRIVAL_MAX,  max(0, pct(arrivals, 90))), 3),
    }


# ============================================================
# QUICK SUMMARY
# ============================================================

def print_summary(dataset):
    """Print a concise summary of the loaded VANET dataset."""
    print()
    print("=" * 50)
    print("VANET TRAFFIC DATASET SUMMARY")
    print("=" * 50)
    print(f"Total rows loaded : {len(dataset):,}")

    # Label distribution
    labels = {}
    for r in dataset:
        labels[r["label"]] = labels.get(r["label"], 0) + 1

    print("\nTraffic state distribution:")
    for label, count in sorted(labels.items()):
        pct = 100.0 * count / len(dataset)
        print(f"  {label:<12}: {count:>7,} rows  ({pct:.1f}%)")

    # Per-scenario stats
    print("\nPer-scenario averages (from real data):")
    for sc in ("light", "moderate", "heavy"):
        f = get_scenario_flows(dataset, sc)
        print(
            f"  {sc:<10}: "
            f"queue={f['avg_queue']:>5.1f} veh  "
            f"arrival={f['avg_arrival_rate']:>5.3f} veh/10s  "
            f"speed={f['avg_speed_kmph']:>5.1f} km/h  "
            f"wait={f['avg_wait_time_s']:>5.1f}s"
        )

    # Data-driven fuzzy bounds
    bounds = get_fuzzy_input_bounds(dataset)
    print("\nData-driven fuzzy membership centres (-> used to seed PSO):")
    print(
        f"  Queue   low/med/high : "
        f"{bounds['queue_low_center']} / "
        f"{bounds['queue_med_center']} / "
        f"{bounds['queue_high_center']}"
    )
    print(
        f"  Arrival low/med/high : "
        f"{bounds['arrival_low_center']} / "
        f"{bounds['arrival_med_center']} / "
        f"{bounds['arrival_high_center']}"
    )
    print("=" * 50)


# ============================================================
# TEST
# ============================================================

if __name__ == "__main__":

    print("Loading VANET dataset (this may take a few seconds)...")
    ds = load_dataset()
    print_summary(ds)

    print("\nComposite flow plan for SUMO (600-second simulation):")
    plan = get_composite_flow_plan(ds)
    for w in plan:
        total = (w["vehicles_LR"] + w["vehicles_RL"] +
                 w["vehicles_BT"] + w["vehicles_TB"])
        print(
            f"  {w['begin']:>4}-{w['end']:<4}s  "
            f"scenario={w['scenario']:<10}  "
            f"total={total:>4} vehicles  "
            f"avg_queue={w['avg_queue']:>4.1f}  "
            f"emergency={w['include_emergency']}"
        )
