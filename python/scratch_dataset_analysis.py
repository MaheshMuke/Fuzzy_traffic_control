import csv
from collections import defaultdict
import statistics

path = '../data/vanet_traffic_data.csv'
data = defaultdict(list)

with open(path, newline='', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for r in reader:
        lbl = r['label'].strip()
        data[lbl].append({
            'flow': float(r['flow_veh_per_hr']),
            'speed': float(r['avg_speed_kmph']),
            'wait': float(r['avg_wait_time_s']),
            'density': float(r['density_veh_per_km']),
            'queue': float(r['queue_length_veh'])
        })

total = sum(len(v) for v in data.values())
print(f"Total records in dataset: {total}")
header = f"{'Label':<12} | {'Count':<7} | {'Share (%)':<9} | {'Mean Flow':<10} | {'Mean Speed':<10} | {'Mean Density':<12} | {'Mean Queue':<10} | {'Mean Wait':<10}"
print("-" * len(header))
print(header)
print("-" * len(header))

for lbl, rows in sorted(data.items()):
    cnt = len(rows)
    pct = cnt / total * 100
    m_flow = statistics.mean(r['flow'] for r in rows)
    m_spd = statistics.mean(r['speed'] for r in rows)
    m_dens = statistics.mean(r['density'] for r in rows)
    m_q = statistics.mean(r['queue'] for r in rows)
    m_wt = statistics.mean(r['wait'] for r in rows)
    print(f"{lbl:<12} | {cnt:<7} | {pct:>8.2f}% | {m_flow:>10.1f} | {m_spd:>10.1f} | {m_dens:>12.1f} | {m_q:>10.1f} | {m_wt:>10.1f}")
print("-" * len(header))
