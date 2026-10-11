import os
import traci
import numpy as np
import pandas as pd
from fuzzy_parameterized import create_fuzzy_controller

NETWORK_FILE = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sumo", "intersection.net.xml"))
TRAIN_ROUTE  = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sumo", "traffic.rou.xml"))

CONTROLLERS = {
    "Baseline_Fuzzy": [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2],
    "Old_PSO_Synthetic": [9.8354, 2.9543, 17.78, 8, 22.3155, 7.6659, 1.6186, 0.5, 4.2632, 0.5, 7.165, 4],
    "Single_Scenario_PSO": [10.0, 4.8843, 22.0119, 4.6249, 22.4976, 1.0, 1.4879, 4.0, 6.7085, 3.2259, 7.3769, 0.5283],
    "Robust_Multi_PSO": [2.0796, 2.9734, 15.5670, 5.8443, 26.4280, 3.6692, 0.7954, 2.4311, 5.9661, 3.1725, 9.0442, 3.3297]
}

def eval_ctrl(p, seed):
    ctrl = create_fuzzy_controller(p)
    traci.start(['sumo', '-n', NETWORK_FILE, '-r', TRAIN_ROUTE, '--no-step-log', 'true', '--seed', str(seed)])
    prev_phase = -1
    em_seen, em_start, em_arr = False, None, None
    tot_delay, tot_stops, completed, tot_q, q_meas, max_q, teleports = 0.0, 0, 0, 0, 0, 0, 0
    prev_tl, was_mov = {}, {}
    seen_arr, arr_cnt, arr_rate, last_arr = set(), 0, 0, 0
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()
            teleports += traci.simulation.getStartingTeleportNumber()
            if 'emergency1' in traci.vehicle.getIDList():
                if not em_seen: em_seen, em_start = True, t
            else:
                if em_seen and em_arr is None: em_arr = t
            completed += traci.simulation.getArrivedNumber()
            for vid in traci.vehicle.getIDList():
                tl = traci.vehicle.getTimeLoss(vid)
                tot_delay += max(0.0, tl - prev_tl.get(vid, 0.0))
                prev_tl[vid] = tl
                spd = traci.vehicle.getSpeed(vid)
                mov = spd > 0.1
                if vid not in was_mov: was_mov[vid] = mov
                else:
                    if was_mov[vid] and not mov: tot_stops += 1
                    was_mov[vid] = mov
            curr_in = set()
            for e in ['A1A0', 'B0A0', 'bottom0A0', 'left0A0']:
                curr_in.update(traci.edge.getLastStepVehicleIDs(e))
            new_in = curr_in - seen_arr
            arr_cnt += len(new_in)
            seen_arr.update(new_in)
            if t - last_arr >= 10:
                arr_rate, arr_cnt, last_arr = arr_cnt, 0, t
            q_len = 0
            for e in ['A1A0', 'B0A0', 'bottom0A0', 'left0A0']:
                for l in range(traci.edge.getLaneNumber(e)):
                    for vid in traci.lane.getLastStepVehicleIDs(f'{e}_{l}'):
                        if traci.vehicle.getSpeed(vid) < 0.1: q_len += 1
            tot_q += q_len
            q_meas += 1
            max_q = max(max_q, q_len)
            ph = traci.trafficlight.getPhase('A0')
            if ph != prev_phase:
                prev_phase = ph
                if 'emergency1' in traci.vehicle.getIDList() and traci.vehicle.getRoadID('emergency1') in ['A1A0', 'B0A0', 'bottom0A0', 'left0A0']:
                    traci.trafficlight.setPhase('A0', 0)
                    traci.trafficlight.setPhaseDuration('A0', 30)
                    continue
                if ph in [0, 2]:
                    ctrl.input['queue'] = q_len
                    ctrl.input['arrival'] = arr_rate
                    ctrl.compute()
                    gc = ctrl.output['green_change']
                    traci.trafficlight.setPhaseDuration('A0', max(20, min(60, 42 + gc)))
        d = tot_delay / completed if completed > 0 else float('inf')
        s = tot_stops / completed if completed > 0 else 0.0
        mq = tot_q / q_meas if q_meas > 0 else 0.0
        em_t = (em_arr - em_start) if (em_start and em_arr) else -1
        em_pen = max(0.0, em_t - 90.0) / 90.0 if em_t > 0 else 0.0
        fit = 0.50 * d + 0.15 * s + 0.20 * mq + 0.15 * em_pen * d
        return {'delay': d, 'stops': s, 'mean_q': mq, 'max_q': max_q, 'em_t': em_t, 'teleports': teleports, 'fitness': fit}
    finally:
        traci.close()

if __name__ == "__main__":
    seeds = [42, 101, 202, 303, 404]
    rows = []
    for cname, p in CONTROLLERS.items():
        for s in seeds:
            res = eval_ctrl(p, s)
            rows.append({'controller': cname, 'seed': s, **res})

    df = pd.DataFrame(rows)
    out_dir = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results"))
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.normpath(os.path.join(out_dir, "training_scenario_benchmark.csv"))
    df.to_csv(out_csv, index=False)
    print("\nTRAINING SCENARIO BENCHMARK RESULTS ACROSS 5 SEEDS:")
    print("=" * 115)
    print(f"{'Controller':<24} | {'Delay (s)':<16} | {'Stops':<14} | {'Mean Queue':<14} | {'Max Q':<8} | {'EmTime (s)':<12} | {'Teleports'}")
    print("=" * 115)
    for cname, grp in df.groupby('controller'):
        d_m, d_s = grp['delay'].mean(), grp['delay'].std()
        s_m, s_s = grp['stops'].mean(), grp['stops'].std()
        q_m, q_s = grp['mean_q'].mean(), grp['mean_q'].std()
        mq_m     = grp['max_q'].mean()
        em_m, em_s = grp['em_t'].mean(), grp['em_t'].std()
        tp_m     = grp['teleports'].mean()
        print(f"{cname:<24} | {d_m:>6.2f} +/- {d_s:<6.2f} | {s_m:>4.2f} +/- {s_s:<4.2f} | {q_m:>5.2f} +/- {q_s:<5.2f} | {mq_m:>4.1f}   | {em_m:>5.1f} +/- {em_s:<4.1f} | {tp_m:>4.1f}")
    print("=" * 115)
