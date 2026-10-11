"""
generate_routes.py
------------------
Generates a SUMO-compatible traffic route file (traffic.rou.xml)
from the real-world dataset's composite flow plan.

The 600-second simulation is divided into 5 time windows that
cover light, moderate, and heavy traffic periods derived from
the dataset — making the SUMO simulation realistic instead of
using a flat uniform flow.

Usage
-----
    python generate_routes.py            # writes sumo/traffic.rou.xml
    python generate_routes.py --preview  # prints XML to screen only
"""

import os
import sys

from dataset_loader import load_dataset, get_composite_flow_plan


# ============================================================
# PATHS
# ============================================================

PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)

DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "sumo", "traffic.rou.xml")


# ============================================================
# XML BUILDER
# ============================================================

def build_route_xml(flow_plan):
    """
    Build the full XML string for a SUMO .rou.xml file from
    the composite flow plan returned by dataset_loader.

    Parameters
    ----------
    flow_plan : list of dicts
        Each dict has: begin, end, vehicles_LR, vehicles_RL,
        vehicles_BT, vehicles_TB, include_emergency

    Returns
    -------
    str : complete XML content
    """
    lines = []
    lines.append('<routes>')
    lines.append('')

    # ----------------------------------------------------------
    # Vehicle types
    # ----------------------------------------------------------
    lines.append('    <!-- Standard car -->')
    lines.append('    <vType id="car"')
    lines.append('           accel="2.6"')
    lines.append('           decel="4.5"')
    lines.append('           length="5"')
    lines.append('           maxSpeed="13.9"/>')
    lines.append('')
    lines.append('    <!-- Emergency vehicle (ambulance / fire / police) -->')
    lines.append('    <vType id="emergency"')
    lines.append('           accel="3.0"')
    lines.append('           decel="5.0"')
    lines.append('           length="5"')
    lines.append('           maxSpeed="15.0"')
    lines.append('           color="1,0,0"/>')
    lines.append('')

    # ----------------------------------------------------------
    # Routes (unchanged from original network)
    # ----------------------------------------------------------
    lines.append('    <!-- Routes -->')
    lines.append('    <route id="leftRight"')
    lines.append('           edges="left0A0 A0A1 A1B1 B1right1"/>')
    lines.append('')
    lines.append('    <route id="rightLeft"')
    lines.append('           edges="right1B1 B1A1 A1A0 A0left0"/>')
    lines.append('')
    lines.append('    <route id="bottomTop"')
    lines.append('           edges="bottom0A0 A0B0 B0B1 B1top1"/>')
    lines.append('')
    lines.append('    <route id="topBottom"')
    lines.append('           edges="top1B1 B1B0 B0A0 A0bottom0"/>')
    lines.append('')

    # ----------------------------------------------------------
    # Time-varying flows from dataset
    # ----------------------------------------------------------
    lines.append('    <!-- Time-varying flows derived from real traffic dataset -->')
    lines.append('    <!-- Each window corresponds to a real traffic period:    -->')
    lines.append('    <!--   light = night / off-peak                           -->')
    lines.append('    <!--   moderate = regular daytime                         -->')
    lines.append('    <!--   heavy = morning / evening rush hour                -->')
    lines.append('')

    flow_id = 0

    for window in flow_plan:
        begin = window["begin"]
        end   = window["end"]

        lines.append(f'    <!-- Window {begin}s - {end}s -->')

        for direction, route in [
            ("LR", "leftRight"),
            ("RL", "rightLeft"),
            ("BT", "bottomTop"),
            ("TB", "topBottom"),
        ]:
            n = window[f"vehicles_{direction}"]
            lines.append(
                f'    <flow id="flow_{flow_id}" type="car" route="{route}"'
                f' begin="{begin}" end="{end}" number="{n}"/>'
            )
            flow_id += 1

        # Emergency vehicle in the rush-hour window
        if window["include_emergency"]:
            em_depart = begin + (end - begin) // 2
            lines.append(
                f'    <vehicle id="emergency1" type="emergency"'
                f' route="leftRight" depart="{em_depart}"/>'
            )

        lines.append('')

    lines.append('</routes>')

    return "\n".join(lines) + "\n"


# ============================================================
# WRITE FILE
# ============================================================

def generate_route_file(output_path=DEFAULT_OUTPUT, preview=False):
    """
    Load dataset, build composite flow plan, write route file.

    Parameters
    ----------
    output_path : str
        Where to write the .rou.xml file.
    preview : bool
        If True, print XML to stdout instead of writing.
    """
    print("Loading dataset...", flush=True)
    dataset   = load_dataset()
    flow_plan = get_composite_flow_plan(dataset)

    print("Building route XML from composite flow plan:", flush=True)
    for w in flow_plan:
        total = (w["vehicles_LR"] + w["vehicles_RL"] +
                 w["vehicles_BT"] + w["vehicles_TB"])
        print(
            f"  {w['begin']:>4}-{w['end']:<4}s  "
            f"total={total:>4} vehicles  "
            f"emergency={w['include_emergency']}",
            flush=True
        )

    xml = build_route_xml(flow_plan)

    if preview:
        print()
        print(xml)
        return

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w") as f:
        f.write(xml)

    print()
    print(f"Route file written: {output_path}")

    total_vehicles = sum(
        w["vehicles_LR"] + w["vehicles_RL"] +
        w["vehicles_BT"] + w["vehicles_TB"]
        for w in flow_plan
    )
    print(f"Total vehicles across all windows: {total_vehicles}")


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    preview = "--preview" in sys.argv
    generate_route_file(preview=preview)
