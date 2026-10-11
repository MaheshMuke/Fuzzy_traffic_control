import os

PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
TRAIN_DIR = os.path.join(PROJECT_ROOT, "sumo", "training_scenarios")
TEST_DIR  = os.path.join(PROJECT_ROOT, "sumo", "test_scenarios")
os.makedirs(TRAIN_DIR, exist_ok=True)
os.makedirs(TEST_DIR, exist_ok=True)

COMMON_VTYPES = """    <vType id="car" accel="2.6" decel="4.5" length="5" maxSpeed="13.9"/>
    <vType id="emergency" accel="3.0" decel="5.0" length="5" maxSpeed="15.0" color="1,0,0"/>

    <route id="leftRight" edges="left0A0 A0A1 A1B1 B1right1"/>
    <route id="rightLeft" edges="right1B1 B1A1 A1A0 A0left0"/>
    <route id="bottomTop" edges="bottom0A0 A0B0 B0B1 B1top1"/>
    <route id="topBottom" edges="top1B1 B1B0 B0A0 A0bottom0"/>
"""

# ============================================================
# TRAINING SCENARIO 1: Balanced Multi-Phase Dynamic
# ============================================================
train_balanced_xml = f"""<routes>
{COMMON_VTYPES}
    <!-- 0 - 100s: Light (14 EW, 12 NS) -->
    <flow id="b_0" type="car" route="leftRight" begin="0" end="100" number="14"/>
    <flow id="b_1" type="car" route="rightLeft" begin="0" end="100" number="14"/>
    <flow id="b_2" type="car" route="bottomTop" begin="0" end="100" number="12"/>
    <flow id="b_3" type="car" route="topBottom" begin="0" end="100" number="12"/>

    <!-- 100 - 250s: Moderate (16 EW, 14 NS) -->
    <flow id="b_4" type="car" route="leftRight" begin="100" end="250" number="16"/>
    <flow id="b_5" type="car" route="rightLeft" begin="100" end="250" number="16"/>
    <flow id="b_6" type="car" route="bottomTop" begin="100" end="250" number="14"/>
    <flow id="b_7" type="car" route="topBottom" begin="100" end="250" number="14"/>

    <!-- 250 - 400s: Heavy Peak (8 EW, 7 NS + Emergency EW) -->
    <flow id="b_8" type="car" route="leftRight" begin="250" end="400" number="8"/>
    <flow id="b_9" type="car" route="rightLeft" begin="250" end="400" number="8"/>
    <flow id="b_10" type="car" route="bottomTop" begin="250" end="400" number="7"/>
    <flow id="b_11" type="car" route="topBottom" begin="250" end="400" number="7"/>
    <vehicle id="emergency1" type="emergency" route="leftRight" depart="325"/>

    <!-- 400 - 520s: Moderate (13 EW, 11 NS) -->
    <flow id="b_12" type="car" route="leftRight" begin="400" end="520" number="13"/>
    <flow id="b_13" type="car" route="rightLeft" begin="400" end="520" number="13"/>
    <flow id="b_14" type="car" route="bottomTop" begin="400" end="520" number="11"/>
    <flow id="b_15" type="car" route="topBottom" begin="400" end="520" number="11"/>

    <!-- 520 - 600s: Light (11 EW, 9 NS) -->
    <flow id="b_16" type="car" route="leftRight" begin="520" end="600" number="11"/>
    <flow id="b_17" type="car" route="rightLeft" begin="520" end="600" number="11"/>
    <flow id="b_18" type="car" route="bottomTop" begin="520" end="600" number="9"/>
    <flow id="b_19" type="car" route="topBottom" begin="520" end="600" number="9"/>
</routes>
"""

# ============================================================
# TRAINING SCENARIO 2: North-South Heavy Tidal Flow (NS:EW = 2.5:1)
# ============================================================
train_ns_heavy_xml = f"""<routes>
{COMMON_VTYPES}
    <!-- 0 - 150s: Warmup (NS heavier) -->
    <flow id="ns_0" type="car" route="leftRight" begin="0" end="150" number="8"/>
    <flow id="ns_1" type="car" route="rightLeft" begin="0" end="150" number="8"/>
    <flow id="ns_2" type="car" route="bottomTop" begin="0" end="150" number="22"/>
    <flow id="ns_3" type="car" route="topBottom" begin="0" end="150" number="22"/>

    <!-- 150 - 380s: Peak Tidal Surge on NS Corridor -->
    <flow id="ns_4" type="car" route="leftRight" begin="150" end="380" number="12"/>
    <flow id="ns_5" type="car" route="rightLeft" begin="150" end="380" number="12"/>
    <flow id="ns_6" type="car" route="bottomTop" begin="150" end="380" number="36"/>
    <flow id="ns_7" type="car" route="topBottom" begin="150" end="380" number="34"/>
    <vehicle id="emergency1" type="emergency" route="bottomTop" depart="280"/>

    <!-- 380 - 600s: Recovery & Moderate NS -->
    <flow id="ns_8" type="car" route="leftRight" begin="380" end="600" number="14"/>
    <flow id="ns_9" type="car" route="rightLeft" begin="380" end="600" number="14"/>
    <flow id="ns_10" type="car" route="bottomTop" begin="380" end="600" number="26"/>
    <flow id="ns_11" type="car" route="topBottom" begin="380" end="600" number="26"/>
</routes>
"""

# ============================================================
# TRAINING SCENARIO 3: East-West Heavy Arterial Flow (EW:NS = 2.5:1)
# ============================================================
train_ew_heavy_xml = f"""<routes>
{COMMON_VTYPES}
    <!-- 0 - 150s: Warmup (EW heavier) -->
    <flow id="ew_0" type="car" route="leftRight" begin="0" end="150" number="22"/>
    <flow id="ew_1" type="car" route="rightLeft" begin="0" end="150" number="22"/>
    <flow id="ew_2" type="car" route="bottomTop" begin="0" end="150" number="8"/>
    <flow id="ew_3" type="car" route="topBottom" begin="0" end="150" number="8"/>

    <!-- 150 - 380s: Peak Arterial Surge on EW Corridor -->
    <flow id="ew_4" type="car" route="leftRight" begin="150" end="380" number="36"/>
    <flow id="ew_5" type="car" route="rightLeft" begin="150" end="380" number="34"/>
    <flow id="ew_6" type="car" route="bottomTop" begin="150" end="380" number="12"/>
    <flow id="ew_7" type="car" route="topBottom" begin="150" end="380" number="12"/>
    <vehicle id="emergency1" type="emergency" route="leftRight" depart="290"/>

    <!-- 380 - 600s: Moderate EW -->
    <flow id="ew_8" type="car" route="leftRight" begin="380" end="600" number="26"/>
    <flow id="ew_9" type="car" route="rightLeft" begin="380" end="600" number="26"/>
    <flow id="ew_10" type="car" route="bottomTop" begin="380" end="600" number="14"/>
    <flow id="ew_11" type="car" route="topBottom" begin="380" end="600" number="14"/>
</routes>
"""

# ============================================================
# UNSEEN TEST SCENARIO 1: Extended Asymmetric Demand (301 vehicles, 700s)
# ============================================================
test_asymmetric_xml = f"""<routes>
{COMMON_VTYPES}
    <flow id="test_flow_0" type="car" route="leftRight" begin="0" end="150" number="18"/>
    <flow id="test_flow_1" type="car" route="rightLeft" begin="0" end="150" number="18"/>
    <flow id="test_flow_2" type="car" route="bottomTop" begin="0" end="150" number="22"/>
    <flow id="test_flow_3" type="car" route="topBottom" begin="0" end="150" number="20"/>

    <flow id="test_flow_4" type="car" route="leftRight" begin="150" end="380" number="25"/>
    <flow id="test_flow_5" type="car" route="rightLeft" begin="150" end="380" number="25"/>
    <flow id="test_flow_6" type="car" route="bottomTop" begin="150" end="380" number="32"/>
    <flow id="test_flow_7" type="car" route="topBottom" begin="150" end="380" number="28"/>
    <vehicle id="emergency1" type="emergency" route="leftRight" depart="260"/>

    <flow id="test_flow_8" type="car" route="leftRight" begin="380" end="550" number="16"/>
    <flow id="test_flow_9" type="car" route="rightLeft" begin="380" end="550" number="16"/>
    <flow id="test_flow_10" type="car" route="bottomTop" begin="380" end="550" number="18"/>
    <flow id="test_flow_11" type="car" route="topBottom" begin="380" end="550" number="18"/>

    <flow id="test_flow_12" type="car" route="leftRight" begin="550" end="700" number="10"/>
    <flow id="test_flow_13" type="car" route="rightLeft" begin="550" end="700" number="10"/>
    <flow id="test_flow_14" type="car" route="bottomTop" begin="550" end="700" number="12"/>
    <flow id="test_flow_15" type="car" route="topBottom" begin="550" end="700" number="12"/>
</routes>
"""

# ============================================================
# UNSEEN TEST SCENARIO 2: Hard Dynamic Stress Test (332 vehicles, 750s)
# Tidal direction flips mid-simulation!
# ============================================================
test_stress_xml = f"""<routes>
{COMMON_VTYPES}
    <!-- Window 1: 0 - 200s (Heavy East-West rush) -->
    <flow id="st_0" type="car" route="leftRight" begin="0" end="200" number="30"/>
    <flow id="st_1" type="car" route="rightLeft" begin="0" end="200" number="30"/>
    <flow id="st_2" type="car" route="bottomTop" begin="0" end="200" number="12"/>
    <flow id="st_3" type="car" route="topBottom" begin="0" end="200" number="12"/>

    <!-- Window 2: 200 - 450s (Sudden Flip: Heavy North-South rush + cross Emergency) -->
    <flow id="st_4" type="car" route="leftRight" begin="200" end="450" number="15"/>
    <flow id="st_5" type="car" route="rightLeft" begin="200" end="450" number="15"/>
    <flow id="st_6" type="car" route="bottomTop" begin="200" end="450" number="42"/>
    <flow id="st_7" type="car" route="topBottom" begin="200" end="450" number="40"/>
    <vehicle id="emergency1" type="emergency" route="bottomTop" depart="340"/>

    <!-- Window 3: 450 - 620s (Gridlock release: High balanced cross traffic) -->
    <flow id="st_8" type="car" route="leftRight" begin="450" end="620" number="24"/>
    <flow id="st_9" type="car" route="rightLeft" begin="450" end="620" number="24"/>
    <flow id="st_10" type="car" route="bottomTop" begin="450" end="620" number="24"/>
    <flow id="st_11" type="car" route="topBottom" begin="450" end="620" number="24"/>

    <!-- Window 4: 620 - 750s (Tail free-flow) -->
    <flow id="st_12" type="car" route="leftRight" begin="620" end="750" number="10"/>
    <flow id="st_13" type="car" route="rightLeft" begin="620" end="750" number="10"/>
    <flow id="st_14" type="car" route="bottomTop" begin="620" end="750" number="10"/>
    <flow id="st_15" type="car" route="topBottom" begin="620" end="750" number="10"/>
</routes>
"""

with open(os.path.join(TRAIN_DIR, "train_balanced.rou.xml"), "w") as f:
    f.write(train_balanced_xml)
with open(os.path.join(TRAIN_DIR, "train_ns_heavy.rou.xml"), "w") as f:
    f.write(train_ns_heavy_xml)
with open(os.path.join(TRAIN_DIR, "train_ew_heavy.rou.xml"), "w") as f:
    f.write(train_ew_heavy_xml)

with open(os.path.join(TEST_DIR, "test_unseen_asymmetric.rou.xml"), "w") as f:
    f.write(test_asymmetric_xml)
with open(os.path.join(TEST_DIR, "test_unseen_stress.rou.xml"), "w") as f:
    f.write(test_stress_xml)

print("Generated 3 Training Scenarios in sumo/training_scenarios/")
print("Generated 2 Unseen Test Scenarios in sumo/test_scenarios/")
