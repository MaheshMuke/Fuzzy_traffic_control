import os
import numpy as np
import matplotlib.pyplot as plt
import skfuzzy as fuzz
from fuzzy_parameterized import create_fuzzy_controller

_PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
RESULTS_DIR = os.path.join(_PROJECT_ROOT, "results")
os.makedirs(RESULTS_DIR, exist_ok=True)

# 3 KEY PARAMETER SETS
BASELINE_PARAMS   = [0, 5, 15, 5, 30, 5, 0, 2, 5, 2, 10, 2]
SINGLE_PSO_PARAMS = [10.0, 4.8843, 22.0119, 4.6249, 22.4976, 1.0, 1.4879, 4.0, 6.7085, 3.2259, 7.3769, 0.5283]
ROBUST_PSO_PARAMS = [2.0796, 2.9734, 15.5670, 5.8443, 26.4280, 3.6692, 0.7954, 2.4311, 5.9661, 3.1725, 9.0442, 3.3297]

def plot_membership_functions():
    q_x = np.linspace(0, 30, 500)
    a_x = np.linspace(0, 10, 500)

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle("Comparison of Fuzzy Membership Functions Across Controllers\n(Revealing Parameter Boundary Saturation & Semantic Collapse)", fontsize=13, fontweight="bold")

    configs = [
        ("Baseline (Hand-Tuned Uniform)", BASELINE_PARAMS, axes[0]),
        ("Single-Scenario PSO (Overfitted - Notice Collapse at Q=22)", SINGLE_PSO_PARAMS, axes[1]),
        ("Robust Multi-Scenario PSO (Semantically Regularized)", ROBUST_PSO_PARAMS, axes[2])
    ]

    colors = ["#2980b9", "#27ae60", "#c0392b"]

    for name, p, (ax_q, ax_a) in configs:
        # Queue MFs
        ax_q.set_title(f"{name} - Queue Length [0-30 veh]", fontsize=10, fontweight="bold")
        q_l = fuzz.gaussmf(q_x, p[0], p[1])
        q_m = fuzz.gaussmf(q_x, p[2], p[3])
        q_h = fuzz.gaussmf(q_x, p[4], p[5])
        ax_q.plot(q_x, q_l, label=f"Low (c={p[0]:.1f}, σ={p[1]:.1f})", color=colors[0], lw=2)
        ax_q.plot(q_x, q_m, label=f"Med (c={p[2]:.1f}, σ={p[3]:.1f})", color=colors[1], lw=2)
        ax_q.plot(q_x, q_h, label=f"High (c={p[4]:.1f}, σ={p[5]:.1f})", color=colors[2], lw=2)
        ax_q.set_ylabel("Degree μ")
        ax_q.grid(True, linestyle="--", alpha=0.5)
        ax_q.legend(fontsize=8, loc="upper right")
        ax_q.set_ylim(-0.05, 1.05)

        # Arrival MFs
        ax_a.set_title(f"{name} - Arrival Rate [0-10 veh/10s]", fontsize=10, fontweight="bold")
        a_l = fuzz.gaussmf(a_x, p[6], p[7])
        a_m = fuzz.gaussmf(a_x, p[8], p[9])
        a_h = fuzz.gaussmf(a_x, p[10], p[11])
        ax_a.plot(a_x, a_l, label=f"Low (c={p[6]:.1f}, σ={p[7]:.1f})", color=colors[0], lw=2)
        ax_a.plot(a_x, a_m, label=f"Med (c={p[8]:.1f}, σ={p[9]:.1f})", color=colors[1], lw=2)
        ax_a.plot(a_x, a_h, label=f"High (c={p[10]:.1f}, σ={p[11]:.1f})", color=colors[2], lw=2)
        ax_a.set_ylabel("Degree μ")
        ax_a.grid(True, linestyle="--", alpha=0.5)
        ax_a.legend(fontsize=8, loc="upper right")
        ax_a.set_ylim(-0.05, 1.05)

    axes[2, 0].set_xlabel("Queue Length (vehicles)", fontweight="bold")
    axes[2, 1].set_xlabel("Arrival Rate (vehicles / 10s)", fontweight="bold")

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "membership_functions_comparison.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_response_surfaces():
    q_vals = np.linspace(0, 30, 25)
    a_vals = np.linspace(0, 10, 25)
    Q, A = np.meshgrid(q_vals, a_vals)

    ctrl_base   = create_fuzzy_controller(BASELINE_PARAMS)
    ctrl_single = create_fuzzy_controller(SINGLE_PSO_PARAMS)
    ctrl_robust = create_fuzzy_controller(ROBUST_PSO_PARAMS)

    Z_base   = np.zeros_like(Q)
    Z_single = np.zeros_like(Q)
    Z_robust = np.zeros_like(Q)

    for i in range(Q.shape[0]):
        for j in range(Q.shape[1]):
            # Baseline
            ctrl_base.input["queue"] = Q[i, j]
            ctrl_base.input["arrival"] = A[i, j]
            ctrl_base.compute()
            Z_base[i, j] = ctrl_base.output["green_change"]

            # Single PSO
            ctrl_single.input["queue"] = Q[i, j]
            ctrl_single.input["arrival"] = A[i, j]
            ctrl_single.compute()
            Z_single[i, j] = ctrl_single.output["green_change"]

            # Robust PSO
            ctrl_robust.input["queue"] = Q[i, j]
            ctrl_robust.input["arrival"] = A[i, j]
            ctrl_robust.compute()
            Z_robust[i, j] = ctrl_robust.output["green_change"]

    fig = plt.figure(figsize=(16, 5))
    fig.suptitle("Fuzzy Control Response Surfaces: Green Time Adjustment ΔG(t) [seconds]", fontsize=13, fontweight="bold")

    surf_configs = [
        ("Baseline (Smooth Linear Gradient)", Z_base, 1),
        ("Single-Scenario PSO (Pathological Cliff Edge)", Z_single, 2),
        ("Robust Multi-Scenario PSO (Smooth Regularized Adaptivity)", Z_robust, 3)
    ]

    for title, Z, idx in surf_configs:
        ax = fig.add_subplot(1, 3, idx, projection="3d")
        surf = ax.plot_surface(Q, A, Z, cmap="coolwarm", edgecolor="none", alpha=0.85)
        ax.set_title(title, fontsize=10, fontweight="bold")
        ax.set_xlabel("Queue (veh)")
        ax.set_ylabel("Arrival (veh/10s)")
        ax.set_zlabel("ΔG (s)")
        ax.set_zlim(-10, 10)
        fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "fuzzy_response_surfaces.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

if __name__ == "__main__":
    plot_membership_functions()
    plot_response_surfaces()
