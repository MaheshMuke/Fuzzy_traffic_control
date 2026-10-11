import os
import pandas as pd
import matplotlib.pyplot as plt

PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
)
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")

def plot_benchmark_comparison():
    benchmark_file = os.path.join(RESULTS_DIR, "vanet_benchmark_results.csv")
    if not os.path.exists(benchmark_file):
        print(f"File not found: {benchmark_file}")
        return

    df = pd.read_csv(benchmark_file)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    fig.suptitle("Adaptive Traffic Signal Control Performance Comparison\n(VANET Dynamic Traffic Scenario)", fontsize=14, fontweight="bold")

    controllers = df["Controller"]
    colors = ["#e74c3c", "#f39c12", "#2ecc71"]

    # 1. Mean Time Loss
    axes[0, 0].bar(controllers, df["Mean_Time_Loss_s"], color=colors, edgecolor="black", alpha=0.85)
    axes[0, 0].set_ylabel("Seconds / Vehicle")
    axes[0, 0].set_title("Mean Time Loss (Delay)")
    axes[0, 0].grid(axis="y", linestyle="--", alpha=0.7)
    for i, v in enumerate(df["Mean_Time_Loss_s"]):
        axes[0, 0].text(i, v + 4, f"{v:.1f}s", ha="center", fontweight="bold")

    # 2. Mean Queue Length
    axes[0, 1].bar(controllers, df["Mean_Queue_veh"], color=colors, edgecolor="black", alpha=0.85)
    axes[0, 1].set_ylabel("Vehicles")
    axes[0, 1].set_title("Mean Queue Length")
    axes[0, 1].grid(axis="y", linestyle="--", alpha=0.7)
    for i, v in enumerate(df["Mean_Queue_veh"]):
        axes[0, 1].text(i, v + 0.6, f"{v:.1f} veh", ha="center", fontweight="bold")

    # 3. Maximum Queue Length
    axes[1, 0].bar(controllers, df["Max_Queue_veh"], color=colors, edgecolor="black", alpha=0.85)
    axes[1, 0].set_ylabel("Vehicles")
    axes[1, 0].set_title("Maximum Queue Length (Gridlock Indicator)")
    axes[1, 0].grid(axis="y", linestyle="--", alpha=0.7)
    for i, v in enumerate(df["Max_Queue_veh"]):
        axes[1, 0].text(i, v + 1, f"{v} veh", ha="center", fontweight="bold")

    # 4. Emergency Response Time
    axes[1, 1].bar(controllers, df["Emergency_Time_s"], color=colors, edgecolor="black", alpha=0.85)
    axes[1, 1].set_ylabel("Seconds")
    axes[1, 1].set_title("Emergency Vehicle Clearance Time")
    axes[1, 1].grid(axis="y", linestyle="--", alpha=0.7)
    for i, v in enumerate(df["Emergency_Time_s"]):
        axes[1, 1].text(i, v + 15, f"{v:.0f}s", ha="center", fontweight="bold")

    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "vanet_performance_comparison.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_pso_convergence():
    conv_file = os.path.join(RESULTS_DIR, "pso_convergence.csv")
    if not os.path.exists(conv_file):
        print(f"File not found: {conv_file}")
        return

    df = pd.read_csv(conv_file)

    fig, ax1 = plt.subplots(figsize=(9, 5))

    color = "#2980b9"
    ax1.set_xlabel("PSO Iteration", fontweight="bold")
    ax1.set_ylabel("Best Swarm Fitness (Cost)", color=color, fontweight="bold")
    ax1.plot(df["iteration"], df["best_fitness"], marker="o", color=color, linewidth=2, label="Fitness Cost")
    ax1.tick_params(axis="y", labelcolor=color)
    ax1.grid(True, linestyle="--", alpha=0.6)

    if "inertia_weight" in df.columns:
        ax2 = ax1.twinx()
        color_w = "#e67e22"
        ax2.set_ylabel("Inertia Weight (W)", color=color_w, fontweight="bold")
        ax2.plot(df["iteration"], df["inertia_weight"], color=color_w, linestyle=":", linewidth=2, label="Inertia (W)")
        ax2.tick_params(axis="y", labelcolor=color_w)

    plt.title("PSO Convergence & Adaptive Inertia Decay\n(VANET-Based Objective Function)", fontsize=13, fontweight="bold")
    plt.tight_layout()
    out_path = os.path.join(RESULTS_DIR, "pso_convergence.png")
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

if __name__ == "__main__":
    plot_benchmark_comparison()
    plot_pso_convergence()