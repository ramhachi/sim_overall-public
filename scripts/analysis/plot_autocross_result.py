
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

def plot_results(csv_file, output_prefix):
    try:
        df = pd.read_csv(csv_file)
        
        # 1. Trajectory colored by Speed
        plt.figure(figsize=(12, 10))
        sc = plt.scatter(df['x'], df['y'], c=df['speed'], cmap='viridis', s=5, label='Trajectory')
        plt.colorbar(sc, label='Speed [m/s]')
        plt.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', label='Start', markersize=10)
        plt.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'rx', label='End', markersize=10)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title(f'Trajectory colored by Speed: {csv_file}')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.savefig(f"{output_prefix}_trajectory_speed.png")
        print(f"Saved {output_prefix}_trajectory_speed.png")
        plt.close()

        # 2. Trajectory colored by Lateral Error (Absolute)
        plt.figure(figsize=(12, 10))
        # Use abs(n) for error magnitude
        lat_error = df['n'].abs()
        sc = plt.scatter(df['x'], df['y'], c=lat_error, cmap='jet', s=5, label='Trajectory')
        plt.colorbar(sc, label='Lateral Error [m]')
        plt.axis('equal')
        plt.grid(True)
        plt.title(f'Trajectory colored by Lateral Error: {csv_file}')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.savefig(f"{output_prefix}_trajectory_error.png")
        print(f"Saved {output_prefix}_trajectory_error.png")
        plt.close()

        # 3. Controls and Errors over Time
        fig, axes = plt.subplots(3, 1, figsize=(12, 15), sharex=True)
        
        # Speed
        axes[0].plot(df['time'], df['speed'], label='Speed')
        axes[0].plot(df['time'], df['target_speed'], '--', label='Target Speed')
        axes[0].set_ylabel('Speed [m/s]')
        axes[0].legend()
        axes[0].grid(True)
        axes[0].set_title('Vehicle Speed')

        # Steering & Throttle/Brake
        axes[1].plot(df['time'], df['steering'], label='Steering [rad]', color='blue')
        axes[1].set_ylabel('Steering [rad]', color='blue')
        ax1_twin = axes[1].twinx()
        ax1_twin.plot(df['time'], df['throttle'], label='Throttle', color='green', alpha=0.6)
        ax1_twin.plot(df['time'], df['brake'], label='Brake', color='red', alpha=0.6)
        ax1_twin.set_ylabel('Input [-]', color='black')
        axes[1].grid(True)
        # Combine legends
        lines, labels = axes[1].get_legend_handles_labels()
        lines2, labels2 = ax1_twin.get_legend_handles_labels()
        axes[1].legend(lines + lines2, labels + labels2, loc='upper left')
        axes[1].set_title('Driver Inputs')

        # Lateral Error
        axes[2].plot(df['time'], df['n'], label='Lateral Error (n)')
        axes[2].axhline(0, color='black', linestyle='--', linewidth=0.5)
        axes[2].set_ylabel('Lateral Error [m]')
        axes[2].set_xlabel('Time [s]')
        axes[2].legend()
        axes[2].grid(True)
        axes[2].set_title('Lateral Error')

        plt.tight_layout()
        plt.savefig(f"{output_prefix}_telemetry.png")
        print(f"Saved {output_prefix}_telemetry.png")
        plt.close()

    except Exception as e:
        print(f"Error plotting: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_autocross_result.py <csv_file> <output_prefix>")
        sys.exit(1)
    plot_results(sys.argv[1], sys.argv[2])
