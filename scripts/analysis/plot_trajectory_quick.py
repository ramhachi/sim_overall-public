import pandas as pd
import matplotlib.pyplot as plt
import sys

def plot_trajectory(csv_file, output_file):
    try:
        df = pd.read_csv(csv_file)
        plt.figure(figsize=(10, 10))
        plt.plot(df['x'], df['y'], label='Trajectory')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title(f'Trajectory: {csv_file}')
        plt.savefig(output_file)
        print(f"Plot saved to {output_file}")
        
        # Print start and end stats
        print(f"Start: ({df['x'].iloc[0]:.2f}, {df['y'].iloc[0]:.2f}) psi={df['psi'].iloc[0]:.2f}")
        print(f"End:   ({df['x'].iloc[-1]:.2f}, {df['y'].iloc[-1]:.2f})")
        
    except Exception as e:
        print(f"Error plotting: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python plot_trajectory.py <csv_file> <output_png>")
        sys.exit(1)
    plot_trajectory(sys.argv[1], sys.argv[2])
