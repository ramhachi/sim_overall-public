
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

def compare_tracks(raw_csv, sim_csv, output_file):
    try:
        # Load Raw CSV
        df_raw = pd.read_csv(raw_csv)
        
        # Load Sim CSV
        df_sim = pd.read_csv(sim_csv)
        
        plt.figure(figsize=(10, 10))
        
        # Plot Raw
        plt.plot(df_raw['x'], df_raw['y'], 'k--', label='Raw CSV (Ground Truth)', linewidth=2, alpha=0.7)
        plt.scatter(df_raw['x'].iloc[0], df_raw['y'].iloc[0], c='green', s=100, label='Raw Start', marker='o')
        
        # Plot Sim
        plt.plot(df_sim['x'], df_sim['y'], 'r-', label='Simulation (Reconstructed)', linewidth=1.5, alpha=0.8)
        plt.scatter(df_sim['x'].iloc[0], df_sim['y'].iloc[0], c='blue', s=80, label='Sim Start', marker='x')
        
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Track Shape Comparison: Raw CSV vs Simulation')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        
        plt.savefig(output_file)
        print(f"Saved comparison plot to {output_file}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python compare_track_shape.py <raw_csv> <sim_csv> <output_png>")
        # Hardcode defaults for quick test if args missing
        raw = "ASE_autocross_sim.csv"
        sim = "lat_error_test_drift_fix.csv"
        out = "track_comparison.png"
        compare_tracks(raw, sim, out)
    else:
        compare_tracks(sys.argv[1], sys.argv[2], sys.argv[3])
