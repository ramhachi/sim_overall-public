import matplotlib.pyplot as plt
import csv
import sys
import math

def plot_velocity_profile(csv_file, output_file):
    data = {'time': [], 's': [], 'speed': [], 'target_speed': [], 'kappa': []}
    
    try:
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    data['time'].append(float(row['time']))
                    data['s'].append(float(row['s']))
                    data['speed'].append(float(row['speed']))
                    data['target_speed'].append(float(row['target_speed']))
                    data['kappa'].append(float(row['kappa']))
                except ValueError:
                    continue
    except FileNotFoundError:
        print(f"Error: File {csv_file} not found.")
        return

    if not data['time']:
        print("Error: No data found in CSV.")
        return

    # Create subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 1. Speed Profile
    ax = axes[0]
    ax.plot(data['s'], data['speed'], label='Actual Speed', color='blue', linewidth=1.5)
    ax.plot(data['s'], data['target_speed'], label='Target Speed', color='red', linestyle='--', linewidth=1.5)
    ax.set_ylabel('Speed [m/s]')
    ax.set_title('Velocity Profile Analysis')
    ax.legend()
    ax.grid(True)
    
    # 2. Curvature
    ax = axes[1]
    ax.plot(data['s'], data['kappa'], label='Curvature (kappa)', color='green', linewidth=1.0)
    ax.set_ylabel('Curvature [1/m]')
    ax.legend()
    ax.grid(True)
    
    # 3. Acceleration
    # Calculate acceleration from speed derivative
    lon_accel = []
    lat_accel = []
    s_points = []
    
    for i in range(1, len(data['time'])):
        dt = data['time'][i] - data['time'][i-1]
        if dt > 0:
            dv = data['speed'][i] - data['speed'][i-1]
            lon_accel.append(dv / dt)
            
            # Lat accel = v^2 * kappa
            v = data['speed'][i]
            k = data['kappa'][i]
            lat_accel.append(v*v * k)
            s_points.append(data['s'][i])
            
    ax = axes[2]
    ax.plot(s_points, lon_accel, label='Lon Accel (derived)', color='orange', alpha=0.7)
    ax.plot(s_points, lat_accel, label='Lat Accel (v^2*k)', color='purple', alpha=0.7)
    ax.set_ylabel('Acceleration [m/s^2]')
    ax.set_xlabel('Distance [m]')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    plt.savefig(output_file)
    print(f"Saved plot to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 plot_velocity_profile.py <csv_file> <output_png>")
        sys.exit(1)
    
    plot_velocity_profile(sys.argv[1], sys.argv[2])
