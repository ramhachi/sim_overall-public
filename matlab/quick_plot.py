#!/usr/bin/env python3
"""Quick visualization of laptime simulation results"""
import csv
import matplotlib.pyplot as plt
import numpy as np

# Read CSV
data = {}
with open('laptime_sim_circle.csv', 'r') as f:
    reader = csv.DictReader(f)
    for key in reader.fieldnames:
        data[key] = []
    for row in reader:
        for key in row:
            data[key].append(float(row[key]))

# Create figure with subplots
fig, axes = plt.subplots(2, 3, figsize=(15, 10))

# 1. Trajectory (x-y plot)
ax = axes[0, 0]
scatter = ax.scatter(data['x'], data['y'], c=data['speed'], cmap='viridis', s=5)
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_title('Vehicle Trajectory (colored by speed)')
ax.axis('equal')
ax.grid(True)
plt.colorbar(scatter, ax=ax, label='Speed [m/s]')

# Draw circle track (radius 30m)
theta = np.linspace(0, 2*np.pi, 100)
track_x = 30 * np.cos(theta)
track_y = 30 * np.sin(theta)
ax.plot(track_x, track_y, 'r--', label='Track centerline', alpha=0.5)
ax.legend()

# 2. Speed profile
ax = axes[0, 1]
ax.plot(data['time'], data['speed'], label='Actual speed')
ax.plot(data['time'], data['target_speed'], label='Target speed', linestyle='--')
ax.set_xlabel('Time [s]')
ax.set_ylabel('Speed [m/s]')
ax.set_title('Speed Profile')
ax.grid(True)
ax.legend()

# 3. Lateral error
ax = axes[0, 2]
ax.plot(data['time'], data['lateral_error'])
ax.set_xlabel('Time [s]')
ax.set_ylabel('Lateral error [m]')
ax.set_title('Lateral Error (n)')
ax.grid(True)
ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

# 4. Heading error
ax = axes[1, 0]
ax.plot(data['time'], data['heading_error_deg'])
ax.set_xlabel('Time [s]')
ax.set_ylabel('Heading error [deg]')
ax.set_title('Heading Error')
ax.grid(True)
ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
ax.axhline(y=180, color='r', linestyle='--', linewidth=0.5, alpha=0.5)
ax.axhline(y=-180, color='r', linestyle='--', linewidth=0.5, alpha=0.5)

# 5. Steering angle
ax = axes[1, 1]
ax.plot(data['time'], [np.degrees(s) for s in data['steering']])
ax.set_xlabel('Time [s]')
ax.set_ylabel('Steering angle [deg]')
ax.set_title('Steering Angle')
ax.grid(True)

# 6. Arc length (s)
ax = axes[1, 2]
ax.plot(data['time'], data['s'])
ax.set_xlabel('Time [s]')
ax.set_ylabel('Arc length s [m]')
ax.set_title('Arc Length Along Track')
ax.grid(True)

plt.tight_layout()
plt.savefig('laptime_analysis.png', dpi=150)
print("Saved plot to: laptime_analysis.png")

# Print statistics
print("\n=== Analysis ===")
print(f"Duration: {data['time'][-1]:.2f} s")
print(f"Final s: {data['s'][-1]:.2f} m")
print(f"Final position: ({data['x'][-1]:.2f}, {data['y'][-1]:.2f}) m")
print(f"Final psi: {data['psi'][-1]:.2f} rad = {np.degrees(data['psi'][-1]):.1f} deg")
print(f"Final heading error: {data['heading_error_deg'][-1]:.1f} deg")
print(f"Max |heading error|: {max(abs(h) for h in data['heading_error_deg']):.1f} deg")
print(f"Mean speed: {np.mean(data['speed']):.2f} m/s")
print(f"Max lateral error: {max(abs(e) for e in data['lateral_error']):.2f} m")
rms_lat = np.sqrt(np.mean([e**2 for e in data['lateral_error']]))
print(f"RMS lateral error: {rms_lat:.2f} m")

# Check for lap completion
track_length = 2 * np.pi * 30  # 2πr for circle
print(f"\nTrack length: {track_length:.2f} m")
if data['s'][-1] > track_length * 0.9:
    print("✓ Lap nearly completed!")
else:
    print(f"✗ Only {data['s'][-1]/track_length*100:.1f}% of lap completed")
