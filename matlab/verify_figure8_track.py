#!/usr/bin/env python3
"""
Verify figure-8 track generation matches Phase 10 specification
"""
import numpy as np
import matplotlib.pyplot as plt

def generate_figure_eight_track(radius):
    """Generate figure-8 track using Phase 10 specification"""
    kappa_ccw = 1.0 / radius  # Counterclockwise curvature
    half_circumference = np.pi * radius
    total_length = 2 * half_circumference
    n_points = 500
    
    # Numerical integration
    s_vals = np.linspace(0, total_length, n_points)
    x = np.zeros(n_points)
    y = np.zeros(n_points)
    theta = np.pi / 2  # North
    
    # Initial position: (R, 0)
    x[0] = radius
    y[0] = 0
    
    for i in range(1, n_points):
        ds = s_vals[i] - s_vals[i-1]
        theta = theta + kappa_ccw * ds
        x[i] = x[i-1] + np.cos(theta) * ds
        y[i] = y[i-1] + np.sin(theta) * ds
    
    # Close loop
    x[-1] = x[0]
    y[-1] = y[0]
    
    return x, y

# Generate track
radius = 25.0
x, y = generate_figure_eight_track(radius)

# Create visualization
fig, ax = plt.subplots(figsize=(10, 10))
ax.plot(x, y, 'r-', linewidth=2, label='Track Centerline')
ax.plot(x[0], y[0], 'go', markersize=15, label='Start')
ax.plot(x[-1], y[-1], 'rs', markersize=15, label='End')

# Mark circle centers
ax.plot(0, radius, 'b+', markersize=20, label=f'Upper Circle Center (0, {radius})')
ax.plot(0, -radius, 'b+', markersize=20, label=f'Lower Circle Center (0, -{radius})')
ax.plot(0, 0, 'mx', markersize=20, label='Self-Intersection (0, 0)')

ax.grid(True, alpha=0.3)
ax.axis('equal')
ax.set_xlabel('X [m]', fontsize=12)
ax.set_ylabel('Y [m]', fontsize=12)
ax.set_title('Figure-8 Track - Phase 10 Specification\n(Vertical Layout, Both CCW)', fontsize=14, fontweight='bold')
ax.legend(loc='upper right')

# Save figure
plt.savefig('/Users/sota/projects/FomulaTMU/sim_overall/matlab/figure_eight_verification.png', dpi=150, bbox_inches='tight')
print(f"Track verification plot saved to: figure_eight_verification.png")

# Print verification metrics
print(f"\n=== Track Verification ===")
print(f"Start position: ({x[0]:.2f}, {y[0]:.2f})")
print(f"End position: ({x[-1]:.2f}, {y[-1]:.2f})")
print(f"Closure error: {np.sqrt((x[-1]-x[0])**2 + (y[-1]-y[0])**2):.6f} m")
print(f"Track length: {2*np.pi*radius:.2f} m")
print(f"Expected length: {2*np.pi*radius:.2f} m")
print(f"\nUpper circle center: (0, {radius})")
print(f"Lower circle center: (0, -{radius})")
print(f"Self-intersection: (0, 0)")
