#!/usr/bin/env python3
import numpy as np
import pandas as pd

# Read CSV
csv_path = "data/ASE_autocross_sim.csv"
df = pd.read_csv(csv_path)
points = df[['x', 'y']].values

n_points = len(points)
print("=== BASIC METRICS ===")
print(f"Total point count: {n_points}")

# Compute segment lengths
segments = np.diff(points, axis=0)
segment_lengths = np.linalg.norm(segments, axis=1)
total_length = np.sum(segment_lengths)
print(f"Total polyline length: {total_length:.6f} m")

# Closure distance
closure_dist = np.linalg.norm(points[-1] - points[0])
print(f"Closure distance (first to last): {closure_dist:.6f} m")

print(f"\n=== FIRST 20 SEGMENT LENGTHS ===")
for i, length in enumerate(segment_lengths[:20]):
    print(f"Segment {i:2d} (pt {i} to {i+1}): {length:.6f} m")

# Curvature using triplet method for first 12 interior points
print(f"\n=== CURVATURE FOR FIRST 12 INTERIOR POINTS (triplet method) ===")
def compute_curvature_triplet(p1, p2, p3):
    """Compute curvature using Menger curvature formula for three points"""
    a = np.linalg.norm(p2 - p1)
    b = np.linalg.norm(p3 - p2)
    c = np.linalg.norm(p3 - p1)
    
    # Area of triangle using Heron's formula
    s = (a + b + c) / 2.0
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0:
        return 0.0
    area = np.sqrt(area_sq)
    
    # Menger curvature: kappa = 4*Area / (a*b*c)
    if a * b * c < 1e-10:
        return 0.0
    kappa = 4.0 * area / (a * b * c)
    return kappa

for i in range(1, 13):  # Interior points 1 through 12
    p_prev = points[i-1]
    p_curr = points[i]
    p_next = points[i+1]
    kappa = compute_curvature_triplet(p_prev, p_curr, p_next)
    if kappa > 1e-6:
        print(f"Point {i:2d}: kappa = {kappa:.6f} (1/m), radius = {1.0/kappa:.3f} m")
    else:
        print(f"Point {i:2d}: kappa = {kappa:.6f} (1/m), radius = inf")

# Compute cumulative distances
cumulative_distances = np.zeros(n_points)
for i in range(1, n_points):
    cumulative_distances[i] = cumulative_distances[i-1] + segment_lengths[i-1]

print(f"\n=== CURVATURE AT SPECIFIC CUMULATIVE DISTANCES ===")
print(f"Using triplet method with nearest point and neighbors")

# Parameters
mu = 1.8
g = 9.81
safety = 0.85
target_distances = [5, 10, 15, 20, 30, 40, 60, 80]

min_safe_entry_speed = 1e9
results = []

for d_target in target_distances:
    # Find closest point to target cumulative distance
    idx = np.argmin(np.abs(cumulative_distances - d_target))
    actual_d = cumulative_distances[idx]
    
    # Use triplet method if we have neighbors
    if idx > 0 and idx < n_points - 1:
        kappa = compute_curvature_triplet(points[idx-1], points[idx], points[idx+1])
    elif idx == 0:
        kappa = compute_curvature_triplet(points[0], points[1], points[2])
    else:
        kappa = compute_curvature_triplet(points[idx-2], points[idx-1], points[idx])
    
    if kappa > 1e-4:
        radius = 1.0 / kappa
        v_max = np.sqrt(mu * g * radius * safety)
        braking_decel = 1.0 * g  # From C++ code
        v_at_entry = np.sqrt(v_max**2 + 2.0 * braking_decel * d_target)
        min_safe_entry_speed = min(min_safe_entry_speed, v_at_entry)
    else:
        radius = float('inf')
        v_max = float('inf')
        v_at_entry = float('inf')
    
    results.append({
        'd_target': d_target,
        'd_actual': actual_d,
        'idx': idx,
        'kappa': kappa,
        'radius': radius,
        'v_max': v_max,
        'v_at_entry': v_at_entry
    })
    
    print(f"\nLookahead d={d_target:2d} m (actual={actual_d:.3f} m, idx={idx}):")
    print(f"  kappa = {kappa:.6f} (1/m)")
    if np.isfinite(radius):
        print(f"  radius = {radius:.3f} m")
    else:
        print(f"  radius = inf")
    if np.isfinite(v_max):
        print(f"  v_max = {v_max:.3f} m/s")
    else:
        print(f"  v_max = inf")
    if np.isfinite(v_at_entry):
        print(f"  v_at_entry = {v_at_entry:.3f} m/s")
    else:
        print(f"  v_at_entry = inf")

print(f"\n=== SAFETY LIMITER ANALYSIS ===")
print(f"Minimum safe entry speed across all lookaheads: {min_safe_entry_speed:.3f} m/s")

# Check if 6 m/s would trigger limiter
current_speed = 6.0
threshold = min_safe_entry_speed * 1.1
print(f"\nCurrent speed: {current_speed:.3f} m/s")
print(f"Limiter threshold (min_safe_speed * 1.1): {threshold:.3f} m/s")
print(f"Would trigger limiter: {current_speed > threshold}")
print(f"  (Logic: current_speed > min_safe_speed * 1.1)")
print(f"  ({current_speed:.3f} > {threshold:.3f} = {current_speed > threshold})")

if current_speed > threshold:
    overspeed_ratio = (current_speed - min_safe_entry_speed) / min_safe_entry_speed
    brake_intensity = np.clip(overspeed_ratio * 1.5, 0.0, 1.0)
    print(f"\nLimiter action:")
    print(f"  Overspeed ratio: {overspeed_ratio:.3f}")
    print(f"  Brake intensity: {brake_intensity:.3f}")
    print(f"  Throttle: 0.0")

print(f"\n=== C++ CODE CONFIRMATION ===")
print("From cpp/include/fsae/driver/rl_control_safety.hpp:")
print("  mu = 1.8")
print("  g = 9.81")
print("  safety = 0.85")
print("  lookahead_distances[] = {5, 10, 15, 20, 30, 40, 60, 80}")
print("  v_max = sqrt(mu * g * radius * safety)")
print("  braking_decel = 1.0 * g")
print("  v_at_entry = sqrt(v_max * v_max + 2.0 * braking_decel * d)")
print("  Trigger condition: current_speed > min_safe_speed * 1.1")
