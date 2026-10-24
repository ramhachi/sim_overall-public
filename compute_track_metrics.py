#!/usr/bin/env python3
import numpy as np
import pandas as pd
import json

# Read CSV
df = pd.read_csv('data/ASE_autocross_sim.csv')

# Extract x, y coordinates
x = df['x'].values
y = df['y'].values

point_count = len(x)

# Compute segment lengths
segments = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
polyline_length = float(np.sum(segments))

# Closure distance
closure_distance = float(np.sqrt((x[-1] - x[0])**2 + (y[-1] - y[0])**2))

# First 20 segment lengths
first20_segment_lengths = [float(s) for s in segments[:20]]

# Compute curvature for interior points using Menger curvature
def menger_curvature(p1, p2, p3):
    """Compute curvature using three points (Menger curvature)"""
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    # Area of triangle using cross product
    area = 0.5 * abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))
    
    # Side lengths
    a = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    b = np.sqrt((x3 - x2)**2 + (y3 - y2)**2)
    c = np.sqrt((x3 - x1)**2 + (y3 - y1)**2)
    
    # Avoid division by zero
    if a * b * c < 1e-10:
        return 0.0
    
    # Menger curvature: kappa = 4*Area / (a*b*c)
    kappa = 4.0 * area / (a * b * c)
    return kappa

# First 12 interior curvature (points 1 to 12, using neighbors)
first12_interior_curvature = []
for i in range(1, 13):
    p1 = (x[i-1], y[i-1])
    p2 = (x[i], y[i])
    p3 = (x[i+1], y[i+1])
    kappa = menger_curvature(p1, p2, p3)
    first12_interior_curvature.append(float(kappa))

# Build cumulative arclength array
cumulative_s = np.zeros(point_count)
for i in range(1, point_count):
    cumulative_s[i] = cumulative_s[i-1] + segments[i-1]

# Interpolation function to get (x, y) at arclength s
def interpolate_at_s(s_target):
    """Interpolate position at arclength s_target"""
    if s_target <= 0:
        return x[0], y[0]
    if s_target >= cumulative_s[-1]:
        return x[-1], y[-1]
    
    # Find segment containing s_target
    idx = np.searchsorted(cumulative_s, s_target)
    if idx == 0:
        return x[0], y[0]
    
    # Linear interpolation within segment
    s0 = cumulative_s[idx-1]
    s1 = cumulative_s[idx]
    t = (s_target - s0) / (s1 - s0) if s1 > s0 else 0.0
    
    xi = x[idx-1] + t * (x[idx] - x[idx-1])
    yi = y[idx-1] + t * (y[idx] - y[idx-1])
    return xi, yi

# Lookahead samples
lookahead_distances = [5, 10, 15, 20, 30, 40, 60, 80]
mu = 1.8
g = 9.81
safety = 0.85

lookahead_samples = []

for d in lookahead_distances:
    # Compute curvature at lookahead point using triplet at s-1m, s, s+1m
    s_center = d
    s_before = s_center - 1.0
    s_after = s_center + 1.0
    
    # Get points
    p_before = interpolate_at_s(s_before)
    p_center = interpolate_at_s(s_center)
    p_after = interpolate_at_s(s_after)
    
    # Compute curvature
    kappa = menger_curvature(p_before, p_center, p_after)
    
    # v_max = sqrt(mu * g / kappa) if kappa > 0
    if kappa > 1e-10:
        v_max = np.sqrt(mu * g / kappa)
    else:
        v_max = 999.0  # Very large for straight sections
    
    # v_at_entry with safety factor
    v_at_entry = v_max * safety
    
    lookahead_samples.append({
        "d": d,
        "kappa": float(kappa),
        "v_max": float(v_max),
        "v_at_entry": float(v_at_entry)
    })

# Minimum safe entry speed
min_safe_entry_speed = float(min(sample["v_at_entry"] for sample in lookahead_samples))

# Trigger logic from rl_control_safety.hpp line 92: current_speed > min_safe_speed * 1.1
current_speed = 6.0
trigger_threshold = min_safe_entry_speed * 1.1
trigger_if_current_speed_is_6 = current_speed > trigger_threshold

# Build result
result = {
    "point_count": int(point_count),
    "polyline_length": float(polyline_length),
    "closure_distance": float(closure_distance),
    "first20_segment_lengths": first20_segment_lengths,
    "first12_interior_curvature": first12_interior_curvature,
    "lookahead_samples": lookahead_samples,
    "min_safe_entry_speed": float(min_safe_entry_speed),
    "trigger_if_current_speed_is_6": bool(trigger_if_current_speed_is_6)
}

print(json.dumps(result, indent=2))
