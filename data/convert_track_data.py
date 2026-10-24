
import csv
import sys
import os

input_file = '/Users/sota/projects/FomulaTMU/sim_overall/data/ASE_autocross.csv'
output_file = '/Users/sota/projects/FomulaTMU/sim_overall/data/ASE_autocross_sim.csv'

def smooth_points(points, window_size=5):
    if len(points) < window_size:
        return points
    
    smoothed = []
    half_window = window_size // 2
    n = len(points)
    
    # Simple moving average (handling loop closer would be better, but simple clamp for now)
    # Actually, let's treat it as a closed loop for smoothing if user says it's a circuit
    # Assuming ASE Autocross is a loop (usually is).
    
    for i in range(n):
        sum_x = 0
        sum_y = 0
        count = 0
        for j in range(-half_window, half_window + 1):
            idx = (i + j) % n # Cyclic for closed loop
            sum_x += points[idx][0]
            sum_y += points[idx][1]
            count += 1
        smoothed.append((sum_x / count, sum_y / count))
        
    return smoothed

def convert_track():
    if not os.path.exists(input_file):
        print(f"Error: Input file not found: {input_file}")
        return

    print(f"Reading {input_file}...")
    
    data = []
    header = []
    try:
        with open(input_file, 'r') as f:
            reader = csv.reader(f)
            try:
                header = next(reader)
            except StopIteration:
                print("Error: Empty file")
                return
                
            # Find indices
            try:
                # Strip whitespace just in case
                header = [h.strip() for h in header]
                idx_x = header.index('x')
                idx_z = header.index('z') # Use z as y
            except ValueError:
                 print(f"Error: Input CSV must contain 'x' and 'z' columns. Found: {header}")
                 return

            for row in reader:
                if not row: continue
                try:
                    x = float(row[idx_x])
                    y = float(row[idx_z]) # z -> y
                    data.append((x, y))
                except (IndexError, ValueError):
                    continue
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    print(f"Loaded {len(data)} points. Applying smoothing (window=5)...")
    smoothed_data = smooth_points(data, window_size=5)

    # Write
    try:
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'width'])
            for pt in smoothed_data:
                 writer.writerow([pt[0], pt[1], 4.5])
        print(f"Success: Converted and smoothed {len(smoothed_data)} points to {output_file}")
    except Exception as e:
        print(f"Error writing file: {e}")

if __name__ == "__main__":
    convert_track()
