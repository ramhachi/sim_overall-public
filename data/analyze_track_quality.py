
import csv
import math

filename = '/Users/sota/projects/FomulaTMU/sim_overall/data/ASE_autocross_sim.csv'

def analyze_track():
    points = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        next(reader) # skip header
        for row in reader:
            try:
                points.append((float(row[0]), float(row[1])))
            except ValueError:
                continue

    print(f"Total points: {len(points)}")

    # Calculate distances and curvatures
    max_curvature = 0
    max_dist = 0
    min_dist = 1e9
    
    for i in range(1, len(points)-1):
        p0 = points[i-1]
        p1 = points[i]
        p2 = points[i+1]

        # Distances
        d1 = math.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2)
        d2 = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        
        max_dist = max(max_dist, d1)
        min_dist = min(min_dist, d1)

        # Curvature approximation (Menger curvature)
        # Area of triangle
        x1, y1 = p0
        x2, y2 = p1
        x3, y3 = p2
        area = 0.5 * abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        
        # Side lengths
        a = d1
        b = d2
        c = math.sqrt((p2[0]-p0[0])**2 + (p2[1]-p0[1])**2)
        
        if area > 1e-6:
            curvature = 4 * area / (a * b * c)
            if curvature > max_curvature:
                max_curvature = curvature
                # print(f"High curvature at index {i}: {curvature:.4f} (r={1/curvature:.2f}m)")

    print(f"Point spacing: Min={min_dist:.4f}m, Max={max_dist:.4f}m")
    print(f"Max curvature: {max_curvature:.4f} (Min radius: {1/max_curvature if max_curvature > 0 else 'inf'}m)")

if __name__ == "__main__":
    analyze_track()
