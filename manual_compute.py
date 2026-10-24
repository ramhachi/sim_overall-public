#!/usr/bin/env python3
"""
Manual computation of track metrics from ASE_autocross_sim.csv
This script computes:
- Point count
- Polyline length
- Closure distance
- First 20 segment lengths
- First 12 interior curvature values (Menger curvature)
- Lookahead samples with curvature and speed calculations
- Minimum safe entry speed
- Trigger condition for current speed = 6 m/s
"""

import math
import json

# Parse CSV data (manually extracted)
csv_lines = """27.189984130859376,-51.61313171386719,4.5
27.315266799926757,-52.90124740600586,4.5
27.312076568603516,-54.18523025512695,4.5
27.17869415283203,-55.44683990478516,4.5
26.91482276916504,-56.67017440795898,4.5
26.52163772583008,-57.84217834472656,4.5
26.001780700683593,-58.95250701904297,4.5
25.359328079223634,-59.992941284179686,4.5
24.599750518798828,-60.95678024291992,4.5
23.7298583984375,-61.83843154907227,4.5
22.757744216918944,-62.63334579467774,4.5
21.692655944824217,-63.33828582763672,4.5
20.5447940826416,-63.95177536010742,4.5
19.325012969970704,-64.47451324462891,4.5
18.04443492889404,-64.90962982177734,4.5
16.71402053833008,-65.26270446777343,4.5
15.344137191772461,-65.54149627685547,4.5
13.944178199768066,-65.75546722412109,4.5
12.522270202636719,-65.91512451171874,4.5
11.08510456085205,-66.03130340576172,4.5"""

# Parse all points
points = []
for line in csv_lines.strip().split('\n'):
    parts = line.split(',')
    x, y = float(parts[0]), float(parts[1])
    points.append((x, y))

point_count = len(points)
print(f"Sample points parsed: {point_count}")
print(f"First point: {points[0]}")
print(f"Last in sample: {points[-1]}")

# For the actual computation, I need the full dataset
# Let me create a simpler inline version that processes the raw data
