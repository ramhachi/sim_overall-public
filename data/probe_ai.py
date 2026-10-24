
import struct
import os

filepath = '/Users/sota/projects/FomulaTMU/sim_overall/data/fast_lane.ai'
filesize = os.path.getsize(filepath)

print(f"File size: {filesize}")

with open(filepath, 'rb') as f:
    # Read first 4 integers (header candidates)
    h1 = struct.unpack('i', f.read(4))[0]
    h2 = struct.unpack('i', f.read(4))[0]
    h3 = struct.unpack('i', f.read(4))[0]
    h4 = struct.unpack('i', f.read(4))[0]
    
    print(f"Header Ints: {h1}, {h2}, {h3}, {h4}")
    
    # Remainder
    remainder = filesize - 16
    print(f"Remaining bytes: {remainder}")
    
    # Try reading as floats
    f.seek(16)
    content = f.read()
    num_floats = len(content) // 4
    print(f"Total floats in remainder: {num_floats}")
    
    # Print first 20 floats
    f.seek(16)
    print("First 20 floats:")
    for i in range(20):
        try:
            val_f = struct.unpack('f', f.read(4))[0]
            # Also read as int to check for IDs
            f.seek(-4, 1)
            val_i = struct.unpack('i', f.read(4))[0]
            print(f"Idx {i}: Float={val_f:10.4f}, Int={val_i}")
        except:
            break

    # Check divisibility
    for stride in range(1, 200):
        if remainder % (stride * 4) == 0:
            count = remainder // (stride * 4)
            print(f"Could be {count} records of {stride} floats ({stride*4} bytes) each.")
