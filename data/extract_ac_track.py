
import struct
import csv
import os

filepath = '/Users/sota/projects/FomulaTMU/sim_overall/data/fast_lane.ai'
output_csv = '/Users/sota/projects/FomulaTMU/sim_overall/data/fast_lane.csv'

def extract_track():
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        return

    points = []
    
    with open(filepath, 'rb') as f:
        # Header (16 bytes)
        # Version, Count(?), ? , ?
        header_data = f.read(16)
        version, count, u1, u2 = struct.unpack('iiii', header_data)
        print(f"Header: Version={version}, Count={count}")
        
        # Read points
        # Structure: float x, float y, float z, float dist, int id
        # Total size: 20 bytes
        
        # Read until end of file
        content = f.read()
        num_floats = len(content) // 4
        num_records_file = len(content) // 20
        
        print(f"Header says {count} records. File size allows {num_records_file} records.")
        
        # Only read the number of records specified in the header
        # The rest of the file might be candidate lines, pit lines, or old data
        records_to_read = count
        
        for i in range(records_to_read):
            offset = i * 20
            record_data = content[offset:offset+20]
            if len(record_data) < 20:
                break
                
            x, y, z, dist = struct.unpack('ffff', record_data[:16])
            idx = struct.unpack('i', record_data[16:])[0]
            
            points.append([x, y, z, dist, idx])

    # Write CSV
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'dist', 'id'])
        writer.writerows(points)
        
    print(f"Extracted {len(points)} points to {output_csv}")

if __name__ == "__main__":
    extract_track()
