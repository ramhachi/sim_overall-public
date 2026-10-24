#!/bin/bash
set -e

# Configuration (matching MATLAB defaults or likely usage)
TRACK_FILE="/workspace/data/ASE_autocross_sim.csv"
RADIUS=0.0
SIM_TIME=10.0
INIT_SPEED=5.0
OUTPUT_CSV="manual_test.csv"

# PID Gains (Arbitrary stable-ish values for testing execution)
KP_STEER=1.2
KD_STEER=0.5
KI_STEER=0.05
KH_STEER=1.0
KFF_STEER=1.0
KP_SPEED=0.2
KI_SPEED=0.05
MAX_THROTTLE=0.5

echo "========================================================"
echo "1. Building the simulation (Clean & Build)"
echo "========================================================"
docker compose run --rm fsae-sim bash -c "cd build && cmake .. && make laptime_sim -j$(nproc)"

echo ""
echo "========================================================"
echo "2. Running Simulation (mimicking MATLAB command)"
echo "========================================================"

# Mimic the MATLAB command construction exactly
# Note: We output to a temp file in build, then cp it to a visible location
CMD="cd build && ./cpp/laptime_sim \
  --track ${TRACK_FILE} \
  --radius ${RADIUS} \
  --time ${SIM_TIME} \
  --initial-speed ${INIT_SPEED} \
  --kp-steer ${KP_STEER} \
  --kd-steer ${KD_STEER} \
  --ki-steer ${KI_STEER} \
  --kh-steer ${KH_STEER} \
  --kff-steer ${KFF_STEER} \
  --kp-speed ${KP_SPEED} \
  --ki-speed ${KI_SPEED} \
  --max-throttle ${MAX_THROTTLE} \
  --output ${OUTPUT_CSV} \
  && cp ${OUTPUT_CSV} /workspace/${OUTPUT_CSV}"

echo "Docker Command to run:"
echo "docker compose run --rm fsae-sim bash -c \"${CMD}\""

docker compose run --rm fsae-sim bash -c "${CMD}"

echo ""
echo "========================================================"
echo "3. Checking Results"
echo "========================================================"
if [ -f "${OUTPUT_CSV}" ]; then
    echo "SUCCESS: Output file ${OUTPUT_CSV} created."
    echo "Last 5 lines:"
    tail -n 5 ${OUTPUT_CSV}
else
    echo "FAILURE: Output file ${OUTPUT_CSV} was not created on host."
fi
