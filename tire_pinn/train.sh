#!/bin/bash
# Quick training script for PINN tire model

set -e

echo "=========================================="
echo "PINN Tire Model Training"
echo "=========================================="

# Check for --list-tires flag
if [[ "$1" == "--list-tires" ]] || [[ "$1" == "-l" ]]; then
    python3 tire_pinn/main_pinn_train.py --list-tires
    exit 0
fi

# Check Python version
python3 --version

# Check PyTorch installation
python3 -c "import torch; print(f'PyTorch {torch.__version__}')" || {
    echo "ERROR: PyTorch not installed!"
    echo "Install with: pip3 install torch torchvision torchaudio"
    exit 1
}

# Check CUDA availability
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"

# Create output directory
mkdir -p output/pinn/checkpoints
mkdir -p output/pinn/tensorboard

# Run training
python3 tire_pinn/main_pinn_train.py \
    --config tire_pinn/config/pinn_config.yaml \
    "$@"

echo ""
echo "=========================================="
echo "Training completed!"
echo "Results saved to: output/pinn/"
echo "=========================================="
