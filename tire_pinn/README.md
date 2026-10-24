# PINN Tire Model

Physics-Informed Neural Network (PINN) for tire force prediction, combining data-driven learning with physics constraints.

## Features

- **Hybrid approach**: Data loss + Physics constraints
- **4 Physics constraints**:
  1. Zero slip: Fx=Fy=0 when SR=SA=0
  2. Symmetry: Fy(-SA) = -Fy(SA) when IA=0
  3. Friction circle: √(Fx²+Fy²) ≤ μ*Fz
  4. Monotonicity: ∂Fy/∂Fz ≥ 0
- **Curriculum learning**: 4-phase gradual constraint introduction
- **Smooth activations**: Swish (C∞) for autograd-friendly gradients
- **LayerNorm**: Batch-independent normalization for collocation points
- **C++ export**: High-precision JSON export for simulation

## Architecture

```
Input (5D): [SR, SA, Fz, IA, TSTC]
  ↓
8 hidden layers: [256, 256, 128, 128, 64, 64, 32, 32]
  ↓ (Swish + LayerNorm)
Output (4D): [Fx, Fy, Mx, Mz]
```

Total parameters: ~165k

## Installation

```bash
# Install PyTorch (choose your CUDA version)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install other dependencies
pip3 install numpy pandas pyyaml tqdm
```

## Usage

### 1. List Available Tire Models

```bash
# List all available tire models in dataset
python3 tire_pinn/main_pinn_train.py --list-tires

# Or using the quick script
./tire_pinn/train.sh --list-tires
```

Output shows:
- ✓ = Complete dataset (DriveBrake + Cornering)
- ✗ = Incomplete (missing Db or Cr data)

### 2. Training

```bash
# Default tire model (from config)
python3 tire_pinn/main_pinn_train.py --config tire_pinn/config/pinn_config.yaml

# Select specific tire model
python3 tire_pinn/main_pinn_train.py --tire-model MRF_18p0X6p0-10

# Multiple options
python3 tire_pinn/main_pinn_train.py \
    --tire-model Hoosier_20p5X7p0-13_R20 \
    --device cuda \
    --batch-size 1024 \
    --epochs 260
```

Options:
- `--tire-model <name>`: Select tire model (overrides config)
- `--list-tires`: List available tire models and exit
- `--device cuda/cpu`: Override device
- `--batch-size 512`: Override batch size
- `--epochs 260`: Override max epochs
- `--resume path/to/checkpoint.pt`: Resume training
- `--allow-nondeployable`: Keep exit code 0 for exploratory runs that do not pass deployment gates

## Available Tire Models

The dataset includes multiple tire models:

**Complete Datasets (Recommended)**:
- `Hoosier_18p0X6p0-10_R20_Rim6` - Hoosier 18.0×6.0-10 R20, 6-inch rim (default)
- `Hoosier_18p0X6p0-10_R20_Rim7` - Hoosier 18.0×6.0-10 R20, 7-inch rim
- `Hoosier_18p0X6p0-10_LCO_C2000_Rim6` - Hoosier 18.0×6.0-10 LCO C2000, 6-inch rim
- `Hoosier_18p0X6p0-10_LCO_C2000_Rim7` - Hoosier 18.0×6.0-10 LCO C2000, 7-inch rim
- `MRF_18p0X6p0-10` - MRF 18.0×6.0-10
- `Goodyear_18p0X6p5-10` - Goodyear 18.0×6.5-10
- `Hoosier_20p5X7p0-13_R20` - Hoosier 20.5×7.0-13 R20
- `Goodyear_20p0X7p0-13` - Goodyear 20.0×7.0-13

**Incomplete Datasets** (Cornering only):
- `Hoosier_16p0X6p0-10_R20_C2000`
- `Hoosier_16p0X7p5-10_R20_C2000`

Rim width is not part of the 5D PINN input, so Hoosier TTC datasets are kept in
separate tire keys when rim width differs. The wheel/rim diameter is encoded in
the tire size suffix, for example `-10`.

### 3. Validation

```bash
python3 tire_pinn/validation/physics_tests.py output/pinn/best_model.pt
```

### 4. C++ Export

```bash
python3 tire_pinn/export/cpp_exporter.py output/pinn/best_model.pt cpp_export/
```

Exports:
- `pinn_architecture.json`: Model structure
- `pinn_weights.json`: All parameters (15-digit precision)
- `pinn_normalization.json`: Input/output normalization params
- `test_vectors.json`: Numerical consistency test vectors

## Training Schedule

### Phase A (Epochs 1-50): Data Loss Only
- Pre-train on TTC data
- Establish baseline accuracy
- Target: R² > 0.95

### Phase B (Epochs 51-120): + Zero Slip + Symmetry
- Ramp physics weights: 0 → 5.0 (zero), 0 → 2.0 (sym)
- Critical constraints first

### Phase C (Epochs 121-200): + Friction Circle + Monotonicity
- Continue ramping: zero 5→10, sym 2→5
- Add friction (0→2), monotonicity (0→1)

### Phase D (Epochs 201-260): Final Weights
- Fixed weights: zero=10, sym=5, fric=2, mono=1
- Fine-tuning phase

## Validation Targets

- **R² (Fx, Fy)**: ≥ 0.90 by default (`validation_targets.r2_min`)
- **Zero slip error**: < 2N
- **Symmetry error**: < 1%
- **Friction violation rate**: < 2%
- **Monotonicity violations**: 0%

Non-deployable training/evaluation exits with a non-zero status by default and does not export
runtime assets. Use `--allow-nondeployable` only when generating exploratory diagnostics.

## Directory Structure

```
tire_pinn/
├── models/              # PINN architecture
├── losses/              # Data + physics losses
├── training/            # Trainer, curriculum, collocation
├── data/                # TTC data loader
├── validation/          # Physics tests
├── export/              # C++ exporter
├── config/              # YAML configs
└── main_pinn_train.py   # Entry point
```

## Key Differences from Standard DNN

| Aspect | Standard DNN | PINN |
|--------|-------------|------|
| Normalization | BatchNorm | LayerNorm |
| Activation | ReLU | Swish |
| Regularization | Dropout | Physics constraints |
| Training data | TTC only | TTC + Collocation points |
| Loss function | MSE | MSE + 4 physics terms |

## References

- Raissi et al. (2019): "Physics-informed neural networks"
- TTC Consortium: Tire test data format
- SAE J670e: Coordinate system conventions

## License

Internal use only - Formula TMU Racing Team
