# Installation & Quick Start Guide

**FSAE Frame Optimization Program (MATLAB Version)**

Last updated: 2026-02-24

---

## Prerequisites

Before you begin, ensure you have:

- **MATLAB R2018a or newer** (for C++14 MEX support)
- **C++ compiler** configured in MATLAB
- **Eigen3 library** (version 3.3 or newer)

---

## Step-by-Step Installation

### 1. Check MATLAB C++ Compiler

Open MATLAB and run:

```matlab
mex -setup C++
```

**Expected output:**
```
MEX configured to use 'Xcode Clang++' for C++ language compilation.
```

If no compiler is found:
- **macOS**: Install Xcode Command Line Tools: `xcode-select --install`
- **Linux**: Install GCC: `sudo apt-get install build-essential`
- **Windows**: Install Microsoft Visual Studio 2017 or newer

---

### 2. Install Eigen3

Eigen3 is a header-only C++ library for linear algebra. It's required for the MEX FEM solver.

#### macOS (Recommended: Homebrew)

```bash
brew install eigen
```

**Verify installation:**
```bash
ls /opt/homebrew/include/eigen3/Eigen/Dense  # ARM Mac (M1/M2)
# OR
ls /usr/local/include/eigen3/Eigen/Dense     # Intel Mac
```

#### Linux (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install libeigen3-dev
```

**Verify installation:**
```bash
ls /usr/include/eigen3/Eigen/Dense
```

#### Windows

1. Download Eigen from: https://eigen.tuxfamily.org/index.php?title=Main_Page
2. Extract ZIP file to `C:\eigen3`
3. Verify `C:\eigen3\Eigen\Dense` exists

---

### 3. Navigate to Project Directory

In MATLAB:

```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/frame_optimize
```

---

### 4. Build MEX Solver

**Build command:**

```matlab
cd matlab
build_mex()
```

**Expected successful output:**

```
Building MEX FEM solver...
Found Eigen3 at: /opt/homebrew/include/eigen3
Compiling with command:
  mex -v -R2018a -I/opt/homebrew/include/eigen3 ...

Building with 'Xcode Clang++'.
...
MEX completed successfully.

✓ MEX compilation successful!
  Location: /Users/sota/projects/FomulaTMU/sim_overall/frame_optimize/mex
  MEX file: beam_fem_solver.mexmaca64
```

**Common errors and solutions:**

| Error | Solution |
|-------|----------|
| `Eigen3 library not found` | Install Eigen3 (see Step 2 above) |
| `No supported compiler` | Run `mex -setup C++` and install compiler |
| `mex.h: No such file` | Update MATLAB to R2018a or newer |

---

### 5. Run Basic Tests

**Test MATLAB functions** (no MEX required):

```matlab
cd ../tests
test_basic_functionality
```

**Expected output:**

```
=== Frame Optimization Week 1 Tests ===

Test 1: Load materials from JSON...
  ✓ PASSED: Loaded 4 materials

Test 2: Load pipe catalog...
  ✓ PASSED: Loaded 17 pipes

...

=== TEST SUMMARY ===
Passed: 7 / 7
✓ ALL TESTS PASSED
```

---

### 6. Run Demo Simulation

**Simple 16-node frame torsion test:**

```matlab
cd ../examples
demo_simple_frame
```

**Expected output:**

```
=== FSAE Frame Optimization Demo ===

1. Loading materials and pipe catalog...
   Loaded 4 materials, 17 pipes

2. Creating simple frame geometry...
   Created 16 nodes, 16 elements

3. Setting up boundary conditions and loads...
   BCs: 12 constraints
   Loads: Applied ±1000 Nm torsion

4. Running MEX FEM solver...
   ✓ Solve completed in 3.245 ms

=== RESULTS ===
Mass:               9.34 kg
Torsional rigidity: 245.6 Nm/deg
Max displacement:   2.134 mm

✓ Demo completed successfully!
```

**Expected figures:**
- Figure 1: Frame geometry (3D plot with nodes and elements)
- Figure 2: Deformed shape (magnified 1000×)

---

## Troubleshooting

### Problem: "Eigen3 library not found"

**Solution 1**: Manual path specification

Edit `matlab/build_mex.m` and add at line 25:

```matlab
function eigen_path = detect_eigen_path()
    % Force custom path
    eigen_path = '/your/custom/path/to/eigen3';  % ← ADD THIS LINE
    return;  % ← ADD THIS LINE

    % (rest of auto-detection code...)
```

**Solution 2**: Check pkg-config (Linux/macOS)

```bash
pkg-config --cflags eigen3
```

If this fails, Eigen3 is not properly installed.

---

### Problem: MEX compilation fails with C++ errors

**Check compiler version:**

```matlab
mex -v -setup C++
```

Required:
- GCC ≥ 7.0
- Clang ≥ 5.0
- MSVC ≥ 2017 (Visual Studio 2017)

**Update compiler** if version is too old.

---

### Problem: "Invalid MEX-file" when running demo

This means the MEX file was compiled for a different platform.

**Solution**: Recompile MEX on your current machine:

```matlab
cd matlab
delete(['../mex/beam_fem_solver.', mexext])  % Delete old MEX file
build_mex()
```

---

### Problem: Demo runs but results are unrealistic

**Check:**
1. Torsional rigidity: Should be 100-500 Nm/deg for a simple frame
2. Mass: Should be 5-20 kg
3. Max displacement: Should be 1-5 mm

If results are far outside these ranges:
- Check node coordinates (units should be meters)
- Check element properties (D_outer in mm, E in GPa)
- Verify boundary conditions (at least 6 DOFs constrained)

---

## Next Steps

After successful installation:

1. **Explore the catalog**:
   ```matlab
   materials = loadMaterials('data/materials.json');
   catalog = loadPipeCatalog('data/pipe_catalog.json', materials);

   % View all Size A pipes
   for i = 1:length(catalog)
       if strcmp(catalog(i).size_class, 'A')
           fprintf('%s: %.1fmm × %.2fmm, I=%.0f mm⁴\n', ...
                   catalog(i).id, catalog(i).D_outer_mm, catalog(i).t_mm, catalog(i).Iy_mm4);
       end
   end
   ```

2. **Modify demo frame**:
   - Edit `examples/demo_simple_frame.m`
   - Change geometry (wheelbase, track width, height)
   - Try different pipe sizes from catalog
   - Apply different loads

3. **Prepare for Week 2**:
   - Week 2 will add regulation checking and exclusion zones
   - No additional installation required
   - Same MEX file will be used

---

## System Requirements

**Tested platforms:**
- macOS 14+ (ARM/Intel)
- Ubuntu 22.04 LTS
- Windows 11

**MATLAB versions:**
- R2018a (minimum)
- R2023b (recommended)
- R2024a (latest tested)

**Disk space:**
- ~10 MB (code + data)
- ~50 MB (MATLAB path cache)

**RAM:**
- 2 GB minimum
- 8 GB recommended for large frames (500+ elements)

---

## File Checklist

After installation, verify these files exist:

```
frame_optimize/
├── mex/
│   └── beam_fem_solver.mexmaca64    ← MEX binary (platform-specific extension)
├── data/
│   ├── materials.json                ← 4 materials
│   ├── pipe_catalog.json             ← 17 pipes
│   └── regulation_params.json        ← FSAE rules
└── matlab/
    └── (6 .m files)
```

**MEX extension by platform:**
- macOS ARM: `.mexmaca64`
- macOS Intel: `.mexmaca64`
- Linux: `.mexa64`
- Windows: `.mexw64`

---

## Getting Help

**Installation issues:**
1. Check MATLAB version: `version`
2. Check compiler: `mex -setup C++`
3. Check Eigen3: Run `build_mex()` and read error messages

**Simulation issues:**
1. Run `test_basic_functionality.m` first
2. Check demo output for warnings
3. Verify input units (meters, mm, GPa)

**Contact:**
- Project: Formula TMU FSAE Team
- Implementation: Claude Sonnet 4.5
- Date: 2026-02-24

---

## Uninstallation

To remove the MEX solver:

```matlab
cd frame_optimize/mex
delete(['beam_fem_solver.', mexext])
```

To remove Eigen3:

```bash
# macOS
brew uninstall eigen

# Linux
sudo apt-get remove libeigen3-dev
```

The MATLAB code and data files can be deleted by removing the `frame_optimize/` directory.

---

**Installation complete! 🎉**

You're now ready to optimize FSAE frames with the MEX FEM solver.
