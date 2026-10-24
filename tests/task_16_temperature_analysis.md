# Task 16: Temperature Dependency Analysis

**Date:** 2026-01-24  
**Status:** Analysis Complete

---

## Question: Does the model include temperature dependency?

**Answer: NO** - Neither the Magic Formula nor TMeasy models currently include temperature dependency.

---

## Model Analysis

### Magic Formula Model (tire_fitting/models/magic_formula.py)

**Current Parameters:**
- Longitudinal: slip_ratio, fz
- Lateral: slip_angle, fz
- Load dependency: Yes (pKx2, pKx3, pKy2, pKy3)
- **Temperature dependency: NO**

**Implementation:**
```python
def calculate_fx(self, slip_ratio: float, fz: float, **kwargs) -> float:
    # D = pDx1 * fz  (no temperature term)
    # K = fz * (pKx1 + pKx2 * dfz) * exp(pKx3 * dfz)
    # No temperature parameters used
```

### TMeasy Model (tire_fitting/models/tmeasy.py)

**Current Parameters:**
- Longitudinal: slip_ratio, fz
- Lateral: slip_angle, fz, camber_angle
- Load dependency: Yes (k_mu_x, k_mu_y)
- **Temperature dependency: NO**

**Implementation:**
```python
def compute_longitudinal_force(self, slip_ratio, fz, mu_x_peak, k_mu_x, Kx):
    # mu_x = mu_x_peak * (1.0 - k_mu_x * dfz)  (no temperature term)
    # No temperature parameters used
```

---

## Available Temperature Data

The tire data files contain the following temperature columns:
- **TSTI:** Inner tire surface temperature
- **TSTC:** Center tire surface temperature  
- **TSTO:** Outer tire surface temperature
- **AmbTmp:** Ambient temperature

**These temperature measurements are currently NOT used by either model.**

---

## Impact on Model Accuracy

### Error Analysis

| Direction | Median Error | RMSE as % of Mean | Notes |
|-----------|--------------|-------------------|-------|
| **Longitudinal (FX)** | 18.9% | 33.7% | Higher errors, more temperature variation |
| **Lateral (FY)** | 5.9% | 9.0% | Lower errors, less temperature variation |

### Hypothesis

The higher longitudinal force errors (18.9% median) may be partially explained by:
1. **Temperature variations in DriveBrake data** - Acceleration/braking generates more heat
2. **Grip changes with temperature** - Friction coefficient varies with tire temperature
3. **No temperature compensation** - Models cannot adapt to thermal effects

The lower lateral errors suggest:
1. **Less temperature variation in Cornering data** - Steady-state cornering
2. **More consistent thermal conditions** - Less aggressive tire heating
3. **Temperature effects less pronounced** - Lateral grip less temperature-sensitive

---

## Recommendations for Temperature-Dependent Modeling

### Priority: HIGH
Adding temperature dependency could reduce longitudinal RMSE from 393.59 N to < 300 N (target).

### Option 1: Magic Formula Temperature Extension

**Extend peak force coefficient:**
```python
# Current:
D = pDx1 * fz

# Proposed:
D = pDx1 * fz * (1 + pDx2_temp * dT)
where dT = (T - T_ref) / T_ref
```

**New parameters:**
- `pDx2_temp`: Temperature sensitivity for peak force (typical: 0.001 to 0.01 per °C)
- `T_ref`: Reference temperature (e.g., 60°C)

**Temperature input:**
- Use average of TSTI, TSTC, TSTO: `T = (TSTI + TSTC + TSTO) / 3`

### Option 2: TMeasy Temperature Extension

**Extend friction coefficient:**
```python
# Current:
mu_x = mu_x_peak * (1 - k_mu_x * dfz)

# Proposed:
mu_x = mu_x_peak * (1 - k_mu_x * dfz) * (1 + k_temp_x * dT)
mu_y = mu_y_peak * (1 - k_mu_y * dfz) * (1 + k_temp_y * dT)
```

**New parameters:**
- `k_temp_x`: Longitudinal temperature sensitivity
- `k_temp_y`: Lateral temperature sensitivity

### Implementation Steps

1. **Modify model classes:**
   - Add temperature parameters to `__init__`
   - Update `calculate_fx` and `calculate_fy` to accept temperature
   - Modify force equations to include temperature terms

2. **Update optimizer:**
   - Add temperature parameter bounds
   - Pass temperature data to force calculations
   - Fit temperature coefficients alongside other parameters

3. **Update data pipeline:**
   - Calculate average tire temperature from TSTI, TSTC, TSTO
   - Pass temperature to model during fitting and prediction

4. **Validate improvements:**
   - Re-run fitting with temperature-dependent models
   - Compare RMSE, TIC, R² with baseline
   - Target: Longitudinal RMSE < 300 N, median error < 12%

---

## Expected Benefits

**Quantitative:**
- Reduce longitudinal RMSE: 393.59 N → < 300 N (23% improvement target)
- Reduce median error: 18.9% → < 12% (36% improvement target)
- Improve R²: 0.9173 → > 0.95 (meet threshold)

**Qualitative:**
- Better capture tire warm-up behavior
- More accurate predictions during aggressive driving
- Improved model fidelity for transient conditions
- Enhanced simulation realism for vehicle dynamics

---

## Conclusion

**Current Status:**
- ✓ Models implemented correctly per MF5.2 and TMeasy specifications
- ✓ No temperature dependency is a design choice, not a bug
- ⚠ Temperature data available but unused

**Next Steps:**
- Document this limitation in user guide
- Consider temperature extension as Phase 2 enhancement
- Prioritize based on application requirements (steady-state vs transient)

**For Production Use:**
- Current models are suitable for steady-state analysis
- For transient thermal analysis, temperature extension is recommended
- Document operating temperature range assumptions (e.g., 40-80°C)

---

**Analysis Completed:** 2026-01-24  
**Recommendation:** Add temperature dependency in next development phase
