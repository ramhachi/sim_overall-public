"""
Property-based tests for HybridTireModel

These tests use the Hypothesis library to verify universal properties
across many randomly generated model configurations for the HybridTireModel.

Property 6: Model Type Query Accuracy
**Validates: Requirements 2.5**
"""

import pytest
from hypothesis import given, strategies as st, settings
from tire_fitting.models import TMeasyModel, MagicFormulaModel, HybridTireModel


# Strategy for generating valid TMeasy parameters
@st.composite
def tmeasy_params_strategy(draw):
    """Generate valid TMeasy parameters"""
    return {
        'mu_x_peak': draw(st.floats(min_value=0.8, max_value=2.0)),
        'k_mu_x': draw(st.floats(min_value=0.0, max_value=0.5)),
        'Kx': draw(st.floats(min_value=5000.0, max_value=30000.0)),
        'mu_y_peak': draw(st.floats(min_value=0.8, max_value=2.0)),
        'k_mu_y': draw(st.floats(min_value=0.0, max_value=0.5)),
        'Ky': draw(st.floats(min_value=5000.0, max_value=30000.0)),
        'C_gamma': draw(st.floats(min_value=0.0, max_value=0.1))
    }


# Strategy for generating valid Magic Formula parameters
@st.composite
def magic_formula_params_strategy(draw):
    """Generate valid Magic Formula parameters"""
    return {
        'pDx1': draw(st.floats(min_value=0.5, max_value=3.0)),
        'pCx1': draw(st.floats(min_value=1.0, max_value=2.5)),
        'pEx1': draw(st.floats(min_value=-1.0, max_value=1.0)),
        'pKx1': draw(st.floats(min_value=5.0, max_value=50.0)),
        'pKx2': draw(st.floats(min_value=-5.0, max_value=5.0)),
        'pKx3': draw(st.floats(min_value=-1.0, max_value=1.0)),
        'pDy1': draw(st.floats(min_value=0.5, max_value=3.0)),
        'pCy1': draw(st.floats(min_value=1.0, max_value=2.5)),
        'pEy1': draw(st.floats(min_value=-1.0, max_value=1.0)),
        'pKy1': draw(st.floats(min_value=5.0, max_value=50.0)),
        'pKy2': draw(st.floats(min_value=-5.0, max_value=5.0)),
        'pKy3': draw(st.floats(min_value=-1.0, max_value=1.0)),
        'fz0': draw(st.floats(min_value=500.0, max_value=2000.0))
    }


# Strategy for model type combinations
model_type_strategy = st.sampled_from(['tmeasy', 'magic_formula'])


class TestHybridModelTypeQueryAccuracy:
    """Property-based tests for model type query accuracy"""
    
    @settings(max_examples=50)
    @given(
        fx_model_type=model_type_strategy,
        fy_model_type=model_type_strategy,
        tmeasy_params=tmeasy_params_strategy(),
        mf_params=magic_formula_params_strategy(),
        fz_nominal=st.floats(min_value=500.0, max_value=2000.0)
    )
    def test_property_6_model_type_query_accuracy(
        self, 
        fx_model_type, 
        fy_model_type,
        tmeasy_params,
        mf_params,
        fz_nominal
    ):
        """
        Property 6: Model Type Query Accuracy
        
        For any instantiated tire model (TMeasy, Magic Formula, or Hybrid),
        querying the model type should return the type that was specified
        during instantiation.
        
        This property verifies that:
        1. TMeasy models return 'tmeasy'
        2. Magic Formula models return 'magic_formula'
        3. Hybrid models return 'hybrid(fx=<type>, fy=<type>)' with correct sub-types
        
        **Validates: Requirements 2.5**
        """
        # Create models based on randomly selected types
        if fx_model_type == 'tmeasy':
            fx_model = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        else:  # magic_formula
            fx_model = MagicFormulaModel(params=mf_params)
        
        if fy_model_type == 'tmeasy':
            fy_model = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        else:  # magic_formula
            fy_model = MagicFormulaModel(params=mf_params)
        
        # Test individual model type queries
        assert fx_model.get_model_type() == fx_model_type, \
            f"fx_model should return '{fx_model_type}', got '{fx_model.get_model_type()}'"
        
        assert fy_model.get_model_type() == fy_model_type, \
            f"fy_model should return '{fy_model_type}', got '{fy_model.get_model_type()}'"
        
        # Create hybrid model
        hybrid = HybridTireModel(fx_model=fx_model, fy_model=fy_model)
        
        # Query hybrid model type
        hybrid_type = hybrid.get_model_type()
        
        # Verify hybrid type format
        assert hybrid_type.startswith('hybrid('), \
            f"Hybrid model type should start with 'hybrid(', got '{hybrid_type}'"
        
        assert 'fx=' in hybrid_type, \
            f"Hybrid model type should contain 'fx=', got '{hybrid_type}'"
        
        assert 'fy=' in hybrid_type, \
            f"Hybrid model type should contain 'fy=', got '{hybrid_type}'"
        
        # Verify correct sub-model types are reported
        expected_hybrid_type = f"hybrid(fx={fx_model_type}, fy={fy_model_type})"
        assert hybrid_type == expected_hybrid_type, \
            f"Expected '{expected_hybrid_type}', got '{hybrid_type}'"
        
        # Verify consistency: get_parameters should also report correct model types
        params = hybrid.get_parameters()
        assert params['fx_model'] == fx_model_type, \
            f"get_parameters()['fx_model'] should be '{fx_model_type}', got '{params['fx_model']}'"
        
        assert params['fy_model'] == fy_model_type, \
            f"get_parameters()['fy_model'] should be '{fy_model_type}', got '{params['fy_model']}'"
    
    @settings(max_examples=20)
    @given(
        tmeasy_params=tmeasy_params_strategy(),
        fz_nominal=st.floats(min_value=500.0, max_value=2000.0)
    )
    def test_tmeasy_model_type_consistency(self, tmeasy_params, fz_nominal):
        """
        Test that TMeasy models always return 'tmeasy' as their type
        
        **Validates: Requirements 2.5**
        """
        model = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        
        # Query type multiple times - should always be consistent
        type1 = model.get_model_type()
        type2 = model.get_model_type()
        type3 = model.get_model_type()
        
        assert type1 == 'tmeasy'
        assert type2 == 'tmeasy'
        assert type3 == 'tmeasy'
        assert type1 == type2 == type3
    
    @settings(max_examples=20)
    @given(
        mf_params=magic_formula_params_strategy()
    )
    def test_magic_formula_model_type_consistency(self, mf_params):
        """
        Test that Magic Formula models always return 'magic_formula' as their type
        
        **Validates: Requirements 2.5**
        """
        model = MagicFormulaModel(params=mf_params)
        
        # Query type multiple times - should always be consistent
        type1 = model.get_model_type()
        type2 = model.get_model_type()
        type3 = model.get_model_type()
        
        assert type1 == 'magic_formula'
        assert type2 == 'magic_formula'
        assert type3 == 'magic_formula'
        assert type1 == type2 == type3
    
    @settings(max_examples=30)
    @given(
        fx_model_type=model_type_strategy,
        fy_model_type=model_type_strategy,
        tmeasy_params=tmeasy_params_strategy(),
        mf_params=magic_formula_params_strategy(),
        fz_nominal=st.floats(min_value=500.0, max_value=2000.0)
    )
    def test_hybrid_model_type_consistency_after_parameter_changes(
        self,
        fx_model_type,
        fy_model_type,
        tmeasy_params,
        mf_params,
        fz_nominal
    ):
        """
        Test that hybrid model type remains consistent even after parameter changes
        
        Changing parameters should not change the model type.
        
        **Validates: Requirements 2.5**
        """
        # Create models
        if fx_model_type == 'tmeasy':
            fx_model = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        else:
            fx_model = MagicFormulaModel(params=mf_params)
        
        if fy_model_type == 'tmeasy':
            fy_model = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        else:
            fy_model = MagicFormulaModel(params=mf_params)
        
        hybrid = HybridTireModel(fx_model=fx_model, fy_model=fy_model)
        
        # Get initial type
        initial_type = hybrid.get_model_type()
        expected_type = f"hybrid(fx={fx_model_type}, fy={fy_model_type})"
        assert initial_type == expected_type
        
        # Modify parameters
        if fx_model_type == 'tmeasy':
            hybrid.set_parameters({'fx_params': {'mu_x_peak': 1.5}})
        else:
            hybrid.set_parameters({'fx_params': {'pDx1': 2.0}})
        
        if fy_model_type == 'tmeasy':
            hybrid.set_parameters({'fy_params': {'mu_y_peak': 1.5}})
        else:
            hybrid.set_parameters({'fy_params': {'pDy1': 2.0}})
        
        # Type should remain unchanged
        final_type = hybrid.get_model_type()
        assert final_type == initial_type
        assert final_type == expected_type
    
    @settings(max_examples=20)
    @given(
        tmeasy_params=tmeasy_params_strategy(),
        mf_params=magic_formula_params_strategy(),
        fz_nominal=st.floats(min_value=500.0, max_value=2000.0)
    )
    def test_hybrid_model_type_distinguishes_direction(
        self,
        tmeasy_params,
        mf_params,
        fz_nominal
    ):
        """
        Test that hybrid model type correctly distinguishes fx and fy model types
        
        hybrid(fx=A, fy=B) should be different from hybrid(fx=B, fy=A)
        
        **Validates: Requirements 2.5**
        """
        tmeasy = TMeasyModel(fz_nominal=fz_nominal, params=tmeasy_params)
        mf = MagicFormulaModel(params=mf_params)
        
        # Create two hybrids with swapped models
        hybrid1 = HybridTireModel(fx_model=tmeasy, fy_model=mf)
        hybrid2 = HybridTireModel(fx_model=mf, fy_model=tmeasy)
        
        type1 = hybrid1.get_model_type()
        type2 = hybrid2.get_model_type()
        
        # Types should be different
        assert type1 != type2
        
        # Verify correct format
        assert type1 == "hybrid(fx=tmeasy, fy=magic_formula)"
        assert type2 == "hybrid(fx=magic_formula, fy=tmeasy)"
