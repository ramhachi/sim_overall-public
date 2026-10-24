"""
Fit Evaluator Module

This module provides the FitEvaluator class for evaluating tire model
fitting results through metrics calculation and visualization.
"""

import numpy as np
import matplotlib.pyplot as plt
import logging
from typing import Dict, Optional, Union
from pathlib import Path


class FitEvaluator:
    """
    Evaluator for tire model fitting results.
    
    This class provides methods for:
    - Calculating evaluation metrics (RMSE, NRMSE, R², TIC)
    - Computing peak friction coefficient errors
    - Generating visualization plots (force vs slip, residuals)
    
    The evaluator helps assess the quality of fitted parameters and
    provides graphical verification of model accuracy.
    """
    
    def __init__(self):
        """Initialize the evaluator with logging."""
        self.logger = logging.getLogger(__name__)
    
    def calculate_metrics(self,
                         y_true: np.ndarray,
                         y_pred: np.ndarray) -> Dict[str, float]:
        """
        Calculate evaluation metrics for fitted model.
        
        Computes four key metrics:
        - RMSE: Root Mean Square Error
        - NRMSE: Normalized RMSE (by range of measured values)
        - R²: Coefficient of determination
        - TIC: Theil Inequality Coefficient
        
        Args:
            y_true: Measured values (e.g., force data) [N]
            y_pred: Predicted values from model [N]
            
        Returns:
            Dictionary containing:
                - rmse: Root mean square error [N]
                - nrmse: Normalized RMSE [-]
                - r2: R-squared coefficient [-]
                - tic: Theil inequality coefficient [-]
                
        Notes:
            - Implements Requirements 9.1, 9.2, 9.3, 9.4
            - All metrics are computed from the same data
            - NRMSE is dimensionless and easier to interpret
            - R² indicates proportion of variance explained
            - TIC is scale-independent and ranges from 0 to 1
        """
        self.logger.debug(f"Calculating metrics for {len(y_true)} data points")
        
        # Ensure inputs are numpy arrays
        y_true = np.asarray(y_true)
        y_pred = np.asarray(y_pred)
        
        # Calculate RMSE (Requirement 9.1)
        # RMSE = sqrt(mean((y_true - y_pred)^2))
        rmse = np.sqrt(np.mean((y_true - y_pred)**2))
        self.logger.debug(f"  RMSE = {rmse:.2f}")
        
        # Calculate NRMSE (Requirement 9.2)
        # NRMSE = RMSE / (max(y_true) - min(y_true))
        y_range = np.max(y_true) - np.min(y_true)
        nrmse = rmse / y_range if y_range > 0 else 0.0
        self.logger.debug(f"  NRMSE = {nrmse:.4f}")
        
        # Calculate R-squared (Requirement 9.3)
        # R² = 1 - SS_res / SS_tot
        # SS_res = sum((y_true - y_pred)^2)
        # SS_tot = sum((y_true - mean(y_true))^2)
        ss_res = np.sum((y_true - y_pred)**2)
        ss_tot = np.sum((y_true - np.mean(y_true))**2)
        r2 = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0
        self.logger.debug(f"  R² = {r2:.4f}")
        
        # Calculate Theil Inequality Coefficient (Requirement 9.4)
        # TIC = sqrt(sum((y_true - y_pred)^2)) / (sqrt(sum(y_true^2)) + sqrt(sum(y_pred^2)))
        numerator = np.sqrt(np.sum((y_true - y_pred)**2))
        denominator = np.sqrt(np.sum(y_true**2)) + np.sqrt(np.sum(y_pred**2))
        tic = numerator / denominator if denominator > 0 else 0.0
        self.logger.debug(f"  TIC = {tic:.4f}")
        
        self.logger.info(f"Metrics calculated: RMSE={rmse:.2f}, NRMSE={nrmse:.4f}, R²={r2:.4f}, TIC={tic:.4f}")
        
        return {
            'rmse': rmse,
            'nrmse': nrmse,
            'r2': r2,
            'tic': tic
        }
    
    def calculate_peak_friction_error(self,
                                     force_data: np.ndarray,
                                     fz_data: np.ndarray,
                                     force_pred: np.ndarray) -> float:
        """
        Calculate peak friction coefficient error.
        
        Computes the relative error between measured and predicted peak
        friction coefficients:
        - Peak friction = max(|Force|) / Fz at that point
        - Error = |mu_peak_data - mu_peak_pred| / mu_peak_data * 100
        
        Args:
            force_data: Measured force values [N]
            fz_data: Vertical load values [N]
            force_pred: Predicted force values [N]
            
        Returns:
            Peak friction coefficient error [%]
            
        Notes:
            - Implements Requirement 9.5
            - Finds peak friction from measured data
            - Compares with predicted friction at same conditions
            - Returns percentage error for easy interpretation
        """
        # Ensure inputs are numpy arrays
        force_data = np.asarray(force_data)
        fz_data = np.asarray(fz_data)
        force_pred = np.asarray(force_pred)
        
        # Calculate friction coefficients
        # mu = |Force| / Fz
        mu_data = np.abs(force_data) / fz_data
        mu_pred = np.abs(force_pred) / fz_data
        
        # Find peak friction coefficient from measured data
        mu_peak_data = np.max(mu_data)
        
        # Handle edge case where peak friction is zero
        if mu_peak_data == 0.0:
            return 0.0
        
        # Find predicted friction at the same index
        peak_idx = np.argmax(mu_data)
        mu_peak_pred = mu_pred[peak_idx]
        
        # Calculate percentage error
        # Error = |mu_peak_data - mu_peak_pred| / mu_peak_data * 100
        error = np.abs(mu_peak_data - mu_peak_pred) / mu_peak_data * 100.0
        
        return error
    
    def plot_force_vs_slip(self,
                          slip_data: np.ndarray,
                          force_data: np.ndarray,
                          force_pred: np.ndarray,
                          load_groups: Optional[Dict[str, np.ndarray]] = None,
                          xlabel: str = "Slip",
                          ylabel: str = "Force [N]",
                          title: str = "Force vs Slip",
                          save_path: Optional[str] = None):
        """
        Generate force vs slip plot with measured and predicted data.
        
        Creates a scatter plot of measured data and line plot of predicted
        data. If load groups are provided, plots each group in different colors.
        
        Args:
            slip_data: Slip ratio or slip angle values
            force_data: Measured force values [N]
            force_pred: Predicted force values [N]
            load_groups: Optional dict mapping group names to boolean masks
                        e.g., {'Low': mask_low, 'Medium': mask_med, 'High': mask_high}
            xlabel: X-axis label (default: "Slip")
            ylabel: Y-axis label (default: "Force [N]")
            title: Plot title (default: "Force vs Slip")
            save_path: Optional path to save figure as PNG
                      If None, displays plot interactively
                      
        Notes:
            - Implements Requirements 10.1, 10.2, 10.5
            - Measured data shown as scatter points
            - Predicted data shown as lines
            - Different colors for different load groups
            - Saves to PNG format if save_path provided
        """
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 6))
        
        if load_groups is not None:
            # Plot each load group separately
            colors = {'Low': 'blue', 'Medium': 'green', 'High': 'red'}
            
            for group_name, mask in load_groups.items():
                color = colors.get(group_name, 'black')
                
                # Plot measured data as scatter
                ax.scatter(slip_data[mask], force_data[mask],
                          alpha=0.5, s=20, color=color,
                          label=f'{group_name} (measured)')
                
                # Plot predicted data as line
                # Sort by slip for smooth line
                sort_idx = np.argsort(slip_data[mask])
                ax.plot(slip_data[mask][sort_idx], force_pred[mask][sort_idx],
                       linewidth=2, color=color, linestyle='--',
                       label=f'{group_name} (predicted)')
        else:
            # Plot all data together
            ax.scatter(slip_data, force_data,
                      alpha=0.5, s=20, color='blue',
                      label='Measured')
            
            # Sort by slip for smooth line
            sort_idx = np.argsort(slip_data)
            ax.plot(slip_data[sort_idx], force_pred[sort_idx],
                   linewidth=2, color='red', linestyle='--',
                   label='Predicted')
        
        # Set labels and title
        ax.set_xlabel(xlabel, fontsize=12)
        ax.set_ylabel(ylabel, fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Tight layout
        plt.tight_layout()
        
        # Save or show
        if save_path is not None:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            plt.close()
        else:
            plt.show()
    
    def plot_residuals(self,
                      y_true: np.ndarray,
                      y_pred: np.ndarray,
                      save_path: Optional[str] = None):
        """
        Generate residual plot for model evaluation.
        
        Creates two subplots:
        1. Residuals vs predicted values (to check for patterns)
        2. Histogram of residuals (to check for normality)
        
        Args:
            y_true: Measured values [N]
            y_pred: Predicted values [N]
            save_path: Optional path to save figure as PNG
                      If None, displays plot interactively
                      
        Notes:
            - Implements Requirements 10.4, 10.5
            - Residuals = y_true - y_pred
            - Good fit shows random scatter around zero
            - Histogram should be approximately normal
            - Saves to PNG format if save_path provided
        """
        # Calculate residuals
        residuals = y_true - y_pred
        
        # Create figure with two subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        # Subplot 1: Residuals vs Predicted
        ax1.scatter(y_pred, residuals, alpha=0.5, s=20, color='blue')
        ax1.axhline(y=0, color='red', linestyle='--', linewidth=2)
        ax1.set_xlabel('Predicted Force [N]', fontsize=12)
        ax1.set_ylabel('Residuals [N]', fontsize=12)
        ax1.set_title('Residuals vs Predicted Values', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        
        # Subplot 2: Histogram of Residuals
        ax2.hist(residuals, bins=50, color='blue', alpha=0.7, edgecolor='black')
        ax2.axvline(x=0, color='red', linestyle='--', linewidth=2)
        ax2.set_xlabel('Residuals [N]', fontsize=12)
        ax2.set_ylabel('Frequency', fontsize=12)
        ax2.set_title('Distribution of Residuals', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Add statistics text
        mean_res = np.mean(residuals)
        std_res = np.std(residuals)
        ax2.text(0.05, 0.95, f'Mean: {mean_res:.2f} N\nStd: {std_res:.2f} N',
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
                fontsize=10)
        
        # Tight layout
        plt.tight_layout()
        
        # Save or show
        if save_path is not None:
            Path(save_path).parent.mkdir(parents=True, exist_ok=True)
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            plt.close()
        else:
            plt.show()
