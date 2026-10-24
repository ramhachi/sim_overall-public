"""
Curriculum learning scheduler for PINN training.

Implements 4-phase training to avoid loss balancing problems:
- Phase A (1-50): Data loss only (pre-training)
- Phase B (51-120): + Zero slip + Symmetry
- Phase C (121-200): + Friction circle + Monotonicity + trend constraints
- Phase D (201-260): Final weights (full physics constraints)
"""

from typing import Any, Dict


class CurriculumScheduler:
    """
    Manages physics loss weight scheduling across training phases.

    Loss balancing problem mitigation:
    - Start with data loss only to establish baseline accuracy
    - Gradually introduce physics constraints
    - Avoid catastrophic forgetting by keeping data weight high
    """

    BASE_TOTAL_EPOCHS = 260
    BASE_PHASE_A_END = 50
    BASE_PHASE_B_END = 120
    BASE_PHASE_C_END = 200
    BASE_PHASE_D_END = 260
    DEFAULT_FINAL_WEIGHTS = {
        "zero": 10.0,  # Most critical
        "sym": 5.0,
        "fric": 2.0,
        "mono": 1.0,  # Expensive autograd constraint
        "fy_sa_zero": 15.0,  # SA=0 Fy leakage constraint (critical)
        "combined": 0.50,  # Weak combined-slip shape prior
        "stiffness": 0.05,  # Weak small-slip load trend prior
        "high_sa_lateral": 0.10,  # High-|SA| |Fy| should not drop with load
        "mz_trail": 0.10,  # Weak Mz/pneumatic-trail sanity prior
        "camber": 0.10,  # Weak camber-thrust symmetry/cap prior
        "smoothness": 0.02,  # Weak pure-section curvature prior
    }

    def __init__(
        self,
        total_epochs: int = 260,
        data_weight: float = 1.0,
        final_weights: Dict[str, float] = None,
    ):
        """
        Args:
            total_epochs: Total training epochs
            data_weight: Data loss weight (fixed at 1.0)
            final_weights: Target physics weights at Phase D
        """
        self.total_epochs = max(int(total_epochs), 4)
        self.data_weight = data_weight
        self.PHASE_A_END, self.PHASE_B_END, self.PHASE_C_END, self.PHASE_D_END = (
            self._scale_phase_boundaries(self.total_epochs)
        )

        defaults = self.DEFAULT_FINAL_WEIGHTS.copy()
        if final_weights is None:
            final_weights = defaults
        else:
            final_weights = {**defaults, **final_weights}
        self.final_weights = final_weights

    @classmethod
    def _scale_phase_boundaries(cls, total_epochs: int) -> tuple[int, int, int, int]:
        phase_a_end = max(
            1, int(round(total_epochs * cls.BASE_PHASE_A_END / cls.BASE_TOTAL_EPOCHS))
        )
        phase_b_end = max(
            phase_a_end + 1,
            int(round(total_epochs * cls.BASE_PHASE_B_END / cls.BASE_TOTAL_EPOCHS)),
        )
        phase_c_end = max(
            phase_b_end + 1,
            int(round(total_epochs * cls.BASE_PHASE_C_END / cls.BASE_TOTAL_EPOCHS)),
        )
        phase_d_end = max(phase_c_end + 1, total_epochs)
        return phase_a_end, phase_b_end, phase_c_end, phase_d_end

    def get_weights(self, epoch: int) -> Dict[str, float]:
        """
        Get loss weights for current epoch.

        Args:
            epoch: Current epoch (1-indexed)

        Returns:
            dict with curriculum weights for data and all physics constraints.
        """
        weights = {"data": self.data_weight}
        physics_keys = [key for key in self.DEFAULT_FINAL_WEIGHTS]

        if epoch <= self.PHASE_A_END:
            # Phase A: Data loss only
            weights.update({key: 0.0 for key in physics_keys})

        elif epoch <= self.PHASE_B_END:
            # Phase B: gently introduce the easy constraints without overwhelming the data fit.
            progress = (epoch - self.PHASE_A_END) / (self.PHASE_B_END - self.PHASE_A_END)
            zero_target = 0.5 * self.final_weights["zero"]
            sym_target = 0.5 * self.final_weights["sym"]
            fy_sa_zero_target = 0.5 * self.final_weights["fy_sa_zero"]
            mz_trail_target = 0.25 * self.final_weights["mz_trail"]
            smoothness_target = 0.25 * self.final_weights["smoothness"]
            weights.update({key: 0.0 for key in physics_keys})
            weights.update(
                {
                    "zero": progress * zero_target,
                    "sym": progress * sym_target,
                    "fy_sa_zero": progress * fy_sa_zero_target,
                    "mz_trail": progress * mz_trail_target,
                    "smoothness": progress * smoothness_target,
                }
            )

        elif epoch <= self.PHASE_C_END:
            # Phase C: ramp every constraint to its configured final target.
            progress = (epoch - self.PHASE_B_END) / (self.PHASE_C_END - self.PHASE_B_END)
            zero_start = 0.5 * self.final_weights["zero"]
            sym_start = 0.5 * self.final_weights["sym"]
            fy_sa_zero_start = 0.5 * self.final_weights["fy_sa_zero"]
            mz_trail_start = 0.25 * self.final_weights["mz_trail"]
            smoothness_start = 0.25 * self.final_weights["smoothness"]
            weights.update(
                {
                    "zero": zero_start + progress * (self.final_weights["zero"] - zero_start),
                    "sym": sym_start + progress * (self.final_weights["sym"] - sym_start),
                    "fric": progress * self.final_weights["fric"],
                    "mono": progress * self.final_weights["mono"],
                    "fy_sa_zero": fy_sa_zero_start
                    + progress * (self.final_weights["fy_sa_zero"] - fy_sa_zero_start),
                    "combined": progress * self.final_weights["combined"],
                    "stiffness": progress * self.final_weights["stiffness"],
                    "high_sa_lateral": progress * self.final_weights["high_sa_lateral"],
                    "mz_trail": mz_trail_start
                    + progress * (self.final_weights["mz_trail"] - mz_trail_start),
                    "camber": progress * self.final_weights["camber"],
                    "smoothness": smoothness_start
                    + progress * (self.final_weights["smoothness"] - smoothness_start),
                }
            )

        else:
            # Phase D: Final weights
            weights.update(self.final_weights)

        return weights

    def get_phase(self, epoch: int) -> str:
        """Get current phase name."""
        if epoch <= self.PHASE_A_END:
            return "A"
        elif epoch <= self.PHASE_B_END:
            return "B"
        elif epoch <= self.PHASE_C_END:
            return "C"
        else:
            return "D"

    def get_phase_info(self, epoch: int) -> Dict[str, Any]:
        """
        Get detailed phase information.

        Returns:
            dict with keys: 'phase', 'epoch_in_phase', 'progress', 'weights'
        """
        phase = self.get_phase(epoch)
        weights = self.get_weights(epoch)

        if phase == "A":
            epoch_in_phase = epoch
            total_in_phase = self.PHASE_A_END
        elif phase == "B":
            epoch_in_phase = epoch - self.PHASE_A_END
            total_in_phase = self.PHASE_B_END - self.PHASE_A_END
        elif phase == "C":
            epoch_in_phase = epoch - self.PHASE_B_END
            total_in_phase = self.PHASE_C_END - self.PHASE_B_END
        else:
            epoch_in_phase = epoch - self.PHASE_C_END
            total_in_phase = self.PHASE_D_END - self.PHASE_C_END

        progress = epoch_in_phase / total_in_phase

        return {
            "phase": phase,
            "epoch_in_phase": epoch_in_phase,
            "total_in_phase": total_in_phase,
            "progress": progress,
            "weights": weights,
        }

    def should_validate(self, epoch: int, val_interval: int = 5) -> bool:
        """Determine if validation should run this epoch."""
        # Always validate at phase boundaries
        if epoch in [self.PHASE_A_END, self.PHASE_B_END, self.PHASE_C_END, self.PHASE_D_END]:
            return True
        # Regular interval
        return epoch % val_interval == 0

    def get_early_stopping_metric(self, epoch: int) -> str:
        """
        Get the metric name for early stopping based on phase.

        Phase A-B: Focus on data accuracy
        Phase C-D: Focus on physics constraints
        """
        phase = self.get_phase(epoch)
        if phase in ["A", "B"]:
            return "val_r2_mean"  # Data accuracy
        else:
            return "val_physics_loss_total"  # Physics satisfaction

    def print_schedule(self):
        """Print the weight schedule for all phases."""
        print("=" * 80)
        print("PINN Curriculum Learning Schedule")
        print("=" * 80)

        for epoch in [1, self.PHASE_A_END, self.PHASE_B_END, self.PHASE_C_END, self.PHASE_D_END]:
            info = self.get_phase_info(epoch)
            print(f"\nEpoch {epoch:3d} | Phase {info['phase']} | Progress: {info['progress']:.2%}")
            print(
                f"  Weights: data={info['weights']['data']:.1f}, "
                f"zero={info['weights']['zero']:.1f}, "
                f"sym={info['weights']['sym']:.1f}, "
                f"fric={info['weights']['fric']:.1f}, "
                f"mono={info['weights']['mono']:.1f}, "
                f"combined={info['weights']['combined']:.2f}, "
                f"stiffness={info['weights']['stiffness']:.2f}, "
                f"high_sa_lateral={info['weights']['high_sa_lateral']:.2f}, "
                f"mz_trail={info['weights']['mz_trail']:.2f}, "
                f"camber={info['weights']['camber']:.2f}, "
                f"smoothness={info['weights']['smoothness']:.2f}"
            )

        print("\n" + "=" * 80)


# Example usage
if __name__ == "__main__":
    scheduler = CurriculumScheduler()
    scheduler.print_schedule()
