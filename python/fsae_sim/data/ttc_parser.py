"""
TTCデータパーサー
FSAE Tire Test Consortiumデータの読み込みと処理
"""

import numpy as np
import pandas as pd
from pathlib import Path
from typing import Dict, Any, Optional
import json


class TTCDataParser:
    """TTCデータの読み込みと処理"""
    
    def __init__(self, data_dir: Path):
        self.data_dir = Path(data_dir)
    
    def load_mat_file(self, filename: str) -> Dict[str, np.ndarray]:
        """MATファイルを読み込み（scipy.io.loadmat使用）"""
        from scipy.io import loadmat
        
        filepath = self.data_dir / filename
        mat_data = loadmat(str(filepath))
        
        # MATLAB構造体から必要なデータを抽出
        return self._extract_tire_data(mat_data)
    
    def _extract_tire_data(self, mat_data: dict) -> Dict[str, np.ndarray]:
        """MATデータから標準形式に変換"""
        # TTCデータの典型的な構造
        result = {}
        
        # 一般的なチャンネル名
        channels = {
            'SA': 'slip_angle',      # スリップ角 [deg]
            'FY': 'lateral_force',   # 横力 [N]
            'FX': 'longitudinal_force',  # 縦力 [N]
            'FZ': 'vertical_load',   # 垂直荷重 [N]
            'MZ': 'aligning_moment', # セルフアライニングトルク [Nm]
            'SR': 'slip_ratio',      # スリップ率 [-]
            'IA': 'inclination_angle',  # キャンバー角 [deg]
            'P': 'pressure',         # 内圧 [kPa]
            'V': 'velocity',         # 速度 [m/s]
        }
        
        for mat_key, std_key in channels.items():
            if mat_key in mat_data:
                result[std_key] = np.array(mat_data[mat_key]).flatten()
        
        return result
    
    def fit_pacejka(self, data: Dict[str, np.ndarray]) -> Dict[str, float]:
        """Pacejkaパラメータをフィッティング（簡易版）"""
        from scipy.optimize import curve_fit
        
        # Magic Formula: y = D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))
        def magic_formula(x, B, C, D, E):
            Bx = B * x
            return D * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))
        
        # 横力フィッティング
        sa = data.get('slip_angle', np.array([]))
        fy = data.get('lateral_force', np.array([]))
        fz = data.get('vertical_load', np.array([]))
        
        if len(sa) == 0 or len(fy) == 0:
            return {}
        
        # 基準荷重でフィルタリング
        fz_nominal = 1500  # [N]
        mask = np.abs(fz - fz_nominal) < 200
        
        if np.sum(mask) < 10:
            return {}
        
        sa_fit = sa[mask]
        fy_fit = fy[mask]
        
        # 正規化
        fy_norm = fy_fit / fz_nominal
        
        try:
            popt, _ = curve_fit(
                magic_formula, 
                np.radians(sa_fit), 
                fy_norm,
                p0=[10, 1.3, 1.8, 0.1],
                bounds=([1, 0.5, 0.5, -1], [30, 2.0, 3.0, 1])
            )
            return {
                'B': float(popt[0]),
                'C': float(popt[1]),
                'D': float(popt[2]),
                'E': float(popt[3]),
                'Fz_nom': fz_nominal
            }
        except Exception as e:
            print(f"Fitting failed: {e}")
            return {}
    
    def export_to_json(self, params: Dict[str, Any], output_path: Path):
        """パラメータをJSON形式で出力"""
        with open(output_path, 'w') as f:
            json.dump(params, f, indent=2)


def process_ttc_data(input_dir: str, output_dir: str):
    """TTCデータ処理のメインエントリ"""
    parser = TTCDataParser(Path(input_dir))
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # 全MATファイルを処理
    mat_files = list(Path(input_dir).glob("*.mat"))
    
    all_params = {}
    for mat_file in mat_files:
        print(f"Processing: {mat_file.name}")
        data = parser.load_mat_file(mat_file.name)
        params = parser.fit_pacejka(data)
        
        if params:
            tire_name = mat_file.stem
            all_params[tire_name] = params
    
    # 結果を保存
    parser.export_to_json(all_params, output_path / "tire_params.json")
    print(f"Saved: {output_path / 'tire_params.json'}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 3:
        process_ttc_data(sys.argv[1], sys.argv[2])
    else:
        print("Usage: python ttc_parser.py <input_dir> <output_dir>")
