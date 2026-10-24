#!/usr/bin/env python3
"""YAMLファイルを読み込んで、読みやすい形式で再保存"""

import yaml
import numpy as np

# 元のYAMLファイルを読み込む（unsafe_loadを使用）
with open('output/tire_params/hoosier_18x6_r20.yaml', 'r') as f:
    data = yaml.unsafe_load(f)

# numpy型をPython型に変換
def convert_numpy(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.integer, np.floating)):
        return float(obj)
    elif isinstance(obj, dict):
        return {k: convert_numpy(v) for k, v in obj.items()}
    elif isinstance(obj, (list, tuple)):
        return [convert_numpy(item) for item in obj]
    else:
        return obj

data = convert_numpy(data)

# 読みやすい形式で保存
with open('output/tire_params/hoosier_18x6_r20.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False, sort_keys=False)

print("YAMLファイルを修正しました")
