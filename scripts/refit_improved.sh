#!/bin/bash
# 改善されたパラメータでタイヤフィッティングを再実行

echo "=========================================="
echo "改善されたパラメータでフィッティング開始"
echo "=========================================="
echo ""
echo "変更点:"
echo "  1. パラメータ境界の調整:"
echo "     - mu_x_peak, mu_y_peak: [1.3, 2.5] (下限を1.0→1.3に上昇)"
echo "     - k_mu_x, k_mu_y: [0.05, 0.5] (下限を0.0→0.05に上昇)"
echo "     - Kx: [15000, 50000] (下限を5000→15000に上昇)"
echo "     - Ky: [10000, 50000] (下限を5000→10000に上昇)"
echo ""
echo "  2. データフィルタリングの緩和:"
echo "     - 温度範囲: 30-90°C (従来: 40-80°C)"
echo "     - 最小速度: 25 kph (従来: 30 kph)"
echo ""
echo "実行中..."
echo ""

python3 tire_fitting/main.py \
    --drivebrake data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv \
    --cornering data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv \
    --output output/tire_params_v2 \
    --tire-name Hoosier_18x6_R20_v2 \
    --min-speed 25.0 \
    --temp-min 30.0 \
    --temp-max 90.0 \
    --remove-outliers

echo ""
echo "=========================================="
echo "フィッティング完了"
echo "=========================================="
echo ""
echo "次のステップ: 検証を実行"
echo "  python3 tire_fitting/validate_predictions.py \\"
echo "    --params output/tire_params_v2/hoosier_18x6_r20_v2.yaml \\"
echo "    --output output/tire_params_v2 \\"
echo "    --n-samples 100"
echo ""
