function [Fx, Fy, Mx, Mz] = nn_tire_predict(SR, SA, Fz, IA, TSTC, net, norm_params)
% NN_TIRE_PREDICT ニューラルネットワークタイヤモデルによる力予測
%
% Inputs:
%   SR          - スリップ率 [-]
%   SA          - スリップ角 [deg]
%   Fz          - 垂直荷重 [N]
%   IA          - キャンバー角 [deg]
%   TSTC        - タイヤ表面温度 [°C]
%   net         - 学習済みNNモデル
%   norm_params - 正規化パラメータ構造体
%                 .input_mean (1x5)
%                 .input_std (1x5)
%                 .output_mean (1x4)
%                 .output_std (1x4)
%
% Outputs:
%   Fx  - 前後力 [N]
%   Fy  - 横力 [N]
%   Mx  - 転覆モーメント [Nm]
%   Mz  - セルフアライニングトルク [Nm]

% 入力ベクトル作成
X = [SR, SA, Fz, IA, TSTC];

% 入力正規化（Z-score normalization）
X_norm = (X - norm_params.input_mean) ./ norm_params.input_std;

% NN推論
Y_norm = predict(net, X_norm);

% 出力逆正規化
Y = Y_norm .* norm_params.output_std + norm_params.output_mean;

% 出力変数への展開
Fx = Y(1);
Fy = Y(2);
Mx = Y(3);
Mz = Y(4);
end
