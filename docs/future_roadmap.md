# 将来ロードマップ

この文書は、**現行実装とは切り分けて**今後の拡張方針を整理したメモです。

## 1. 14DOF 系

### 現状

- 標準実装は `vehicle_14dof_new.hpp` に一本化したい
- MATLAB / RL / ラップタイム系で同じ状態定義の参照表を持ちたい

### 今後の計画

1. 14DOF 状態定義表を C++ / MATLAB / docs で共有する
2. 代表イベントごとの検証ケースを固定化する
3. 内部状態の物理的意味を表にして保守しやすくする

## 2. タイヤモデル

### 現状

- 運用上は PINN が主系統
- Magic Formula / TMeasy は比較・フォールバックとして重要
- 学習済み deployable モデルの配置規約をさらに明確化したい

### 今後の計画

1. PINN 配布物の配置規約を固定する
2. C++ のモデル選択規則をさらに見える化する
3. 物理モデルと学習モデルの比較ベンチマークを常設する

## 3. サスペンション設計

### 現状

- フロント A アーム、リア A アーム、POU が分かれている
- `suspension_base_geometry.json` が中間成果物になっている

### 今後の計画

1. Front / Rear / POU の一貫パイプラインをさらに自動化する
2. 制約条件と目標値の由来を入力仕様として文書化する
3. 14DOF 用パラメータへの反映手順を簡略化する

## 4. フレーム最適化

### 現状

- GA + FEM の構成は動いている
- サスペンション取付点との連携が重要
- 規定、重量、剛性、ドライバー空間の同時最適化が核
- V2 schema により FBH / Front Hoop / Main Hoop / Main Hoop Brace / リア終端台形は設計変数として扱える入口ができた
- V2 はデフマウント入力を必須化し、torsion / lateral / braking / rollover / diff reaction の複数荷重ケースを artifact に保存する
- 2026-05-10 の初回 MATLAB `ga` 15000 評価では、torsion_only 探索 + 最終 full 再評価で 0.0215 s/eval まで戻ったが、V2 full load-case 目的関数は 0.2347 s/eval でまだ重かった
- 同日の安定化後は、MEX backend 優先、サスペンション hardpoint cache、有限 rejection penalty、feasible-neighborhood initial population、V2 係数再スケールにより、MATLAB `ga` の population 統計から `Inf` は消えた
- 現在の代表スモークは 309 評価、61.2 s、0.1980 s/eval、finite score rate 100%、true valid individual rate 62.5%、45.06 kg、4909 Nm/deg、規定 pass。MEX 単体は約 0.0014 s/eval なので、残る速度課題は MATLAB 目的関数/規定チェック/GA overhead 側
- 25 kg / 7000 Nm/deg 目標にはまだ未達で、現 topology の総管長 40 m級が重量下限を押し上げている
- 2026-05-11 の規定最小 topology probe では、固定 97 要素 topology は最軽量でも 31.03 kg だった一方、diff mount ありの sparse topology は 27.23 kg / 3357 Nm/deg、suspension-only sparse topology は 24.28 kg / 3357 Nm/deg まで落ちた
- ただし現 V2 の Upper Side Impact node model は F.6.4 の 290-345 mm 高さ帯を管全体で厳密表現していないため、sparse topology は最終適合設計ではなく下限確認として扱う

### 今後の計画

1. Side Impact と Front Hoop Bracing / FBH Support のノード幾何を rulebook F.6.2-F.6.4 に合わせて作り直す
2. 規定最小 topology を正式な初期値 / initial population seed として昇格し、そこから剛性を足す探索へ切り替える
3. V2 の optimizer 比較を 3 seed 以上・同一評価予算で継続し、`mixed_ga` から既定を変える根拠を作る
4. Side Impact / FBH support / MH brace support / rear box の不要要素削減と topology switch 化を進める
5. C++ FEM 側に応力 / 要素内力出力を追加し、V2 の荷重ケース評価を変位・剛性中心から強度評価へ拡張する
6. デフ envelope、リアサス、エンジン、排気、ドライバー template の除外領域を CAD / artifact 由来の実寸へ寄せる
7. 最適化結果を 14DOF パラメータへ戻す導線を強化する
8. V2 の高速化は、全ケース一律評価ではなく cheap check、段階評価、上位個体だけの full case、feasible-preserving encoding、MEX/TCP バックエンド比較の順で進める

## 5. ドライバーモデル

### 現状

- PID と RL が主系統
- MPC は比較対象として残っている

### 今後の計画

1. PID / RL / MPC の比較レポートを自動化する
2. 報酬関数、観測、行動の仕様を固定文書化する
3. イベント別のドライバー標準設定を用意する

## 6. 運用 UI

### 現状

- 標準入口は EasyStart
- Dashboard は一部機能を担う

### 今後の計画

1. EasyStart と Dashboard の責務分担を明確化する
2. 結果可視化をより UI から辿りやすくする
3. 実行ログと生成物の参照先を統一する
