# ドキュメント索引

`docs/` は、**現在使っている実装に一致する説明だけ**を置く棚です。  
理論はルートの [理論総合ガイド](../学生フォーミュラ車両運動理論総合ガイド.md)、実装と運用の大枠は [実装総合ガイド](../学生フォーミュラ車両運動シミュレーション総合ガイド.md) に分け、ここでは現役入口とソース別説明を整理します。

## まず読む文書

1. [理論総合ガイド](../学生フォーミュラ車両運動理論総合ガイド.md)
2. [実装総合ガイド](../学生フォーミュラ車両運動シミュレーション総合ガイド.md)
3. [現行システムマップ](current_system_map.md)
4. [ソース別リファレンス](source_reference/README.md)
5. [将来ロードマップ](future_roadmap.md)

## 現在の標準入口

| 区分 | 主入口 | 補足 |
|:---|:---|:---|
| 日常運用 | `EasyStart.m` | もっとも優先する入口 |
| 14DOF 本体 | `cpp/src/main_14dof.cpp` | 正本は `vehicle_14dof_new.hpp` |
| ラップタイム | `cpp/src/main_laptime_sim.cpp` | PID / RL ドライバーを統合 |
| 統合実行 | `matlab/run_full_pipeline.m` | 出力は `output/` 配下へ集約 |
| タイヤ | `tire_pinn/`, `tire_fitting/` | 運用上は PINN 主系統 |
| サスペンション | `suspension_kinematics_design/` | Front / Rear A アーム最適化 |
| フレーム | `frame_optimize/` | GA + FEM |
| 再現性 | [docs/reproducibility.md](reproducibility.md) | 依存固定点と更新手順 |

## docs/ が扱わないもの

- `build/`, `vcpkg_installed/`, `output/` などの生成物
- 配布データやベンダー資産
- 現役導線から外れた検討メモ

## 運用ルール

1. 入口が変わったら `README.md`、総合ガイド、`current_system_map.md` を同時更新する
2. ソースを移動したら `source_reference/` のパスも合わせる
3. 未実装や構想は `future_roadmap.md` に寄せる
