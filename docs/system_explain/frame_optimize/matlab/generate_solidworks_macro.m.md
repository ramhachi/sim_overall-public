# generate_solidworks_macro.m
## ファイル目的
- 最適化済みフレームを SolidWorks で再構築する `.swb` VBA マクロを自動生成する。

## 実行フロー
- オプションを解釈し、フレーム座標を SolidWorks 座標へ変換する。
- 要素を Front/Main Hoop、FBH、Side Impact、Rear、Bracket などへ分類する。
- ベント部のフィレット半径とグループ別プロファイル情報を作る。
- VBA 各セクションを生成し、late binding 化と静的検証を行う。
- UTF-8 の `.swb` として書き出す。

## 関数詳細
### `output_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials, options)`
- シグネチャ: `function output_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials, options)`
- 入力:
  - `nodes`, `elements_geom`
  - `best_x`: 先頭 5 要素に管プロファイル添字
  - `catalog`, `materials`, `options`
- 出力:
  - `output_path`: 生成 `.swb` パス
- 責務:
  - SolidWorks VBA 全体生成
- 主要ロジック:
  - `materials` は現実装では未使用
  - `emit_vba_*` でセクション分割生成
  - `convert_to_late_binding` と `validate_vba` を必ず通す
- 依存関係:
  - 下記ローカル関数群
- 前提条件・注意事項:
  - `emit_vba_diagnostic()` を呼んでいるが、現ファイルには定義が見当たらず実行時エラー要因になり得る

### `opts = parse_options(options)`
- シグネチャ: `function opts = parse_options(options)`
- 入力: オプション struct
- 出力: 既定値補完済み `opts`
- 責務: 出力先・ファイル名・フィレット係数設定
- 主要ロジック: `get_field` で既定値を適用
- 依存関係: `get_field`
- 注意: `body_export_dir` 既定は `output_dir/pipe_bodies`

### `val = get_field(s, field, default)`
- シグネチャ: `function val = get_field(s, field, default)`
- 入力: struct, field 名, 既定値
- 出力: 値
- 責務: 安全参照
- 主要ロジック: `isfield`
- 依存関係: なし
- 注意: 単純補助

### `sw = frame_to_sw(frame_coords)`
- シグネチャ: `function sw = frame_to_sw(frame_coords)`
- 入力: フレーム座標 `[Xrear+, Yright+, Zup+]`
- 出力: SolidWorks 座標 `[Xfwd+, Yup+, Zright+]`
- 責務: 座標系変換
- 主要ロジック: `[-X, Z, Y]`
- 依存関係: なし
- 注意: 一貫してこの変換を前提に VBA へ出力

### `components = classify_elements(elements_geom, best_x, catalog)`
- シグネチャ: `function components = classify_elements(elements_geom, best_x, catalog)`
- 入力: 接続情報、最良解、カタログ
- 出力: グループ別要素集合
- 責務: 形状生成単位への分解
- 主要ロジック:
  - Front/Main Hoop の bent path と bottom pipe を固定定義
  - group 4 を FBH/支持/後部/ブラケット/その他へ細分類
- 依存関係: `detect_fbh_nodes`
- 注意: `best_x`, `catalog` はこの関数内では実質未使用

### `fbh_nodes = detect_fbh_nodes(elements_geom, conn, groups)`
- シグネチャ: `function fbh_nodes = detect_fbh_nodes(elements_geom, conn, groups)`
- 入力: 接続・グループ
- 出力: FBH ノード番号列
- 責務: 動的 FBH ノード範囲推定
- 主要ロジック: group 4 内の 24〜27 を優先使用
- 依存関係: なし
- 注意: ノード体系変更に弱い

### `radius_m = compute_fillet_radius(OD_mm, factor, sw_nodes, path)`
- シグネチャ: `function radius_m = compute_fillet_radius(OD_mm, factor, sw_nodes, path)`
- 入力: 外径、係数、座標、経路
- 出力: フィレット半径
- 責務: 曲げ半径計算
- 主要ロジック: `factor*OD` を基準に最短セグメントの 45% でクリップ
- 依存関係: `norm`
- 注意: 小さなセグメントでは半径が自動縮小

### `profiles = build_profile_info(best_x, catalog)`
- シグネチャ: `function profiles = build_profile_info(best_x, catalog)`
- 入力: 最適化変数、カタログ
- 出力: 5 グループ分プロファイル情報
- 責務: VBA 定数生成用プロファイル整形
- 主要ロジック: `OD`, `t`, `material_id`, `OD_m`, `ID_m` を格納
- 依存関係: なし
- 注意: group 名は固定 5 種

### `code = emit_vba_header(opts, n_nodes, profiles)`
- シグネチャ: `function code = emit_vba_header(opts, n_nodes, profiles)`
- 入力: オプション、ノード数、プロファイル情報
- 出力: VBA 文字列
- 責務: 定数・モジュール変数宣言生成
- 主要ロジック: `PIPE_OD_i`, `PIPE_T_i`, SolidWorks API 定数を埋め込む
- 依存関係: なし
- 注意: `.swb` 前提のため `Attribute VB_Name` は出さない

### `code = emit_vba_node_data(sw_nodes)`
- シグネチャ: `function code = emit_vba_node_data(sw_nodes)`
- 入力: SW 座標ノード
- 出力: `InitNodes` VBA
- 責務: ノード配列初期化コード生成
- 主要ロジック: `nd(i,1..3)` へ全座標を書き込む
- 依存関係: なし
- 注意: 全ノードを 1 行ずつ直書き

### `code = emit_vba_helpers()`
- シグネチャ: `function code = emit_vba_helpers()`
- 入力: なし
- 出力: ヘルパー VBA 群
- 責務: 共通 VBA 関数生成
- 主要ロジック:
  - `DrawLine3D`
  - `CreateDirRecursive`
  - `FindPartTemplate`
  - `SaveTubeProfile`
- 依存関係: なし
- 注意: 日本語/英語 Front Plane 名へ両対応

### `code = emit_vba_profile_gen(profiles)`
- シグネチャ: `function code = emit_vba_profile_gen(profiles)`
- 入力: プロファイル情報
- 出力: `CreateCustomProfiles`
- 責務: `.sldlfp` 管プロファイル群生成
- 主要ロジック:
  - OD/t の重複組をまとめる
  - `C:\FSAE_WeldProfiles\FSAE\tube` 以下へ保存
  - SolidWorks ユーザ設定へパス登録
- 依存関係: `SaveTubeProfile` を使う VBA を生成
- 注意: Windows 固定パスを埋め込む

### `code = emit_vba_hoop_bent(name, path, fillet_nodes, fillet_radius, profile, grp_idx)`
- シグネチャ: `function code = emit_vba_hoop_bent(name, path, fillet_nodes, fillet_radius, profile, grp_idx)`
- 入力: 名称、経路、フィレット点、半径、プロファイル、グループ番号
- 出力: bent hoop 生成 VBA
- 責務: 3D スケッチ + フィレット + 単一グループ weldment 生成
- 主要ロジック:
  - 経路線分を描く
  - 各コーナーへ `CreateFillet`
  - `InsertStructuralWeldment5(profilePath, 1, True, ...)`
- 依存関係: `DrawLine3D`, profile file
- 注意: プロファイルファイル存在確認を先に行う

### `code = emit_vba_straight_members(name, elements, profile)`
- シグネチャ: `function code = emit_vba_straight_members(name, elements, profile)`
- 入力: グループ名、要素群、プロファイル
- 出力: 直線部材生成 VBA
- 責務: 直線グループの構築
- 主要ロジック:
  - 全線分を 3D スケッチへ描く
  - 各セグメントを独立 `StructuralMemberGroup` として weldment 化
- 依存関係: `DrawLine3D`
- 注意: 空グループはスキップコードのみ出力

### `code = emit_vba_trim()`
- シグネチャ: `function code = emit_vba_trim()`
- 入力: なし
- 出力: `TrimMembers`
- 責務: 後処理トリムのひな形生成
- 主要ロジック:
  - 全 solid body 数を取得し診断表示
  - 実際の複雑な自動トリムは未実装で、ログ出力中心
- 依存関係: なし
- 注意: コメントどおり手動調整前提が残る

### `code = emit_vba_cutlist_props(components, profiles)`
- シグネチャ: `function code = emit_vba_cutlist_props(components, profiles)`
- 入力: 分類済み要素、プロファイル
- 出力: `SetCutListProps`
- 責務: cut list カテゴリ付与
- 主要ロジック:
  - 先頭側の主要部材数 `n_primary` を算定
  - `Category` に主要鋼材/補助鋼材を設定
- 依存関係: なし
- 注意: ボディ順序が生成順に依存

### `code = emit_vba_body_export(opts)`
- シグネチャ: `function code = emit_vba_body_export(opts)`
- 入力: オプション
- 出力: `ExportBodies`
- 責務: 各ボディの個別書き出し
- 主要ロジック:
  - body 名をファイル名向けにサニタイズ
  - `CopyDocument` を優先し、失敗時 `SaveAs3`
- 依存関係: `CreateDirRecursive`
- 注意: フルモデル保存 fallback があり単一ボディ保証は弱い

### `code = emit_vba_main(opts)`
- シグネチャ: `function code = emit_vba_main(opts)`
- 入力: オプション
- 出力: `Main`
- 責務: 実行順序定義
- 主要ロジック:
  - テンプレート探索 → 新規 Part → ノード初期化 → プロファイル生成
  - Hoops/FBH → Primary → Secondary → Trim → CutList → Export
- 依存関係: 生成済み全 `Build_*` / `TrimMembers` / `SetCutListProps` / `ExportBodies`
- 注意: `DiagnosticWeldmentTest` 呼び出しも含むが、その定義は現ファイルで確認できない

### `output_path = write_bas_file(vba_sections, opts)`
- シグネチャ: `function output_path = write_bas_file(vba_sections, opts)`
- 入力: VBA セクション配列, オプション
- 出力: 出力パス
- 責務: `.swb` 書き出し
- 主要ロジック: UTF-8 で順次書き込む
- 依存関係: `mkdir`, `fopen`
- 注意: 既存ファイルは上書き

### `vba_code = convert_to_late_binding(vba_code)`
- シグネチャ: `function vba_code = convert_to_late_binding(vba_code)`
- 入力: VBA 文字列
- 出力: 置換後文字列
- 責務: early binding 依存除去
- 主要ロジック: `SldWorks.*` 型名を `Object` へ一括置換
- 依存関係: `strrep`
- 注意: 列挙体プレフィクス除去は説明にあるが実装は型置換中心

### `validate_vba(vba_sections)`
- シグネチャ: `function validate_vba(vba_sections)`
- 入力: VBA セクション配列
- 出力: なし
- 責務: 生成コードの静的検査
- 主要ロジック:
  - early-binding 型残存
  - enum プレフィクス残存
  - `Sub/End Sub`, `Function/End Function` の対応
  - loop 内 `Dim`
  - `Attribute VB_Name` 混入
- 依存関係: `strip_vba_comment`
- 注意: 失敗時は詳細行番号つきで `CompileCheck`

### `code_part = strip_vba_comment(line)`
- シグネチャ: `function code_part = strip_vba_comment(line)`
- 入力: VBA 1 行
- 出力: コメント除去後コード
- 責務: 静的検査の前処理
- 主要ロジック:
  - 文字列リテラル内 `'` は保持
  - 文字列外 `'` 以降を切り捨て
- 依存関係: なし
- 注意: `"` の入れ子切替のみで単純実装
