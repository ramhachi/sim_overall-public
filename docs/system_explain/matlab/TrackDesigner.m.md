# `matlab/TrackDesigner.m`

## 1. ファイルの目的

MATLAB 上で FSAE 用トラック中心線を対話的に作成・編集し、CSV に保存する GUI ツールです。  
スプライン区間、直線区間、コーナーフィレット、スラローム生成を 1 つの画面で扱います。

## 2. 主な入出力または依存関係

- 入力:
  - GUI 上のマウス操作・キー入力
  - CSV 読み込み時のトラック点列
- 出力:
  - `state.waypoints`
  - `state.segmentTypes`
  - `state.trackWidth`
  - 保存時の CSV（各行は `x, y, trackWidth`）
- 依存関係:
  - MATLAB GUI API (`figure`, `uicontrol`, `uibuttongroup`, `uigetfile`, `uiputfile` など)
  - `readmatrix`, `writematrix`

## 3. 処理の流れ / アルゴリズム

1. `TrackDesigner()` が内部状態 `state` を初期化し、`createUI()` で画面を構築します。
2. ユーザー操作に応じてウェイポイント追加・移動・削除を行います。
3. 各区間は `segmentTypes` で spline / straight を切り替えます。
4. `onFillet()` は選択点の前後 3 点から接線長と円弧中心を求め、角を円弧点列へ置換します。
5. `onSlalom()` は末端方向ベクトルに対し法線方向へ交互に振れた点列を生成して追記します。
6. `generateTrackPoints()` が最終中心線を生成し、`updatePlot()` が中心線と左右境界を再描画します。
7. `onSave()` が生成済み中心線を CSV に書き出します。

## 4. ファイル内の全関数

### `TrackDesigner()`

- エントリポイントです。
- `waypoints`, `segmentTypes`, `mode`, `trackWidth`, `isClosed`, `selectedIdx` など GUI 全体の状態を保持します。
- ネスト関数群がこの `state` を共有して編集します。

### `createUI()`

- メインウィンドウ、制御パネル、描画軸、各種ボタンや入力欄を作成します。
- モード切替、区間種別、フィレット半径、スラローム設定、トラック幅、閉ループ設定、CSV 入出力をここで結線します。
- 初回描画として `updateUIState()` と `updatePlot()` を呼びます。

### `onMouseDown(~, ~)`

- 現在のマウス座標を取得し、編集モードに応じて挙動を分岐します。
- `add` では点を末尾追加、`move` では近傍点を選択、`delete` では選択点を削除します。

### `onMouseMove(~, ~)`

- `move` モードかつ選択点がある時だけ、その点を現在カーソル位置へ追従させます。
- ドラッグ中の再描画を担います。

### `onMouseUp(~, ~)`

- 現在は処理を持たず、ドラッグ終了のフックだけ用意しています。

### `onKeyPress(~, event)`

- Delete / Backspace で選択点を削除します。
- GUI 上の削除操作をキーボードからも可能にします。

### `onSlalom(~, ~)`

- 最終ウェイポイントから見た進行方向を推定し、その法線方向へ左右交互に振れる点を追加します。
- `Cones`, `Gap`, `Amp` の UI 値を使い、最後に中心へ戻る退出点も 1 点追加します。

### `onFillet(~, ~)`

- 選択した頂点の前後点を調べ、接線長 `d_tangent = R / tan(halfAngle)` を用いて円弧接続へ置換します。
- 半径過大、端点、直線近傍などの不正ケースは警告して中断します。
- 生成後は入口・出口区間を直線扱いに調整します。

### `generateTrackPoints(waypoints, types, closed)`

- 実際に保存・表示する細分化点列を生成する中核関数です。
- 直線区間は線形補間、スプライン区間は Catmull-Rom 風の Hermite 補間で点列化します。
- 閉ループ時は前後点を循環参照し、開ループ時は端点外挿で接線を作ります。

### `updatePlot()`

- 軸をクリアして中心線、左右境界、制御点、開始点ラベルを描き直します。
- 境界線は中心線の数値微分から法線ベクトルを作って `trackWidth/2` だけオフセットしています。

### `updateUIState()`

- 現在選択中の点に対応する `segmentTypes` を UI ラジオボタンへ反映します。

### `onModeChange(~, event)`

- 追加・移動・削除モードを `state.mode` に反映します。

### `onSegTypeChange(~, ~, typeVal)`

- 選択点から次点への区間種別を spline (`0`) または straight (`1`) に更新します。

### `onWidthChange(src, ~)`

- トラック幅入力欄の文字列を数値化し、正値なら `state.trackWidth` を更新します。

### `onClosedChange(src, ~)`

- 閉ループ設定を `state.isClosed` に反映して再描画します。

### `onClear(~, ~)`

- すべての点列と選択状態をリセットします。

### `onLoad(~, ~)`

- CSV を読み込み、点数が多い時は dense polygon を straight として扱うか問い合わせます。
- 3 列目があればトラック幅も取り込みます。

### `onSave(~, ~)`

- `generateTrackPoints()` で確定中心線を作り、幅列を付加して CSV 保存します。

### `findNearestPoint(x, y)`

- 全ウェイポイントとの二乗距離を比較し、閾値未満なら最寄点 index、遠ければ 0 を返します。

## 5. 注意点・前提条件

- GUI ツールなので MATLAB デスクトップ環境前提です。
- `segmentTypes(i)` は「点 `i` から次の点への区間種別」を表します。
- `findNearestPoint()` の閾値は距離二乗 100 で固定なので、座標スケールが極端に変わると操作感も変わります。
- `onFillet()` は選択点の前後 1 点ずつが必要で、開ループの先頭・末尾には適用できません。
