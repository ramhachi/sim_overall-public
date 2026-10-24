# `dashboard/static/js/main.js`

## 1. 役割

`main.js` は FSAE Dashboard のフロントエンド制御を一手に担うメインスクリプトです。  
サイドバーによるページ切替、FastAPI バックエンドへの `fetch()` 呼び出し、車両パラメータ編集 UI、Plotly グラフ描画、シミュレーション実行結果表示、エクスポート内容のプレビュー/ダウンロードを担当します。

このファイルは「画面ごとの小さなコントローラ群」を 1 本にまとめた構成で、クラスは持たず、トップレベル関数と共有状態 `vehicleParams` の組み合わせで動作します。

## 2. 依存関係と前提

- ブラウザ標準 API
  - `document.querySelector*`
  - `fetch`
  - `Blob`
  - `URL.createObjectURL`
- グローバル変数
  - `Plotly`: グラフ描画ライブラリ
  - `T`: サーバーから HTML に注入される翻訳辞書
- バックエンド API
  - `GET /api/engine`
  - `GET /api/params`
  - `POST /api/params`
  - `POST /api/params/reset`
  - `POST /api/simulate`
  - `GET /api/regulation`
  - `GET /api/tire?Fz=...`
  - `GET /api/export?...`

## 3. DOM 契約

このファイルは以下のような要素 ID / セレクタが HTML 側に存在することを前提にします。

- ナビゲーション
  - `#sidebar a[data-page]`
  - `.page`
- エンジン画面
  - `#engineInfo`
  - `#engineChart`
- 車両パラメータ画面
  - `#vehicleForm`
  - `#vehicleMsg`
- シミュレーション画面
  - `#runSimBtn`
  - `#simResults`
  - `#simScenario`
- レギュレーション画面
  - `#regOverall`
  - `#regTable tbody`
- タイヤ画面
  - `#tireFz`
  - `#fxChart`
  - `#fyChart`
- エクスポート画面
  - `#exportFmt`
  - `#exportEngine`
  - `#exportTire`
  - `#exportPreview`

HTML 側でこれらが欠けると、対象画面の描画やイベント処理は成立しません。

## 4. トップレベル状態

### `PLOT_LAYOUT`

Plotly 用の共通ダークテーマ設定です。

- 背景色
- フォント色
- マージン
- 軸グリッド色

各グラフでスプレッド構文によりベース設定として再利用されます。

### `PLOT_CONFIG`

Plotly の表示挙動設定です。

- `responsive: true`
- `displayModeBar: true`
- `displaylogo: false`

### `vehicleParams`

`/api/params` から受け取った現在の車両パラメータを保持する共有オブジェクトです。  
フォーム描画時と保存時の両方で参照されます。

### `SECTION_LABELS`

`T.vehicle.sections` を元にしたセクション名辞書です。  
車両パラメータ画面で、内部キーを画面表示用ラベルへ変換するために使います。

## 5. 初期化と全体制御フロー

1. ファイル読込時に Plotly 設定と共有状態を定義します。
2. `#sidebar a[data-page]` 全件へクリックイベントを登録します。
3. ユーザーがサイドバーを押すと `goPage()` が呼ばれます。
4. `goPage()` は表示中ページを切り替え、対象ページに応じて必要な API 読み込みを遅延実行します。
5. 各ページ関数は API 応答を DOM または Plotly へ反映します。
6. 車両パラメータ画面だけは `vehicleParams` を編集用状態として保持し、保存時にまとめて POST します。

この設計により、初期ロード時に全 API を一括取得せず、「ページを開いた時だけ読む」lazy-load 型になっています。

## 6. 関数ごとの詳細

### `goPage(name)`

#### 役割

サイドバーとメイン表示領域のアクティブページを切り替えます。

#### 処理内容

1. `.page` から `active` を外します。
2. サイドバーリンクから `active` を外します。
3. `page-${name}` に一致する要素へ `active` を付けます。
4. `data-page="${name}"` のリンクへ `active` を付けます。
5. ページ名に応じて以下を遅延呼び出しします。
   - `engine` → `loadEngineData()`
   - `vehicle` → `loadVehicleParams()`
   - `tire` → `loadTireData()`

#### 注意点

- 画面切替は DOM クラス操作のみで、ルーティングは行いません。
- 対象要素が見つからない場合は `if (page)` / `if (link)` で安全に無視します。

### サイドバー click ハンドラ

```js
document.querySelectorAll('#sidebar a[data-page]').forEach(...)
```

#### 役割

ナビゲーションの実際の入口です。  
リンクのデフォルト遷移を止めて `goPage(a.dataset.page)` を呼びます。

#### 意味

- HTML 側リンクを SPA 風のページ切替へ変換します。
- URL 遷移よりも軽量で、同じ HTML 内のセクション表示切替に特化しています。

### `switchLang(lang)`

#### 役割

言語切替をクエリパラメータ方式で行います。

#### API / 挙動

- 入力: `lang`
- 動作: `window.location.href = '/?lang=' + lang`
- 出力: なし
- 副作用: ページ全体を再読み込みします。

### `loadEngineData()`

#### 役割

エンジン画面の情報カードとトルク/出力グラフを構築します。

#### 入力 API

- `GET /api/engine`

#### 処理内容

1. API 応答から `engine` と `curves` を取り出します。
2. `#engineInfo` に 6 枚の情報カード HTML を流し込みます。
   - 排気量
   - 構成
   - 最大出力
   - 最大トルク
   - アイドル回転数
   - レッドライン
3. トルク系列 `traceTorque` と出力系列 `tracePower` を構築します。
4. `layout` に軸タイトル、凡例、レッドライン、最大トルク回転数マーカー、注釈を設定します。
5. `Plotly.newPlot('engineChart', ...)` で二軸グラフを描画します。

#### エラー処理

- 例外時は `console.error('Engine load error:', err)` のみです。
- UI に明示的エラー表示は行いません。

### `loadVehicleParams()`

#### 役割

車両パラメータ編集画面のデータ読込入口です。

#### 入力 API

- `GET /api/params`

#### 処理内容

1. 応答 JSON を `vehicleParams` に保存します。
2. `renderParamForm(vehicleParams)` を呼び、フォーム全体を再描画します。

#### エラー処理

- `console.error('Params load error:', err)` に記録します。

### `renderParamForm(params)`

#### 役割

車両パラメータ辞書を、セクション単位の動的フォームへ変換します。

#### 入力

- `params`: セクション名 → 値辞書のネスト構造

#### 処理内容

1. `#vehicleForm` を空にします。
2. `Object.entries(params)` で各セクションを反復します。
3. セクション名を `SECTION_LABELS` で翻訳し、見出しを作ります。
4. セクション内の各キーを見て入力欄を生成します。
   - 配列値 → `data-array="true"` を付与した `type="text"`
   - それ以外 → `type="number"` の数値入力
5. セクションごとに `div.param-section` を組み立ててコンテナへ追加します。

#### 制約

- 値が「辞書の 1 階層下」にある前提です。
- より深いネスト構造やオブジェクト配列は扱いません。

### `saveParams()`

#### 役割

動的フォームの内容を `vehicleParams` に反映し、サーバーへ保存します。

#### 入力 API

- `POST /api/params`

#### 処理内容

1. `#vehicleForm input` 全件を走査します。
2. `data-section` / `data-key` から格納先を決定します。
3. `data-array="true"` の場合は
   - `,` で分割
   - `trim()`
   - `parseFloat()` で数値配列化
4. スカラー値は `parseFloat(inp.value)` で数値化します。
5. 変換後の `vehicleParams` 全体を JSON として POST します。
6. 成功時 `showMsg('vehicleMsg', T.vehicle.saved_msg, 'success')`
7. 失敗時 `showMsg('vehicleMsg', T.common.error, 'error')`

#### 注意点

- 差分送信ではなく全体送信です。
- 空欄があると `parseFloat('')` により `NaN` が入り得ます。

### `resetParams()`

#### 役割

現在の編集値をサーバー側の既定値に戻し、フォームを再描画します。

#### 入力 API

- `POST /api/params/reset`

#### 処理内容

1. API 応答の `data.params` を `vehicleParams` に代入します。
2. `renderParamForm(vehicleParams)` を再実行します。
3. 成功/失敗メッセージを `showMsg()` で表示します。

### `showMsg(id, text, type)`

#### 役割

短時間だけメッセージを表示する共通 UI ヘルパです。

#### API

- 入力:
  - `id`: 対象 DOM ID
  - `text`: 表示文言
  - `type`: CSS クラス種別（`success` / `error` など）

#### 処理内容

1. 対象要素を取得します。
2. `className = 'msg ' + type` を設定します。
3. `textContent` を更新します。
4. 3 秒後にテキストとクラスを初期状態へ戻します。

### `runSim()`

#### 役割

ダッシュボードからシミュレーション実行 API を叩き、結果を整形表示します。

#### 入力 API

- `POST /api/simulate`

#### 処理内容

1. `#runSimBtn` と `#simResults` を取得します。
2. 実行中はボタンを無効化し、文言を `T.simulation.running` に変更します。
3. `#simResults` に loading 文言を表示します。
4. `#simScenario` の値を読み、`{ scenario }` を POST します。
5. 応答 JSON を解釈します。
   - `data.success === true` → `data.output` を `<pre>` 表示
   - 失敗 → `data.error || data.stderr || 'Unknown error'` を `<pre class="error">` 表示
6. 例外時は `err.message` をエスケープして表示します。
7. `finally` でボタン状態と文言を元に戻します。

#### セキュリティ面

- 表示前に `escapeHtml()` を通すため、標準出力に HTML 断片が含まれてもそのまま実行されません。

### `runRegCheck()`

#### 役割

レギュレーション判定結果を総合表示と表形式で反映します。

#### 入力 API

- `GET /api/regulation`

#### 処理内容

1. 応答の `overall` を見て `#regOverall` のクラスと文言を更新します。
2. `#regTable tbody` を空にします。
3. `data.checks` を 1 件ずつ行 HTML へ変換します。
4. 各行で `pass` / `fail` のバッジクラスを切り替えます。

#### 出力項目

- ルール ID
- カテゴリ
- ルール説明
- 現在値
- 要求値
- 合否

### `loadTireData()`

#### 役割

指定鉛直荷重に対するタイヤ力カーブを 2 つの Plotly グラフとして描きます。

#### 入力 API

- `GET /api/tire?Fz=...`

#### 処理内容

1. `#tireFz` の値を取得し、空なら `1000` を使います。
2. API 応答から以下を取得します。
   - `slip_ratios`
   - `fx_values`
   - `slip_angles_deg`
   - `fy_values`
3. `fxChart` に Fx-Slip Ratio グラフを描画します。
4. `fyChart` に Fy-Slip Angle グラフを描画します。

#### エラー処理

- 失敗時は `console.error('Tire load error:', err)` のみです。

### `previewExport()`

#### 役割

現在のエクスポート設定でプレビュー内容をサーバーから取得します。

#### 入力 API

- `GET /api/export?fmt=...&include_engine=...&include_tire=...`

#### 処理内容

1. `#exportFmt` の値を取得します。
2. `#exportEngine`, `#exportTire` のチェック状態を読みます。
3. クエリ文字列付きで API を呼びます。
4. 応答の `data.content` を `#exportPreview.textContent` に表示します。
5. 失敗時は `Error: ...` を表示します。

### `downloadExport()`

#### 役割

プレビュー済みのテキストをファイルとしてダウンロードさせます。

#### 処理内容

1. `#exportPreview` のテキストを取得します。
2. 中身が空なら `previewExport()` を呼び、処理を終えます。
3. `fmt` に応じて拡張子を決定します。
   - `yaml` → `yaml`
   - `cpp_header` → `hpp`
   - それ以外 → `json`
4. `Blob` を作成し、一時 `<a>` 要素へ Object URL を設定します。
5. `a.click()` で保存を開始します。

#### 注意点

- プレビュー未取得時は「先に取得して終了」であり、その場では即ダウンロードしません。

### `escapeHtml(str)`

#### 役割

任意文字列を HTML として解釈させないためのユーティリティです。

#### 処理内容

1. 一時 `div` を生成します。
2. `textContent = str` を設定します。
3. `innerHTML` を返します。

#### 主用途

- シミュレーション標準出力/エラー表示の安全化

## 7. 画面別 API 対応表

| 画面 | 入口関数 | API | 主な描画先 |
| --- | --- | --- | --- |
| Engine | `loadEngineData()` | `GET /api/engine` | `#engineInfo`, `#engineChart` |
| Vehicle | `loadVehicleParams()` / `saveParams()` / `resetParams()` | `GET/POST /api/params`, `POST /api/params/reset` | `#vehicleForm`, `#vehicleMsg` |
| Simulation | `runSim()` | `POST /api/simulate` | `#simResults` |
| Regulation | `runRegCheck()` | `GET /api/regulation` | `#regOverall`, `#regTable tbody` |
| Tire | `loadTireData()` | `GET /api/tire` | `#fxChart`, `#fyChart` |
| Export | `previewExport()` / `downloadExport()` | `GET /api/export` | `#exportPreview` |

## 8. 設計上の特徴

- **単一ファイル集中型 UI 制御**  
  モジュール分割やクラス設計よりも、少数ページを素早く制御する実装を優先しています。
- **lazy-load**  
  実際に開いた画面だけデータ取得します。
- **状態保持は最小限**  
  フロント側の共有状態は `vehicleParams` が中心で、他はその場で取得・描画します。
- **サーバー主導翻訳**  
  文言は `T` 経由で参照され、JS 側には翻訳ロジックを持ちません。

## 9. 注意点・既知の制約

- `renderParamForm()` は深いネスト構造を汎用的には扱えません。
- `saveParams()` は全項目上書き保存で、部分更新 API ではありません。
- 失敗時 UI はページによって差があり、`console.error` のみで終わる箇所があります。
- `downloadExport()` は Object URL を解放していないため、厳密には都度 `URL.revokeObjectURL()` を呼ぶ余地があります。
- 初期表示時に自動で全ページをロードするわけではないため、各画面内容はそのページを開いた時点で初めて最新化されます。
