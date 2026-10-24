# `matlab/compute_track_width_mass_distribution.m`

## 1. ファイルの目的

前後トレッド拡大量に応じて、sprung mass penalty と各輪の unsprung mass 増分を一貫したルールで見積もる関数です。

## 2. 主な入出力または依存関係

- 入力:
  - `base_unsprung`
  - `track_f`
  - `track_r`
  - オプション `base_track`, `sprung_penalty_per_meter`, `unsprung_penalty_per_meter`
- 出力:
  - `total_mass_penalty`
  - `m_unsprung`
  - `sprung_mass_penalty`
  - `unsprung_delta`
- 依存関係: `inputParser`

## 3. 処理の流れ / アルゴリズム

1. 前後トレッドと基準トレッドとの差を計算します。
2. 負の差分は 0 に切り上げ、拡幅分だけを評価対象にします。
3. 前後差分を左右輪へ複製して `unsprung_delta` を作ります。
4. 軸単位の差分合計から `sprung_mass_penalty` を計算します。
5. `base_unsprung` に unsprung 増分を足し、最終 `m_unsprung` と総ペナルティを返します。

## 4. ファイル内の関数

### `compute_track_width_mass_distribution(base_unsprung, track_f, track_r, varargin)`

- 本ファイル唯一の関数です。
- 「トレッドを広げると軽くなる」ことは許さず、基準より狭い場合もペナルティ 0 のままです。
- 前輪 2 輪、後輪 2 輪へ同じ unsprung 増分を割り当てます。

## 5. 注意点・前提条件

- `base_unsprung` は 4 要素配列前提です。
- モデルは線形近似で、拡幅に伴う幾何や剛性の二次影響は含みません。
