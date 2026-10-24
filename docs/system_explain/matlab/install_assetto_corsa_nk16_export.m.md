# `matlab/install_assetto_corsa_nk16_export.m`

## 1. 役割

この関数は、`export_assetto_corsa_nk16()` が生成したオーバーレイを、  
**展開済みの Assetto Corsa 車両フォルダへ直接コピーするインストーラ** です。

生成だけで終わらせず、そのまま NK16_ICV に反映したい場合の最終段です。

## 2. 想定する入力

- `exportDir`: `output/assetto_corsa/NK16_ICV/` のような export 結果フォルダ
- `targetCarDir`: ユーザーが手元で展開した NK16_ICV フォルダ

関数は、`exportDir/data` と `exportDir/ui` を `targetCarDir` 以下へコピーします。

## 3. 安全確認

今回の実装で最重要なのは、**誤ディレクトリへの上書き防止** です。  
この関数はコピー前に次を検査します。

1. `exportDir` が存在するか
2. `exportDir` に `data/` と `ui/` が揃っているか
3. `targetCarDir` が存在するか
4. `targetCarDir` が AC 車両フォルダらしいか
   - `data/`
   - `data.acd`
   - `ui/`
   - `ui/ui_car.json`
   のいずれかを持つ
5. `data.acd` しかなく `data/` が未展開ならエラーにする

このチェックにより、単なる任意フォルダへ `data/` `ui/` を新規作成してしまう事故を防ぎます。

## 4. コピー方式

`copyOverlayDirectory()` は再帰的にディレクトリを辿り、既存ファイルを `copyfile(..., 'f')` で上書きします。  
対象は `data/` と `ui/` のみで、必要なら `sim_overall_export_metadata.json` もルートへコピーします。

このため、

- AC 車両全体を丸ごと作り直すのではなく
- **physics / ui に相当する部分だけを overlay 更新**

する設計になっています。

## 5. 失敗させる条件

この関数は曖昧な状態で黙って成功しないよう、かなり厳しく失敗します。

- export 元が不完全
- target が AC 車両に見えない
- target が `data.acd` のみで未展開

のいずれでも `error(...)` を投げます。

これは「成功したように見えたのに実際は間違った場所へコピーされた」という最悪ケースを避けるためです。

## 6. システム上の位置付け

`export_assetto_corsa_nk16.m` が生成器なら、この関数は配備器です。  
`EasyStart.m` の [10] メニューでは、生成だけでなくこの関数を介した直接反映まで一気通貫で行えます。
