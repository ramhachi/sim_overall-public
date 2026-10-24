# test_launch.m

## 概要
`test_launch.m` は、リポジトリのルートを `MATLAB` パスへ追加したうえで `EasyStart` をそのまま実行する最小構成の起動確認スクリプトです。

## 種別
スクリプトです。関数定義やローカルヘルパーはありません。

## 実行内容
コードは 2 行だけです。

```matlab
addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..'));
EasyStart
```

### 1. パス追加
`fileparts(mfilename('fullpath'))` でこのスクリプト自身のディレクトリを求め、そこから 2 階層上を `addpath` します。コメントでも「`EasyStart` を含むリポジトリルートを念のためパスへ追加する」と示しています。

### 2. `EasyStart` 実行
パス追加後、`EasyStart` を引数なしで呼び出します。このファイル側では戻り値の受け取り、例外処理、結果検査は行いません。`EasyStart` の実行に成功するかどうかがそのままテスト結果になります。

## 外部依存
- `EasyStart`
- `addpath`
- `fullfile`
- `fileparts`
- `mfilename`

## 想定している確認内容
このスクリプト自身は判定文や表示文を持たないため、コード上で確認しているのは「`EasyStart` が呼び出せる場所にあり、実際に実行へ進めること」です。失敗時の挙動は `EasyStart` 側または `MATLAB` の標準エラー処理に委ねています。
