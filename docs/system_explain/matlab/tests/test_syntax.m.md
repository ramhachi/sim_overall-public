# test_syntax.m

## 概要
`test_syntax.m` は、`EasyStart` を `pcode` に通すことで構文上の問題が出るかを確認し、結果メッセージを表示するスクリプトです。

## 種別
スクリプトです。ローカル関数はありません。

## エントリとなる処理
全体が `try` / `catch` で囲まれています。

```matlab
try
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..'));
    pcode EasyStart
    disp('Syntax OK');
catch e
    disp('Syntax Error:');
    disp(e.message);
end
```

## 実行フロー
1. `addpath(...)` で、このスクリプト位置から 2 階層上のリポジトリルートをパスへ追加します。
2. `pcode EasyStart` を実行します。コード上ではこの処理を構文確認の中心に置いています。
3. 例外が発生しなければ `Syntax OK` を表示します。
4. 例外が発生した場合は `Syntax Error:` を表示し、続けて `e.message` を表示します。

## 出力
- 正常時: `Syntax OK`
- 失敗時: `Syntax Error:` と例外メッセージ

## 外部依存
- `EasyStart`
- `pcode`
- `addpath`
- `fullfile`
- `fileparts`
- `mfilename`
- `disp`

## 注意点
- `catch` 節ではメッセージ表示のみを行い、例外の再送出はしません。
- このスクリプトは構文確認結果を文字列で通知しますが、戻り値や明示的な終了コードは定義していません。
