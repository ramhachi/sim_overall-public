# tests/verify_exporter.py

## 役割
既に生成済みの YAML / C++ ヘッダを読み直し、Checkpoint 9 の出力物が期待構造を満たしているかを確認する検証スクリプトです。

## 入力と出力
- 入力: `output/tire_params/hoosier_18x6_r20.yaml`, `output/tire_params/hoosier_18x6_r20.hpp`
- 出力: 必須項目・構文要素の合否ログ
- 一時生成物: C++ 構文確認用の一時 `.cpp` と `/tmp/test.o`（スクリプト内で削除）

## 主な処理
- `verify_yaml_export()` は YAML の parse 成功、基本メタデータ、`longitudinal/lateral/camber/fitting_info` の必須キーを確認します。
- `verify_cpp_header_export()` は include guard、`fsae::tire` namespace、`struct`、`static constexpr double`、単位コメント、主要パラメータの存在を確認します。
- 同関数は生成ヘッダを include する小さな C++ ソースを作って `g++ -c` し、ヘッダの構文妥当性も見ます。

## 存在理由
- エクスポータ本体の単体テストだけでは拾いにくい「実ファイルとして後段で読めるか」を確認するための現物検証です。
