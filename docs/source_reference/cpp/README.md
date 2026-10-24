# cpp/ ソースリファレンス

- 対象: `cpp/src/`, `cpp/include/`, `cpp/bindings/`, `cpp/tests/`
- 方針: 実装コードに現れる関数・クラス・依存関係を基準に、日本語で役割を整理
- 主な下位ディレクトリ:
  - `src/`: 実行プログラム、手動検証用テスト、明示的インスタンス化
  - `include/fsae/`: 車両・タイヤ・制御・トラック・ユーティリティの主要ヘッダ
  - `bindings/`: Python 連携
  - `tests/`: Google Test ベースの単体/統合テスト

