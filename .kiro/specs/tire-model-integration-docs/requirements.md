# 要件定義書: タイヤモデル統合と文書化

## はじめに

本プロジェクトは、既に実装・学習済みのMATLABニューラルネットワークタイヤモデルを14DOFシミュレータに統合するための文書を作成することを目的とする。約140,000サンプルのTTCタイヤテストデータで学習済みのニューラルネットワークモデルと、そのエクスポート機能は既に完成している。本文書化プロジェクトでは、この既存実装の使用方法、C++シミュレータへの統合手順、検証方法を明確に記述し、他の開発者が容易に理解・利用できるようにする。また、既存のプロジェクトドキュメントとの整合性を保ちながら、統合的なドキュメント構造を提案する。

## 用語集

- **System**: タイヤモデル統合文書化システム
- **TMeasy**: 物理ベースのタイヤモデル（C++実装、14DOFシミュレータで使用）
- **NN_Model**: MATLABニューラルネットワークタイヤモデル
- **TTC_Data**: TTCタイヤテストデータ（約140,000サンプル）
- **Documentation_System**: プロジェクトドキュメント管理システム
- **Integration_Workflow**: データ収集から学習、検証、C++統合までの一連のプロセス
- **SR**: スリップ率（Slip Ratio）
- **SA**: スリップ角（Slip Angle）
- **Fz**: 垂直荷重（Vertical Load）
- **IA**: キャンバー角（Inclination Angle）
- **TSTC**: タイヤ表面温度（Tire Surface Temperature）
- **Fx**: 前後力（Longitudinal Force）
- **Fy**: 横力（Lateral Force）
- **Mx**: 転覆モーメント（Overturning Moment）
- **Mz**: セルフアライニングトルク（Self-Aligning Torque）

## 要件

### 要件1: ニューラルネットワークタイヤモデル統合の目的と利点の文書化

**ユーザーストーリー:** エンジニアとして、なぜMATLABニューラルネットワークタイヤモデルをシミュレーションに組み込むのか、その利点と目的を理解したい。

#### 受入基準

1. THE Documentation_System SHALL NN_Modelを14DOFシミュレータに統合する主要な目的を記述すること
2. THE Documentation_System SHALL NN_ModelとTMeasyモデルの精度比較結果を含むこと
3. THE Documentation_System SHALL NN_Model統合による予測精度向上の具体的なメリットを説明すること
4. THE Documentation_System SHALL TTC_Dataに基づく学習の利点（実測データベース）を記述すること
5. WHERE 比較が必要な場合 THE Documentation_System SHALL TMeasyとNN_Modelの使い分けシナリオを示すこと

### 要件2: MATLABニューラルネットワークモデルの既存実装の文書化

**ユーザーストーリー:** 開発者として、既に実装されているMATLABニューラルネットワークタイヤモデルの構造と使用方法を理解したい。

#### 受入基準

1. THE Documentation_System SHALL 既存NN_Modelの入力変数（SR、SA、Fz、IA、TSTC）を文書化すること
2. THE Documentation_System SHALL 既存NN_Modelの出力変数（Fx、Fy、Mx、Mz）を文書化すること
3. THE Documentation_System SHALL 既存NN_Modelのアーキテクチャ（層構成、活性化関数、正規化手法）を記述すること
4. THE Documentation_System SHALL 既存の学習済みモデルファイルの場所と形式を文書化すること
5. THE Documentation_System SHALL 既存のエクスポート機能の使用方法を説明すること
6. THE Documentation_System SHALL TTC_Dataの構造とサンプル数（約140,000）を記述すること

### 要件3: 既存エクスポート機能の使用方法の文書化

**ユーザーストーリー:** 開発者として、既に実装されているエクスポート機能を使用して、学習済みモデルをC++で利用可能な形式に変換する方法を理解したい。

#### 受入基準

1. THE Documentation_System SHALL 既存エクスポート機能の呼び出し方法を文書化すること
2. THE Documentation_System SHALL エクスポートされるファイル形式（ONNX、C++コード、パラメータファイル等）を説明すること
3. THE Documentation_System SHALL エクスポート時に保存される正規化パラメータの形式を記述すること
4. THE Documentation_System SHALL エクスポートプロセスで生成されるファイル一覧を提供すること
5. THE Documentation_System SHALL エクスポート機能を実行するMATLABスクリプトまたは関数を特定すること
6. THE Documentation_System SHALL エクスポート時のオプションやパラメータ設定を説明すること

### 要件4: C++シミュレータへの統合方法の文書化

**ユーザーストーリー:** C++開発者として、学習済みニューラルネットワークモデルを14DOFシミュレータに統合し、TMeasyモデルと置き換える方法を理解したい。

#### 受入基準

1. THE Documentation_System SHALL 学習済みNN_ModelをC++コードにエクスポートする具体的な手順を文書化すること
2. THE Documentation_System SHALL C++統合時に必要なライブラリまたは依存関係（ONNX Runtime、TensorFlow Lite等）を列挙すること
3. THE Documentation_System SHALL 14DOFシミュレータのタイヤモデル呼び出しインターフェースを示すこと
4. THE Documentation_System SHALL 推論時の入力正規化（SR、SA、Fz、IA、TSTC）と出力の逆正規化（Fx、Fy、Mx、Mz）の処理を説明すること
5. THE Documentation_System SHALL TMeasyモデルからNN_Modelへの切り替え方法を記述すること
6. THE Documentation_System SHALL C++コードでのNN_Model推論のサンプルコードを含むこと

### 要件5: 統合後の検証とテスト方法の文書化

**ユーザーストーリー:** テストエンジニアとして、C++シミュレータに統合したニューラルネットワークモデルが正しく動作することを検証したい。

#### 受入基準

1. THE Documentation_System SHALL NN_Model統合後の検証手順を文書化すること
2. THE Documentation_System SHALL MATLABでの推論結果とC++での推論結果の比較方法を説明すること
3. THE Documentation_System SHALL 許容誤差範囲（数値精度の違いによる誤差）を定義すること
4. THE Documentation_System SHALL 14DOFシミュレータでの統合テストシナリオを提供すること
5. WHEN 検証が実行される THEN THE Documentation_System SHALL 入力データセット（テスト用TTC_Data）の準備方法を示すこと
6. THE Documentation_System SHALL 推論性能（計算時間）の測定方法を記述すること

### 要件6: ドキュメント構造の整理と統合

**ユーザーストーリー:** プロジェクトマネージャーとして、NN_Model統合に関するドキュメントを既存プロジェクト文書と整合させ、情報の検索性を向上させたい。

#### 受入基準

1. THE Documentation_System SHALL docs/フォルダ内のNN_Model統合ドキュメントの配置場所を提案すること
2. THE Documentation_System SHALL 既存ドキュメント（学生フォーミュラ車両運動シミュレーション総合ガイド.md、設計書_統合版.md）との関係性を示すこと
3. THE Documentation_System SHALL matlab/dl_tire_model/内の既存READMEとの重複を避け、参照関係を明確化すること
4. THE Documentation_System SHALL 日本語ドキュメントと英語ドキュメントの棲み分けルールを定義すること
5. THE Documentation_System SHALL 対象読者（初心者、開発者、研究者）ごとのドキュメント推奨を提供すること

### 要件7: データ形式と既存前処理の文書化

**ユーザーストーリー:** データエンジニアとして、既存の学習で使用されたTTC_Dataの形式と前処理方法を理解したい。

#### 受入基準

1. THE Documentation_System SHALL 既存TTC_Dataのファイル形式（CSV、MAT等）を文書化すること
2. THE Documentation_System SHALL 入力変数（SR、SA、Fz、IA、TSTC）と出力変数（Fx、Fy、Mx、Mz）の物理的意味と単位を定義すること
3. THE Documentation_System SHALL 既存実装で使用された前処理ステップ（外れ値除去、正規化、分割）を説明すること
4. THE Documentation_System SHALL 既存の正規化パラメータ（平均、標準偏差）の保存場所と形式を記述すること
5. THE Documentation_System SHALL matlab/dl_tire_model/DATA_FORMAT.mdとの整合性を保つこと
6. THE Documentation_System SHALL 約140,000サンプルのデータセット構成を説明すること

### 要件7: モデル検証と性能評価の文書化

**ユーザーストーリー:** 研究者として、ニューラルネットワークモデルの精度と性能を評価し、物理ベースモデルと比較したい。

#### 受入基準

1. THE Documentation_System SHALL NN_Modelの評価指標（RMSE、R²、MAE等）を定義すること
2. THE Documentation_System SHALL 各出力変数（Fx、Fy、Mx、Mz）ごとの精度目標を記述すること
3. THE Documentation_System SHALL TMeasyモデルとNN_Modelの精度比較結果を含むこと
4. THE Documentation_System SHALL 検証データセットでの性能評価手順を説明すること
5. WHEN モデルが検証される THEN THE Documentation_System SHALL 予測値と実測値の比較プロット生成方法を示すこと

### 要件8: 既存MATLABスクリプトとリソースの文書化

**ユーザーストーリー:** MATLAB開発者として、既存のmatlab/dl_tire_model/ディレクトリ内のスクリプト群の役割と使用方法を理解したい。

#### 受入基準

1. THE Documentation_System SHALL matlab/dl_tire_model/内の主要スクリプトの役割を説明すること
2. THE Documentation_System SHALL 学習スクリプト、エクスポートスクリプト、検証スクリプトを特定すること
3. THE Documentation_System SHALL 各スクリプトの入力パラメータと出力を文書化すること
4. THE Documentation_System SHALL matlab/dl_tire_model/README.md、DATA_FORMAT.md、IMPROVED_TRAINING_GUIDE.mdとの参照関係を明確化すること
5. THE Documentation_System SHALL NN_Modelのアーキテクチャ（層構成、活性化関数）を記述すること

## 成功基準

- 既存MATLABニューラルネットワークタイヤモデルの実装が完全に文書化されている
- 既存エクスポート機能の使用方法が明確に記述されている
- エクスポートされたモデルのC++統合手順が具体的に文書化されている
- 統合後の検証とテスト方法が定義されている
- 既存TTC_Dataの形式と前処理方法が文書化されている
- 既存MATLABスクリプトの役割と使用方法が明確である
- 既存プロジェクトドキュメントとの整合性が保たれている
- 14DOFシミュレータでのNN_Model呼び出し方法が実装可能である
- 他の開発者が文書を読んで既存実装を理解・利用できる
