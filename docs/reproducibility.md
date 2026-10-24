# 再現性と依存固定 (2026-10-21)

## 固定点

| 対象 | 固定方法 |
|:---|:---|
| C++ 依存 | `vcpkg.json` の `builtin-baseline`（`67972e92...`） |
| Python (開発・評価) | `python/requirements-dev.lock`（uv、全依存に SHA-256） |
| Python (PINN 単体) | `tire_pinn/requirements.lock`（uv、全依存に SHA-256） |
| Docker base | `ubuntu:22.04@sha256:ce4a593b...`（multi-arch index） |
| Ubuntu packages | `UBUNTU_SNAPSHOT=20261021T000000Z` の snapshot.ubuntu.com sources |

## 更新手順

```bash
# Python 開発・評価 lock
uv pip compile python/pyproject.toml --extra dev --extra rl \
  --python-version 3.10 --generate-hashes --no-header \
  -o python/requirements-dev.lock

# PINN 単体 lock
uv pip compile tire_pinn/requirements.txt --python-version 3.10 \
  --generate-hashes --no-header -o tire_pinn/requirements.lock

# C++ vcpkg baseline
git ls-remote https://github.com/microsoft/vcpkg.git HEAD
```

`Dockerfile` は `python/requirements-dev.lock` を `--require-hashes` でインストールします。

## 実行環境の記録

`suspension_kinematics_design` の成果物を保存するとき、`save_suspension_geometry_artifact` が
`artifact.environment` に MATLAB バージョン、OS、CPU、コア数、BLAS/LAPACK、C++ コンパイラ、
Eigen のバージョンを記録します。`artifact.optimization.stages` 側には seed、solver options、
targets/sources/weights、コード commit が保存されます。

## 未固定・既知の限界

- Ubuntu の `apt-get install` は `Dockerfile` の `UBUNTU_SNAPSHOT` で snapshot repository に固定している。更新時は snapshot 日時と base digest を同時に更新する。
- NVIDIA/CUDA ドライバ、CPU、OS カーネルはホスト依存。`vcpkg` の triplet もビルド環境で変わる。
- 数値結果の再現には、commit だけでなく実行環境の CPU・control period・tire manifest も記録する。
- `data/tiredata/` は再生成不能な測定データ。公開・配布条件は `docs/repository_hygiene.md` を参照。

## 検証

```bash
python3 -m pytest tests/ -q
ctest --test-dir build --output-on-failure
```

`.github/workflows/secret-scan.yml` がPRと主要ブランチで追跡ファイルをgitleaksにより走査します。
過去履歴のpurgeと漏えい資格情報の失効・ローテーションはリポジトリ外の運用作業です。
