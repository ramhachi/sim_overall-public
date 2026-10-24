@echo off
REM FSAE Sim 開発環境セットアップスクリプト (Windows)
REM 使用法: setup_env.bat

echo ========================================
echo  FSAE Vehicle Dynamics Simulator Setup
echo ========================================
echo.

REM 1. vcpkgチェック
if not defined VCPKG_ROOT (
    echo [INFO] VCPKG_ROOT not set
    if exist "%USERPROFILE%\vcpkg" (
        set VCPKG_ROOT=%USERPROFILE%\vcpkg
        echo [INFO] Using %USERPROFILE%\vcpkg
    ) else (
        echo [INFO] Cloning vcpkg...
        git clone https://github.com/microsoft/vcpkg.git "%USERPROFILE%\vcpkg"
        call "%USERPROFILE%\vcpkg\bootstrap-vcpkg.bat" -disableMetrics
        set VCPKG_ROOT=%USERPROFILE%\vcpkg
    )
)

REM 環境変数を永続化
setx VCPKG_ROOT "%VCPKG_ROOT%" >nul 2>&1

REM 2. 依存ライブラリインストール
echo.
echo [INFO] Installing vcpkg dependencies...
cd /d "%~dp0"
"%VCPKG_ROOT%\vcpkg.exe" install --triplet=x64-windows --x-manifest-root="%~dp0"

REM 3. Python環境セットアップ
echo.
echo [INFO] Setting up Python environment...
cd /d "%~dp0"
cd python

python -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)"
if errorlevel 1 (
    echo [ERROR] Python 3.10 or newer is required.
    exit /b 1
)

if not exist ".venv" (
    python -m venv .venv
)

call .venv\Scripts\activate.bat
pip install --upgrade pip
pip install -e ".[dev]"

REM 4. 出力ディレクトリ作成
echo.
echo [INFO] Creating output directories...
cd /d "%~dp0"
if not exist "output" mkdir output
if not exist "data\ttc" mkdir data\ttc
if not exist "data\aero" mkdir data\aero
if not exist "data\vehicle" mkdir data\vehicle

REM 5. CMakeビルド
echo.
echo [INFO] Configuring CMake...
cmake --preset=release

echo.
echo [INFO] Building C++ project...
cmake --build build/release --config Release

echo.
echo [INFO] Building frame optimization FEM binaries...
cmake -S frame_optimize\cpp -B frame_optimize\cpp\build ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
    -DVCPKG_TARGET_TRIPLET=x64-windows ^
    -DVCPKG_MANIFEST_DIR="%~dp0"
cmake --build frame_optimize\cpp\build --config Release
cmake --install frame_optimize\cpp\build --config Release

echo.
echo ========================================
echo  Setup Complete!
echo ========================================
echo.
echo Next steps:
echo   1. Activate Python: python\.venv\Scripts\activate
echo   2. Run MATLAB verification: matlab\quarter_car\run_verification.m
echo   3. Build Debug: cmake --preset=debug ^&^& cmake --build build/debug
echo.
pause
