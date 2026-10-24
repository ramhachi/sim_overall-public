# FSAE Vehicle Dynamics Simulator - Development Environment
# Ubuntu 22.04 + C++20 + 全依存ライブラリ込み

FROM ubuntu:22.04@sha256:ce4a593b4e323dcc3dd728e397e0a866a1bf516a1b7c31d6aa06991baec4f2e0

ARG UBUNTU_SNAPSHOT=20261021T000000Z

LABEL maintainer="FSAE Team"
LABEL description="FSAE Vehicle Dynamics Simulator Development Environment"

# タイムゾーン設定（インタラクティブ入力を回避）
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Tokyo

# APT sources are pinned to an Ubuntu snapshot so OS package versions are reproducible.
RUN rm -f /etc/apt/sources.list.d/*.list \
    && printf '%s\n' \
      "deb https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy main restricted universe multiverse" \
      "deb https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-updates main restricted universe multiverse" \
      "deb https://snapshot.ubuntu.com/ubuntu/${UBUNTU_SNAPSHOT} jammy-security main restricted universe multiverse" \
      > /etc/apt/sources.list

# 基本パッケージ
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    wget \
    pkg-config \
    gdb \
    valgrind \
    clang-format \
    clang-tidy \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# C++開発用ライブラリ
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    nlohmann-json3-dev \
    libspdlog-dev \
    libgtest-dev \
    libpython3-dev \
    pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

# OSQP v1.0.0（ソースからビルド、Ubuntu 22.04にはlibosqp-devがない）
RUN git clone --depth 1 --branch v1.0.0 --recursive https://github.com/osqp/osqp.git /tmp/osqp \
    && mkdir -p /tmp/osqp/build && cd /tmp/osqp/build \
    && cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . -j$(nproc) \
    && cmake --install . \
    && rm -rf /tmp/osqp

# EnTT (ヘッダオンリー)
RUN git clone --depth 1 --branch v3.13.2 https://github.com/skypjack/entt.git /opt/entt \
    && mkdir -p /opt/entt/build && cd /opt/entt/build \
    && cmake .. -DENTT_BUILD_TESTING=OFF \
    && cmake --install .

# autodiff (ヘッダオンリー自動微分ライブラリ)
RUN git clone --depth 1 --branch v1.1.2 https://github.com/autodiff/autodiff.git /opt/autodiff \
    && mkdir -p /opt/autodiff/build && cd /opt/autodiff/build \
    && cmake .. -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF \
    && cmake --install .

# Python環境（hash固定。更新は python/requirements-dev.lock を再生成）
COPY python/requirements-dev.lock /tmp/requirements-dev.lock
RUN pip3 install --no-cache-dir --require-hashes -r /tmp/requirements-dev.lock

# 作業ディレクトリ
WORKDIR /workspace

# ビルド用スクリプト
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# CMakeが見つけられるようにパスを設定
ENV CMAKE_PREFIX_PATH="/opt/entt:/opt/autodiff"

# デフォルトコマンド
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["/bin/bash"]
