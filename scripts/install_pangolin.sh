#!/usr/bin/env bash
set -euo pipefail
# Prerequisites are listed in README_CN.md. Install to a user-owned prefix by default.
prefix="${1:-$HOME/.local}"
work="${PANGOLIN_BUILD_ROOT:-$HOME/.cache/orbslam3-pangolin-v0.9.2}"
jobs="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
if [[ ! -d "$work/source/.git" ]]; then
  git clone --depth 1 --branch v0.9.2 https://github.com/stevenlovegrove/Pangolin.git "$work/source"
fi
cmake -S "$work/source" -B "$work/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build "$work/build" --parallel "$jobs"
cmake --install "$work/build"
