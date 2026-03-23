#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

git submodule update --init --recursive

export PATH="$HOME/.local/bin:$PATH"

if ! command -v conan >/dev/null 2>&1; then
  echo "conan not found in PATH. Run ./install_jetson_orin.sh first."
  exit 1
fi

conan profile detect --force
conan install . -s build_type=Release -b missing -of build

cmake -S . -B build   -G "Unix Makefiles"   -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/build/conan_toolchain.cmake"   -DCMAKE_POLICY_DEFAULT_CMP0091=NEW   -DCMAKE_BUILD_TYPE=Release   -DBUILD_EXAMPLES=ON

cmake --build build --target example_teleoperation_with_joint_mapping_kgs -j"$(nproc)"

echo "Built: $SCRIPT_DIR/build/examples/cpp/example_teleoperation_with_joint_mapping_kgs"
