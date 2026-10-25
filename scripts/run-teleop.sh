#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
lib_dir="${MUJOCO_312_LIB_DIR:-$repo_root/target/mujoco-3.12.0/lib}"

if [[ ! -f "$lib_dir/libmujoco.so" ]]; then
  if [[ -n "${MUJOCO_312_LIB_DIR:-}" ]]; then
    echo "MuJoCo 3.12 library not found in MUJOCO_312_LIB_DIR: $lib_dir" >&2
    exit 1
  fi
  "$repo_root/scripts/setup-mujoco.sh"
fi

export MUJOCO_DYNAMIC_LINK_DIR="$lib_dir"
export LD_LIBRARY_PATH="$lib_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$repo_root"
cargo build --release --bin simulator --bin keyboard --bin keyboard_tty --bin teleop
exec dora run dataflow.teleop.yml "$@"
