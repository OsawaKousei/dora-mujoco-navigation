#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
archive="$(mktemp)"
trap 'rm -f "$archive"' EXIT

url="https://github.com/google-deepmind/mujoco/releases/download/3.12.0/mujoco-3.12.0-linux-x86_64.tar.gz"
expected_sha256="a9367911e6d5eaeade17c2197304687421c1fc932cdf7bcd4cb8cfaf0374dcb2"

if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo "Automatic MuJoCo setup currently supports Linux x86_64 only." >&2
  exit 1
fi

mkdir -p "$repo_root/target"
curl --fail --location --silent --show-error "$url" -o "$archive"
actual_sha256="$(sha256sum "$archive" | cut -d ' ' -f 1)"
if [[ "$actual_sha256" != "$expected_sha256" ]]; then
  echo "MuJoCo download checksum mismatch." >&2
  exit 1
fi

tar -xzf "$archive" -C "$repo_root/target" \
  mujoco-3.12.0/lib/libmujoco.so \
  mujoco-3.12.0/lib/libmujoco.so.3.12.0
echo "MuJoCo 3.12 library installed in target/mujoco-3.12.0/lib"
