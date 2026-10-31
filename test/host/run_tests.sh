#!/usr/bin/env bash
# Host tests for the pure protocol unit. Run from anywhere; builds into a temp dir.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror -I"$root/src" \
  "$root/test/host/test_command_protocol.cpp" "$root/src/core/CommandProtocol.cpp" \
  -o "$out/test_command_protocol"
"$out/test_command_protocol"
