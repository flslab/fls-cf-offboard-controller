#!/bin/bash
# Offline only. Argument: isolated candidate directory named firmware.
set -euo pipefail
firmware_path="${1:?Supply the isolated firmware source path}"
tests_path="$(cd "$(dirname "$0")/tests" && pwd)"
output_path="$(dirname "$firmware_path")/verification"
mkdir -p "$output_path"
for test_name in test_response test_parameter_module test_runtime; do
  clang -std=c11 -O1 -g -fsanitize=address,undefined -Wall -Wextra -Werror \
    -I"$tests_path/stubs" -I"$(dirname "$firmware_path")" \
    -I"$firmware_path/src/modules/interface/kalman_core" \
    "$tests_path/$test_name.c" -o "$output_path/$test_name"
  "$output_path/$test_name" | tee "$output_path/$test_name.log"
done
