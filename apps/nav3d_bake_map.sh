#!/usr/bin/env bash
# Bake a full map's 4-yd air volume and 0.5-yd offline water tiles with resume.
#
# Usage: nav3d_bake_map.sh <generator> <data-dir> <output-dir> <map-id> [workers]
#
# The generator is the built src/tools/nav3d_generator binary. Each mode splits
# the 64x64 tile grid into <workers> chunks, runs one process per chunk, and
# appends a per-tile manifest. Existing output tiles are skipped, so rerunning
# the command resumes.

set -euo pipefail

generator=${1:?generator binary}
data_dir=${2:?data dir}
output=${3:?output dir}
map=${4:?map id}
workers=${5:-6}

work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

mkdir -p "$output"
awk 'BEGIN { for (x = 0; x < 64; ++x) for (y = 0; y < 64; ++y) print x, y }' > "$work/tiles.txt"

for mode in air water; do
  if [ "$mode" = air ]; then
    cell_size=4.0
    mode_flag=--air-volume
  else
    cell_size=0.5
    mode_flag=--terrain-only
  fi

  for i in $(seq 0 $((workers - 1))); do
    awk -v i="$i" -v n="$workers" 'NR % n == i' "$work/tiles.txt" \
      > "$work/chunk-$mode-$i.txt"
    "$generator" \
      --data-dir "$data_dir" \
      --output "$output" \
      --map "$map" \
      --cell-size "$cell_size" \
      "$mode_flag" \
      --tiles-file "$work/chunk-$mode-$i.txt" \
      --skip-existing \
      --manifest "$work/manifest-$mode-$i.txt" \
      > "$work/log-$mode-$i.txt" 2>&1 &
  done
  wait

  cat "$work"/manifest-"$mode"-*.txt >> "$output/manifest-$mode.txt"
  echo "$mode: $(find "$output" -name "*_$mode.n3t" | wc -l) tiles"
done
