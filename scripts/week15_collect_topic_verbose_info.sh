#!/usr/bin/env bash
set -euo pipefail

mkdir -p scripts
mkdir -p results/week15_autoware/day3_topic_analysis

ROOT="$HOME/avp_core_implementation"
OUT="$ROOT/results/week15_autoware/day3_topic_analysis"

mkdir -p "$OUT"

collect_info() {
    local input_file="$1"
    local output_file="$2"

    echo "# Topic verbose info generated at $(date)" > "$output_file"
    echo "" >> "$output_file"

    while IFS= read -r topic; do
        if [[ -z "$topic" ]]; then
            continue
        fi

        echo "============================================================" >> "$output_file"
        echo "TOPIC: $topic" >> "$output_file"
        echo "------------------------------------------------------------" >> "$output_file"

        timeout 5s ros2 topic info -v "$topic" >> "$output_file" 2>&1 || {
            echo "[WARN] Failed to read topic info: $topic" >> "$output_file"
        }

        echo "" >> "$output_file"
    done < "$input_file"

}

collect_info "$OUT/planning_candidates.txt" "$OUT/planning_topic_info_verbose.txt"
collect_info "$OUT/control_candidates.txt" "$OUT/control_topic_info_verbose.txt"
collect_info "$OUT/localization_candidates.txt" "$OUT/localization_topic_info_verbose.txt"
collect_info "$OUT/vehicle_candidates.txt" "$OUT/vehicle_topic_info_verbose.txt"
collect_info "$OUT/operation_candidates.txt" "$OUT/operation_topic_info_verbose.txt"

echo "Done. Results saved under: $OUT"