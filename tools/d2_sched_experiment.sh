#!/usr/bin/env bash

set -u

OUT=results/week17/day2   # path
CORE=3                    # core number
PRIO=80                   # realtime priority
SECS=20                   # measurement period

VICTIM=""
STRESS=""
mkdir -p "$OUT"

cleanup() {
  [ -n "$VICTIM" ] && kill "$VICTIM" 2>/dev/null
  pkill -f stress-ng 2>/dev/null
  return 0
}
trap cleanup EXIT

run_load() {
  taskset -c "$CORE" stress-ng --cpu 2 --timeout $((SECS + 10))s >/dev/null 2>&1 &
  STRESS=$!
  sleep 3
  ps -o pid,psr,comm -C stress-ng
}

echo "=== victim start (core $CORE fix) ==="
python3 tools/victim.py 1 300 500 > "$OUT/victim_stdout.log" 2>&1 &
VICTIM=$!
sleep 2
taskset -cp "$CORE" "$VICTIM"
ps -o pid,psr,comm -p "$VICTIM"
chrt -p "$VICTIM"

echo
echo "=== Run 1/3 : non-load, SCHED_OTHER (${SECS}s) ==="
pidstat -w -p "$VICTIM" 1 "$SECS" > "$OUT/victim_1_idle.txt"

echo
echo "=== Run 2/3 : load, SCHED_OTHER (${SECS}s) ==="
run_load
pidstat -w -p "$VICTIM" 1 "$SECS" > "$OUT/victim_2_other_load.txt"
wait "$STRESS" 2>/dev/null

echo
echo "=== SCHED_FIFO $PRIO ==="
sudo chrt -f -p "$PRIO" "$VICTIM"
chrt -p "$VICTIM"

echo
echo "=== Run 3/3 : load, SCHED_FIFO $PRIO (${SECS}s) ==="
run_load
pidstat -w -p "$VICTIM" 1 "$SECS" > "$OUT/victim_3_fifo_load.txt"
wait "$STRESS" 2>/dev/null

echo
echo "=== result ==="
for f in "$OUT"/victim_[123]_*.txt; do
  printf "%-26s " "$(basename "$f")"
  awk '/^(Average:|Average:)/ && $2 != "UID" {
         printf "cswch/s=%-9s nvcswch/s=%s\n", $4, $5
       }' "$f"
done