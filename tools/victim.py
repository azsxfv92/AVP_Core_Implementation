#!/usr/bin/env python3
import os
import sys
import time

period_ms = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
work_us = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

period_s = period_ms / 1000.0
work_s = work_us / 1_000_000.0


print(f"victim PID={os.getpid()} period={period_ms}ms duration={duration_s}s work_us={work_us}us", flush=True)

end = time.monotonic() + duration_s
wakeups = 0
while time.monotonic() < end:    
      time.sleep(period_s)
      wakeups += 1

      t0 = time.monotonic()
      while time.monotonic() - t0 < work_s:
            pass

print(f"victim done: {wakeups} wakeups in {duration_s}s "
      f"({wakeups / duration_s:.1f}/s)", flush=True)
