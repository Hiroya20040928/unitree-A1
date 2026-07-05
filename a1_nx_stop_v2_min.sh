#!/usr/bin/env bash
set -u

echo "[A1 v2-min STOP] send stop"
python3 - <<'PY'
import time
for p in ["/tmp/a1_follow_cmd_raw", "/tmp/a1_follow_cmd"]:
    with open(p, "w") as f:
        f.write("0 0.00000 0.00000 0.00000 %.6f\n" % time.time())
PY

echo "[A1 v2-min STOP] kill processes"
sudo pkill -9 -f '[a]1_high_follow_driver' 2>/dev/null || true
sudo pkill -9 -f '[a]1_high_follow_driver_v2' 2>/dev/null || true
pkill -9 -f '[a]1_follow_lowlatency_depth_server' 2>/dev/null || true
pkill -9 -f '[a]1_cmd_passthrough_loop' 2>/dev/null || true

echo "[A1 v2-min STOP] DONE"
