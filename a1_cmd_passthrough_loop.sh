#!/usr/bin/env bash
while true; do
  if [ -s /tmp/a1_follow_cmd_raw ]; then
    cp /tmp/a1_follow_cmd_raw /tmp/a1_follow_cmd.tmp
    mv /tmp/a1_follow_cmd.tmp /tmp/a1_follow_cmd
  fi
  sleep 0.02
done
