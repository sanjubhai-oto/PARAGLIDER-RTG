#!/usr/bin/env bash
# Kill all PARAGLIDER-RTG processes + free ports

echo "==> Killing all sim processes"
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {} 2>/dev/null || true

echo "==> Freeing ports"
for port in 14550 14551 14552 5760 9002; do
    for p in $(lsof -iTCP:$port -t 2>/dev/null) $(lsof -iUDP:$port -t 2>/dev/null); do
        kill -9 $p 2>/dev/null || true
    done
done

sleep 2
echo "==> Remaining sim processes:"
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2, $11}' | head
echo "==> Done"
