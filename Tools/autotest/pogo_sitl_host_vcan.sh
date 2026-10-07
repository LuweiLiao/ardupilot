#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_DIR="${POGO_SITL_RUN_DIR:-/tmp/ardupilot-sitl-host-vcan-dual}"
VENV_DIR="${POGO_SITL_VENV:-/tmp/ardupilot-sitl-venv}"
COPTER_BIN="${POGO_COPTER_BIN:-$ROOT_DIR/build/sitl/sitl/bin/arducopter}"
PERIPH_BIN="${POGO_PERIPH_BIN:-$ROOT_DIR/build/sitl-periph/sitl_periph_universal/bin/AP_Periph}"

for binary in "$COPTER_BIN" "$PERIPH_BIN" "$VENV_DIR/bin/python3"; do
    if [[ ! -x "$binary" ]]; then
        printf 'missing executable: %s\n' "$binary" >&2
        exit 1
    fi
done

if [[ "$(id -u)" -eq 0 ]]; then
    SUDO=()
else
    SUDO=(sudo)
fi

for iface in vcan0 vcan1; do
    if ! ip link show "$iface" >/dev/null 2>&1; then
        "${SUDO[@]}" modprobe vcan
        "${SUDO[@]}" ip link add dev "$iface" type vcan
    fi
    if ! ip -details link show "$iface" | grep -q 'link/can'; then
        printf '%s exists but is not a CAN interface\n' "$iface" >&2
        exit 1
    fi
    if ! ip link show "$iface" | grep -q 'mtu 72'; then
        "${SUDO[@]}" ip link set "$iface" mtu 72
    fi
    if ! ip link show "$iface" | grep -q '<[^>]*UP'; then
        "${SUDO[@]}" ip link set "$iface" up
    fi
done

mkdir -p "$RUN_DIR/a" "$RUN_DIR/b"
printf '%s\n' \
    'CAN_P1_DRIVER 1' 'CAN_P2_DRIVER 0' 'CAN_D1_PROTOCOL 1' \
    'SIM_CAN_TYPE1 2' 'SIM_CAN_TYPE2 0' 'SIM_CAN_SRV_MSK 255' > "$RUN_DIR/a/arducopter.parm"
printf '%s\n' 'SIM_CAN_TYPE1 2' 'SIM_CAN_TYPE2 0' 'CAN_PROTOCOL 1' \
    'CAN2_PROTOCOL 0' 'CAN_NODE 125' > "$RUN_DIR/a/periph.parm"
printf '%s\n' \
    'CAN_P1_DRIVER 0' 'CAN_P2_DRIVER 1' 'CAN_D1_PROTOCOL 1' \
    'SIM_CAN_TYPE1 0' 'SIM_CAN_TYPE2 2' 'SIM_CAN_SRV_MSK 255' > "$RUN_DIR/b/arducopter.parm"
printf '%s\n' 'SIM_CAN_TYPE1 0' 'SIM_CAN_TYPE2 2' 'CAN_PROTOCOL 0' \
    'CAN2_PROTOCOL 1' 'CAN_NODE 126' > "$RUN_DIR/b/periph.parm"

cleanup() {
    kill "${periph_a_pid:-}" "${periph_b_pid:-}" "${mav_a_pid:-}" "${mav_b_pid:-}" \
        "${copter_a_pid:-}" "${copter_b_pid:-}" "${dump_a_pid:-}" "${dump_b_pid:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for domain in a b; do
    iface=vcan0
    [[ "$domain" == b ]] && iface=vcan1
    # Keep a bounded capture for repeatable evidence without filling /tmp.
    (cd "$RUN_DIR/$domain" && exec candump -L -n 10000 "$iface" > can.log 2> can.err) &
    printf -v "dump_${domain}_pid" '%s' "$!"
done

start_copter() {
    local domain="$1" instance="$2" port="$3"
    (cd "$RUN_DIR/$domain" && exec "$COPTER_BIN" --wipe --model quad --instance "$instance" \
        --defaults "$RUN_DIR/$domain/arducopter.parm" > arducopter.log 2>&1) &
    printf -v "copter_${domain}_pid" '%s' "$!"
    (cd "$RUN_DIR/$domain" && exec "$VENV_DIR/bin/python3" -u -c '
from pymavlink import mavutil
mav = mavutil.mavlink_connection("tcp:127.0.0.1:'"$port"'", source_system=250)
mav.wait_heartbeat(timeout=15)
while True:
    mav.recv_match(blocking=True, timeout=1)
' > mavclient.log 2>&1) &
    printf -v "mav_${domain}_pid" '%s' "$!"
}

start_copter a 0 5760
start_copter b 1 5770
sleep 5

for domain in a b; do
    instance=0
    [[ "$domain" == b ]] && instance=1
    (cd "$RUN_DIR/$domain" && exec "$PERIPH_BIN" --instance "$instance" \
        --defaults "$RUN_DIR/$domain/periph.parm" > periph.log 2>&1) &
    printf -v "periph_${domain}_pid" '%s' "$!"
done

printf 'A: vcan0, ArduCopter %s, AP_Periph %s, MAVLink TCP 5760\n' "$copter_a_pid" "$periph_a_pid"
printf 'B: vcan1, ArduCopter %s, AP_Periph %s, MAVLink TCP 5770\n' "$copter_b_pid" "$periph_b_pid"
printf 'Logs: %s/{a,b}/\n' "$RUN_DIR"
printf 'Stop with Ctrl-C.\n'

wait -n "$copter_a_pid" "$copter_b_pid" "$periph_a_pid" "$periph_b_pid"
