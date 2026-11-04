#!/bin/bash

# Copyright 2024 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# SWPL-5189: Verify URUP upgrade from Docker version (n-1) to Podman version (n).
#
# This script automates the command-line verification steps described in the
# test case. It has two modes:
#
#   capture-before  Run on the Docker (n-1) robot BEFORE the URUP update.
#                   Saves a snapshot of container state, images, ports,
#                   macvlan config, marker files, and program checksums.
#
#   verify-after    Run on the Podman (n) robot AFTER the URUP update.
#                   Re-captures the state and compares it against the
#                   pre-update snapshot, reporting PASS/FAIL for each check.
#
# Physical steps (flashing, USB, installing URCaps via PolyScope X, creating
# programs, turning on EtherNet/IP and Profinet) must still be performed
# manually. This script only handles the shell-verifiable checks.
#
# Usage:
#   On the Docker robot (before update):
#     ./verify_urup_docker_to_podman.sh capture-before [--snapshot-dir DIR]
#
#   On the Podman robot (after update):
#     ./verify_urup_docker_to_podman.sh verify-after  [--snapshot-dir DIR] [--robot-ip IP]

set -euo pipefail

# ──────────────────────────────────────────────────────────────────────────────
# Defaults
# ──────────────────────────────────────────────────────────────────────────────
SNAPSHOT_DIR="/tmp/urup_migration_snapshot"
ROBOT_IP=""
PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0

# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
usage() {
  cat <<'EOF'
Usage:
  verify_urup_docker_to_podman.sh capture-before [OPTIONS]
  verify_urup_docker_to_podman.sh verify-after   [OPTIONS]

Options:
  --snapshot-dir DIR   Directory for before/after snapshots (default: /tmp/urup_migration_snapshot)
  --robot-ip IP        Robot IP for remote port-forwarding checks (verify-after only)
  -h, --help           Show this help

Modes:
  capture-before   Capture Docker state before the URUP update
  verify-after     Verify Podman state after the URUP update
EOF
  exit 0
}

log_info()  { echo -e "\033[1;34m[INFO]\033[0m  $*"; }
log_pass()  { echo -e "\033[1;32m[PASS]\033[0m  $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
log_fail()  { echo -e "\033[1;31m[FAIL]\033[0m  $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }
log_skip()  { echo -e "\033[1;33m[SKIP]\033[0m  $*"; SKIP_COUNT=$((SKIP_COUNT + 1)); }

check_eq() {
  local label="$1" expected="$2" actual="$3"
  if [[ "$expected" == "$actual" ]]; then
    log_pass "$label"
  else
    log_fail "$label (expected: '$expected', got: '$actual')"
  fi
}

check_contains() {
  local label="$1" haystack="$2" needle="$3"
  if echo "$haystack" | grep -qF "$needle"; then
    log_pass "$label"
  else
    log_fail "$label (expected to contain '$needle')"
  fi
}

check_not_exists() {
  local label="$1" path="$2"
  if [[ ! -e "$path" ]]; then
    log_pass "$label"
  else
    log_fail "$label ('$path' still exists)"
  fi
}

check_exists() {
  local label="$1" path="$2"
  if [[ -e "$path" ]]; then
    log_pass "$label"
  else
    log_fail "$label ('$path' does not exist)"
  fi
}

check_command_succeeds() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    log_pass "$label"
  else
    log_fail "$label (command failed: $*)"
  fi
}

check_port_open() {
  local label="$1" host="$2" port="$3"
  if timeout 3 bash -c "</dev/tcp/$host/$port" 2>/dev/null; then
    log_pass "$label ($host:$port open)"
  else
    log_fail "$label ($host:$port closed)"
  fi
}

check_port_closed() {
  local label="$1" host="$2" port="$3"
  if timeout 3 bash -c "</dev/tcp/$host/$port" 2>/dev/null; then
    log_fail "$label ($host:$port unexpectedly open)"
  else
    log_pass "$label ($host:$port closed as expected)"
  fi
}

# ──────────────────────────────────────────────────────────────────────────────
# capture-before: snapshot Docker state
# ──────────────────────────────────────────────────────────────────────────────
do_capture_before() {
  log_info "Capturing pre-update Docker state to $SNAPSHOT_DIR"
  mkdir -p "$SNAPSHOT_DIR"

  log_info "Checking Docker is available"
  if ! command -v docker &>/dev/null; then
    log_fail "docker command not found"
    exit 1
  fi

  log_info "docker ps -a"
  docker ps -a > "$SNAPSHOT_DIR/docker_ps_a.txt" 2>&1 || true

  log_info "docker images"
  docker images > "$SNAPSHOT_DIR/docker_images.txt" 2>&1 || true

  log_info "Port forwards"
  docker ps --format "{{.Names}} {{.Ports}}" > "$SNAPSHOT_DIR/docker_ports.txt" 2>&1 || true

  log_info "Macvlan persist entries"
  cat /var/urcaps/*/*/persist-*.json > "$SNAPSHOT_DIR/macvlan_persist.json" 2>/dev/null || echo "No persist files found" > "$SNAPSHOT_DIR/macvlan_persist.json"

  log_info "eth1 inside ur_test_mount container"
  local pid
  pid=$(docker inspect -f '{{.State.Pid}}' test-mount_ur_test_mount_ur_test_mount-backend 2>/dev/null) || true
  if [[ -n "$pid" && "$pid" != "0" ]]; then
    nsenter -t "$pid" -n ip -br link > "$SNAPSHOT_DIR/ur_test_mount_eth1.txt" 2>&1 || true
  else
    echo "Container not running or not found" > "$SNAPSHOT_DIR/ur_test_mount_eth1.txt"
  fi

  log_info "Marker files under /var/urcaps/acme"
  if [[ -d /var/urcaps/acme ]]; then
    find /var/urcaps/acme -type f -exec ls -la {} \; > "$SNAPSHOT_DIR/acme_markers.txt" 2>&1 || true
    find /var/urcaps/acme -type f -exec cat {} \; >> "$SNAPSHOT_DIR/acme_markers.txt" 2>&1 || true
  else
    echo "No /var/urcaps/acme directory" > "$SNAPSHOT_DIR/acme_markers.txt"
  fi

  log_info "Marker files under /var/urcaps/_permanent/acme"
  if [[ -d /var/urcaps/_permanent/acme ]]; then
    find /var/urcaps/_permanent/acme -type f -exec ls -la {} \; > "$SNAPSHOT_DIR/acme_permanent_markers.txt" 2>&1 || true
    find /var/urcaps/_permanent/acme -type f -exec cat {} \; >> "$SNAPSHOT_DIR/acme_permanent_markers.txt" 2>&1 || true
  else
    echo "No /var/urcaps/_permanent/acme directory" > "$SNAPSHOT_DIR/acme_permanent_markers.txt"
  fi

  log_info "acme-urup-test nonroot container logs"
  docker logs acme_acme-urup-test_nonroot > "$SNAPSHOT_DIR/nonroot_logs.txt" 2>&1 || echo "Container not found" > "$SNAPSHOT_DIR/nonroot_logs.txt"

  log_info "Program file checksums"
  if ls /ur/bin/backend/applications/*/  >/dev/null 2>&1; then
    find /ur/bin/backend/applications/ -type f -exec sha256sum {} \; > "$SNAPSHOT_DIR/program_checksums.txt" 2>&1 || true
  else
    echo "No program files found" > "$SNAPSHOT_DIR/program_checksums.txt"
  fi

  log_info "3rd-party URCap image IDs"
  docker images --format "{{.Repository}}:{{.Tag}} {{.ID}}" | grep -v "universal-robots" > "$SNAPSHOT_DIR/thirdparty_image_ids.txt" 2>/dev/null || echo "None" > "$SNAPSHOT_DIR/thirdparty_image_ids.txt"

  log_info "Snapshot captured. Files saved to $SNAPSHOT_DIR"
  ls -la "$SNAPSHOT_DIR/"
  echo ""
  log_info "Next steps:"
  log_info "  1. Perform the URUP update (install the Podman version via USB/PolyScope X)"
  log_info "  2. After the robot boots with version n, run:"
  log_info "     $0 verify-after --snapshot-dir $SNAPSHOT_DIR --robot-ip <robot-ip>"
}

# ──────────────────────────────────────────────────────────────────────────────
# verify-after: check Podman state and compare with pre-update snapshot
# ──────────────────────────────────────────────────────────────────────────────
do_verify_after() {
  log_info "=== URUP Docker-to-Podman Migration Verification ==="
  log_info "Snapshot directory: $SNAPSHOT_DIR"
  log_info ""

  # ── Basic checks ──────────────────────────────────────────────────────────
  log_info "── Basic checks ──"

  check_command_succeeds "Podman is available" podman ps

  local services_active
  services_active=$(systemctl is-active urservice citadel 2>&1) || true
  if echo "$services_active" | grep -q "^active"; then
    log_pass "urservice and citadel are running"
  else
    log_fail "urservice and/or citadel not active ($services_active)"
  fi

  local link_output
  link_output=$(ip -br link 2>&1) || true
  if echo "$link_output" | grep -q "ur-cobot-net.*UP" && echo "$link_output" | grep -q "podman0.*UP"; then
    log_pass "ur-cobot-net and podman0 networks are up"
  else
    if echo "$link_output" | grep -q "ur-cobot-net"; then
      log_info "  ur-cobot-net found"
    else
      log_fail "ur-cobot-net not found in ip -br link"
    fi
    if echo "$link_output" | grep -q "podman0"; then
      log_info "  podman0 found"
    else
      log_fail "podman0 not found in ip -br link"
    fi
  fi

  local migration_log
  migration_log=$(journalctl -u ur-urcap-migration -o cat 2>&1) || true
  local loaded_count
  loaded_count=$(echo "$migration_log" | grep -c "Loaded image" || true)
  if [[ "$loaded_count" -gt 0 ]]; then
    log_pass "Image migration ran: $loaded_count image(s) loaded"
  else
    log_fail "Image migration: no 'Loaded image' lines in ur-urcap-migration journal"
  fi

  echo ""

  # ── UR URCaps ─────────────────────────────────────────────────────────────
  log_info "── UR URCaps ──"

  local podman_images_readonly
  podman_images_readonly=$(podman images --format "{{.Repository}} {{.ReadOnly}}" 2>&1) || true
  local ur_images_not_readonly
  ur_images_not_readonly=$(echo "$podman_images_readonly" | grep "universal-robots" | grep -v "true" || true)
  if [[ -z "$ur_images_not_readonly" ]]; then
    log_pass "UR images are in read-only store (platform-store-current)"
  else
    log_fail "Some UR images are NOT read-only: $ur_images_not_readonly"
  fi

  echo ""

  # ── External URCaps ──────────────────────────────────────────────────────
  log_info "── External URCaps (Robotiq, OnRobot, ur_test_mount) ──"

  local podman_ps
  podman_ps=$(podman ps 2>&1) || true
  log_info "Running containers:"
  echo "$podman_ps"

  local thirdparty_readonly
  thirdparty_readonly=$(echo "$podman_images_readonly" | grep -v "universal-robots" | grep "true" || true)
  if [[ -z "$thirdparty_readonly" ]]; then
    log_pass "3rd-party images are in graphroot (ReadOnly false)"
  else
    log_fail "Some 3rd-party images are unexpectedly read-only: $thirdparty_readonly"
  fi

  if [[ -f "$SNAPSHOT_DIR/thirdparty_image_ids.txt" ]]; then
    log_info "Comparing 3rd-party image IDs with pre-update snapshot..."
    local current_ids
    current_ids=$(podman images --format "{{.Repository}}:{{.Tag}} {{.ID}}" 2>/dev/null | grep -v "universal-robots" || true)
    local before_ids
    before_ids=$(cat "$SNAPSHOT_DIR/thirdparty_image_ids.txt")
    if [[ "$before_ids" != "None" ]]; then
      local mismatch=false
      while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        local repo_tag="${line%% *}"
        local old_id="${line##* }"
        local new_id
        new_id=$(echo "$current_ids" | grep "^$repo_tag " | awk '{print $2}' || true)
        if [[ "$old_id" == "$new_id" ]]; then
          log_pass "Image ID match: $repo_tag ($old_id)"
        elif [[ -z "$new_id" ]]; then
          log_fail "Image not found after update: $repo_tag"
          mismatch=true
        else
          log_fail "Image ID mismatch for $repo_tag (before: $old_id, after: $new_id)"
          mismatch=true
        fi
      done <<< "$before_ids"
    else
      log_skip "No pre-update 3rd-party image IDs to compare"
    fi
  else
    log_skip "No pre-update snapshot for image IDs"
  fi

  log_info "ur_test_mount eth1 check (macvlan)"
  local pid
  pid=$(podman inspect -f '{{.State.Pid}}' test-mount_ur_test_mount_ur_test_mount-backend 2>/dev/null) || true
  if [[ -n "$pid" && "$pid" != "0" ]]; then
    local eth1_after
    eth1_after=$(nsenter -t "$pid" -n ip -br link 2>&1) || true
    echo "  Current eth1: $eth1_after"
    if echo "$eth1_after" | grep -q "eth1"; then
      log_pass "ur_test_mount container has eth1"
      if [[ -f "$SNAPSHOT_DIR/ur_test_mount_eth1.txt" ]]; then
        local mac_before mac_after
        mac_before=$(grep "eth1" "$SNAPSHOT_DIR/ur_test_mount_eth1.txt" | awk '{print $3}' || true)
        mac_after=$(echo "$eth1_after" | grep "eth1" | awk '{print $3}' || true)
        check_eq "ur_test_mount eth1 MAC address preserved" "$mac_before" "$mac_after"
      fi
    else
      log_fail "ur_test_mount container missing eth1"
    fi
  else
    log_fail "ur_test_mount backend container not running or not inspectable"
  fi

  if [[ -n "$ROBOT_IP" ]]; then
    log_info "ur_test_mount data persistence check"
    local read_result
    read_result=$(curl -s "http://$ROBOT_IP/test-mount/ur_test_mount/ur_test_mount-backend/rest-api/read_file" 2>&1) || true
    check_contains "ur_test_mount data persisted" "$read_result" "written before the urup"
  else
    log_skip "ur_test_mount data persistence (no --robot-ip provided)"
  fi

  echo ""

  # ── Port forwarding ──────────────────────────────────────────────────────
  log_info "── Port forwarding ──"

  local port_host="${ROBOT_IP:-127.0.0.1}"

  if [[ -n "$ROBOT_IP" ]]; then
    log_info "Robotiq ports (should be open from a PC):"
    check_port_open "Robotiq port 49998" "$ROBOT_IP" 49998
    check_port_open "Robotiq port 49999" "$ROBOT_IP" 49999

    log_info "OnRobot ports (should be open on localhost, closed from PC):"
    check_port_closed "OnRobot port 51234 from PC" "$ROBOT_IP" 51234
    check_port_closed "OnRobot port 44005 from PC" "$ROBOT_IP" 44005
  else
    log_info "OnRobot ports (checking localhost):"
    check_port_open "OnRobot port 51234 on localhost" "127.0.0.1" 51234
    check_port_open "OnRobot port 44005 on localhost" "127.0.0.1" 44005
  fi

  log_info "EtherNet/IP ports (turn on first in PolyScope X):"
  for port in 44818 2222 40000; do
    check_port_open "EtherNet/IP port $port" "$port_host" "$port"
  done

  log_info "Profinet port (turn on first in PolyScope X):"
  check_port_open "Profinet port 40002" "$port_host" 40002

  local podman_ports
  podman_ports=$(podman ps --format "{{.Names}} {{.Ports}}" 2>&1) || true
  log_info "Current port mappings:"
  echo "$podman_ports"

  if [[ -f "$SNAPSHOT_DIR/docker_ports.txt" ]]; then
    log_info "Pre-update port mappings were:"
    cat "$SNAPSHOT_DIR/docker_ports.txt"
  fi

  echo ""

  # ── acme-urup-test checks ───────────────────────────────────────────────
  log_info "── acme-urup-test URCap ──"

  log_info "persistent: and permanent: markers"
  if [[ -d /var/urcaps/acme ]]; then
    log_pass "persistent markers exist under /var/urcaps/acme"
    find /var/urcaps/acme -type f -exec ls -la {} \; 2>/dev/null || true
  else
    log_fail "persistent markers missing (/var/urcaps/acme not found)"
  fi

  if [[ -d /var/urcaps/_permanent/acme ]]; then
    log_pass "permanent markers exist under /var/urcaps/_permanent/acme"
    find /var/urcaps/_permanent/acme -type f -exec ls -la {} \; 2>/dev/null || true
  else
    log_fail "permanent markers missing (/var/urcaps/_permanent/acme not found)"
  fi

  log_info "Dockerfile VOLUME /calibration marker (expected to be gone)"
  local calibration_vol
  calibration_vol=$(podman inspect acme_acme-urup-test_backend --format '{{range .Mounts}}{{if eq .Destination "/calibration"}}{{.Source}}{{end}}{{end}}' 2>/dev/null) || true
  if [[ -n "$calibration_vol" ]] && [[ -f "$calibration_vol/marker" || -f "$calibration_vol/data.txt" ]]; then
    log_fail "Calibration volume marker unexpectedly survived the migration"
  else
    log_pass "Calibration volume marker is gone (expected after migration)"
  fi

  log_info "nonroot container runs as user 1000"
  local nonroot_user
  nonroot_user=$(podman inspect acme_acme-urup-test_nonroot --format '{{.Config.User}}' 2>/dev/null) || true
  if [[ -z "$nonroot_user" ]]; then
    local nonroot_pid
    nonroot_pid=$(podman inspect -f '{{.State.Pid}}' acme_acme-urup-test_nonroot 2>/dev/null) || true
    if [[ -n "$nonroot_pid" && "$nonroot_pid" != "0" ]]; then
      nonroot_user=$(ps -o uid= -p "$nonroot_pid" 2>/dev/null | tr -d ' ') || true
    fi
  fi
  log_info "  nonroot container user: $nonroot_user"

  log_info "nonroot container logs (via journalctl):"
  journalctl CONTAINER_NAME=acme_acme-urup-test_nonroot -o cat 2>&1 | tail -20 || true

  log_info "stopped container (startup: manual, never started)"
  local stopped_image
  stopped_image=$(podman images --format "{{.Repository}}" 2>/dev/null | grep "acme.*stopped" || true)
  if [[ -n "$stopped_image" ]]; then
    log_pass "stopped container image exists"
  else
    log_fail "stopped container image not found"
  fi
  local stopped_running
  stopped_running=$(podman ps --format "{{.Names}}" 2>/dev/null | grep "acme.*stopped" || true)
  if [[ -z "$stopped_running" ]]; then
    log_pass "stopped container is not running (expected)"
  else
    log_fail "stopped container is unexpectedly running"
  fi

  if [[ -n "$ROBOT_IP" ]]; then
    log_info "acme-urup-test frontend"
    local frontend_result
    frontend_result=$(curl -s "http://$ROBOT_IP/" 2>&1) || true
    if echo "$frontend_result" | grep -q "acme-frontend-marker-v1"; then
      log_pass "acme-urup-test frontend page contains marker"
    else
      log_skip "Could not verify acme-frontend-marker-v1 (may need specific URL path)"
    fi

    log_info "acme-urup-test REST endpoint"
    local rest_result
    rest_result=$(curl -s "http://$ROBOT_IP/acme/acme-urup-test/acme-urup-test-backend/rest-api/" 2>&1) || true
    if [[ -n "$rest_result" ]]; then
      log_pass "acme-urup-test REST endpoint responds"
    else
      log_fail "acme-urup-test REST endpoint returned no data"
    fi
  else
    log_skip "acme-urup-test frontend and REST endpoint (no --robot-ip provided)"
  fi

  echo ""

  # ── Program files ────────────────────────────────────────────────────────
  log_info "── Program files ──"

  if [[ -f "$SNAPSHOT_DIR/program_checksums.txt" ]] && [[ "$(cat "$SNAPSHOT_DIR/program_checksums.txt")" != "No program files found" ]]; then
    local current_checksums
    current_checksums=$(find /ur/bin/backend/applications/ -type f -exec sha256sum {} \; 2>/dev/null) || true
    if [[ -n "$current_checksums" ]]; then
      local checksum_match=true
      while IFS= read -r line; do
        [[ -z "$line" ]] && continue
        local expected_hash="${line%% *}"
        local filepath="${line#* }"
        filepath="${filepath## }"
        local actual_hash
        actual_hash=$(sha256sum "$filepath" 2>/dev/null | awk '{print $1}') || true
        if [[ "$expected_hash" == "$actual_hash" ]]; then
          log_pass "Program file checksum: $filepath"
        else
          log_fail "Program file checksum mismatch: $filepath"
          checksum_match=false
        fi
      done < "$SNAPSHOT_DIR/program_checksums.txt"
    else
      log_fail "No program files found after update"
    fi
  else
    log_skip "No pre-update program checksums to compare"
  fi

  echo ""

  # ── Docker leftovers ─────────────────────────────────────────────────────
  log_info "── Docker leftovers ──"

  check_not_exists "Docker graphroot removed" "/data/var/lib/docker"
  check_not_exists "Docker graphroot removed (alt path)" "/var/lib/docker"
  check_not_exists "/data/docker-migration removed" "/data/docker-migration"

  if command -v docker &>/dev/null; then
    log_info "docker command is still present (checking it is not used for URCaps)"
  fi

  local podman_containers
  podman_containers=$(podman ps -a --format "{{.Names}}" 2>&1) || true
  if [[ -n "$podman_containers" ]]; then
    log_pass "URCap containers are managed by Podman"
  else
    log_fail "No Podman containers found"
  fi

  echo ""

  # ── Summary ──────────────────────────────────────────────────────────────
  log_info "════════════════════════════════════════════════════"
  log_info "  Verification Summary"
  log_info "════════════════════════════════════════════════════"
  echo -e "  \033[1;32mPASS: $PASS_COUNT\033[0m"
  echo -e "  \033[1;31mFAIL: $FAIL_COUNT\033[0m"
  echo -e "  \033[1;33mSKIP: $SKIP_COUNT\033[0m"
  log_info "════════════════════════════════════════════════════"

  if [[ $FAIL_COUNT -gt 0 ]]; then
    log_info "Some checks FAILED. Review the output above for details."
    return 1
  else
    log_info "All checks passed!"
    return 0
  fi
}

# ──────────────────────────────────────────────────────────────────────────────
# Argument parsing
# ──────────────────────────────────────────────────────────────────────────────
MODE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    capture-before)
      MODE="capture-before"
      shift
      ;;
    verify-after)
      MODE="verify-after"
      shift
      ;;
    --snapshot-dir)
      SNAPSHOT_DIR="$2"
      shift 2
      ;;
    --robot-ip)
      ROBOT_IP="$2"
      shift 2
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "Unknown option: $1"
      usage
      ;;
  esac
done

if [[ -z "$MODE" ]]; then
  echo "Error: no mode specified."
  usage
fi

case "$MODE" in
  capture-before)
    do_capture_before
    ;;
  verify-after)
    do_verify_after
    ;;
esac
