#!/usr/bin/env bash
set -euo pipefail

API_BASE="${RAK_CAR_API_BASE:-http://192.168.3.60:5050}"
API_PREFIX="${RAK_CAR_API_PREFIX:-/v1}"

echo "[1] health"
curl -s "${API_BASE}${API_PREFIX}/health"
echo
echo

echo "[2] actions"
curl -s "${API_BASE}${API_PREFIX}/actions"
echo
echo

echo "[3] 同步 beep"
curl -s -X POST "${API_BASE}${API_PREFIX}/execute" \
  -H 'Content-Type: application/json' \
  -d '{
    "target": "car",
    "name": "beep",
    "timeout": 40
  }'
echo
echo

echo "[4] 同步 shooting"
curl -s -X POST "${API_BASE}${API_PREFIX}/execute" \
  -H 'Content-Type: application/json' \
  -d '{
    "target": "car",
    "name": "shooting",
    "timeout": 50
  }'
echo
echo

echo "[5] 同步 move_for"
curl -s -X POST "${API_BASE}${API_PREFIX}/execute" \
  -H 'Content-Type: application/json' \
  -d '{
    "target": "car",
    "name": "move_for",
    "args": [[0.05, 0.0, 0.0]],
    "timeout": 60
  }'
echo
echo

echo "[6] emergency stop"
curl -s -X POST "${API_BASE}${API_PREFIX}/control/emergency-stop"
echo
