#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${SCRIPT_DIR}/clips"
URL="https://www.youtube.com/watch?v=o4TdHrMi6do"
FULL_VIDEO="${OUT_DIR}/o4TdHrMi6do_full.mp4"
CLIP_VIDEO="${OUT_DIR}/o4TdHrMi6do_01m12s_01m24s.mp4"

mkdir -p "${OUT_DIR}"

if ! command -v yt-dlp >/dev/null 2>&1; then
  echo "yt-dlp is required but not installed."
  echo "Install it first, then run this script again."
  exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg is required but not installed."
  exit 1
fi

echo "Downloading source video..."
yt-dlp \
  --format "bv*[ext=mp4]+ba[ext=m4a]/b[ext=mp4]/b" \
  --merge-output-format mp4 \
  --output "${FULL_VIDEO}" \
  "${URL}"

echo "Extracting clip 00:01:12 -> 00:01:24..."
if ! ffmpeg -y -ss 00:01:12 -to 00:01:24 -i "${FULL_VIDEO}" -c copy "${CLIP_VIDEO}"; then
  ffmpeg -y -ss 00:01:12 -to 00:01:24 -i "${FULL_VIDEO}" -c:v libx264 -c:a aac "${CLIP_VIDEO}"
fi

echo "Clip ready: ${CLIP_VIDEO}"
