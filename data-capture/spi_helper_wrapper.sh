#!/bin/bash

# spi_helper_wrapper.sh
# Run the spi_capture_helper with real-time priority, negative nice level,
# and pinned to a specific CPU to minimize scheduler jitter.

set -euo pipefail

# Default helper path relative to this script.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER="${SCRIPT_DIR}/spi_capture_helper"

if [[ ! -x "${HELPER}" ]]; then
    echo "Error: spi_capture_helper binary not found at ${HELPER}" >&2
    exit 1
fi

# CPU to pin to (change if desired).
CPU_LIST="${SPI_HELPER_CPU:-2}"
# Real-time priority (1-99). Use an environment override if needed.
RT_PRIO="${SPI_HELPER_RT_PRIO:-80}"
# Nice adjustment (negative values raise priority).
NICE_LEVEL="${SPI_HELPER_NICE:- -10}"

exec taskset --cpu-list "${CPU_LIST}" \
    chrt --fifo "${RT_PRIO}" \
    nice -n "${NICE_LEVEL}" \
    "${HELPER}" "$@"
