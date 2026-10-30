#!/usr/bin/env bash
# Runs a command in the firmware container (compose.yaml next to this script), as your user, so
# PlatformIO doesn't have to be installed on your PC. From the repository's root:
#   .docker/firmware/run.sh pio run -e teensy40   # build the firmware for Remo's Teensy 4.0
#   .docker/firmware/run.sh pio run -e teensy31   # or for DiffBot's Teensy 3.2
#   .docker/firmware/run.sh bash                  # a shell in the container
# From another folder, use the path to this script from there, for example
# ../.docker/firmware/run.sh in diffbot_firmware.
# It builds the image first, then runs the command in diffbot_firmware. Paths in the command are
# paths in the container, where the repository is /workspace.
# LOCAL_UID and LOCAL_GID are your user and group ID unless you set them, for example
#   LOCAL_UID=1234 LOCAL_GID=1234 .docker/firmware/run.sh pio run -e teensy40
set -euo pipefail

usage() {
    echo "Usage: $0 COMMAND [ARGUMENT...], for example: $0 pio run -e teensy40"
    echo "Runs COMMAND in the firmware container, in diffbot_firmware, as your user."
}

# Without a command, say how to use the script, before Docker starts
if [ "$#" -eq 0 ]; then
    usage >&2
    exit 2
fi
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
    exit 0
fi

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LOCAL_UID="${LOCAL_UID:-$(id -u)}"
export LOCAL_GID="${LOCAL_GID:-$(id -g)}"
compose=(docker compose -f "$here/compose.yaml")

# Build the image. The first build shows Docker's progress, because it downloads the base image
# and the Python packages. Later builds usually take seconds, because Docker reuses the cached
# result of each step when the Dockerfile and requirements.txt haven't changed. The script keeps
# the output of these later builds and prints it only when the build fails.
image="$("${compose[@]}" config --images firmware)"
if ! docker image inspect "$image" > /dev/null 2>&1; then
    "${compose[@]}" build firmware
else
    status=0
    build_log="$("${compose[@]}" --progress plain build firmware 2>&1)" || status=$?
    if [ "$status" -ne 0 ]; then
        printf '%s\n' "$build_log" >&2
        exit "$status"
    fi
fi

# Without a terminal for the input or the output, for example in CI, run the container without
# one. Docker Compose checks only the output before version 2.39.4.
tty_flag=()
if [ ! -t 0 ] || [ ! -t 1 ]; then
    tty_flag=(-T)
fi
exec "${compose[@]}" run --rm ${tty_flag[@]+"${tty_flag[@]}"} firmware "$@"
