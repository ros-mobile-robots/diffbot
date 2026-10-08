#!/usr/bin/env bash
# Runs on the host before the dev container starts (initializeCommand), and before
# `docker run` for plain Docker. Prepares X11 access for RViz, Gazebo and rqt:
# - the X socket directory, which is mounted into the container
# - a copy of the display's X11 cookie that works inside the container. The cookie's
#   address family is set to "wild" (ffff), so it matches any hostname.
# Native Linux desktops (X11 or Xwayland) need the cookie and xauth installed. WSLg needs
# no cookie, so the file may stay empty there. No `xhost +` is needed.
set -u

mkdir -p /tmp/.X11-unix

xauth_file="/tmp/.diffbot-${USER:-$(id -un)}.xauth"
: > "$xauth_file"
chmod 600 "$xauth_file"

if [ -n "${DISPLAY:-}" ] && command -v xauth > /dev/null; then
    xauth nlist "$DISPLAY" 2> /dev/null | sed -e 's/^..../ffff/' | xauth -f "$xauth_file" nmerge - 2> /dev/null
fi
exit 0
