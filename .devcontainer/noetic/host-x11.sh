#!/usr/bin/env bash
# Runs on the host before the dev container starts (initializeCommand), and before
# `docker run` for plain Docker. Prepares X11 access for RViz, Gazebo and rqt:
# - the X socket directory, which is mounted into the container
# - .x11/xauth next to this script: a copy of the display's X11 cookie that works inside
#   the container. The cookie's address family is set to "wild" (ffff), so it matches any
#   hostname. The container sees the file through the workspace mount, so a refreshed
#   cookie reaches a running container too. The folder is ignored by Git and Docker.
# Native Linux desktops (X11 or Xwayland) need the cookie and xauth installed. WSLg needs
# no cookie, so the file may stay empty there. No `xhost +` is needed.
set -u

mkdir -p /tmp/.X11-unix

x11_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.x11"
mkdir -p "$x11_dir"
chmod 700 "$x11_dir"

tmp_file="$x11_dir/xauth.new"
rm -f "$tmp_file"
touch "$tmp_file"
chmod 600 "$tmp_file"
if [ -n "${DISPLAY:-}" ] && command -v xauth > /dev/null; then
    xauth nlist "$DISPLAY" 2> /dev/null | sed -e 's/^..../ffff/' | xauth -f "$tmp_file" nmerge - 2> /dev/null
fi
mv -f "$tmp_file" "$x11_dir/xauth"
exit 0
