#!/usr/bin/env bash
# Runs on the host before the dev container starts (initializeCommand), and before
# `docker run` for plain Docker. Prepares the display access for RViz, Gazebo and rqt in the
# container: the X socket folder, and .x11/xauth, a copy of your display's X11 cookie.
# The container reads the cookie through the workspace mount, so a refreshed cookie also
# reaches a running container. Git and Docker ignore the .x11 folder.
# Native Linux desktops (X11 or Xwayland) need xauth installed. WSLg needs no cookie, so the
# file may stay empty there. No `xhost +` is needed.
set -u

# Create the X socket folder if it's missing, so mounting it into the container never fails
mkdir -p /tmp/.X11-unix

# Create the folder .x11 next to this script; only you can read it and the cookie in it
# (folder 700, file 600)
x11_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.x11"
mkdir -p "$x11_dir"
chmod 700 "$x11_dir"

# Write the cookie to a temporary file and rename it at the end, so a running container never
# reads a half-written cookie
tmp_file="$x11_dir/xauth.new"
rm -f "$tmp_file"
touch "$tmp_file"
chmod 600 "$tmp_file"
# Copy the current display's cookie, if there is a display and xauth is installed. The first four
# characters of each entry are the address family; ffff ("wild") makes it match any hostname,
# including the container's.
if [ -n "${DISPLAY:-}" ] && command -v xauth > /dev/null; then
    xauth nlist "$DISPLAY" 2> /dev/null | sed -e 's/^..../ffff/' | xauth -f "$tmp_file" nmerge - 2> /dev/null
fi
mv -f "$tmp_file" "$x11_dir/xauth"
# Never fail: without a cookie (WSLg, or no display), the container still starts
exit 0
