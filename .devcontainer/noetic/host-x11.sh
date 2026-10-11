#!/usr/bin/env bash
# Runs on the host before the dev container starts (initializeCommand), and before
# `docker run` for plain Docker. Prepares the display access for the GUI programs in the
# container (RViz, Gazebo, rqt):
# - makes sure the host's X socket folder /tmp/.X11-unix exists
# - writes .x11/xauth next to this script: a copy of your display's X11 cookie, the secret the
#   X server asks programs for
# Your desktop makes a new cookie at each login. This script runs before every container start,
# so the copy is renewed then. The container reads .x11/xauth through the mount of your clone
# (workspaceMount in devcontainer.json): if you log in again while a container keeps running,
# run this script again and the container gets the new cookie too.
# .gitignore and .dockerignore exclude .x11, so the cookie never ends up in Git or in the image.
# Native Linux desktops (X11 or Xwayland) need xauth installed. WSLg accepts programs without a
# cookie, so the file stays empty there.
# The cookie gives access only to programs that have it, such as the container's; `xhost +`,
# which many Docker guides use instead, would let every program on the machine use your display.
# The method comes from the ROS tutorial http://wiki.ros.org/docker/Tutorials/GUI and from
# rocker, OSRF's tool for running ROS in Docker: https://github.com/osrf/rocker
set -u

# Create the X socket folder if it's missing, so mounting it into the container never fails
mkdir -p /tmp/.X11-unix

# Create the folder .x11 next to this script: BASH_SOURCE[0] is the script's path, dirname its
# folder, and cd + pwd make that absolute, wherever the script is called from. Only you can read
# the folder and the cookie in it (folder 700, file 600).
x11_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.x11"
mkdir -p "$x11_dir"
chmod 700 "$x11_dir"

# Start with an empty temporary file that only you can read (after removing a leftover of an
# interrupted run), write the cookie into it and rename it at the end. So a running container
# never reads a half-written cookie, and xauth also exists when no cookie is copied (WSLg).
tmp_file="$x11_dir/xauth.new"
rm -f "$tmp_file"
touch "$tmp_file"
chmod 600 "$tmp_file"
# Copy the current display's cookie, if there is a display and xauth is installed:
# - xauth nlist prints the display's entries from your Xauthority file, in hex
# - each entry starts with its address family (four characters), followed by the hostname;
#   0100 means "local connections from this hostname". sed replaces it with ffff ("wild",
#   any address), so the cookie also works in a container with another hostname
# - xauth nmerge adds them to the temporary file
# Errors are hidden: WSLg has no Xauthority file, so there's nothing to copy.
if [ -n "${DISPLAY:-}" ] && command -v xauth > /dev/null; then
    xauth nlist "$DISPLAY" 2> /dev/null | sed -e 's/^..../ffff/' | xauth -f "$tmp_file" nmerge - 2> /dev/null
fi
mv -f "$tmp_file" "$x11_dir/xauth"
# Never fail: without a cookie (WSLg, or no display), the container still starts
exit 0
