#!/usr/bin/env bash
# Records a short video of DiffBot driving in Gazebo, without a screen and without a GPU, as on
# CI's runners. DiffBot drives the simulation test's commands (test/test_drive.py) once more, in a
# run of its own: the video shows what the test's drive looks like, not a test run, and this
# script checks nothing about the drive. CI's workflow "Simulation video"
# (.github/workflows/sim_video.yml) runs it on request.
#
# Usage, in the dev container (.devcontainer/noetic), with the workspace built and loaded:
#   rosrun diffbot_gazebo record_drive_video.sh [OUTPUT_FOLDER]
# OUTPUT_FOLDER must be new or empty, so everything in it comes from this run. Without it, the
# script creates ~/.ros/sim_video/<date>-<time>. After a successful run, the folder holds:
#   diffbot_drive.mp4   the video: H.264, 960x540, 30 frames per second of simulation time
#   diffbot_drive.json  model, commit, commands, the image's software and the settings
#   record.log          this script's messages
#   simulation.log      the messages of roslaunch, Gazebo and the controllers
#   xvfb.log            the messages of Xvfb, the virtual display
# Any failed step stops the script with an error. The folder then holds no video, only the logs.
#
# The steps:
# 1. make_video_world.py writes a copy of the test's world (worlds/diffbot_empty.world) with a
#    camera, which saves every frame it renders as a JPEG file, and a floor grid
# 2. Xvfb, a virtual X server, is the display that the Gazebo server needs to render the camera.
#    Mesa renders on the CPU (llvmpipe)
# 3. roslaunch starts the Gazebo server with lockstep, and the controllers
#    (launch/sim_video.launch). With lockstep, the physics wait for the camera, so the camera
#    saves a frame every 1/30 s of simulation time, however slowly the computer renders
# 4. drive_for_video.py waits until the simulation, the controller and the camera are ready,
#    drives, and notes how many frames the camera had saved when the drive started and ended
# 5. The simulation stops
# 6. encode_drive_video.py cuts the frames to the drive, encodes the MP4, checks it and writes the
#    JSON file
set -euo pipefail

readonly MODEL=diffbot
readonly VIDEO_NAME=${MODEL}_drive
# The camera: image size (pixels) and frames per second of simulation time
readonly WIDTH=960
readonly HEIGHT=540
readonly RATE=30
# Simulation time (s) with the robot at rest, before and after the drive
readonly PAD=0.5
# The ROS master's and the Gazebo server's ports. Not the defaults (11311 and 11345), which a
# simulation open in the dev container may use, nor the simulation tests' (11351 and 11352)
readonly ROS_PORT=11361
readonly GAZEBO_PORT=11362
# Wall time limits (s)
readonly DISPLAY_TIMEOUT=10   # for Xvfb to start
readonly DRIVE_TIMEOUT=600    # for drive_for_video.py, which also waits for Gazebo to start
readonly STOP_TIMEOUT=60      # for roslaunch to stop the nodes, after SIGINT
readonly ENCODE_TIMEOUT=300   # for encode_drive_video.py

script_dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
out_dir=
work_dir=
ros_home=
tee_pid=
xvfb_pid=
launch_pid=
driver_pid=
encoder_pid=

log() {
    echo "record_drive_video.sh: $*"
}

fail() {
    log "error: $*" >&2
    exit 1
}

# Return whether the process runs. A child that has ended but isn't reaped yet (a zombie) counts
# as ended
process_running() {
    local state
    state=$(ps -o stat= -p "$1") || return 1
    [[ ${state// /} != Z* ]]
}

# Send the process a signal, give it some seconds to end, kill it if it hasn't, and reap it
end_process() {
    local pid=$1 signal=$2 seconds=$3 tenths=0
    kill -s "$signal" "$pid" 2> /dev/null || true
    while process_running "$pid" && ((tenths < seconds * 10)); do
        sleep 0.1
        tenths=$((tenths + 1))
    done
    if process_running "$pid"; then
        log "process $pid didn't end within $seconds s of SIG$signal, killing it"
        kill -s KILL "$pid" 2> /dev/null || true
    fi
    wait "$pid" 2> /dev/null || true
}

# Return whether something on this computer listens on the TCP port
port_in_use() {
    (: < "/dev/tcp/127.0.0.1/$1") 2> /dev/null
}

# Create the output folder, or check that the given one is empty
prepare_output() {
    out_dir=${1:-${ROS_HOME:-$HOME/.ros}/sim_video/$(date +%Y%m%d-%H%M%S)}
    if [[ -e $out_dir && -n $(ls -A "$out_dir") ]]; then
        fail "$out_dir isn't empty: give a new or empty folder, so it holds only this run's files"
    fi
    mkdir -p "$out_dir"
    out_dir=$(readlink -f "$out_dir")
}

# Start Xvfb on a display number of its own, with a new cookie (the secret that X programs need
# to use a display) that only this script's programs get through XAUTHORITY
start_display() {
    local display_number tries=0
    export XAUTHORITY=$work_dir/xauthority
    touch "$XAUTHORITY"
    for display_number in $(seq 99 199); do
        # Skip numbers in use. The dev container shares the host's X socket folder
        # /tmp/.X11-unix, but not its lock files /tmp/.X<n>-lock: Xvfb would replace the socket
        # of a host's display that has no lock file in the container
        if [[ -e /tmp/.X$display_number-lock || -e /tmp/.X11-unix/X$display_number ]]; then
            continue
        fi
        xauth -q -f "$XAUTHORITY" add ":$display_number" . "$(mcookie)"
        # setsid: Xvfb gets a session of its own, so a Ctrl+C in the terminal reaches only this
        # script, which then stops the simulation before the display. -displayfd 3: Xvfb writes
        # the display number to file descriptor 3 once it accepts programs
        setsid Xvfb ":$display_number" -auth "$XAUTHORITY" -screen 0 640x480x24 -nolisten tcp \
            -displayfd 3 3> "$work_dir/display_ready" >> "$out_dir/xvfb.log" 2>&1 &
        xvfb_pid=$!
        local tenths=0
        while [[ ! -s $work_dir/display_ready ]] && process_running "$xvfb_pid" \
            && ((tenths < DISPLAY_TIMEOUT * 10)); do
            sleep 0.1
            tenths=$((tenths + 1))
        done
        if [[ -s $work_dir/display_ready ]]; then
            export DISPLAY=:$display_number
            log "Xvfb runs on display $DISPLAY"
            return 0
        fi
        # Another X server may use the number without a socket file, through an abstract socket
        end_process "$xvfb_pid" KILL 5
        xvfb_pid=
        tries=$((tries + 1))
        ((tries < 5)) || break
    done
    fail "Xvfb didn't start, see $out_dir/xvfb.log"
}

start_simulation() {
    log "Starting Gazebo with lockstep and the controllers (launch/sim_video.launch)"
    # setsid, as for Xvfb. roslaunch starts the ROS master and each node in a session of its own
    setsid roslaunch -p "$ROS_PORT" diffbot_gazebo sim_video.launch model:="$MODEL" \
        world_name:="$work_dir/video.world" gazebo_port:="$GAZEBO_PORT" \
        extra_gazebo_args:=--lockstep > "$out_dir/simulation.log" 2>&1 &
    launch_pid=$!
}

# Stop the simulation: roslaunch stops its nodes, after SIGINT, escalating to SIGTERM and SIGKILL
# for each node that doesn't end. If roslaunch itself doesn't end, kill what it started: each of
# its processes runs in ROS_HOME, a folder of this run
stop_simulation() {
    if [[ -n $launch_pid ]]; then
        end_process "$launch_pid" INT "$STOP_TIMEOUT"
        launch_pid=
    fi
    [[ -n $ros_home ]] || return 0
    local proc
    for proc in /proc/[0-9]*; do
        if [[ $(readlink "$proc/cwd" 2> /dev/null) == "$ros_home" ]]; then
            log "killing ${proc#/proc/}, which roslaunch left running"
            kill -s KILL "${proc#/proc/}" 2> /dev/null || true
        fi
    done
}

# Runs at every exit: stop what this script started, delete the frames, and on failure delete
# the video and its JSON file, so a failed run leaves only its logs
cleanup() {
    local status=$?
    set +e
    if [[ -n $driver_pid ]]; then
        end_process "$driver_pid" TERM 10
    fi
    # timeout passes SIGTERM on to the encoder's whole process group, ffmpeg included
    if [[ -n $encoder_pid ]]; then
        end_process "$encoder_pid" TERM 10
    fi
    stop_simulation
    if [[ -n $xvfb_pid ]]; then
        end_process "$xvfb_pid" TERM 10
    fi
    if [[ -n $work_dir ]]; then
        rm -rf "$work_dir"
    fi
    if ((status != 0)) && [[ -n $out_dir ]]; then
        rm -f "$out_dir/$VIDEO_NAME.mp4" "$out_dir/$VIDEO_NAME.json"
        if [[ -s $out_dir/simulation.log ]]; then
            log "the end of simulation.log:"
            tail -n 20 "$out_dir/simulation.log"
        fi
        log "FAILED with exit status $status, no video. The logs are in $out_dir" >&2
    fi
    # Close the output to tee and give tee a moment to write the rest of it to record.log
    if [[ -n $tee_pid ]]; then
        exec 1>&- 2>&-
        local tenths=0
        while process_running "$tee_pid" && ((tenths < 50)); do
            sleep 0.1
            tenths=$((tenths + 1))
        done
    fi
    exit "$status"
}

main() {
    local tool
    for tool in Xvfb xauth mcookie setsid ffmpeg ffprobe roslaunch rospack python3; do
        command -v "$tool" > /dev/null || fail "$tool is missing, run this in the dev container"
    done
    prepare_output "${1:-}"
    trap cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    # Show the messages and also write them to record.log. tee ignores Ctrl+C and SIGTERM, which
    # also reach it, so it writes the cleanup's messages too, and ends when the script closes
    # its output
    exec > >(trap '' INT TERM && exec tee -a "$out_dir/record.log") 2>&1
    tee_pid=$!
    work_dir=$(mktemp -d)
    log "Recording $MODEL's drive into $out_dir"

    # This run's ROS graph and Gazebo server, reachable only from this computer
    export ROS_MASTER_URI=http://localhost:$ROS_PORT
    export ROS_HOSTNAME=localhost
    export GAZEBO_IP=127.0.0.1
    # ROS_HOME is where roslaunch runs the nodes and writes their logs
    ros_home=$work_dir/ros_home
    mkdir -p "$ros_home"
    export ROS_HOME=$ros_home
    # Mesa's software renderer, also on computers with a GPU, so the video looks the same
    export LIBGL_ALWAYS_SOFTWARE=1
    local port
    for port in "$ROS_PORT" "$GAZEBO_PORT"; do
        if port_in_use "$port"; then
            fail "port $port is in use: is another recording running?"
        fi
    done

    local package_dir
    package_dir=$(rospack find diffbot_gazebo)
    python3 "$script_dir/make_video_world.py" "$package_dir/worlds/diffbot_empty.world" \
        "$work_dir/video.world" --frames "$work_dir/frames" --width "$WIDTH" --height "$HEIGHT" \
        --rate "$RATE"
    start_display
    start_simulation

    # In the background, so a signal to this script ends the wait at once
    timeout --kill-after=10 "$DRIVE_TIMEOUT" python3 "$script_dir/drive_for_video.py" \
        --frames "$work_dir/frames" --pad "$PAD" --output "$work_dir/drive.json" &
    driver_pid=$!
    local driver_status=0
    wait "$driver_pid" || driver_status=$?
    driver_pid=
    ((driver_status == 0)) || fail "drive_for_video.py failed with exit status $driver_status"

    log "Stopping the simulation"
    stop_simulation
    # Like the driver: in the background, so a signal ends the wait at once. The video and its
    # JSON file go to the work folder first and into the output folder only when both are
    # complete, so a failed or interrupted encoding leaves no file of them in the output
    timeout --kill-after=10 "$ENCODE_TIMEOUT" python3 "$script_dir/encode_drive_video.py" \
        --frames "$work_dir/frames" --drive "$work_dir/drive.json" \
        --world "$work_dir/video.world" --model "$MODEL" --lockstep \
        --video "$work_dir/$VIDEO_NAME.mp4" --info "$work_dir/$VIDEO_NAME.json" &
    encoder_pid=$!
    local encoder_status=0
    wait "$encoder_pid" || encoder_status=$?
    encoder_pid=
    ((encoder_status == 0)) || fail "encode_drive_video.py failed with exit status $encoder_status"
    mv "$work_dir/$VIDEO_NAME.mp4" "$work_dir/$VIDEO_NAME.json" "$out_dir/"
    log "Done: $out_dir/$VIDEO_NAME.mp4"
}

main "$@"
