# Build environment for the Teensy firmware in diffbot_firmware: PlatformIO Core and a compiler
# for the PC, independent of ROS and of the robot image. The boards' toolchains aren't in the
# image: PlatformIO downloads them into its core folder (/pio-core) on the first build.
# Use it through compose.yaml in this folder, which mounts the repository.
FROM python:3.12-slim-bookworm

# A C/C++ compiler for builds on the PC (native tests), Git for PlatformIO's libraries
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        git \
    && rm -rf /var/lib/apt/lists/*

# PlatformIO Core, in the version that CI used before (see requirements.txt)
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# A user with your user and group ID, so that the build's files in the mounted repository belong
# to your user. compose.yaml passes the two IDs as the build arguments USER_UID and USER_GID,
# from LOCAL_UID and LOCAL_GID, which run.sh sets to `id -u` and `id -g`. USER_UID and USER_GID
# have no default, so a build without both stops instead of silently using 1000. The user's home
# and the core folder belong to it, so a new volume mounted at /pio-core is writable too.
ARG USER_UID
ARG USER_GID
RUN if [ -z "$USER_UID" ] || [ -z "$USER_GID" ]; then \
        echo "USER_UID and USER_GID are missing: build through compose.yaml or run.sh" >&2; \
        exit 1; \
    fi
RUN (getent group "$USER_GID" || groupadd --gid "$USER_GID" pio) \
    && useradd --non-unique --uid "$USER_UID" --gid "$USER_GID" --create-home pio \
    && mkdir /pio-core \
    && chown "$USER_UID:$USER_GID" /pio-core
ENV PLATFORMIO_CORE_DIR=/pio-core
USER pio
WORKDIR /workspace/diffbot_firmware
