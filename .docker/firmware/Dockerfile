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

# A user with your user and group ID (compose.yaml passes them), so that the build's files in the
# mounted repository belong to you. Its home and the core folder belong to it, so a new volume
# mounted at /pio-core is writable too.
ARG USER_UID=1000
ARG USER_GID=1000
RUN (getent group "$USER_GID" || groupadd --gid "$USER_GID" pio) \
    && useradd --non-unique --uid "$USER_UID" --gid "$USER_GID" --create-home pio \
    && mkdir /pio-core \
    && chown "$USER_UID:$USER_GID" /pio-core
ENV PLATFORMIO_CORE_DIR=/pio-core
USER pio
WORKDIR /workspace/diffbot_firmware
