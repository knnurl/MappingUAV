#!/usr/bin/env bash
# filepath: scripts/run_qgc.sh
# Launch QGroundControl on the Jetson inside the Ubuntu 24.04 container
# (docker/qgc/Dockerfile). Build once:  ./scripts/run_qgc.sh --build
set -e

IMAGE=qgc:5.1.4

# A taskbar launcher has no terminal to type a sudo password into, so use
# docker directly when this user is in the docker group and fall back to sudo
# for a plain shell run. Run once to make the launcher work:
#   sudo usermod -aG docker $USER   (then log out and back in)
if docker info >/dev/null 2>&1; then DOCKER=(docker); else DOCKER=(sudo docker); fi

# -t only when there is a terminal: from the taskbar launcher there is none,
# and "docker run -t" fails with "the input device is not a TTY".
if [ -t 0 ]; then TTY_ARGS=(-it); else TTY_ARGS=(-i); fi
QGC_HOME="${HOME}/.qgc-docker"     # QGC settings, saved params, firmware cache

if [ "$1" = "--build" ]; then
    "${DOCKER[@]}" build -t "${IMAGE}" "$(dirname "$0")/../docker/qgc"
    exit 0
fi

mkdir -p "${QGC_HOME}"

# Hardware GL: the Tegra driver (libGLX_nvidia) lives on the host and is
# injected by the nvidia runtime's CSV mode. Without it Mesa cannot get a
# context in the container ("failed to create drawable"), so fall back to
# LIBGL_ALWAYS_SOFTWARE=1 ./scripts/run_qgc.sh if this misbehaves.
GPU_ARGS=()
if [ "${QGC_NO_GPU:-0}" != "1" ]; then
    GPU_ARGS=(--runtime nvidia
              -e NVIDIA_VISIBLE_DEVICES=all
              -e NVIDIA_DRIVER_CAPABILITIES=all)
fi

# X11: the container runs as uid 1000, same as this user, so grant that user
# and hand it the session's cookie (gdm keeps it outside $HOME).
XAUTH="${XAUTHORITY:-${HOME}/.Xauthority}"
xhost "+SI:localuser:$(id -un)" >/dev/null

# --privileged + full /dev bind (NOT --device=/dev/ttyACM0): during firmware
# flash the FC reboots into its bootloader and re-enumerates as a different USB
# device. A device mapping pinned at container start goes stale mid-upload and
# the flash fails partway. /run/udev lets QtSerialPort enumerate ports by name.
"${DOCKER[@]}" run --rm "${TTY_ARGS[@]}" \
    --privileged \
    "${GPU_ARGS[@]}" \
    -v /dev:/dev \
    -v /run/udev:/run/udev:ro \
    -e DISPLAY="${DISPLAY}" \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-}" \
    -e XAUTHORITY=/tmp/.Xauthority \
    -v "${XAUTH}":/tmp/.Xauthority:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${QGC_HOME}":/qgc-home \
    --network host \
    "${IMAGE}" "$@"

xhost "-SI:localuser:$(id -un)" >/dev/null
