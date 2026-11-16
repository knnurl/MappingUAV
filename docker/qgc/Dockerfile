# filepath: docker/qgc/Dockerfile
# QGroundControl 5.1.x (aarch64) on Ubuntu 24.04 — the AppImage needs glibc 2.38,
# the Jetson host (JetPack 6) has 2.35. Container supplies the newer glibc only;
# the FC is reached through the host's /dev, so this is still a local GCS.
#
# The AppImage is extracted at build time rather than FUSE-mounted at runtime:
# no libfuse2 in the container, and no --cap-add needed for the mount.
FROM ubuntu:24.04

ARG QGC_VERSION=v5.1.4
ARG QGC_URL=https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/QGroundControl-aarch64.AppImage

ENV DEBIAN_FRONTEND=noninteractive

# The 188 MB download is its own early layer so that iterating on the runtime
# dependency list below does not re-fetch it.
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl \
    && curl -fL --retry 3 -o /tmp/qgc.AppImage "${QGC_URL}" \
    && chmod +x /tmp/qgc.AppImage \
    && cd /opt && /tmp/qgc.AppImage --appimage-extract >/dev/null \
    && mv /opt/squashfs-root /opt/qgc \
    && rm /tmp/qgc.AppImage \
    && rm -rf /var/lib/apt/lists/*

# Qt6 platform + GL + audio runtime deps. The AppImage bundles Qt, GStreamer and
# libudev itself; these are the host-side libs it links against.
# The libxcb-glx/dri/present/xshmfence/libdrm set is what Mesa needs to create a
# GLX drawable — without them even LIBGL_ALWAYS_SOFTWARE=1 fails with
# "failed to create drawable".
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglx-mesa0 libegl1 libopengl0 libgl1-mesa-dri libglu1-mesa \
        libxcb-glx0 libxcb-dri2-0 libxcb-dri3-0 libxcb-present0 \
        libxcb-cursor0 libxcb-xinerama0 libxcb-icccm4 libxcb-image0 \
        libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
        libxcb-sync1 libxcb-xfixes0 libxcb-xkb1 libxkbcommon-x11-0 \
        libx11-xcb1 libxshmfence1 libdrm2 libxext6 libxdamage1 libxfixes3 \
        libxrandr2 libxi6 libxrender1 libsm6 libice6 \
        libdbus-1-3 libpulse0 libasound2t64 \
        fontconfig fonts-dejavu-core libfreetype6 \
    && rm -rf /var/lib/apt/lists/*

# QGC refuses to start as root, so it runs as a user matching the host's
# neric (1000:1000). Ubuntu 24.04 images ship their own uid-1000 "ubuntu"
# user, which has to go first.
#   dialout (20) -> /dev/ttyACM0, the flight controller
#   video   (44) -> /dev/nvmap + /dev/nvhost-*, the Tegra GPU. Without this the
#                   nvidia runtime injects the driver but it dies with
#                   "NvRmMemInitNvmap failed with Permission denied".
#   render (104) -> /dev/dri/renderD*
# GIDs are the host's; -o allows them to collide with the image's own groups.
ARG UID=1000
ARG GID=1000
ARG VIDEO_GID=44
ARG RENDER_GID=104

RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd -g ${GID} qgc \
    && groupadd -o -g ${VIDEO_GID} hostvideo \
    && groupadd -o -g ${RENDER_GID} hostrender \
    && useradd -u ${UID} -g ${GID} -G dialout,hostvideo,hostrender \
               -d /qgc-home -s /bin/bash qgc

# Settings/params/firmware cache live on a host volume so they survive the container.
ENV HOME=/qgc-home
# Qt warns and falls back if the locale is not UTF-8.
ENV LANG=C.UTF-8
RUN mkdir -p /qgc-home && chown ${UID}:${GID} /qgc-home

USER qgc
ENTRYPOINT ["/opt/qgc/AppRun"]
