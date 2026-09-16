# Ubuntu 16.04 (Xenial) 对应 ROS1 Kinetic。
# 该镜像仅用于验证遗留 ROS1 环境的容器化可行性，不包含 Qt 和业务源码。
FROM ros:kinetic-ros-base-xenial

LABEL org.opencontainers.image.title="ROS1 Kinetic feasibility test" \
      org.opencontainers.image.description="Ubuntu 16.04 with ROS1 Kinetic ros-base"

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV ROS_DISTRO=kinetic \
    ROS_ROOT=/opt/ros/kinetic \
    CATKIN_WS=/root/catkin_ws

WORKDIR ${CATKIN_WS}

# 构建镜像时完成环境断言，并验证 catkin 工作空间可以正常配置和编译。
RUN mkdir -p src \
    && grep -q '^VERSION_ID="16.04"$' /etc/os-release \
    && source "${ROS_ROOT}/setup.bash" \
    && test "$(rosversion -d)" = "${ROS_DISTRO}" \
    && command -v roscore >/dev/null \
    && command -v catkin_make >/dev/null \
    && catkin_make

# 基础镜像自带 /ros_entrypoint.sh，会在启动命令前加载 ROS 环境。
EXPOSE 11311
CMD ["roscore"]
