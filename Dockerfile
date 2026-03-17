FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Основные зависимости
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-joy \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-mapviz \
    ros-jazzy-mapviz-plugins \
    ros-jazzy-tile-map \
    libgl1 \
    libgl1-mesa-dri \
    mesa-utils \
    x11-apps \
    python3-future \
    python3-shapely \
    && rm -rf /var/lib/apt/lists/*

# Настройка рабочего пространства
WORKDIR /ros2_ws

# Копируем исходники и собираем
COPY src/ src/

RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Entrypoint
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
