#!/bin/bash
# Запуск симуляции ROS2 из Docker с отображением GUI на рабочем столе

IMAGE_NAME="ros2_agrorobot"
CONTAINER_NAME="agrorobot_sim"
WS_DIR="$(cd "$(dirname "$0")" && pwd)"

# Разрешить Docker использовать X11
xhost +local:docker

# Проверить наличие NVIDIA GPU
DOCKER_GPU_ARGS=""
if command -v nvidia-smi &>/dev/null; then
    echo "[INFO] NVIDIA GPU обнаружен, включаем поддержку GPU"
    DOCKER_GPU_ARGS="--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all"
fi

# Проверить/собрать образ
if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "[INFO] Образ не найден, собираем..."
    docker build -t "$IMAGE_NAME" "$WS_DIR"
fi

# Удалить предыдущий контейнер если есть
docker rm -f "$CONTAINER_NAME" 2>/dev/null || true

echo "[INFO] Запуск контейнера с симуляцией..."

docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --env DISPLAY="$DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_INDIRECT=0 \
    --env XDG_RUNTIME_DIR=/tmp/runtime-root \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$WS_DIR/src":/ros2_ws/src:ro \
    --volume "$WS_DIR/ros2_ws_build":/ros2_ws/build \
    --volume "$WS_DIR/ros2_ws_install":/ros2_ws/install \
    --volume "$WS_DIR/ros2_ws_log":/ros2_ws/log \
    --device /dev/dri \
    --device /dev/input \
    $DOCKER_GPU_ARGS \
    "$IMAGE_NAME" \
    bash -c "cd /ros2_ws && \
             colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5 && \
             source install/setup.bash && \
             ros2 launch my_robot test.launch.py"
