#!/usr/bin/env bash
# ============================================================
#  Быстрый запуск симуляции Fito (грядки + автопилот)
#  Использование:
#    ./run.sh             — собрать и запустить
#    ./run.sh --no-build  — только запустить (без пересборки)
#    ./run.sh --autopilot — запустить и сразу включить автопилот
# ============================================================

WS="$(cd "$(dirname "$0")" && pwd)"
ROS_SETUP=/opt/ros/jazzy/setup.sh

source "$ROS_SETUP"

# ── Зависимости ─────────────────────────────────────────────
export GZ_CONFIG_PATH=/usr/share/gz

if ! python3 -c "import cv_bridge" 2>/dev/null; then
  echo "Устанавливаю ros-jazzy-cv-bridge..."
  sudo apt install -y ros-jazzy-cv-bridge
fi

# ── Флаги ───────────────────────────────────────────────────
BUILD=true
AUTO=false
for arg in "$@"; do
  case $arg in
    --no-build)  BUILD=false ;;
    --autopilot) AUTO=true   ;;
  esac
done

# ── Сборка ──────────────────────────────────────────────────
if [ "$BUILD" = true ]; then
  echo ""
  echo "╔══════════════════════════════════════╗"
  echo "║  Сборка пакетов...                   ║"
  echo "╚══════════════════════════════════════╝"
  cd "$WS"
  if ! colcon build --packages-select my_robot visual_multi_crop_row_navigation \
      2>&1 | grep -E "(Starting|Finished|ERROR|WARNING|failed)"; then
    echo "✗ Ошибка сборки"
    exit 1
  fi
  # colcon возвращает 0 даже при ошибках — проверяем явно
  if [ ! -f "$WS/install/my_robot/share/my_robot/launch/test.launch.py" ]; then
    echo "✗ Сборка не завершена (файлы не установлены)"
    exit 1
  fi
  echo "✓ Сборка завершена"
fi

source "$WS/install/setup.bash"

# ── Включить автопилот после старта (в фоне) ────────────────
AUTO_PID=""
if [ "$AUTO" = true ]; then
  (
    echo "⏳ Жду запуска симуляции (20 сек)..."
    sleep 20
    echo "🚀 Включаю автопилот..."
    ros2 topic pub /autopilot/enable std_msgs/msg/Bool "data: true" --once 2>/dev/null \
      && echo "✓ Автопилот включён" \
      || echo "✗ Не удалось включить автопилот (симуляция не запущена?)"
  ) &
  AUTO_PID=$!
fi

# ── Очистка при выходе ───────────────────────────────────────
cleanup() {
  if [ -n "$AUTO_PID" ]; then
    kill "$AUTO_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ── Запуск ──────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "║  Запуск симуляции...                         ║"
echo "║                                              ║"
echo "║  Топики автопилота в RViz:                   ║"
echo "║    /vs_nav/graphic  — детектор рядов         ║"
echo "║    /vs_nav/mask     — маска зелени           ║"
echo "║    /vs_nav/ExG      — ExG индекс             ║"
echo "║    /scan            — LiDAR                  ║"
echo "║    /odom            — одометрия              ║"
echo "║                                              ║"
echo "║  Включить автопилот вручную:                 ║"
echo "║    ros2 topic pub /autopilot/enable \\        ║"
echo "║      std_msgs/msg/Bool 'data: true' --once   ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

ros2 launch my_robot test.launch.py
