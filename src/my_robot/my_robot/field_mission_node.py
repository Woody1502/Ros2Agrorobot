#!/usr/bin/env python3
"""
Field Mission Node — полностью автономный обход поля.

Режимы:
  FULL_MISSION  — бустрофедон по всем рядам
  GOTO_TEST     — подъехать к одной точке в odom-фрейме (только для теста)

Топики управления:
  /mission/start   (std_msgs/Bool)             — True = начать FULL_MISSION
  /mission/goto    (geometry_msgs/Point)       — GOTO_TEST: целевая точка в odom
  /mission/stop    (std_msgs/Bool)             — экстренная остановка

Публикует статус:
  /mission/status  (std_msgs/String)

Соглашение о знаках руля (acker_odom):
  positive steering → turn_radius > 0 → angular_velocity > 0
  theta -= angular_velocity*dt  → theta уменьшается → ПРАВЫЙ поворот
  → ОТРИЦАТЕЛЬНАЯ команда руля = левый поворот (к +y, к следующему ряду при движении +x)
  Если после теста оказалось наоборот: параметр steer_sign_flip = -1
"""

import math
import rclpy
from enum import IntEnum
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


# ──────────────────────────── состояния ───────────────────────────────────────

class State(IntEnum):
    IDLE            = 0
    GOTO_TEST       = 1   # только для теста: едем к одной точке
    ROW_FOLLOWING   = 2   # VS активен, row_driver активен
    END_BRAKING     = 3   # тормозим в конце ряда
    HEADLAND_0      = 4   # в headland: прямо (headland_depth м)
    HEADLAND_1      = 5   # разворот 90° к следующему ряду
    HEADLAND_2      = 6   # прямо на row_spacing м (к следующему ряду)
    HEADLAND_3      = 7   # разворот 90° вдоль рядов
    ROW_ENTRY       = 8   # ждём захват VS
    MISSION_COMPLETE = 9
    EMERGENCY_STOP  = 10


# ──────────────────────────── нода ────────────────────────────────────────────

class FieldMissionNode(Node):

    # ── конструктор ───────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('field_mission')

        # ── параметры поля ────────────────────────────────────────────────────
        self.declare_parameter('num_rows',          5)
        self.declare_parameter('row_spacing',       2.0)    # м между рядами
        self.declare_parameter('row_length',        26.5)   # м от старта до конца ряда
        self.declare_parameter('end_margin',        3.0)    # м до конца ряда — тормозим
        self.declare_parameter('headland_depth',    9.0)    # м в headland перед разворотом
        self.declare_parameter('row_entry_timeout', 8.0)    # сек ожидания захвата VS

        # ── скорости ──────────────────────────────────────────────────────────
        self.declare_parameter('nav_speed',         1.0)    # рад/с колёса — в headland
        self.declare_parameter('turn_speed',        0.6)    # рад/с — при повороте
        self.declare_parameter('entry_speed',       0.5)    # рад/с — при входе в ряд

        # ── рулевые параметры ─────────────────────────────────────────────────
        # steer_sign_flip: 1 = нормально, -1 = инвертировать если робот едет не туда
        self.declare_parameter('steer_sign_flip',   1)
        self.declare_parameter('max_steer',         0.28)
        self.declare_parameter('goto_steer_gain',   1.0)    # пропорц. коэф. для GOTO
        self.declare_parameter('goto_tolerance',    0.5)    # м — считаем точку достигнутой
        self.declare_parameter('goto_speed',        0.8)    # рад/с — в GOTO режиме

        # ── загрузка параметров ───────────────────────────────────────────────
        self.num_rows          = self.get_parameter('num_rows').value
        self.row_spacing       = self.get_parameter('row_spacing').value
        self.row_length        = self.get_parameter('row_length').value
        self.end_margin        = self.get_parameter('end_margin').value
        self.headland_depth    = self.get_parameter('headland_depth').value
        self.row_entry_timeout = self.get_parameter('row_entry_timeout').value
        self.nav_speed         = self.get_parameter('nav_speed').value
        self.turn_speed        = self.get_parameter('turn_speed').value
        self.entry_speed       = self.get_parameter('entry_speed').value
        self.steer_sign_flip   = self.get_parameter('steer_sign_flip').value
        self.max_steer         = self.get_parameter('max_steer').value
        self.goto_steer_gain   = self.get_parameter('goto_steer_gain').value
        self.goto_tolerance    = self.get_parameter('goto_tolerance').value
        self.goto_speed        = self.get_parameter('goto_speed').value

        # ── состояние FSM ─────────────────────────────────────────────────────
        self.state         = State.IDLE
        self.current_row   = 0          # 0-based
        # +1 = движемся по +x (чётные ряды), -1 = по -x (нечётные)
        self.current_dir   = 1
        self.state_timer   = 0.0        # сколько секунд в текущем состоянии

        # ── одометрия ─────────────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # позиция при старте текущего ряда / шага
        self.step_x0     = 0.0
        self.step_y0     = 0.0
        self.step_theta0 = 0.0

        # ── детекция конца ряда ───────────────────────────────────────────────
        self.row_end_frames  = 0       # кадров подряд с сигналом от VS
        self.ROW_END_CONFIRM = 5       # сколько нужно подряд

        # ── GOTO тест ─────────────────────────────────────────────────────────
        self.goto_target = None        # Point

        # ── подписки ──────────────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',         self._odom_cb,    10)
        self.create_subscription(Bool,     '/vs_nav/row_end', self._row_end_cb, 10)
        self.create_subscription(Bool,     '/mission/start', self._start_cb,   10)
        self.create_subscription(Bool,     '/mission/stop',  self._stop_cb,    10)
        self.create_subscription(Point,    '/mission/goto',  self._goto_cb,    10)

        # ── публикации ────────────────────────────────────────────────────────
        self.steer_pub   = self.create_publisher(Float64MultiArray,
                                                 '/position_controller/commands', 10)
        self.vel_pub     = self.create_publisher(Float64MultiArray,
                                                 '/velocity_controller/commands', 10)
        self.enable_pub  = self.create_publisher(Bool,   '/autopilot/enable',   10)
        self.vs_pub      = self.create_publisher(Bool,   '/mission/vs_active',  10)
        self.status_pub  = self.create_publisher(String, '/mission/status',     10)

        # ── главный цикл 10 Гц ────────────────────────────────────────────────
        self.DT = 0.1
        self.create_timer(self.DT, self._update)

        self.get_logger().info('[MISSION] Ready. Send True to /mission/start or '
                               'geometry_msgs/Point to /mission/goto for test.')

    # ── колбэки ───────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x     = msg.pose.pose.position.x
        self.y     = msg.pose.pose.position.y
        q          = msg.pose.pose.orientation
        siny_cosp  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def _row_end_cb(self, msg: Bool):
        if msg.data:
            self.row_end_frames += 1
        else:
            self.row_end_frames = 0

    def _start_cb(self, msg: Bool):
        if msg.data and self.state == State.IDLE:
            self.get_logger().info('[MISSION] Starting full field coverage.')
            self._start_row()

    def _stop_cb(self, msg: Bool):
        if msg.data:
            self._emergency_stop()

    def _goto_cb(self, msg: Point):
        """Тест: подъехать к точке (x, y) в odom-фрейме."""
        if self.state not in (State.IDLE, State.GOTO_TEST):
            self.get_logger().warn('[MISSION] Cannot accept goto — not in IDLE.')
            return
        self.goto_target = msg
        self._transition(State.GOTO_TEST)
        self.get_logger().info(
            f'[MISSION] GOTO_TEST → target odom ({msg.x:.2f}, {msg.y:.2f})')

    # ── главный цикл ──────────────────────────────────────────────────────────

    def _update(self):
        self.state_timer += self.DT

        if self.state == State.IDLE:
            pass

        elif self.state == State.GOTO_TEST:
            self._tick_goto_test()

        elif self.state == State.ROW_FOLLOWING:
            self._tick_row_following()

        elif self.state == State.END_BRAKING:
            self._tick_end_braking()

        elif self.state in (State.HEADLAND_0, State.HEADLAND_1,
                            State.HEADLAND_2, State.HEADLAND_3):
            self._tick_headland()

        elif self.state == State.ROW_ENTRY:
            self._tick_row_entry()

        elif self.state == State.MISSION_COMPLETE:
            pass  # уже остановлены

    # ── GOTO_TEST ─────────────────────────────────────────────────────────────

    def _tick_goto_test(self):
        """Простой пропорциональный регулятор: едем к goto_target."""
        if self.goto_target is None:
            return

        tx, ty = self.goto_target.x, self.goto_target.y
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        # Логирование 1 Гц
        if int(self.state_timer * 10) % 10 == 0:
            self.get_logger().info(
                f'[GOTO] pos=({self.x:.2f},{self.y:.2f}) '
                f'target=({tx:.2f},{ty:.2f}) dist={dist:.2f}m '
                f'theta={math.degrees(self.theta):.1f}°')

        if dist < self.goto_tolerance:
            self._cmd_wheels(0.0)
            self._cmd_steer(0.0)
            self._publish_status('GOTO_REACHED')
            self.get_logger().info(
                f'[GOTO] Reached ({tx:.2f},{ty:.2f})! '
                f'Final pos: ({self.x:.2f},{self.y:.2f})')
            self._transition(State.IDLE)
            return

        target_heading = math.atan2(dy, dx)
        angle_err = self._norm_angle(target_heading - self.theta)

        steer = self.steer_sign_flip * max(-self.max_steer,
                                          min(self.max_steer,
                                              self.goto_steer_gain * angle_err))
        speed = self.goto_speed
        self._cmd_steer(steer)
        self._cmd_wheels(speed)

    # ── ROW_FOLLOWING ─────────────────────────────────────────────────────────

    MIN_ROW_DIST = 1.5  # м — игнорируем VS row_end пока не проехали хотя бы столько

    def _tick_row_following(self):
        dist_in_row = self._dist_from_step()
        row_end_by_odom = dist_in_row >= (self.row_length - self.end_margin)
        row_end_by_vs   = (self.row_end_frames >= self.ROW_END_CONFIRM
                           and dist_in_row >= self.MIN_ROW_DIST)

        if row_end_by_odom or row_end_by_vs:
            reason = 'odom' if row_end_by_odom else 'VS'
            self.get_logger().info(
                f'[MISSION] Row {self.current_row} end detected ({reason}), '
                f'dist={dist_in_row:.1f}m')
            self._transition(State.END_BRAKING)

    # ── END_BRAKING ───────────────────────────────────────────────────────────

    def _tick_end_braking(self):
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)
        self._set_vs(False)
        self._set_drive(False)

        # 1 секунда чтобы остановиться, потом в headland
        if self.state_timer >= 1.0:
            self._transition(State.HEADLAND_0)
            self.get_logger().info('[MISSION] Starting headland sequence.')

    # ── HEADLAND ──────────────────────────────────────────────────────────────

    def _tick_headland(self):
        """
        Трёхшаговый разворот (square turn):
          HEADLAND_0: прямо headland_depth м (в headland)
          HEADLAND_1: поворот 90° к следующему ряду
          HEADLAND_2: прямо row_spacing м (к следующему ряду)
          HEADLAND_3: поворот 90° вдоль рядов (в обратном направлении)

        Знак руля при повороте:
          Движение +x (dir=+1): нужен левый поворот → steer < 0
          Движение -x (dir=-1): нужен правый поворот → steer > 0
          Итого: turn_steer = -max_steer * current_dir * steer_sign_flip
        """
        turn_steer = self.max_steer * self.current_dir * self.steer_sign_flip
        TURN_DONE_DEG = 82.0  # немного меньше 90 чтобы не перерегулировать

        if self.state == State.HEADLAND_0:
            dist = self._dist_from_step()
            if dist >= self.headland_depth:
                self._transition(State.HEADLAND_1)
            else:
                self._cmd_steer(0.0)
                self._cmd_wheels(self.nav_speed)

        elif self.state == State.HEADLAND_1:
            turned = abs(self._heading_change_from_step())
            if turned >= math.radians(TURN_DONE_DEG):
                self._transition(State.HEADLAND_2)
            else:
                self._cmd_steer(turn_steer)
                self._cmd_wheels(self.turn_speed)

        elif self.state == State.HEADLAND_2:
            dist = self._dist_from_step()
            if dist >= self.row_spacing:
                self._transition(State.HEADLAND_3)
            else:
                self._cmd_steer(0.0)
                self._cmd_wheels(self.nav_speed)

        elif self.state == State.HEADLAND_3:
            turned = abs(self._heading_change_from_step())
            if turned >= math.radians(TURN_DONE_DEG):
                self._finish_headland()
            else:
                self._cmd_steer(turn_steer)
                self._cmd_wheels(self.turn_speed)

    def _finish_headland(self):
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)

        self.current_row += 1
        self.current_dir *= -1   # бустрофедон: чередуем направление
        self.row_end_frames = 0

        if self.current_row >= self.num_rows:
            self._transition(State.MISSION_COMPLETE)
            self.get_logger().info('[MISSION] All rows completed!')
            self._publish_status('MISSION_COMPLETE')
            return

        self.get_logger().info(
            f'[MISSION] Starting row {self.current_row} '
            f'(dir={self.current_dir:+d})')
        self._transition(State.ROW_ENTRY)

    # ── ROW_ENTRY ─────────────────────────────────────────────────────────────

    def _tick_row_entry(self):
        """Активируем VS и едем медленно, ждём захвата ряда."""
        self._set_vs(True)
        self._cmd_wheels(self.entry_speed)

        if self.state_timer >= self.row_entry_timeout:
            self.get_logger().warn(
                f'[MISSION] ROW_ENTRY timeout on row {self.current_row}. '
                'Proceeding anyway (VS may still acquire).')
            self._start_row()

    # ── вспомогательные методы ────────────────────────────────────────────────

    def _start_row(self):
        """Запустить следующий ряд: активировать drive + VS."""
        self._set_step_origin()
        self._set_vs(True)
        self._set_drive(True)
        self._transition(State.ROW_FOLLOWING)
        self._publish_status(f'ROW_FOLLOWING row={self.current_row}')
        self.get_logger().info(
            f'[MISSION] ROW_FOLLOWING row={self.current_row} dir={self.current_dir:+d}')

    def _emergency_stop(self):
        self._cmd_wheels(0.0)
        self._cmd_steer(0.0)
        self._set_vs(False)
        self._set_drive(False)
        self._transition(State.EMERGENCY_STOP)
        self.get_logger().error('[MISSION] EMERGENCY STOP')

    def _transition(self, new_state: State):
        old = State(self.state).name
        self.state = new_state
        self.state_timer = 0.0
        self._set_step_origin()
        self._publish_status(new_state.name)
        self.get_logger().info(f'[MISSION] {old} → {new_state.name}')

    def _set_step_origin(self):
        """Запомнить позицию/курс в начале шага для отслеживания прогресса."""
        self.step_x0     = self.x
        self.step_y0     = self.y
        self.step_theta0 = self.theta

    def _dist_from_step(self) -> float:
        return math.hypot(self.x - self.step_x0, self.y - self.step_y0)

    def _heading_change_from_step(self) -> float:
        return self._norm_angle(self.theta - self.step_theta0)

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── низкоуровневые команды ────────────────────────────────────────────────

    def _cmd_steer(self, angle: float):
        msg = Float64MultiArray()
        msg.data = [float(angle)]
        self.steer_pub.publish(msg)

    def _cmd_wheels(self, speed: float):
        msg = Float64MultiArray()
        msg.data = [float(speed)] * 4
        self.vel_pub.publish(msg)

    def _set_drive(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)

    def _set_vs(self, active: bool):
        msg = Bool()
        msg.data = active
        self.vs_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = f'[row={self.current_row} dir={self.current_dir:+d}] {text}'
        self.status_pub.publish(msg)


# ── точка входа ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FieldMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
