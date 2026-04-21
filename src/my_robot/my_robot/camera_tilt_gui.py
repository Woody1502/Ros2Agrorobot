#!/usr/bin/env python3
"""
camera_tilt_gui.py — Camera tilt, ROI, Drive and Row Transition GUI.

Row transition maneuver (two-arc, gyroscope-tracked):
  Arc 1 (forward):  steer at max angle, drive until IMU reports ~α degrees of turn
  Arc 2 (backward): opposite steer, reverse until total heading change = 180°

  Geometry (center-pivot steering):
    α = arccos(−row_spacing / (2 · turn_radius))
    β = π − α
"""
import threading
import time
import math
import tkinter as tk
import yaml
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import Imu

# ── constants ─────────────────────────────────────────────────────────────────

IMG_W = 1280
IMG_H = 720
PARAMS_PATH = (
    '/ros2_ws/install/visual_multi_crop_row_navigation/share/'
    'visual_multi_crop_row_navigation/configs/params.yaml'
)

DEF_LEFT_BOT  = 150
DEF_LEFT_TOP  = 50
DEF_RIGHT_BOT = 150
DEF_RIGHT_TOP = 50

# Robot geometry from URDF
REAR_TO_PIVOT = 1.863   # m: rear axle → steering joint
PIVOT_TO_AXLE = 0.585   # m: steering joint → front axle


# ── geometry helpers ──────────────────────────────────────────────────────────

def steer_to_radius(steer_angle_rad):
    """Exact turn radius R for this robot's center-pivot mechanism."""
    a = abs(steer_angle_rad)
    if a < 1e-6:
        return float('inf')
    return (PIVOT_TO_AXLE + REAR_TO_PIVOT * math.cos(a)) / math.sin(a)


def compute_arcs(row_spacing, turn_radius):
    """
    Return (alpha_deg, beta_deg) for two-arc headland maneuver.
    α = arccos(−d / 2R),  β = 180° − α
    """
    if turn_radius <= 0 or row_spacing <= 0:
        return 90.0, 90.0
    val = max(-1.0, min(1.0, -row_spacing / (2.0 * turn_radius)))
    alpha = math.acos(val)
    return math.degrees(alpha), math.degrees(math.pi - alpha)


# ── ROI preview helpers ───────────────────────────────────────────────────────

def _roi_polygons(lb, lt, rb, rt):
    right = [[IMG_W - rt, 0], [IMG_W, 0], [IMG_W, IMG_H], [IMG_W - rb, IMG_H]]
    left  = [[0, 0], [lt, 0], [lb, IMG_H], [0, IMG_H]]
    return right, left


def save_roi_to_file(lb, lt, rb, rt, enable):
    if not os.path.exists(PARAMS_PATH):
        return False, f'File not found:\n{PARAMS_PATH}'
    try:
        with open(PARAMS_PATH, 'r') as f:
            data = yaml.safe_load(f)
        p = data['/**']['ros__parameters']
        p['enable_roi'] = bool(enable)
        right, left = _roi_polygons(lb, lt, rb, rt)
        p['p1'], p['p2'], p['p3'], p['p4'] = right
        p['p5'], p['p6'], p['p7'], p['p8'] = left
        with open(PARAMS_PATH, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        return True, 'Saved to file'
    except Exception as e:
        return False, str(e)


CANVAS_W = 400
CANVAS_H = 80

def _scale(x, y):
    return int(x * CANVAS_W / IMG_W), int(y * CANVAS_H / IMG_H)


def draw_roi_preview(canvas, lb, lt, rb, rt, enable):
    canvas.delete('all')
    canvas.create_rectangle(0, 0, CANVAS_W, CANVAS_H, fill='#1a1a1a', outline='')
    if not enable:
        canvas.create_rectangle(0, 0, CANVAS_W, CANVAS_H, fill='#2ecc71', outline='')
        canvas.create_text(CANVAS_W // 2, CANVAS_H // 2,
                           text='ROI disabled — full frame active',
                           fill='white', font=('Arial', 9, 'bold'))
        return
    tl = _scale(lt,         0)
    tr = _scale(IMG_W - rt, 0)
    br = _scale(IMG_W - rb, IMG_H)
    bl = _scale(lb,         IMG_H)
    canvas.create_polygon([0,0, tl[0],0, bl[0],CANVAS_H, 0,CANVAS_H],       fill='#922b21', outline='')
    canvas.create_polygon([tr[0],0, CANVAS_W,0, CANVAS_W,CANVAS_H, br[0],CANVAS_H], fill='#922b21', outline='')
    canvas.create_polygon([tl[0],0, tr[0],0, br[0],CANVAS_H, bl[0],CANVAS_H], fill='#1e8449', outline='')
    canvas.create_line(tl[0], 0, bl[0], CANVAS_H, fill='#58d68d', width=1, dash=(3,3))
    canvas.create_line(tr[0], 0, br[0], CANVAS_H, fill='#58d68d', width=1, dash=(3,3))
    vis_top = IMG_W - lt - rt
    vis_bot = IMG_W - lb - rb
    canvas.create_text(CANVAS_W//2, 10,          text=f'far:  {max(vis_top,0)} px', fill='#abebc6', font=('Arial', 8))
    canvas.create_text(CANVAS_W//2, CANVAS_H-10, text=f'near: {max(vis_bot,0)} px', fill='#abebc6', font=('Arial', 8))
    canvas.create_line(0, 1,          CANVAS_W, 1,          fill='#555', width=1)
    canvas.create_line(0, CANVAS_H-1, CANVAS_W, CANVAS_H-1, fill='#555', width=1)


# ── ROS node ──────────────────────────────────────────────────────────────────

class CameraTiltGui(Node):

    def __init__(self):
        super().__init__('camera_tilt_gui')

        self.tilt_pub  = self.create_publisher(Float64MultiArray, '/camera_tilt_controller/commands', 10)
        self.roi_pub   = self.create_publisher(Int32MultiArray,   '/vs_nav/roi',                      10)
        self.vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands',    10)
        self.steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands',    10)

        self._yaw      = 0.0
        self._yaw_lock = threading.Lock()
        self.create_subscription(Imu, '/imu', self._imu_cb, 10)

        self._stop_event = threading.Event()
        self._running    = False

        self.create_timer(1.0, self._publish_initial)

    # ── IMU ───────────────────────────────────────────────────────────────────

    def _imu_cb(self, msg):
        q = msg.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self._yaw_lock:
            self._yaw = math.atan2(siny, cosy)

    @property
    def yaw(self):
        with self._yaw_lock:
            return self._yaw

    # ── one-shot init timer ───────────────────────────────────────────────────

    def _publish_initial(self):
        self.publish_tilt(1.35)
        if self._timers:
            self.destroy_timer(self._timers[0])

    # ── low-level commands ────────────────────────────────────────────────────

    def publish_tilt(self, val):
        msg = Float64MultiArray(); msg.data = [float(val)]
        self.tilt_pub.publish(msg)

    def publish_roi(self, lb, lt, rb, rt, enable):
        msg = Int32MultiArray()
        msg.data = [int(lb), int(lt), int(rb), int(rt), int(enable)]
        self.roi_pub.publish(msg)

    def cmd_wheels(self, speed):
        msg = Float64MultiArray(); msg.data = [float(speed)] * 4
        self.vel_pub.publish(msg)

    def cmd_steer(self, angle):
        msg = Float64MultiArray(); msg.data = [float(angle)]
        self.steer_pub.publish(msg)

    def full_stop(self):
        self.cmd_wheels(0.0)
        self.cmd_steer(0.0)

    # ── maneuver ──────────────────────────────────────────────────────────────

    @property
    def is_running(self):
        return self._running

    def abort(self):
        self._stop_event.set()
        self.full_stop()

    def start_maneuver(self, direction, params, on_status):
        """
        direction: +1 = Turn Left  (negative steer, yaw increases)
                   -1 = Turn Right (positive steer, yaw decreases)
        """
        if self._running:
            return
        self._stop_event.clear()
        threading.Thread(
            target=self._maneuver,
            args=(direction, params, on_status),
            daemon=True,
        ).start()

    @staticmethod
    def _norm(a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def _wait_angle(self, target_rad, sign, on_status, label):
        """
        Accumulate yaw change until signed progress >= target_rad.
        sign: +1 if yaw expected to increase, -1 if decrease.
        """
        acc  = 0.0
        last = self.yaw
        while not self._stop_event.is_set():
            cur   = self.yaw
            acc  += self._norm(cur - last)
            last  = cur
            prog  = sign * acc
            rem   = math.degrees(target_rad - prog)
            on_status(f'{label}  {rem:.0f}° left')
            if prog >= target_rad - math.radians(2.0):
                return True
            time.sleep(0.05)
        return False

    def _maneuver(self, direction, params, on_status):
        """
        Two-arc headland maneuver.

        Steer sign (from acker_odom.py convention):
          positive steer angle → right turn (yaw decreases in odom/IMU)
          negative steer angle → left  turn (yaw increases in odom/IMU)

        Arc 2 uses the OPPOSITE steer while going BACKWARD,
        which continues yaw change in the SAME direction.
        """
        self._running = True
        try:
            raw_steer   = params['steer_angle']
            sign_flip   = params['sign_flip']     # +1 or -1
            row_spacing = params['row_spacing']
            drive_speed = params['drive_speed']
            turn_speed  = params['turn_speed']

            R = steer_to_radius(raw_steer)
            alpha_deg, beta_deg = compute_arcs(row_spacing, R)
            alpha = math.radians(alpha_deg)
            beta  = math.radians(beta_deg)

            # direction +1 = left: steer1 negative, direction -1 = right: steer1 positive
            steer1 = -direction * sign_flip * raw_steer
            steer2 = -steer1  # arc2: opposite steer + reverse velocity → same yaw direction

            # Arc 1 — forward
            on_status(f'Arc 1  forward  {alpha_deg:.0f}°…')
            self.cmd_steer(steer1)
            self.cmd_wheels(drive_speed)
            if not self._wait_angle(alpha, float(direction), on_status, 'Arc 1'):
                self.full_stop()
                on_status('⛔  Stopped')
                return

            # Arc 2 — backward
            on_status(f'Arc 2  reverse  {beta_deg:.0f}°…')
            self.cmd_steer(steer2)
            self.cmd_wheels(-turn_speed)
            if not self._wait_angle(beta, float(direction), on_status, 'Arc 2'):
                self.full_stop()
                on_status('⛔  Stopped')
                return

            self.full_stop()
            on_status('✓  Done — at next row')
        finally:
            self._running = False


# ── GUI ───────────────────────────────────────────────────────────────────────

def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraTiltGui()
    threading.Thread(target=ros_spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title('Camera, ROI & Drive Control')
    root.geometry('450x950')
    root.resizable(False, False)

    BG  = '#2c2c2c'
    FG  = '#eeeeee'
    DIM = '#888888'
    SEP = '#444444'
    root.configure(bg=BG)

    def lbl(parent, text, bold=False, size=10, fg=FG, **kw):
        f = ('Arial', size, 'bold') if bold else ('Arial', size)
        return tk.Label(parent, text=text, font=f, bg=BG, fg=fg, **kw)

    def section_header(text):
        tk.Frame(root, height=1, bg=SEP).pack(fill='x', padx=10, pady=(10, 0))
        lbl(root, text, bold=True, size=11, fg='#aaaaaa').pack(anchor='w', padx=12, pady=(4, 2))

    def make_slider(parent, label_text, from_, to, default, on_change):
        row = tk.Frame(parent, bg=BG)
        row.pack(fill='x', pady=1)
        lv = tk.StringVar(value=f'{label_text}{default} px')
        tk.Label(row, textvariable=lv, font=('Arial', 10),
                 bg=BG, fg=FG, width=26, anchor='w').pack(side='left')
        def _cb(v):
            lv.set(f'{label_text}{int(float(v))} px')
            on_change()
        s = tk.Scale(parent, from_=from_, to=to, resolution=1,
                     orient=tk.HORIZONTAL, length=416, command=_cb,
                     showvalue=False, bg=BG, fg=FG,
                     troughcolor='#555', highlightthickness=0)
        s.set(default)
        s.pack(padx=14)
        return s

    def param_row(parent, label_text, default, width=8):
        """Label + Entry row, returns StringVar."""
        row = tk.Frame(parent, bg=BG)
        row.pack(fill='x', pady=2, padx=14)
        tk.Label(row, text=label_text, font=('Arial', 10),
                 bg=BG, fg=FG, width=22, anchor='w').pack(side='left')
        var = tk.StringVar(value=str(default))
        tk.Entry(row, textvariable=var, font=('Arial', 10),
                 bg='#3a3a3a', fg=FG, insertbackground=FG,
                 relief='flat', width=width).pack(side='left', padx=(4, 0))
        return var

    bkw = dict(font=('Arial', 11, 'bold'), relief='flat',
               padx=16, pady=8, cursor='hand2', bd=0)

    # ─────────────────────────────────────────────────────────────────────────
    # Camera Tilt
    # ─────────────────────────────────────────────────────────────────────────
    section_header('Camera Tilt')
    tilt_frame = tk.Frame(root, bg=BG)
    tilt_frame.pack(fill='x', padx=14, pady=4)

    tilt_val = tk.StringVar(value='1.35 rad  (~77°)')
    tk.Label(tilt_frame, textvariable=tilt_val, font=('Arial', 11),
             bg=BG, fg=FG).pack()

    def on_tilt(v):
        rad = float(v)
        tilt_val.set(f'{rad:.2f} rad  (~{round(rad * 57.2958, 1)}°)')
        node.publish_tilt(rad)

    tk.Scale(tilt_frame, from_=0.0, to=2.09, resolution=0.01,
             orient=tk.HORIZONTAL, length=416, command=on_tilt,
             showvalue=False, bg=BG, fg=FG,
             troughcolor='#555', highlightthickness=0).pack()
    lbl(tilt_frame, '0° (forward)  ←──────────────────────────→  120° (down)',
        size=8, fg=DIM).pack()

    # ─────────────────────────────────────────────────────────────────────────
    # ROI
    # ─────────────────────────────────────────────────────────────────────────
    section_header('ROI  (perspective-aware trapezoid)')
    roi_frame = tk.Frame(root, bg=BG)
    roi_frame.pack(fill='x', padx=14, pady=4)

    enable_var = tk.BooleanVar(value=True)
    tk.Checkbutton(roi_frame, text='Enable ROI', variable=enable_var,
                   bg=BG, fg=FG, selectcolor=SEP,
                   activebackground=BG, activeforeground=FG,
                   font=('Arial', 10),
                   command=lambda: refresh()).pack(anchor='w')

    lbl(roi_frame, '── Left side ──────────────────────────────',
        size=9, fg=DIM).pack(anchor='w', pady=(6, 0))
    s_lb = make_slider(roi_frame, 'Near (bottom): ', 0, 500, DEF_LEFT_BOT,  lambda: refresh())
    s_lt = make_slider(roi_frame, 'Far  (top):    ', 0, 500, DEF_LEFT_TOP,  lambda: refresh())
    lbl(roi_frame, '── Right side ─────────────────────────────',
        size=9, fg=DIM).pack(anchor='w', pady=(6, 0))
    s_rb = make_slider(roi_frame, 'Near (bottom): ', 0, 500, DEF_RIGHT_BOT, lambda: refresh())
    s_rt = make_slider(roi_frame, 'Far  (top):    ', 0, 500, DEF_RIGHT_TOP, lambda: refresh())

    lbl(roi_frame, 'Preview  (top = far / bottom = near):', size=9, fg=DIM).pack(
        anchor='w', pady=(10, 2))
    canvas = tk.Canvas(roi_frame, width=CANVAS_W, height=CANVAS_H,
                       bg='#1a1a1a', highlightthickness=1, highlightbackground=SEP)
    canvas.pack()

    def refresh():
        draw_roi_preview(canvas, s_lb.get(), s_lt.get(),
                         s_rb.get(), s_rt.get(), enable_var.get())

    refresh()

    roi_status = tk.StringVar(value='')
    tk.Label(roi_frame, textvariable=roi_status, font=('Arial', 9),
             bg=BG, fg=DIM).pack(pady=(6, 2))

    roi_btn_frame = tk.Frame(roi_frame, bg=BG)
    roi_btn_frame.pack(pady=4)

    def on_apply():
        node.publish_roi(s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        roi_status.set('✓ Applied — tracker reset')
        roi_frame.after(3000, lambda: roi_status.set(''))

    def on_save():
        ok, msg = save_roi_to_file(s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        node.publish_roi(s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        roi_status.set(('✓ ' if ok else '✗ ') + msg)
        roi_frame.after(4000, lambda: roi_status.set(''))

    tk.Button(roi_btn_frame, text='Apply', bg='#1e8449', fg='white',
              activebackground='#27ae60', command=on_apply, **bkw).pack(side='left', padx=6)
    tk.Button(roi_btn_frame, text='Save to file', bg='#1a5276', fg='white',
              activebackground='#2980b9', command=on_save, **bkw).pack(side='left', padx=6)

    # ─────────────────────────────────────────────────────────────────────────
    # Drive Controls
    # ─────────────────────────────────────────────────────────────────────────
    section_header('Drive')
    drv_frame = tk.Frame(root, bg=BG)
    drv_frame.pack(fill='x', padx=14, pady=6)

    drv_speed_var = param_row(drv_frame, 'Forward speed (rad/s):', '1.0')

    drv_btn_frame = tk.Frame(drv_frame, bg=BG)
    drv_btn_frame.pack(pady=4)

    def on_forward():
        if node.is_running:
            return
        try:
            spd = float(drv_speed_var.get())
        except ValueError:
            return
        node.cmd_steer(0.0)
        node.cmd_wheels(spd)

    def on_stop():
        node.abort()

    tk.Button(drv_btn_frame, text='▶  Forward', bg='#1e8449', fg='white',
              activebackground='#27ae60', command=on_forward, **bkw).pack(side='left', padx=6)
    tk.Button(drv_btn_frame, text='■  Stop', bg='#922b21', fg='white',
              activebackground='#c0392b', command=on_stop, **bkw).pack(side='left', padx=6)

    # ─────────────────────────────────────────────────────────────────────────
    # Row Transition
    # ─────────────────────────────────────────────────────────────────────────
    section_header('Row Transition  (2-arc gyro maneuver)')
    turn_frame = tk.Frame(root, bg=BG)
    turn_frame.pack(fill='x', padx=14, pady=6)

    steer_var   = param_row(turn_frame, 'Steer angle (rad):', '0.28')
    spacing_var = param_row(turn_frame, 'Row spacing (m):',   '2.0')
    fwd_spd_var = param_row(turn_frame, 'Arc 1 speed (rad/s):', '1.0')
    rev_spd_var = param_row(turn_frame, 'Arc 2 speed (rad/s):', '0.6')

    # Flip-steer checkbox
    flip_var = tk.IntVar(value=1)
    flip_row = tk.Frame(turn_frame, bg=BG)
    flip_row.pack(fill='x', pady=(4, 0), padx=0)
    tk.Checkbutton(flip_row, text='Flip steer direction  (if turns go wrong way)',
                   variable=flip_var, onvalue=-1, offvalue=1,
                   bg=BG, fg=DIM, selectcolor=SEP,
                   activebackground=BG, activeforeground=FG,
                   font=('Arial', 9),
                   command=lambda: _update_arc_info()).pack(anchor='w')

    # Arc info display
    arc_info_var = tk.StringVar(value='')
    tk.Label(turn_frame, textvariable=arc_info_var, font=('Arial', 9),
             bg=BG, fg='#f39c12').pack(anchor='w', pady=(6, 0))

    def _update_arc_info(*_):
        try:
            sa  = float(steer_var.get())
            d   = float(spacing_var.get())
            R   = steer_to_radius(sa)
            a1, a2 = compute_arcs(d, R)
            arc_info_var.set(
                f'Turn radius: {R:.2f} m   →   Arc 1: {a1:.1f}°   Arc 2: {a2:.1f}°'
            )
        except (ValueError, ZeroDivisionError):
            arc_info_var.set('(invalid parameters)')

    # bind entry changes to recompute
    for var in (steer_var, spacing_var):
        var.trace_add('write', _update_arc_info)

    _update_arc_info()

    # Status + indicator
    turn_status_var = tk.StringVar(value='Idle')
    status_row = tk.Frame(turn_frame, bg=BG)
    status_row.pack(fill='x', pady=(6, 2))

    indicator = tk.Label(status_row, text='●', font=('Arial', 14),
                         bg=BG, fg='#555555')
    indicator.pack(side='left', padx=(0, 6))
    tk.Label(status_row, textvariable=turn_status_var, font=('Arial', 9),
             bg=BG, fg=DIM, anchor='w').pack(side='left', fill='x')

    def set_status(text):
        turn_status_var.set(text)
        if 'Arc' in text:
            indicator.config(fg='#f39c12')
        elif '✓' in text:
            indicator.config(fg='#2ecc71')
        elif '⛔' in text:
            indicator.config(fg='#e74c3c')
        else:
            indicator.config(fg='#555555')

    def _on_status_thread(text):
        root.after(0, lambda: set_status(text))

    # Turn buttons
    turn_btn_frame = tk.Frame(turn_frame, bg=BG)
    turn_btn_frame.pack(pady=6)

    def _get_params():
        return {
            'steer_angle': float(steer_var.get()),
            'row_spacing': float(spacing_var.get()),
            'drive_speed': float(fwd_spd_var.get()),
            'turn_speed':  float(rev_spd_var.get()),
            'sign_flip':   flip_var.get(),
        }

    def on_turn(direction):
        if node.is_running:
            return
        try:
            params = _get_params()
        except ValueError:
            set_status('⛔  Invalid parameters')
            return
        set_status('Starting…')
        node.start_maneuver(direction, params, _on_status_thread)

    tk.Button(turn_btn_frame, text='◄  Turn Left', bg='#1a5276', fg='white',
              activebackground='#2980b9',
              command=lambda: on_turn(+1), **bkw).pack(side='left', padx=6)
    tk.Button(turn_btn_frame, text='Turn Right  ►', bg='#1a5276', fg='white',
              activebackground='#2980b9',
              command=lambda: on_turn(-1), **bkw).pack(side='left', padx=6)

    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
