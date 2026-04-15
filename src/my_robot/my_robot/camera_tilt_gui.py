import threading
import tkinter as tk
import yaml
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

IMG_W = 1280
IMG_H = 720
PARAMS_PATH = (
    '/ros2_ws/install/visual_multi_crop_row_navigation/share/'
    'visual_multi_crop_row_navigation/configs/params.yaml'
)

# Defaults: wider exclusion near (bottom), narrower far (top)
DEF_LEFT_BOT  = 150
DEF_LEFT_TOP  = 50
DEF_RIGHT_BOT = 150
DEF_RIGHT_TOP = 50


class CameraTiltGui(Node):
    def __init__(self):
        super().__init__('camera_tilt_gui')
        self.tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera_tilt_controller/commands', 10)
        self.roi_pub = self.create_publisher(
            Int32MultiArray, '/vs_nav/roi', 10)
        self.create_timer(1.0, self._publish_initial)

    def _publish_initial(self):
        self.publish_tilt(1.35)
        self.destroy_timer(self._timers[0] if self._timers else None)

    def publish_tilt(self, val):
        msg = Float64MultiArray()
        msg.data = [float(val)]
        self.tilt_pub.publish(msg)

    def publish_roi(self, lb, lt, rb, rt, enable):
        """[left_bottom, left_top, right_bottom, right_top, enable]"""
        msg = Int32MultiArray()
        msg.data = [int(lb), int(lt), int(rb), int(rt), int(enable)]
        self.roi_pub.publish(msg)


def _roi_polygons(lb, lt, rb, rt):
    """Return (right_strip_pts, left_strip_pts) as lists of [x,y]."""
    right = [
        [IMG_W - rt, 0], [IMG_W, 0],
        [IMG_W, IMG_H], [IMG_W - rb, IMG_H],
    ]
    left = [
        [0, 0], [lt, 0],
        [lb, IMG_H], [0, IMG_H],
    ]
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


# ── Preview canvas ────────────────────────────────────────────────────────────

CANVAS_W = 400
CANVAS_H = 80  # taller to better show the trapezoid

def _scale(x, y):
    return int(x * CANVAS_W / IMG_W), int(y * CANVAS_H / IMG_H)


def draw_roi_preview(canvas, lb, lt, rb, rt, enable):
    canvas.delete('all')

    # background (full frame)
    canvas.create_rectangle(0, 0, CANVAS_W, CANVAS_H, fill='#1a1a1a', outline='')

    if not enable:
        canvas.create_rectangle(0, 0, CANVAS_W, CANVAS_H, fill='#2ecc71', outline='')
        canvas.create_text(CANVAS_W // 2, CANVAS_H // 2,
                           text='ROI disabled — full frame active',
                           fill='white', font=('Arial', 9, 'bold'))
        return

    # Trapezoid corners in image coords
    # visible area: top-left=(lt,0), top-right=(W-rt,0),
    #               bot-right=(W-rb,H), bot-left=(lb,H)
    tl = _scale(lt,          0)
    tr = _scale(IMG_W - rt,  0)
    br = _scale(IMG_W - rb,  IMG_H)
    bl = _scale(lb,          IMG_H)

    # Left exclusion trapezoid
    left_poly = [0, 0,  tl[0], 0,  bl[0], CANVAS_H,  0, CANVAS_H]
    canvas.create_polygon(left_poly, fill='#922b21', outline='')

    # Right exclusion trapezoid
    right_poly = [tr[0], 0,  CANVAS_W, 0,  CANVAS_W, CANVAS_H,  br[0], CANVAS_H]
    canvas.create_polygon(right_poly, fill='#922b21', outline='')

    # Visible trapezoid
    vis_poly = [tl[0], 0,  tr[0], 0,  br[0], CANVAS_H,  bl[0], CANVAS_H]
    canvas.create_polygon(vis_poly, fill='#1e8449', outline='')

    # Perspective guide lines (show convergence)
    canvas.create_line(tl[0], 0, bl[0], CANVAS_H, fill='#58d68d', width=1, dash=(3, 3))
    canvas.create_line(tr[0], 0, br[0], CANVAS_H, fill='#58d68d', width=1, dash=(3, 3))

    # Labels
    vis_top_w  = IMG_W - lt  - rt
    vis_bot_w  = IMG_W - lb  - rb
    canvas.create_text(CANVAS_W // 2, 10,
                       text=f'far:  {max(vis_top_w, 0)} px',
                       fill='#abebc6', font=('Arial', 8))
    canvas.create_text(CANVAS_W // 2, CANVAS_H - 10,
                       text=f'near: {max(vis_bot_w, 0)} px',
                       fill='#abebc6', font=('Arial', 8))

    # Horizon line
    canvas.create_line(0, 1, CANVAS_W, 1, fill='#555', width=1)
    canvas.create_line(0, CANVAS_H - 1, CANVAS_W, CANVAS_H - 1, fill='#555', width=1)


# ── GUI ───────────────────────────────────────────────────────────────────────

def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraTiltGui()
    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Camera & ROI Control')
    root.geometry('450x620')
    root.resizable(False, False)
    root.configure(bg='#2c2c2c')

    BG   = '#2c2c2c'
    FG   = '#eeeeee'
    DIM  = '#888888'
    SEP  = '#444444'

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
        lbl(row, '', fg=FG).pack(side='left')   # spacer
        lbl_w = tk.Label(row, textvariable=lv, font=('Arial', 10),
                         bg=BG, fg=FG, width=26, anchor='w')
        lbl_w.pack(side='left')

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

    # ── Camera tilt ──────────────────────────────────────────────────────────
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

    # ── ROI ──────────────────────────────────────────────────────────────────
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

    # Preview
    lbl(roi_frame, 'Preview  (top = far / bottom = near):', size=9, fg=DIM).pack(
        anchor='w', pady=(10, 2))
    canvas = tk.Canvas(roi_frame, width=CANVAS_W, height=CANVAS_H,
                       bg='#1a1a1a', highlightthickness=1, highlightbackground=SEP)
    canvas.pack()

    def refresh():
        draw_roi_preview(canvas, s_lb.get(), s_lt.get(),
                         s_rb.get(), s_rt.get(), enable_var.get())

    refresh()

    # Status + buttons
    status_var = tk.StringVar(value='')
    tk.Label(roi_frame, textvariable=status_var, font=('Arial', 9),
             bg=BG, fg=DIM).pack(pady=(6, 2))

    btn_frame = tk.Frame(roi_frame, bg=BG)
    btn_frame.pack(pady=4)

    bkw = dict(font=('Arial', 10, 'bold'), relief='flat',
               padx=18, pady=7, cursor='hand2', bd=0)

    def on_apply():
        node.publish_roi(s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        status_var.set('✓ Applied — tracker reset')
        roi_frame.after(3000, lambda: status_var.set(''))

    def on_save():
        ok, msg = save_roi_to_file(
            s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        node.publish_roi(s_lb.get(), s_lt.get(), s_rb.get(), s_rt.get(), enable_var.get())
        status_var.set(('✓ ' if ok else '✗ ') + msg)
        roi_frame.after(4000, lambda: status_var.set(''))

    tk.Button(btn_frame, text='Apply', bg='#1e8449', fg='white',
              activebackground='#27ae60', command=on_apply, **bkw).pack(side='left', padx=6)
    tk.Button(btn_frame, text='Save to file', bg='#1a5276', fg='white',
              activebackground='#2980b9', command=on_save, **bkw).pack(side='left', padx=6)

    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
