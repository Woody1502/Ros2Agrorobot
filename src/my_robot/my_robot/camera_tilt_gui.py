import threading
import tkinter as tk
from tkinter import ttk
import yaml
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

IMG_W = 1280
IMG_H = 720
PARAMS_PATH = '/ros2_ws/install/visual_multi_crop_row_navigation/share/visual_multi_crop_row_navigation/configs/params.yaml'


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

    def publish_roi(self, left_margin, right_margin, enable):
        msg = Int32MultiArray()
        msg.data = [int(left_margin), int(right_margin), int(enable)]
        self.roi_pub.publish(msg)


def ros_spin(node):
    rclpy.spin(node)


def save_roi_to_file(left_margin, right_margin, enable):
    if not os.path.exists(PARAMS_PATH):
        return False, f'File not found:\n{PARAMS_PATH}'
    try:
        with open(PARAMS_PATH, 'r') as f:
            data = yaml.safe_load(f)
        params = data['/**']['ros__parameters']
        params['enable_roi'] = bool(enable)
        params['p1'] = [IMG_W - right_margin, 0]
        params['p2'] = [IMG_W, 0]
        params['p3'] = [IMG_W, IMG_H]
        params['p4'] = [IMG_W - right_margin, IMG_H]
        params['p5'] = [0, 0]
        params['p6'] = [left_margin, 0]
        params['p7'] = [left_margin, IMG_H]
        params['p8'] = [0, IMG_H]
        with open(PARAMS_PATH, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)
        return True, 'Saved to file'
    except Exception as e:
        return False, str(e)


def draw_roi_preview(canvas, left_margin, right_margin, enable, canvas_w=380, canvas_h=36):
    canvas.delete('all')
    scale = canvas_w / IMG_W
    left_px = int(left_margin * scale)
    right_px = canvas_w - int(right_margin * scale)

    if not enable:
        canvas.create_rectangle(0, 0, canvas_w, canvas_h, fill='#2ecc71', outline='')
        canvas.create_text(canvas_w // 2, canvas_h // 2, text='ROI disabled (full frame)',
                           fill='white', font=('Arial', 9))
        return

    # excluded strips
    canvas.create_rectangle(0, 0, left_px, canvas_h, fill='#c0392b', outline='')
    canvas.create_rectangle(right_px, 0, canvas_w, canvas_h, fill='#c0392b', outline='')
    # visible zone
    canvas.create_rectangle(left_px, 0, right_px, canvas_h, fill='#2ecc71', outline='')
    # labels
    if left_px > 18:
        canvas.create_text(left_px // 2, canvas_h // 2, text='✕', fill='white', font=('Arial', 9))
    if canvas_w - right_px > 18:
        canvas.create_text((right_px + canvas_w) // 2, canvas_h // 2,
                           text='✕', fill='white', font=('Arial', 9))
    visible_w = right_px - left_px
    if visible_w > 40:
        canvas.create_text(left_px + visible_w // 2, canvas_h // 2,
                           text=f'{IMG_W - left_margin - right_margin}px visible',
                           fill='white', font=('Arial', 9, 'bold'))


def main():
    rclpy.init()
    node = CameraTiltGui()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Camera & ROI Control')
    root.geometry('440x440')
    root.resizable(False, False)
    root.configure(bg='#2c2c2c')

    def lbl(parent, text, bold=False, size=10, fg='#eeeeee', **kw):
        font = ('Arial', size, 'bold') if bold else ('Arial', size)
        return tk.Label(parent, text=text, font=font, bg='#2c2c2c', fg=fg, **kw)

    def section(parent, title):
        frame = tk.Frame(parent, bg='#2c2c2c')
        frame.pack(fill='x', padx=12, pady=(10, 4))
        lbl(frame, title, bold=True, size=11, fg='#aaaaaa').pack(anchor='w')
        sep = tk.Frame(parent, height=1, bg='#555555')
        sep.pack(fill='x', padx=12)
        return frame

    # ── Camera tilt ──────────────────────────────────────────────
    section(root, 'Camera Tilt')
    tilt_frame = tk.Frame(root, bg='#2c2c2c')
    tilt_frame.pack(fill='x', padx=12, pady=4)

    tilt_val_lbl = lbl(tilt_frame, '1.35 rad  (~77°)', size=11)
    tilt_val_lbl.pack()

    def on_tilt(v):
        rad = float(v)
        deg = round(rad * 57.2958, 1)
        tilt_val_lbl.config(text=f'{rad:.2f} rad  (~{deg}°)')
        node.publish_tilt(rad)

    tilt_slider = tk.Scale(tilt_frame, from_=0.0, to=2.09, resolution=0.01,
                           orient=tk.HORIZONTAL, length=400, command=on_tilt,
                           showvalue=False, bg='#2c2c2c', fg='#eeeeee',
                           troughcolor='#555555', highlightthickness=0)
    tilt_slider.set(1.35)
    tilt_slider.pack()
    lbl(tilt_frame, '0° (forward)  ←─────────────────────────→  120° (down)',
        size=8, fg='#888888').pack()

    # ── ROI ───────────────────────────────────────────────────────
    section(root, 'Region of Interest (ROI)')
    roi_frame = tk.Frame(root, bg='#2c2c2c')
    roi_frame.pack(fill='x', padx=12, pady=4)

    # Enable checkbox
    enable_var = tk.BooleanVar(value=True)
    roi_enable_cb = tk.Checkbutton(roi_frame, text='Enable ROI', variable=enable_var,
                                   bg='#2c2c2c', fg='#eeeeee', selectcolor='#444444',
                                   activebackground='#2c2c2c', activeforeground='#eeeeee',
                                   font=('Arial', 10), command=lambda: refresh_preview())
    roi_enable_cb.pack(anchor='w')

    # Left margin
    left_row = tk.Frame(roi_frame, bg='#2c2c2c')
    left_row.pack(fill='x', pady=2)
    left_lbl = lbl(left_row, 'Left margin:   150 px', size=10)
    left_lbl.pack(side='left')

    def on_left(v):
        left_lbl.config(text=f'Left margin:   {int(float(v))} px')
        refresh_preview()

    left_slider = tk.Scale(roi_frame, from_=0, to=500, resolution=1,
                           orient=tk.HORIZONTAL, length=400, command=on_left,
                           showvalue=False, bg='#2c2c2c', fg='#eeeeee',
                           troughcolor='#555555', highlightthickness=0)
    left_slider.set(150)
    left_slider.pack()

    # Right margin
    right_row = tk.Frame(roi_frame, bg='#2c2c2c')
    right_row.pack(fill='x', pady=2)
    right_lbl = lbl(right_row, 'Right margin: 150 px', size=10)
    right_lbl.pack(side='left')

    def on_right(v):
        right_lbl.config(text=f'Right margin: {int(float(v))} px')
        refresh_preview()

    right_slider = tk.Scale(roi_frame, from_=0, to=500, resolution=1,
                            orient=tk.HORIZONTAL, length=400, command=on_right,
                            showvalue=False, bg='#2c2c2c', fg='#eeeeee',
                            troughcolor='#555555', highlightthickness=0)
    right_slider.set(150)
    right_slider.pack()

    # Preview canvas
    lbl(roi_frame, 'Preview (1280px wide):', size=9, fg='#888888').pack(anchor='w', pady=(6, 1))
    preview_canvas = tk.Canvas(roi_frame, width=400, height=36,
                                bg='#1a1a1a', highlightthickness=1,
                                highlightbackground='#555555')
    preview_canvas.pack()

    def refresh_preview():
        draw_roi_preview(preview_canvas,
                         left_slider.get(), right_slider.get(),
                         enable_var.get(), canvas_w=400)

    refresh_preview()

    # Buttons
    btn_frame = tk.Frame(roi_frame, bg='#2c2c2c')
    btn_frame.pack(pady=8)

    status_lbl = lbl(roi_frame, '', size=9, fg='#aaaaaa')
    status_lbl.pack()

    def on_apply():
        node.publish_roi(left_slider.get(), right_slider.get(), enable_var.get())
        status_lbl.config(text='✓ Applied (tracker reset)', fg='#2ecc71')

    def on_save():
        ok, msg = save_roi_to_file(left_slider.get(), right_slider.get(), enable_var.get())
        node.publish_roi(left_slider.get(), right_slider.get(), enable_var.get())
        status_lbl.config(text=('✓ ' if ok else '✗ ') + msg,
                          fg='#2ecc71' if ok else '#e74c3c')

    btn_style = dict(font=('Arial', 10, 'bold'), relief='flat', padx=16, pady=6,
                     cursor='hand2', bd=0)
    tk.Button(btn_frame, text='Apply', bg='#27ae60', fg='white',
              activebackground='#2ecc71', command=on_apply, **btn_style).pack(side='left', padx=6)
    tk.Button(btn_frame, text='Save to file', bg='#2980b9', fg='white',
              activebackground='#3498db', command=on_save, **btn_style).pack(side='left', padx=6)

    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
