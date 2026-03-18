import threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CameraTiltGui(Node):
    def __init__(self):
        super().__init__('camera_tilt_gui')
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/camera_tilt_controller/commands',
            10
        )
        # Publish initial position after a short delay
        self.create_timer(1.0, self._publish_initial)

    def _publish_initial(self):
        self.publish_tilt(1.35)
        self.destroy_timer(self._timers[0] if self._timers else None)

    def publish_tilt(self, val):
        msg = Float64MultiArray()
        msg.data = [float(val)]
        self.pub.publish(msg)


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraTiltGui()

    spin_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Camera Tilt Control')
    root.geometry('420x160')
    root.resizable(False, False)

    tk.Label(root, text='Camera Pitch (radians)', font=('Arial', 13, 'bold')).pack(pady=8)

    val_label = tk.Label(root, text='1.35 rad  (~77°)', font=('Arial', 11))
    val_label.pack()

    def on_slider(v):
        rad = float(v)
        deg = round(rad * 57.2958, 1)
        val_label.config(text=f'{rad:.2f} rad  (~{deg}°)')
        node.publish_tilt(rad)

    slider = tk.Scale(
        root,
        from_=0.0, to=2.09,
        resolution=0.01,
        orient=tk.HORIZONTAL,
        length=380,
        command=on_slider,
        showvalue=False,
    )
    slider.set(1.35)
    slider.pack(padx=20, pady=4)

    tk.Label(root, text='0° (forward)  ←──────────────────→  ~120° (down)', font=('Arial', 9), fg='gray').pack()

    root.mainloop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
