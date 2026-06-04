#!/usr/bin/env python3

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINT_LIMITS = [
    (-168.0, 168.0),   # J1
    (-135.0, 135.0),   # J2
    (-150.0, 150.0),   # J3
    (-145.0, 145.0),   # J4
    (-155.0, 160.0),   # J5
    (-180.0, 180.0),   # J6
]

DEFAULT_SPEED = 20


class JointSliderPublisher(Node):
    def __init__(self):
        super().__init__("local_joint_slider_publisher")
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/jetcobot/target_angles",
            10,
        )

    def publish_angles(self, angles, speed):
        msg = Float64MultiArray()
        msg.data = [float(a) for a in angles] + [float(speed)]
        self.pub.publish(msg)


class JointSliderGUI:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node

        self.root.title("Local Jetcobot Joint Slider")

        self.angle_vars = []
        self.value_labels = []

        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)

        self.center_angles = [-82.88, 56.42, -19.86, -93.51, 16.78, -124.71]
        self.left_scan_angles = [-82.88, 56.51, -19.33, -93.60, 24.96, -121.46]
        self.right_scan_angles = [-82.88, 56.51, -19.77, -94.65, 2.10, -129.55]
        self.home_angles = [0, 0, 0, 0, 0, 0]

        self.build_ui()

    def build_ui(self):
        title = ttk.Label(
            self.root,
            text="Jetcobot Joint Slider Publisher",
            font=("Arial", 15, "bold"),
        )
        title.pack(pady=10)

        speed_frame = ttk.Frame(self.root)
        speed_frame.pack(fill="x", padx=15, pady=5)

        ttk.Label(speed_frame, text="Speed").pack(side="left")

        speed_slider = ttk.Scale(
            speed_frame,
            from_=1,
            to=100,
            orient="horizontal",
            variable=self.speed_var,
        )
        speed_slider.pack(side="left", fill="x", expand=True, padx=10)

        self.speed_label = ttk.Label(speed_frame, textvariable=self.speed_var, width=5)
        self.speed_label.pack(side="left")

        joint_frame = ttk.Frame(self.root)
        joint_frame.pack(fill="both", expand=True, padx=15, pady=10)

        for i, (low, high) in enumerate(JOINT_LIMITS):
            row = ttk.Frame(joint_frame)
            row.pack(fill="x", pady=6)

            ttk.Label(row, text=f"J{i+1}", width=4).pack(side="left")

            var = tk.DoubleVar(value=0.0)
            self.angle_vars.append(var)

            slider = ttk.Scale(
                row,
                from_=low,
                to=high,
                orient="horizontal",
                variable=var,
                command=lambda value, idx=i: self.on_slider_change(idx, value),
            )
            slider.pack(side="left", fill="x", expand=True, padx=10)

            label = ttk.Label(row, text="0.00", width=8)
            label.pack(side="left")
            self.value_labels.append(label)

            limit_label = ttk.Label(row, text=f"[{low}, {high}]", width=16)
            limit_label.pack(side="left")

        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill="x", padx=15, pady=10)

        ttk.Button(
            button_frame,
            text="Publish 현재 슬라이더",
            command=self.publish_current,
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Home",
            command=lambda: self.go_named_pose("Home", self.home_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Center",
            command=lambda: self.go_named_pose("Center", self.center_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Left Scan",
            command=lambda: self.go_named_pose("Left Scan", self.left_scan_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Right Scan",
            command=lambda: self.go_named_pose("Right Scan", self.right_scan_angles),
        ).pack(side="left", padx=5)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var).pack(fill="x", padx=15, pady=5)

    def on_slider_change(self, idx, value):
        angle = float(value)
        self.value_labels[idx].config(text=f"{angle:.2f}")

    def get_slider_angles(self):
        return [round(var.get(), 2) for var in self.angle_vars]

    def set_slider_angles(self, angles):
        for i, angle in enumerate(angles):
            low, high = JOINT_LIMITS[i]
            angle = max(low, min(high, float(angle)))

            self.angle_vars[i].set(angle)
            self.value_labels[i].config(text=f"{angle:.2f}")

    def publish_current(self):
        angles = self.get_slider_angles()
        speed = int(self.speed_var.get())

        self.ros_node.publish_angles(angles, speed)
        self.status_var.set(f"Published: angles={angles}, speed={speed}")

    def go_named_pose(self, name, angles):
        self.set_slider_angles(angles)

        speed = int(self.speed_var.get())
        self.ros_node.publish_angles(angles, speed)

        self.status_var.set(f"Published {name}: {angles}, speed={speed}")


def spin_ros(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    ros_node = JointSliderPublisher()

    ros_thread = threading.Thread(target=spin_ros, args=(ros_node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    gui = JointSliderGUI(root, ros_node)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()