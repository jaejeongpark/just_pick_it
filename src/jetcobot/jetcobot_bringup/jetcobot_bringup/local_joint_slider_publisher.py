#!/usr/bin/env python3

import queue
import signal
import sys
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty


CMD_JOINT = 0
CMD_COORD = 1

JOINT_LIMITS = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-155.0, 160.0),
    (-180.0, 180.0),
]

COORD_LIMITS = [
    (-280.0, 280.0),
    (-280.0, 280.0),
    (-70.0, 523.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
]

JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6"]
COORD_LABELS = ["X", "Y", "Z", "RX", "RY", "RZ"]

DEFAULT_SPEED = 20


class JetcobotGuiPublisher(Node):
    def __init__(self, status_queue):
        super().__init__("local_jetcobot_gui_publisher")

        self.status_queue = status_queue

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            "/jetcobot/target_pose",
            10,
        )

        self.status_request_pub = self.create_publisher(
            Empty,
            "/jetcobot/request_status",
            10,
        )

        self.status_sub = self.create_subscription(
            Float64MultiArray,
            "/jetcobot/status",
            self.status_callback,
            10,
        )

    def publish_command(self, command_type, values, speed, coord_move_mode=0):
        msg = Float64MultiArray()
        msg.data = (
            [float(command_type)]
            + [float(v) for v in values]
            + [float(speed), float(coord_move_mode)]
        )
        self.command_pub.publish(msg)

    def request_status(self):
        self.status_request_pub.publish(Empty())

    def status_callback(self, msg):
        data = list(msg.data)

        if len(data) != 26:
            self.get_logger().warn(
                f"Invalid status length: {len(data)}. Expected 26."
            )
            return

        status = {
            "tool_reference": data[0:6],
            "world_reference": data[6:12],
            "reference_frame": int(data[12]),
            "end_type": int(data[13]),
            "angles": data[14:20],
            "coords": data[20:26],
        }

        self.status_queue.put(status)


class JetcobotSliderGUI:
    def __init__(self, root, ros_node, status_queue):
        self.root = root
        self.ros_node = ros_node
        self.status_queue = status_queue

        self.root.title("Local Jetcobot Slider Controller")

        self.command_type = CMD_JOINT
        self.pending_mode = None
        self.latest_status = None

        self.value_vars = []
        self.entry_vars = []
        self.name_labels = []
        self.value_labels = []
        self.sliders = []
        self.limit_labels = []

        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.coord_move_mode_var = tk.IntVar(value=0)

        self.center_angles = [-82.88, 56.42, -19.86, -93.51, 16.78, -124.71]
        self.left_scan_angles = [-82.88, 56.51, -19.33, -93.60, 24.96, -121.46]
        self.right_scan_angles = [-82.88, 56.51, -19.77, -94.65, 2.10, -129.55]
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.build_ui()
        self.apply_mode_ui()

        self.root.after(200, self.poll_status_queue)
        self.ros_node.request_status()

    def current_limits(self):
        if self.command_type == CMD_JOINT:
            return JOINT_LIMITS
        return COORD_LIMITS

    def current_labels(self):
        if self.command_type == CMD_JOINT:
            return JOINT_LABELS
        return COORD_LABELS

    def current_mode_name(self):
        if self.command_type == CMD_JOINT:
            return "JOINT / send_angles"
        return "COORD / send_coords"

    def build_ui(self):
        title = ttk.Label(
            self.root,
            text="Jetcobot Slider Publisher",
            font=("Arial", 15, "bold"),
        )
        title.pack(pady=10)

        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", padx=15, pady=5)

        self.mode_button = ttk.Button(
            top_frame,
            text="Mode: JOINT / send_angles",
            command=self.toggle_mode,
        )
        self.mode_button.pack(side="left", padx=5)

        ttk.Button(
            top_frame,
            text="Robot 상태 읽기",
            command=self.request_robot_status,
        ).pack(side="left", padx=5)

        ttk.Label(top_frame, text="Speed").pack(side="left", padx=(15, 5))

        speed_slider = ttk.Scale(
            top_frame,
            from_=1,
            to=100,
            orient="horizontal",
            variable=self.speed_var,
        )
        speed_slider.pack(side="left", fill="x", expand=True, padx=5)

        ttk.Label(top_frame, textvariable=self.speed_var, width=5).pack(side="left")

        self.coord_mode_frame = ttk.Frame(self.root)
        self.coord_mode_frame.pack(fill="x", padx=15, pady=5)

        ttk.Label(self.coord_mode_frame, text="send_coords move mode").pack(side="left")

        ttk.Radiobutton(
            self.coord_mode_frame,
            text="0 angular",
            variable=self.coord_move_mode_var,
            value=0,
        ).pack(side="left", padx=10)

        ttk.Radiobutton(
            self.coord_mode_frame,
            text="1 linear",
            variable=self.coord_move_mode_var,
            value=1,
        ).pack(side="left", padx=10)

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=15, pady=10)

        for i in range(6):
            row = ttk.Frame(main_frame)
            row.pack(fill="x", pady=5)

            name_label = ttk.Label(row, text=f"J{i+1}", width=5)
            name_label.pack(side="left")
            self.name_labels.append(name_label)

            value_var = tk.DoubleVar(value=0.0)
            self.value_vars.append(value_var)

            slider = ttk.Scale(
                row,
                from_=JOINT_LIMITS[i][0],
                to=JOINT_LIMITS[i][1],
                orient="horizontal",
                variable=value_var,
                command=lambda value, idx=i: self.on_slider_change(idx, value),
            )
            slider.pack(side="left", fill="x", expand=True, padx=6)
            self.sliders.append(slider)

            entry_var = tk.StringVar(value="0.00")
            self.entry_vars.append(entry_var)

            entry = ttk.Entry(row, textvariable=entry_var, width=10)
            entry.pack(side="left", padx=5)
            entry.bind("<Return>", lambda event, idx=i: self.apply_entry_to_slider(idx))

            value_label = ttk.Label(row, text="0.00", width=8)
            value_label.pack(side="left", padx=5)
            self.value_labels.append(value_label)

            limit_label = ttk.Label(row, text="[0, 0]", width=18)
            limit_label.pack(side="left")
            self.limit_labels.append(limit_label)

        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill="x", padx=15, pady=10)

        ttk.Button(
            button_frame,
            text="Publish",
            command=self.publish_current,
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Home",
            command=lambda: self.go_joint_pose("Home", self.home_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Center",
            command=lambda: self.go_joint_pose("Center", self.center_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Left Scan",
            command=lambda: self.go_joint_pose("Left Scan", self.left_scan_angles),
        ).pack(side="left", padx=5)

        ttk.Button(
            button_frame,
            text="Right Scan",
            command=lambda: self.go_joint_pose("Right Scan", self.right_scan_angles),
        ).pack(side="left", padx=5)

        status_frame = ttk.LabelFrame(self.root, text="Robot Status")
        status_frame.pack(fill="x", padx=15, pady=8)

        self.tool_ref_var = tk.StringVar(value="tool_reference : -")
        self.world_ref_var = tk.StringVar(value="world_reference: -")
        self.ref_frame_var = tk.StringVar(value="reference_frame: -")
        self.end_type_var = tk.StringVar(value="end_type       : -")
        self.angles_var = tk.StringVar(value="angles         : -")
        self.coords_var = tk.StringVar(value="coords         : -")

        ttk.Label(status_frame, textvariable=self.tool_ref_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.world_ref_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.ref_frame_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.end_type_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.angles_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.coords_var).pack(anchor="w", padx=8)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(self.root, textvariable=self.status_var).pack(fill="x", padx=15, pady=5)

    def request_robot_status(self):
        self.ros_node.request_status()
        self.status_var.set("Robot status requested")

    def poll_status_queue(self):
        try:
            while True:
                status = self.status_queue.get_nowait()
                self.handle_robot_status(status)
        except queue.Empty:
            pass

        self.root.after(100, self.poll_status_queue)

    def handle_robot_status(self, status):
        self.latest_status = status

        tool_reference = self.round_list(status["tool_reference"])
        world_reference = self.round_list(status["world_reference"])
        angles = self.round_list(status["angles"])
        coords = self.round_list(status["coords"])

        self.tool_ref_var.set(f"tool_reference : {tool_reference}")
        self.world_ref_var.set(f"world_reference: {world_reference}")
        self.ref_frame_var.set(f"reference_frame: {status['reference_frame']}")
        self.end_type_var.set(f"end_type       : {status['end_type']}")
        self.angles_var.set(f"angles         : {angles}")
        self.coords_var.set(f"coords         : {coords}")

        if self.pending_mode is not None:
            self.command_type = self.pending_mode
            self.pending_mode = None
            self.apply_mode_ui()

            if self.command_type == CMD_JOINT:
                self.set_slider_values(status["angles"])
                self.status_var.set("Mode changed to JOINT using current robot angles")
            else:
                self.set_slider_values(status["coords"])
                self.status_var.set("Mode changed to COORD using current robot coords")
        else:
            self.status_var.set("Robot status updated")

    def round_list(self, values):
        return [round(float(v), 2) for v in values]

    def toggle_mode(self):
        if self.command_type == CMD_JOINT:
            self.pending_mode = CMD_COORD
            self.status_var.set("Requesting robot coords for COORD mode...")
        else:
            self.pending_mode = CMD_JOINT
            self.status_var.set("Requesting robot angles for JOINT mode...")

        self.ros_node.request_status()

    def apply_mode_ui(self):
        labels = self.current_labels()
        limits = self.current_limits()

        self.mode_button.config(text=f"Mode: {self.current_mode_name()}")

        for i in range(6):
            low, high = limits[i]

            self.name_labels[i].config(text=labels[i])
            self.sliders[i].config(from_=low, to=high)
            self.limit_labels[i].config(text=f"[{low}, {high}]")

            value = self.value_vars[i].get()
            value = self.clamp(value, low, high)

            self.value_vars[i].set(value)
            self.entry_vars[i].set(f"{value:.2f}")
            self.value_labels[i].config(text=f"{value:.2f}")

        if self.command_type == CMD_COORD:
            self.coord_mode_frame.pack(fill="x", padx=15, pady=5)
        else:
            self.coord_mode_frame.pack_forget()

    def clamp(self, value, low, high):
        return max(low, min(high, float(value)))

    def on_slider_change(self, idx, value):
        value = float(value)
        self.value_labels[idx].config(text=f"{value:.2f}")
        self.entry_vars[idx].set(f"{value:.2f}")

    def apply_entry_to_slider(self, idx):
        limits = self.current_limits()
        low, high = limits[idx]

        try:
            value = float(self.entry_vars[idx].get())
        except ValueError:
            value = self.value_vars[idx].get()

        value = self.clamp(value, low, high)

        self.value_vars[idx].set(value)
        self.entry_vars[idx].set(f"{value:.2f}")
        self.value_labels[idx].config(text=f"{value:.2f}")

        return value

    def apply_all_entries_to_sliders(self):
        values = []

        for i in range(6):
            values.append(self.apply_entry_to_slider(i))

        return values

    def get_slider_values(self):
        values = []
        limits = self.current_limits()

        for i, var in enumerate(self.value_vars):
            low, high = limits[i]
            value = self.clamp(var.get(), low, high)
            values.append(round(value, 2))

        return values

    def set_slider_values(self, values):
        limits = self.current_limits()

        for i, value in enumerate(values):
            low, high = limits[i]
            value = self.clamp(value, low, high)

            self.value_vars[i].set(value)
            self.entry_vars[i].set(f"{value:.2f}")
            self.value_labels[i].config(text=f"{value:.2f}")

    def publish_current(self):
        # entry 값이 slider 값보다 우선
        values = self.apply_all_entries_to_sliders()

        speed = int(self.speed_var.get())
        coord_move_mode = int(self.coord_move_mode_var.get())

        self.ros_node.publish_command(
            self.command_type,
            values,
            speed,
            coord_move_mode,
        )

        self.status_var.set(
            f"Published {self.current_mode_name()}: "
            f"values={values}, speed={speed}, coord_move_mode={coord_move_mode}"
        )

    def go_joint_pose(self, name, angles):
        # named pose는 항상 joint command로만 보냄
        self.command_type = CMD_JOINT
        self.pending_mode = None
        self.apply_mode_ui()

        self.set_slider_values(angles)

        speed = int(self.speed_var.get())

        self.ros_node.publish_command(
            CMD_JOINT,
            angles,
            speed,
            0,
        )

        self.status_var.set(f"Published {name}: {angles}, speed={speed}")


def spin_ros(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS spin stopped: {e}")


def main():
    rclpy.init()

    status_queue = queue.Queue()

    ros_node = JetcobotGuiPublisher(status_queue)

    ros_thread = threading.Thread(
        target=spin_ros,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    root = tk.Tk()
    gui = JetcobotSliderGUI(root, ros_node, status_queue)

    def shutdown():
        try:
            root.quit()
        except Exception:
            pass

        try:
            root.destroy()
        except Exception:
            pass

        try:
            ros_node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def handle_sigint(signum, frame):
        shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)
    root.protocol("WM_DELETE_WINDOW", shutdown)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        shutdown()
    finally:
        shutdown()


if __name__ == "__main__":
    main()