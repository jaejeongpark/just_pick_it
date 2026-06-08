#!/usr/bin/env python3

import argparse
import queue
import signal
import socket
import struct
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty

try:
    from PIL import Image, ImageTk
except ImportError:
    Image = None
    ImageTk = None


CMD_JOINT = 0
CMD_COORD = 1

ARM_RELEASE = 0
ARM_POWER_ON = 1

HEADER_FMT = ">IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

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
MIN_CALIB_IMAGES = 15


class UdpVideoReceiver:
    def __init__(self, frame_queue, status_queue):
        self.frame_queue = frame_queue
        self.status_queue = status_queue
        self.sock = None
        self.thread = None
        self.running = False
        self.port = None
        self.frames = {}
        self.last_frame_time = 0.0

    def start(self, port):
        self.stop()
        self.port = int(port)
        self.running = True
        self.frames = {}
        self.last_frame_time = 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", self.port))
            self.sock.settimeout(0.5)
            self.status_queue.put(f"Listening UDP video on port {self.port}")
        except Exception as e:
            self.status_queue.put(f"UDP bind failed on port {self.port}: {e}")
            self.running = False
            return

        while self.running:
            try:
                packet, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as e:
                self.status_queue.put(f"UDP receive error: {e}")
                continue

            if len(packet) <= HEADER_SIZE:
                continue

            try:
                frame_id, packet_idx, total_packets = struct.unpack(
                    HEADER_FMT,
                    packet[:HEADER_SIZE],
                )
                chunk = packet[HEADER_SIZE:]
            except Exception:
                continue

            if total_packets <= 0:
                continue

            if frame_id not in self.frames:
                self.frames[frame_id] = {
                    "total": total_packets,
                    "chunks": {},
                    "time": time.time(),
                }

            self.frames[frame_id]["chunks"][packet_idx] = chunk

            now = time.time()
            old_ids = [
                fid for fid, info in self.frames.items()
                if now - info["time"] > 2.0
            ]
            for fid in old_ids:
                self.frames.pop(fid, None)

            info = self.frames.get(frame_id)
            if info is None:
                continue

            if len(info["chunks"]) == info["total"]:
                try:
                    jpg_data = b"".join(
                        info["chunks"][i] for i in range(info["total"])
                    )
                except KeyError:
                    self.frames.pop(frame_id, None)
                    continue

                self.frames.pop(frame_id, None)

                np_data = np.frombuffer(jpg_data, dtype=np.uint8)
                img_bgr = cv2.imdecode(np_data, cv2.IMREAD_COLOR)

                if img_bgr is None:
                    continue

                img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

                try:
                    while not self.frame_queue.empty():
                        self.frame_queue.get_nowait()
                except queue.Empty:
                    pass

                self.frame_queue.put(img_rgb)
                self.last_frame_time = time.time()

        self.status_queue.put("UDP video receiver stopped")


class JetcobotGuiPublisher(Node):
    def __init__(self, status_queue, robot_name):
        node_name = f"local_{robot_name}_gui_publisher"
        super().__init__(node_name)

        self.status_queue = status_queue
        self.robot_name = robot_name
        self.ns = f"/{robot_name}"

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/target_pose",
            10,
        )

        self.status_request_pub = self.create_publisher(
            Empty,
            f"{self.ns}/request_status",
            10,
        )

        self.tool_reference_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/set_tool_reference",
            10,
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/set_gripper",
            10,
        )

        self.arm_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/set_arm",
            10,
        )

        self.status_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/status",
            self.status_callback,
            10,
        )

        self.get_logger().info(f"GUI namespace: {self.ns}")
        self.get_logger().info(f"Pub: {self.ns}/target_pose")
        self.get_logger().info(f"Pub: {self.ns}/set_gripper")
        self.get_logger().info(f"Pub: {self.ns}/set_tool_reference")
        self.get_logger().info(f"Pub: {self.ns}/set_arm")
        self.get_logger().info(f"Pub: {self.ns}/request_status")
        self.get_logger().info(f"Sub: {self.ns}/status")

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

    def publish_tool_reference(self, values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.tool_reference_pub.publish(msg)

    def publish_gripper(self, value, speed):
        msg = Float64MultiArray()
        msg.data = [float(value), float(speed)]
        self.gripper_pub.publish(msg)

    def publish_arm(self, command):
        msg = Float64MultiArray()
        msg.data = [float(command)]
        self.arm_pub.publish(msg)

    def status_callback(self, msg):
        data = list(msg.data)

        if len(data) not in [26, 27]:
            self.get_logger().warn(
                f"Invalid status length: {len(data)}. Expected 26 or 27."
            )
            return

        status = {
            "tool_reference": data[0:6],
            "world_reference": data[6:12],
            "reference_frame": int(data[12]),
            "end_type": int(data[13]),
            "angles": data[14:20],
            "coords": data[20:26],
            "gripper_value": data[26] if len(data) == 27 else -1.0,
        }

        self.status_queue.put(status)


class JetcobotSliderGUI:
    def __init__(self, root, ros_node, status_queue, robot_name, initial_udp_port=""):
        self.root = root
        self.ros_node = ros_node
        self.status_queue = status_queue
        self.robot_name = robot_name

        self.root.title(f"Local Jetcobot Controller - {robot_name}")

        self.command_type = CMD_JOINT
        self.pending_mode = None
        self.latest_status = None

        self.value_vars = []
        self.entry_vars = []
        self.name_labels = []
        self.value_labels = []
        self.sliders = []
        self.limit_labels = []

        self.tool_ref_entry_vars = []
        self.corner_preview_photos = []

        self.speed_var = tk.IntVar(value=DEFAULT_SPEED)
        self.coord_move_mode_var = tk.IntVar(value=0)

        self.gripper_value_var = tk.DoubleVar(value=50.0)
        self.gripper_entry_var = tk.StringVar(value="50.00")

        self.udp_port_var = tk.StringVar(value=str(initial_udp_port) if initial_udp_port else "")
        self.video_status_var = tk.StringVar(value="UDP port 입력 후 Enter")
        self.capture_status_var = tk.StringVar(value="Capture: -")
        self.calib_status_var = tk.StringVar(value="Calibration: -")

        self.video_frame_queue = queue.Queue(maxsize=1)
        self.video_status_queue = queue.Queue()
        self.video_receiver = UdpVideoReceiver(
            self.video_frame_queue,
            self.video_status_queue,
        )

        self.video_photo_raw = None
        self.video_photo_rect = None
        self.last_video_frame_time = 0.0
        self.latest_frame_rgb = None

        self.capture_dir = Path.cwd() / f"{robot_name}_captures"
        self.capture_dir.mkdir(parents=True, exist_ok=True)

        self.calib_K = None
        self.calib_dist = None
        self.calib_image_size = None

        self.calib_board_w_var = tk.StringVar(value="8")
        self.calib_board_h_var = tk.StringVar(value="6")
        self.calib_square_size_var = tk.StringVar(value="0.025")
        self.calib_output_var = tk.StringVar(
            value=str(self.capture_dir / "camera_calibration.yaml")
        )

        self.center_angles = [57.40, 82.88, -21.80, -97.20, 29.35, -155.91]
        self.left_scan_angles = [83.37, 82.88, -21.80, -97.20, 29.35, -155.91]
        self.right_scan_angles = [38.12, 82.88, -21.80, -97.20, 29.35, -155.91]
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.build_ui()
        self.apply_mode_ui()

        self.load_calibration_if_exists()
        self.refresh_corner_preview_slots()

        self.root.after(100, self.poll_status_queue)
        self.root.after(30, self.poll_video_frame_queue)
        self.root.after(300, self.poll_video_status_queue)
        self.ros_node.request_status()

        if initial_udp_port:
            self.root.after(500, self.start_video_receiver)

    def current_limits(self):
        return JOINT_LIMITS if self.command_type == CMD_JOINT else COORD_LIMITS

    def current_labels(self):
        return JOINT_LABELS if self.command_type == CMD_JOINT else COORD_LABELS

    def current_mode_name(self):
        return "JOINT / send_angles" if self.command_type == CMD_JOINT else "COORD / send_coords"

    def build_ui(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True)

        self.motion_tab = ttk.Frame(self.notebook)
        self.tool_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.motion_tab, text="Motion Control")
        self.notebook.add(self.tool_tab, text="Tool Reference")

        self.build_motion_tab()
        self.build_tool_reference_tab()

    def build_motion_tab(self):
        title = ttk.Label(
            self.motion_tab,
            text=f"Jetcobot Motion Control - {self.robot_name}",
            font=("Arial", 15, "bold"),
        )
        title.pack(pady=8)

        ns_label = ttk.Label(
            self.motion_tab,
            text=f"ROS namespace: /{self.robot_name}",
        )
        ns_label.pack(pady=2)

        body_frame = ttk.Frame(self.motion_tab)
        body_frame.pack(fill="both", expand=True)

        left_panel = ttk.Frame(body_frame)
        left_panel.pack(side="left", fill="both", expand=True)

        right_panel = ttk.Frame(body_frame)
        right_panel.pack(side="right", fill="both", padx=10, pady=5)

        top_frame = ttk.Frame(left_panel)
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

        arm_frame = ttk.LabelFrame(left_panel, text="Arming Control")
        arm_frame.pack(fill="x", padx=15, pady=6)

        ttk.Button(
            arm_frame,
            text="Power ON / Arm",
            command=self.arm_power_on,
        ).pack(side="left", padx=5, pady=5)

        ttk.Button(
            arm_frame,
            text="Release Servos / Disarm",
            command=self.arm_release,
        ).pack(side="left", padx=5, pady=5)

        self.coord_mode_frame = ttk.Frame(left_panel)
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

        main_frame = ttk.Frame(left_panel)
        main_frame.pack(fill="both", expand=True, padx=15, pady=8)

        for i in range(6):
            row = ttk.Frame(main_frame)
            row.pack(fill="x", pady=4)

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

        self.build_gripper_control(left_panel)

        button_frame = ttk.Frame(left_panel)
        button_frame.pack(fill="x", padx=15, pady=8)

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

        status_frame = ttk.LabelFrame(left_panel, text="Robot Status")
        status_frame.pack(fill="x", padx=15, pady=8)

        self.tool_ref_var = tk.StringVar(value="tool_reference : -")
        self.world_ref_var = tk.StringVar(value="world_reference: -")
        self.ref_frame_var = tk.StringVar(value="reference_frame: -")
        self.end_type_var = tk.StringVar(value="end_type       : -")
        self.angles_var = tk.StringVar(value="angles         : -")
        self.coords_var = tk.StringVar(value="coords         : -")
        self.gripper_var = tk.StringVar(value="gripper       : -")

        ttk.Label(status_frame, textvariable=self.tool_ref_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.world_ref_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.ref_frame_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.end_type_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.angles_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.coords_var).pack(anchor="w", padx=8)
        ttk.Label(status_frame, textvariable=self.gripper_var).pack(anchor="w", padx=8)

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(left_panel, textvariable=self.status_var).pack(
            fill="x",
            padx=15,
            pady=5,
        )

        self.build_video_monitor(right_panel)

    def build_video_monitor(self, parent):
        video_frame = ttk.LabelFrame(parent, text=f"Camera UDP Monitor - {self.robot_name}")
        video_frame.pack(fill="both", expand=True)

        top = ttk.Frame(video_frame)
        top.pack(fill="x", padx=8, pady=6)

        ttk.Label(top, text="UDP Port").pack(side="left", padx=5)

        port_entry = ttk.Entry(top, textvariable=self.udp_port_var, width=10)
        port_entry.pack(side="left", padx=5)
        port_entry.bind("<Return>", lambda event: self.start_video_receiver())

        ttk.Button(top, text="Start", command=self.start_video_receiver).pack(side="left", padx=5)
        ttk.Button(top, text="Stop", command=self.stop_video_receiver).pack(side="left", padx=5)

        image_row = ttk.Frame(video_frame)
        image_row.pack(fill="both", expand=True, padx=8, pady=8)

        raw_frame = ttk.LabelFrame(image_row, text="Raw")
        raw_frame.pack(side="left", fill="both", expand=True, padx=4)

        rect_frame = ttk.LabelFrame(image_row, text="Rectified")
        rect_frame.pack(side="left", fill="both", expand=True, padx=4)

        self.video_label_raw = tk.Label(
            raw_frame,
            text="UDP port 입력 후 Enter\n아직 영상을 받지 못했습니다.",
            width=40,
            height=18,
            bg="black",
            fg="white",
            anchor="center",
            justify="center",
        )
        self.video_label_raw.pack(fill="both", expand=True, padx=4, pady=4)

        self.video_label_rect = tk.Label(
            rect_frame,
            text="calibration.yaml 없음\ncalibration 후 rectified 표시",
            width=40,
            height=18,
            bg="black",
            fg="white",
            anchor="center",
            justify="center",
        )
        self.video_label_rect.pack(fill="both", expand=True, padx=4, pady=4)

        capture_row = ttk.Frame(video_frame)
        capture_row.pack(fill="x", padx=8, pady=4)

        ttk.Button(
            capture_row,
            text="Capture Image",
            command=self.capture_current_frame,
        ).pack(side="left", padx=5)

        ttk.Label(capture_row, text=f"Save dir: {self.capture_dir}").pack(side="left", padx=5)

        calib_frame = ttk.LabelFrame(video_frame, text="Camera Calibration")
        calib_frame.pack(fill="x", padx=8, pady=6)

        ttk.Label(
            calib_frame,
            text="최소 15장 이상, 다양한 자세/화면 위치/측정 거리 중심으로 캡쳐하세요.",
        ).pack(anchor="w", padx=5, pady=3)

        row1 = ttk.Frame(calib_frame)
        row1.pack(fill="x", padx=5, pady=4)

        ttk.Label(row1, text="Board W").pack(side="left", padx=3)
        ttk.Entry(row1, textvariable=self.calib_board_w_var, width=6).pack(side="left", padx=3)

        ttk.Label(row1, text="Board H").pack(side="left", padx=3)
        ttk.Entry(row1, textvariable=self.calib_board_h_var, width=6).pack(side="left", padx=3)

        ttk.Label(row1, text="Square m").pack(side="left", padx=3)
        ttk.Entry(row1, textvariable=self.calib_square_size_var, width=8).pack(side="left", padx=3)

        row2 = ttk.Frame(calib_frame)
        row2.pack(fill="x", padx=5, pady=4)

        ttk.Label(row2, text="Output").pack(side="left", padx=3)
        ttk.Entry(row2, textvariable=self.calib_output_var, width=45).pack(
            side="left",
            padx=3,
            fill="x",
            expand=True,
        )

        ttk.Button(row2, text="Run Calibration", command=self.run_camera_calibration).pack(side="left", padx=5)
        ttk.Button(row2, text="Refresh Corners", command=self.refresh_corner_preview_slots).pack(side="left", padx=5)

        ttk.Label(video_frame, textvariable=self.video_status_var).pack(fill="x", padx=8, pady=2)
        ttk.Label(video_frame, textvariable=self.capture_status_var).pack(fill="x", padx=8, pady=2)
        ttk.Label(video_frame, textvariable=self.calib_status_var).pack(fill="x", padx=8, pady=2)

        preview_frame = ttk.LabelFrame(video_frame, text="Detected Corner Preview Slots")
        preview_frame.pack(fill="both", expand=True, padx=8, pady=6)

        self.corner_canvas = tk.Canvas(preview_frame, height=170)
        self.corner_canvas.pack(side="left", fill="both", expand=True)

        self.corner_scrollbar = ttk.Scrollbar(
            preview_frame,
            orient="vertical",
            command=self.corner_canvas.yview,
        )
        self.corner_scrollbar.pack(side="right", fill="y")
        self.corner_canvas.configure(yscrollcommand=self.corner_scrollbar.set)

        self.corner_inner = ttk.Frame(self.corner_canvas)
        self.corner_canvas.create_window((0, 0), window=self.corner_inner, anchor="nw")

        self.corner_inner.bind(
            "<Configure>",
            lambda event: self.corner_canvas.configure(scrollregion=self.corner_canvas.bbox("all")),
        )

    def build_gripper_control(self, parent):
        gripper_frame = ttk.LabelFrame(parent, text="Gripper Control")
        gripper_frame.pack(fill="x", padx=15, pady=8)

        ttk.Label(gripper_frame, text="Open amount").pack(side="left", padx=5)

        self.gripper_slider = ttk.Scale(
            gripper_frame,
            from_=0.0,
            to=100.0,
            orient="horizontal",
            variable=self.gripper_value_var,
            command=self.on_gripper_slider_change,
        )
        self.gripper_slider.pack(side="left", fill="x", expand=True, padx=8)

        self.gripper_entry = ttk.Entry(gripper_frame, textvariable=self.gripper_entry_var, width=10)
        self.gripper_entry.pack(side="left", padx=5)
        self.gripper_entry.bind("<Return>", lambda event: self.apply_gripper_entry())

        self.gripper_value_label = ttk.Label(gripper_frame, text="50.00", width=8)
        self.gripper_value_label.pack(side="left", padx=5)

        ttk.Button(gripper_frame, text="Apply Gripper", command=self.publish_gripper_current).pack(side="left", padx=5)
        ttk.Button(gripper_frame, text="Open 100", command=lambda: self.set_and_publish_gripper(100.0)).pack(side="left", padx=5)
        ttk.Button(gripper_frame, text="Close 0", command=lambda: self.set_and_publish_gripper(0.0)).pack(side="left", padx=5)

    def build_tool_reference_tab(self):
        frame = ttk.Frame(self.tool_tab)
        frame.pack(fill="both", expand=True, padx=15, pady=15)

        ttk.Label(
            frame,
            text=f"Tool Reference Setting - {self.robot_name}",
            font=("Arial", 14, "bold"),
        ).pack(anchor="w", pady=8)

        desc = (
            "tool_reference = [x, y, z, rx, ry, rz]\n"
            "flange 기준 tool/TCP offset 설정값입니다.\n"
            "reference_frame은 base로 유지하고, 여기서는 tool_reference만 변경합니다."
        )
        ttk.Label(frame, text=desc).pack(anchor="w", pady=5)

        editor = ttk.LabelFrame(frame, text="set_tool_reference([x, y, z, rx, ry, rz])")
        editor.pack(fill="x", pady=10)

        labels = ["x", "y", "z", "rx", "ry", "rz"]
        row = ttk.Frame(editor)
        row.pack(fill="x", padx=8, pady=8)

        for label in labels:
            cell = ttk.Frame(row)
            cell.pack(side="left", padx=5)

            ttk.Label(cell, text=label).pack()

            var = tk.StringVar(value="0.00")
            entry = ttk.Entry(cell, textvariable=var, width=10)
            entry.pack()

            self.tool_ref_entry_vars.append(var)

        button_row = ttk.Frame(editor)
        button_row.pack(fill="x", padx=8, pady=8)

        ttk.Button(button_row, text="Apply Tool Reference", command=self.apply_tool_reference).pack(side="left", padx=5)
        ttk.Button(button_row, text="Reset [0,0,0,0,0,0]", command=self.reset_tool_reference).pack(side="left", padx=5)
        ttk.Button(button_row, text="Robot 상태 읽기", command=self.request_robot_status).pack(side="left", padx=5)

        self.tool_ref_status_var = tk.StringVar(value="tool_reference: -")
        ttk.Label(frame, textvariable=self.tool_ref_status_var).pack(anchor="w", pady=8)

    def arm_power_on(self):
        self.ros_node.publish_arm(ARM_POWER_ON)
        self.status_var.set(f"ARM requested to /{self.robot_name}: power_on()")

    def arm_release(self):
        self.ros_node.publish_arm(ARM_RELEASE)
        self.status_var.set(f"DISARM requested to /{self.robot_name}: release_all_servos()")

    def get_capture_image_paths(self):
        return sorted(
            list(self.capture_dir.glob("*.png"))
            + list(self.capture_dir.glob("*.jpg"))
            + list(self.capture_dir.glob("*.jpeg"))
        )

    def get_calib_params(self):
        board_w = int(self.calib_board_w_var.get())
        board_h = int(self.calib_board_h_var.get())
        square_size = float(self.calib_square_size_var.get())
        output_file = Path(self.calib_output_var.get()).expanduser()
        return board_w, board_h, square_size, output_file

    def find_chessboard_on_image(self, img_bgr, board_w, board_h):
        pattern_size = (board_w, board_h)
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if not found:
            return False, None

        refine_criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001,
        )

        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), refine_criteria)
        return True, corners_refined

    def refresh_corner_preview_slots(self):
        if Image is None or ImageTk is None:
            return

        for child in self.corner_inner.winfo_children():
            child.destroy()

        self.corner_preview_photos.clear()

        try:
            board_w, board_h, _, _ = self.get_calib_params()
        except Exception:
            self.calib_status_var.set("Corner preview failed: calibration parameter error")
            return

        paths = self.get_capture_image_paths()
        total = len(paths)

        if total < MIN_CALIB_IMAGES:
            self.calib_status_var.set(
                f"Warning: captured {total}/{MIN_CALIB_IMAGES}. 더 다양한 자세/거리에서 캡쳐하세요."
            )

        valid = 0

        for idx, path in enumerate(paths):
            img_bgr = cv2.imread(str(path))
            if img_bgr is None:
                continue

            found, corners = self.find_chessboard_on_image(img_bgr, board_w, board_h)
            if not found:
                continue

            valid += 1
            display = img_bgr.copy()
            cv2.drawChessboardCorners(display, (board_w, board_h), corners, found)
            display_rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

            h, w = display_rgb.shape[:2]
            thumb_w = 180
            thumb_h = max(1, int(h * thumb_w / w))
            display_rgb = cv2.resize(display_rgb, (thumb_w, thumb_h))

            pil_img = Image.fromarray(display_rgb)
            photo = ImageTk.PhotoImage(pil_img)
            self.corner_preview_photos.append(photo)

            slot = ttk.Frame(self.corner_inner)
            slot.grid(row=valid - 1, column=0, sticky="w", padx=4, pady=4)

            img_label = ttk.Label(slot, image=photo)
            img_label.pack(side="left")

            text = f"{idx + 1:03d}. {path.name}\ncorner: OK"
            ttk.Label(slot, text=text).pack(side="left", padx=8)

        if total >= MIN_CALIB_IMAGES:
            self.calib_status_var.set(
                f"Corner preview: valid={valid}, total={total}. 검출 실패 이미지는 preview에서 제외됨."
            )

    def run_camera_calibration(self):
        try:
            board_w, board_h, square_size, output_file = self.get_calib_params()
        except Exception:
            self.calib_status_var.set("Calibration failed: board/square/output 설정 오류")
            return

        image_paths = self.get_capture_image_paths()
        total = len(image_paths)

        if total < MIN_CALIB_IMAGES:
            self.calib_status_var.set(
                f"Calibration warning: 캡쳐 이미지 {total}/{MIN_CALIB_IMAGES}. 최소 15장 이상 더 찍으세요."
            )
            self.refresh_corner_preview_slots()
            return

        objp = np.zeros((board_h * board_w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
        objp *= square_size

        obj_points = []
        img_points = []
        img_size = None
        ok_count = 0

        for path in image_paths:
            img_bgr = cv2.imread(str(path))
            if img_bgr is None:
                continue

            gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

            if img_size is None:
                img_size = (gray.shape[1], gray.shape[0])

            found, corners = self.find_chessboard_on_image(img_bgr, board_w, board_h)
            if not found:
                continue

            obj_points.append(objp)
            img_points.append(corners)
            ok_count += 1

        self.refresh_corner_preview_slots()

        if ok_count < MIN_CALIB_IMAGES:
            self.calib_status_var.set(
                f"Calibration failed: corner 유효 이미지 {ok_count}/{MIN_CALIB_IMAGES}. 검출 잘되는 이미지를 더 찍으세요."
            )
            return

        ret, K, dist, _, _ = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)

        self.save_camera_calibration_yaml(
            output_file=output_file,
            img_size=img_size,
            K=K,
            dist=dist,
            rms=ret,
            valid_count=ok_count,
            total_count=total,
        )

        self.calib_K = K
        self.calib_dist = dist
        self.calib_image_size = img_size

        self.calib_status_var.set(
            f"Calibration done: RMS={ret:.4f}px, valid={ok_count}/{total}, saved={output_file}"
        )

        if self.latest_frame_rgb is not None:
            self.show_video_frame(self.latest_frame_rgb)

    def save_camera_calibration_yaml(self, output_file, img_size, K, dist, rms, valid_count, total_count):
        output_file = Path(output_file).expanduser()
        output_file.parent.mkdir(parents=True, exist_ok=True)

        calib = {
            "image_width": int(img_size[0]),
            "image_height": int(img_size[1]),
            "camera_name": f"{self.robot_name}_camera",
            "distortion_model": "plumb_bob",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": K.ravel().tolist(),
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(dist.size),
                "data": dist.ravel().tolist(),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": np.eye(3).ravel().tolist(),
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": np.hstack([K, np.zeros((3, 1))]).ravel().tolist(),
            },
            "calibration_info": {
                "robot_name": self.robot_name,
                "rms_reprojection_error_px": float(rms),
                "source_image_dir": str(self.capture_dir),
                "valid_images": int(valid_count),
                "total_images": int(total_count),
            },
        }

        with open(output_file, "w") as f:
            yaml.dump(calib, f, default_flow_style=False, sort_keys=False)

    def load_calibration_if_exists(self):
        path = Path(self.calib_output_var.get()).expanduser()

        if not path.exists():
            self.calib_status_var.set(f"Calibration YAML 없음: {path}. calibration 후 rectified 표시.")
            return False

        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)

            K_data = data["camera_matrix"]["data"]
            dist_data = data["distortion_coefficients"]["data"]

            self.calib_K = np.array(K_data, dtype=np.float64).reshape(3, 3)
            self.calib_dist = np.array(dist_data, dtype=np.float64).reshape(1, -1)
            self.calib_image_size = (int(data["image_width"]), int(data["image_height"]))

            self.calib_status_var.set(f"Calibration loaded: {path}")
            return True

        except Exception as e:
            self.calib_K = None
            self.calib_dist = None
            self.calib_image_size = None
            self.calib_status_var.set(f"Calibration load failed: {e}")
            return False

    def rectify_frame_rgb(self, frame_rgb):
        if self.calib_K is None or self.calib_dist is None:
            return None

        try:
            return cv2.undistort(frame_rgb, self.calib_K, self.calib_dist)
        except Exception:
            return None

    def capture_current_frame(self):
        if self.latest_frame_rgb is None:
            self.capture_status_var.set("Capture failed: 아직 수신된 프레임이 없습니다.")
            return

        try:
            self.capture_dir.mkdir(parents=True, exist_ok=True)

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            save_path = self.capture_dir / f"{self.robot_name}_capture_{timestamp}.png"

            frame_bgr = cv2.cvtColor(self.latest_frame_rgb, cv2.COLOR_RGB2BGR)
            ok = cv2.imwrite(str(save_path), frame_bgr)

            if ok:
                paths = self.get_capture_image_paths()
                if len(paths) < MIN_CALIB_IMAGES:
                    self.capture_status_var.set(
                        f"Captured: {save_path} | {len(paths)}/{MIN_CALIB_IMAGES}, 더 찍으세요."
                    )
                else:
                    self.capture_status_var.set(f"Captured: {save_path}")

                self.refresh_corner_preview_slots()
            else:
                self.capture_status_var.set("Capture failed: cv2.imwrite returned False")

        except Exception as e:
            self.capture_status_var.set(f"Capture failed: {e}")

    def start_video_receiver(self):
        port_text = self.udp_port_var.get().strip()

        try:
            port = int(port_text)
            if not (1 <= port <= 65535):
                raise ValueError()
        except ValueError:
            self.video_status_var.set("Invalid UDP port")
            self.video_label_raw.config(image="", text="Invalid UDP port\n1~65535 사이 값을 입력하세요.")
            return

        if Image is None or ImageTk is None:
            self.video_status_var.set("Pillow not installed: pip install pillow")
            self.video_label_raw.config(image="", text="Pillow가 필요합니다.\npip install pillow")
            return

        self.load_calibration_if_exists()
        self.video_receiver.start(port)
        self.latest_frame_rgb = None
        self.last_video_frame_time = 0.0

        self.video_status_var.set(f"Listening UDP video on port {port}")
        self.capture_status_var.set("Capture: 아직 수신된 프레임 없음")
        self.video_label_raw.config(image="", text=f"Listening on UDP port {port}\n아직 영상을 받지 못했습니다.")

        if self.calib_K is None:
            self.video_label_rect.config(image="", text="calibration.yaml 없음\ncalibration 후 rectified 표시")

    def stop_video_receiver(self):
        self.video_receiver.stop()
        self.video_status_var.set("UDP video stopped")
        self.video_label_raw.config(image="", text="UDP video stopped")

    def poll_video_status_queue(self):
        try:
            while True:
                msg = self.video_status_queue.get_nowait()
                self.video_status_var.set(str(msg))
        except queue.Empty:
            pass

        if self.video_receiver.running:
            now = time.time()
            if self.last_video_frame_time != 0.0 and now - self.last_video_frame_time > 2.0:
                self.video_status_var.set("Listening... 최근 2초 동안 새 프레임 없음")

        self.root.after(300, self.poll_video_status_queue)

    def poll_video_frame_queue(self):
        try:
            frame_rgb = self.video_frame_queue.get_nowait()
        except queue.Empty:
            self.root.after(30, self.poll_video_frame_queue)
            return

        self.last_video_frame_time = time.time()
        self.latest_frame_rgb = frame_rgb.copy()

        self.show_video_frame(frame_rgb)
        self.root.after(30, self.poll_video_frame_queue)

    def show_video_frame(self, frame_rgb):
        if Image is None or ImageTk is None:
            return

        self.show_image_on_label(frame_rgb, self.video_label_raw, is_raw=True)

        rectified = self.rectify_frame_rgb(frame_rgb)

        if rectified is None:
            self.video_label_rect.config(
                image="",
                text="calibration.yaml 없음\ncalibration 후 rectified 표시",
                width=40,
                height=18,
            )
        else:
            self.show_image_on_label(rectified, self.video_label_rect, is_raw=False)

        h, w = frame_rgb.shape[:2]
        if rectified is None:
            self.video_status_var.set(f"Receiving raw video: {w}x{h}")
        else:
            self.video_status_var.set(f"Receiving raw + rectified: {w}x{h}")

    def show_image_on_label(self, frame_rgb, label, is_raw):
        max_w = 380
        max_h = 300

        h, w = frame_rgb.shape[:2]
        scale = min(max_w / w, max_h / h, 1.0)
        new_w = int(w * scale)
        new_h = int(h * scale)

        display_rgb = frame_rgb
        if scale != 1.0:
            display_rgb = cv2.resize(frame_rgb, (new_w, new_h))

        image = Image.fromarray(display_rgb)
        photo = ImageTk.PhotoImage(image=image)

        if is_raw:
            self.video_photo_raw = photo
        else:
            self.video_photo_rect = photo

        label.config(image=photo, text="", width=new_w, height=new_h)

    def request_robot_status(self):
        self.ros_node.request_status()
        self.status_var.set(f"Robot status requested: /{self.robot_name}/request_status")

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
        gripper_value = float(status.get("gripper_value", -1.0))

        self.tool_ref_var.set(f"tool_reference : {tool_reference}")
        self.world_ref_var.set(f"world_reference: {world_reference}")
        self.ref_frame_var.set(f"reference_frame: {status['reference_frame']}")
        self.end_type_var.set(f"end_type       : {status['end_type']}")
        self.angles_var.set(f"angles         : {angles}")
        self.coords_var.set(f"coords         : {coords}")
        self.gripper_var.set(f"gripper       : {gripper_value:.2f}")

        self.set_tool_reference_entries(status["tool_reference"])
        self.tool_ref_status_var.set(f"tool_reference: {tool_reference}")

        if gripper_value >= 0.0:
            self.set_gripper_value(gripper_value)

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
        return [self.apply_entry_to_slider(i) for i in range(6)]

    def set_slider_values(self, values):
        limits = self.current_limits()

        for i, value in enumerate(values):
            low, high = limits[i]
            value = self.clamp(value, low, high)

            self.value_vars[i].set(value)
            self.entry_vars[i].set(f"{value:.2f}")
            self.value_labels[i].config(text=f"{value:.2f}")

    def publish_current(self):
        values = self.apply_all_entries_to_sliders()
        speed = int(self.speed_var.get())
        coord_move_mode = int(self.coord_move_mode_var.get())

        self.ros_node.publish_command(self.command_type, values, speed, coord_move_mode)

        self.status_var.set(
            f"Published to /{self.robot_name}: {self.current_mode_name()}, "
            f"values={values}, speed={speed}, coord_move_mode={coord_move_mode}"
        )

    def go_joint_pose(self, name, angles):
        self.command_type = CMD_JOINT
        self.pending_mode = None
        self.apply_mode_ui()
        self.set_slider_values(angles)

        speed = int(self.speed_var.get())
        self.ros_node.publish_command(CMD_JOINT, angles, speed, 0)

        self.status_var.set(f"Published {name} to /{self.robot_name}: {angles}, speed={speed}")

    def on_gripper_slider_change(self, value):
        value = float(value)
        self.gripper_entry_var.set(f"{value:.2f}")
        self.gripper_value_label.config(text=f"{value:.2f}")

    def apply_gripper_entry(self):
        try:
            value = float(self.gripper_entry_var.get())
        except ValueError:
            value = self.gripper_value_var.get()

        value = max(0.0, min(100.0, value))
        self.set_gripper_value(value)
        return value

    def set_gripper_value(self, value):
        value = max(0.0, min(100.0, float(value)))
        self.gripper_value_var.set(value)
        self.gripper_entry_var.set(f"{value:.2f}")
        self.gripper_value_label.config(text=f"{value:.2f}")

    def publish_gripper_current(self):
        value = self.apply_gripper_entry()
        speed = int(self.speed_var.get())
        self.ros_node.publish_gripper(value, speed)
        self.status_var.set(f"Published gripper to /{self.robot_name}: value={value:.2f}, speed={speed}")

    def set_and_publish_gripper(self, value):
        self.set_gripper_value(value)
        self.publish_gripper_current()

    def read_tool_reference_entries(self):
        values = []
        for var in self.tool_ref_entry_vars:
            try:
                values.append(float(var.get()))
            except ValueError:
                values.append(0.0)
        return values

    def set_tool_reference_entries(self, values):
        for var, value in zip(self.tool_ref_entry_vars, values):
            var.set(f"{float(value):.2f}")

    def apply_tool_reference(self):
        values = self.read_tool_reference_entries()
        self.ros_node.publish_tool_reference(values)
        self.status_var.set(f"set_tool_reference requested to /{self.robot_name}: {values}")
        self.tool_ref_status_var.set(f"tool_reference requested: {values}")

    def reset_tool_reference(self):
        values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.set_tool_reference_entries(values)
        self.ros_node.publish_tool_reference(values)
        self.status_var.set(f"set_tool_reference reset requested to /{self.robot_name}")
        self.tool_ref_status_var.set("tool_reference requested: [0,0,0,0,0,0]")

    def shutdown(self):
        self.video_receiver.stop()


def spin_ros(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS spin stopped: {e}")


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", type=str, default="jetcobot1")
    parser.add_argument("--udp-port", type=str, default="")
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()

    status_queue = queue.Queue()

    ros_node = JetcobotGuiPublisher(
        status_queue=status_queue,
        robot_name=args.robot_name,
    )

    ros_thread = threading.Thread(
        target=spin_ros,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    root = tk.Tk()
    gui = JetcobotSliderGUI(
        root=root,
        ros_node=ros_node,
        status_queue=status_queue,
        robot_name=args.robot_name,
        initial_udp_port=args.udp_port,
    )

    def shutdown():
        print("Shutting down local GUI publisher...")

        try:
            gui.shutdown()
        except Exception:
            pass

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