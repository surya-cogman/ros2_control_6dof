#!/usr/bin/env python3
"""
gui_trap_full.py
ROS2 + PyQt5 GUI with trapezoidal, synchronized trajectories (radians).
Publishes to: /forward_command_controller/commands (Float64MultiArray)
Subscribes to: /joint_states
"""

import sys
import math
import time
import threading
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from PyQt5 import QtWidgets, QtCore

DT = 0.01  # 100 Hz publish rate


# -------- Trapezoid helpers --------
def compute_trap_params(distance: float, vmax: float, amax: float):
    """Return (t_acc, t_flat, t_total, v_cruise) for given positive distance, vmax, amax."""
    if distance <= 0.0:
        return 0.0, 0.0, 0.0, 0.0

    t_acc = vmax / amax
    d_acc = 0.5 * amax * t_acc * t_acc

    if 2 * d_acc >= distance:
        # triangle (no flat)
        t_acc = math.sqrt(distance / amax)
        t_flat = 0.0
        v_cruise = amax * t_acc
    else:
        d_flat = distance - 2 * d_acc
        t_flat = d_flat / vmax
        v_cruise = vmax

    t_total = 2 * t_acc + t_flat
    return t_acc, t_flat, t_total, v_cruise


def gen_trap_trajectory(q0: float, qf: float, t_total: float, vmax_user: float, amax_user: float, dt=DT) -> List[float]:
    """
    Generate sampled trapezoidal trajectory for a single joint that finishes at t_total.
    If t_total > 0, compute vmax that fits within t_total using amax_user as limit.
    If mode is va (t_total = None or 0), we assume vmax_user & amax_user are limits and compute t_total.
    """
    sign = 1.0 if (qf - q0) >= 0 else -1.0
    D = abs(qf - q0)
    if D == 0.0:
        # no motion
        steps = max(1, int(math.ceil((t_total or DT) / dt)))
        return [q0] * steps

    if t_total is not None and t_total > 0.0:
        # Solve for t_acc from quadratic: a*x^2 - a*T*x + D = 0 where x = t_acc, a = amax_user
        a = amax_user
        T = t_total
        if a <= 0:
            # fallback to default amax
            a = 1.0
        A = a
        B = -a * T
        C = D
        disc = B * B - 4 * A * C
        vmax_candidate = None
        if disc >= 0 and A != 0:
            sqrt_disc = math.sqrt(disc)
            x1 = (-B + sqrt_disc) / (2 * A)
            x2 = (-B - sqrt_disc) / (2 * A)
            t_acc_candidate = max(x1, x2)
            if 0 < t_acc_candidate < T / 2.0:
                vmax_candidate = a * t_acc_candidate

        if vmax_candidate is None:
            # fallback: use vmax_user or reasonable default
            vmax_candidate = vmax_user if vmax_user > 0 else min(
                1.0, D / T * 2.0)

        vmax = min(vmax_candidate,
                   vmax_user) if vmax_user > 0 else vmax_candidate
    else:
        # mode va: compute t_total from vmax_user & amax_user
        vmax = vmax_user
        a = amax_user
        if vmax <= 0 or a <= 0:
            # fallback defaults
            vmax = 1.0
            a = 1.0
        # compute times from compute_trap_params
        t_acc, t_flat, t_total_calc, v_cruise = compute_trap_params(D, vmax, a)
        t_total = t_total_calc
        amax_user = a

    # compute final trapezoid params for chosen vmax
    t_acc, t_flat, t_total_calc, v_cruise = compute_trap_params(
        D, vmax, amax_user)
    if t_total_calc <= 0:
        steps = 1
    else:
        steps = max(1, int(math.ceil(t_total_calc / dt)))

    traj = []
    for i in range(steps):
        t = i * dt
        if t <= t_acc:
            s = 0.5 * amax_user * t * t
        elif t <= (t_acc + t_flat):
            s = 0.5 * amax_user * t_acc * t_acc + v_cruise * (t - t_acc)
        elif t <= (2 * t_acc + t_flat):
            td = t - (t_acc + t_flat)
            s = 0.5 * amax_user * t_acc * t_acc + v_cruise * \
                t_flat + v_cruise * td - 0.5 * amax_user * td * td
        else:
            s = D
        pos = q0 + sign * s
        traj.append(pos)
    # ensure final exactly qf
    if len(traj) > 0:
        traj[-1] = qf
    return traj


# -------- GUI + ROS Node --------
class TrapGuiNode(Node, QtWidgets.QWidget):
    def __init__(self):
        Node.__init__(self, "trap_gui_node")
        QtWidgets.QWidget.__init__(self)

        # ROS pub/sub
        self.pub = self.create_publisher(
            Float64MultiArray, "/forward_command_controller/commands", 10)
        self.create_subscription(
            JointState, "/joint_states", self.joint_cb, 10)

        self.joint_count = 6
        self.joint_names = [f"joint_{i+1}" for i in range(self.joint_count)]
        self.current_q = [0.0] * self.joint_count

        # UI
        self.setWindowTitle("Trapezoidal Trajectory GUI (radians)")
        main_layout = QtWidgets.QVBoxLayout()

        # joint inputs
        grid = QtWidgets.QGridLayout()
        self.angle_inputs = []
        for i in range(self.joint_count):
            lbl = QtWidgets.QLabel(f"Joint {i+1} (rad):")
            inp = QtWidgets.QLineEdit("0.0")
            inp.setFixedWidth(120)
            self.angle_inputs.append(inp)
            grid.addWidget(lbl, i, 0)
            grid.addWidget(inp, i, 1)
        main_layout.addLayout(grid)

        # Mode radio buttons
        mode_layout = QtWidgets.QHBoxLayout()
        self.mode_time = QtWidgets.QRadioButton("Specify total time (T, sec)")
        self.mode_va = QtWidgets.QRadioButton(
            "Specify vmax & amax (rad/s, rad/s^2)")
        self.mode_time.setChecked(True)
        mode_layout.addWidget(self.mode_time)
        mode_layout.addWidget(self.mode_va)
        main_layout.addLayout(mode_layout)

        # connect to toggle function to enable/disable fields
        self.mode_time.toggled.connect(self.on_mode_toggle)

        # Time input
        h_time = QtWidgets.QHBoxLayout()
        self.time_label = QtWidgets.QLabel("T (sec):")
        self.time_input = QtWidgets.QLineEdit("3.0")
        self.time_input.setFixedWidth(120)
        h_time.addWidget(self.time_label)
        h_time.addWidget(self.time_input)
        main_layout.addLayout(h_time)

        # V/A inputs
        h_va = QtWidgets.QHBoxLayout()
        self.v_label = QtWidgets.QLabel("vmax (rad/s):")
        self.v_input = QtWidgets.QLineEdit("1.0")
        self.v_input.setFixedWidth(100)
        self.a_label = QtWidgets.QLabel("amax (rad/s^2):")
        self.a_input = QtWidgets.QLineEdit("1.0")
        self.a_input.setFixedWidth(100)
        h_va.addWidget(self.v_label)
        h_va.addWidget(self.v_input)
        h_va.addWidget(self.a_label)
        h_va.addWidget(self.a_input)
        main_layout.addLayout(h_va)

        # Control buttons
        btn_layout = QtWidgets.QHBoxLayout()
        self.send_btn = QtWidgets.QPushButton("Send Trajectory")
        self.stop_btn = QtWidgets.QPushButton("Emergency Stop")
        btn_layout.addWidget(self.send_btn)
        btn_layout.addWidget(self.stop_btn)
        main_layout.addLayout(btn_layout)

        # Status
        self.status_label = QtWidgets.QLabel("Status: Idle")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)
        self.resize(560, 420)

        # Button callbacks
        self.send_btn.clicked.connect(self.on_send)
        self.stop_btn.clicked.connect(self.on_stop)

        # Trajectory thread control
        self.traj_thread = None
        self.traj_stop = threading.Event()

        # tiny timer for UI (keeps it responsive)
        self.qtimer = QtCore.QTimer()
        self.qtimer.timeout.connect(lambda: None)
        self.qtimer.start(100)

        # enforce initial field states
        self.on_mode_toggle()

    def on_mode_toggle(self):
        """Enable/disable fields depending on selected mode."""
        if self.mode_time.isChecked():
            # Only time editable
            self.time_input.setEnabled(True)
            self.v_input.setEnabled(False)
            self.a_input.setEnabled(False)
        else:
            # Only v/a editable
            self.time_input.setEnabled(False)
            self.v_input.setEnabled(True)
            self.a_input.setEnabled(True)

    def joint_cb(self, msg: JointState):
        # robust mapping by names
        name_pos = {n: p for n, p in zip(msg.name, msg.position)}
        for i, jn in enumerate(self.joint_names):
            if jn in name_pos:
                self.current_q[i] = name_pos[jn]

    def on_stop(self):
        self.traj_stop.set()
        self.status_label.setText("Status: Stopped")
        # publish current positions once to hold
        msg = Float64MultiArray()
        msg.data = self.current_q.copy()
        self.pub.publish(msg)

    def on_send(self):
        # read target angles (radians)
        try:
            qf = [float(inp.text()) for inp in self.angle_inputs]
        except ValueError:
            self.status_label.setText("Status: Invalid joint inputs")
            return

        # read mode parameters
        if self.mode_time.isChecked():
            try:
                T = float(self.time_input.text())
                if T <= 0:
                    self.status_label.setText("Status: T must be > 0")
                    return
            except ValueError:
                self.status_label.setText("Status: Invalid T")
                return
            # read v/a as caps but disabled in UI; if user left them blank use defaults
            try:
                vmax_user = float(self.v_input.text()
                                  ) if self.v_input.text() else 1.0
            except ValueError:
                vmax_user = 1.0
            try:
                amax_user = float(self.a_input.text()
                                  ) if self.a_input.text() else 1.0
            except ValueError:
                amax_user = 1.0
            mode = "time"
            T_user = T
        else:
            # v/a mode
            try:
                vmax_user = float(self.v_input.text())
                amax_user = float(self.a_input.text())
                if vmax_user <= 0 or amax_user <= 0:
                    self.status_label.setText(
                        "Status: vmax and amax must be > 0")
                    return
            except ValueError:
                self.status_label.setText("Status: Invalid vmax/amax")
                return
            mode = "va"
            T_user = None

        # snapshot current q
        q0 = self.current_q.copy()

        # prevent multiple runs
        if self.traj_thread and self.traj_thread.is_alive():
            self.status_label.setText("Status: Trajectory already running")
            return

        # clear stop event and start thread
        self.traj_stop.clear()
        self.traj_thread = threading.Thread(
            target=self.execute_synced_traj,
            args=(q0, qf, mode, T_user, vmax_user, amax_user),
            daemon=True
        )
        self.traj_thread.start()

    def execute_synced_traj(self, q0, qf, mode, T_user, vmax_user, amax_user):
        self.status_label.setText("Status: Computing trajectories...")
        dt = DT
        per_trajs = []
        per_times = []

        # build per-joint trajectories
        for i in range(self.joint_count):
            if mode == "time":
                traj = gen_trap_trajectory(
                    q0[i], qf[i], T_user, vmax_user, amax_user, dt=dt)
            else:
                # compute t_total from user vmax/amx
                dist = abs(qf[i] - q0[i])
                if dist == 0:
                    traj = [q0[i]]
                else:
                    t_acc, t_flat, t_total, v_cruise = compute_trap_params(
                        dist, vmax_user, amax_user)
                    traj = gen_trap_trajectory(
                        q0[i], qf[i], t_total, vmax_user, amax_user, dt=dt)
            per_trajs.append(traj)
            per_times.append(len(traj) * dt)

        max_time = max(per_times) if any(per_times) else 0.0
        if max_time <= 0.0:
            self.status_label.setText("Status: No motion required")
            return

        steps = max(1, int(math.ceil(max_time / dt)))
        # pad/truncate to equal length
        synced = []
        for i in range(self.joint_count):
            traj = per_trajs[i]
            if len(traj) < steps:
                last = traj[-1] if traj else qf[i]
                traj_ext = traj + [last] * (steps - len(traj))
            elif len(traj) > steps:
                # downsample/truncate evenly
                indices = [
                    min(len(traj)-1, int(round(j * (len(traj)-1) / (steps-1)))) for j in range(steps)]
                traj_ext = [traj[idx] for idx in indices]
            else:
                traj_ext = traj
            synced.append(traj_ext)

        self.status_label.setText(f"Status: Executing (T={steps*dt:.2f}s)...")
        for s in range(steps):
            if self.traj_stop.is_set():
                self.status_label.setText("Status: Stopped by user")
                return
            msg = Float64MultiArray()
            msg.data = [synced[j][s] for j in range(self.joint_count)]
            self.pub.publish(msg)
            time.sleep(dt)

        # ensure final target publish
        final = Float64MultiArray()
        final.data = qf.copy()
        self.pub.publish(final)
        self.status_label.setText("Status: Completed")

    def closeEvent(self, event):
        self.traj_stop.set()
        if self.traj_thread and self.traj_thread.is_alive():
            self.traj_thread.join(timeout=1.0)
        event.accept()


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    node_gui = TrapGuiNode()

    # spin ROS in background thread
    def ros_spin():
        try:
            while rclpy.ok():
                rclpy.spin_once(node_gui, timeout_sec=0.01)
        except Exception:
            pass

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    node_gui.show()
    app.exec_()

    # teardown
    node_gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
