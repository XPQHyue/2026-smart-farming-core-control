#!/usr/bin/env python3
from __future__ import annotations

import math
import queue
import shlex
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except Exception:
    print("\u7f3a\u5c11\u4f9d\u8d56: pyserial")
    print("\u8bf7\u5148\u5b89\u88c5: pip install -r tools/requirements.txt")
    raise SystemExit(2)

try:
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QLineEdit,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QVBoxLayout,
        QWidget,
    )
    import pyqtgraph as pg
except Exception as exc:
    print("\u7f3a\u5c11 GUI \u4f9d\u8d56")
    print("\u8bf7\u5148\u5b89\u88c5: pip install -r tools/requirements_gui.txt")
    print(f"detail: {exc}")
    raise SystemExit(2)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("STM32 PID \u81ea\u52a8\u6574\u5b9a\u5de5\u5177\uff08\u9ad8\u6027\u80fd\u7248\uff09")
        self.resize(1280, 860)

        self.connected = False
        self.serial_mon: serial.Serial | None = None
        self.rx_buf = bytearray()
        self.auto_proc: subprocess.Popen[str] | None = None
        self.user_stop_requested = False
        self.log_queue: "queue.Queue[str]" = queue.Queue()

        self.mae_history: deque[float] = deque(maxlen=500)
        self.kp_history: deque[float] = deque(maxlen=500)
        self.ki_history: deque[float] = deque(maxlen=500)
        self.kd_history: deque[float] = deque(maxlen=500)
        self.best_score: float | None = None
        self.best_params: tuple[float, float, float] | None = None
        self.metric_count = 0
        self.last_poll_ts = 0.0
        self.last_no_data_log_ts = 0.0
        self.max_log_lines = 800

        self._build_ui()
        self.refresh_ports()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(80)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        v = QVBoxLayout(root)

        serial_box = QGroupBox("\u4e32\u53e3\u8bbe\u7f6e")
        serial_grid = QGridLayout(serial_box)

        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setCurrentText("COM6")
        self.baud_edit = QLineEdit("9600")

        self.refresh_btn = QPushButton("\u5237\u65b0\u4e32\u53e3")
        self.refresh_btn.clicked.connect(self.refresh_ports)

        self.conn_btn = QPushButton("\u8fde\u63a5")
        self.conn_btn.clicked.connect(self.toggle_connection)
        self.state_label = QLabel("\u672a\u8fde\u63a5")

        self.poll_check = QCheckBox("\u81ea\u52a8\u8f6e\u8be2")
        self.poll_check.setChecked(True)
        self.poll_interval_edit = QLineEdit("0.25")

        serial_grid.addWidget(QLabel("\u4e32\u53e3"), 0, 0)
        serial_grid.addWidget(self.port_combo, 0, 1)
        serial_grid.addWidget(QLabel("\u6ce2\u7279\u7387"), 0, 2)
        serial_grid.addWidget(self.baud_edit, 0, 3)
        serial_grid.addWidget(self.refresh_btn, 0, 4)
        serial_grid.addWidget(self.conn_btn, 0, 5)
        serial_grid.addWidget(self.state_label, 0, 6)

        serial_grid.addWidget(self.poll_check, 1, 0, 1, 2)
        serial_grid.addWidget(QLabel("\u8f6e\u8be2\u95f4\u9694(s)"), 1, 2)
        serial_grid.addWidget(self.poll_interval_edit, 1, 3)

        v.addWidget(serial_box)

        send_box = QGroupBox("\u624b\u52a8\u53d1\u9001")
        send_grid = QGridLayout(send_box)
        self.channels_edit = QLineEdit("0,1,2")
        self.values_edit = QLineEdit("1.000,0.001,1.000")
        self.send_btn = QPushButton("\u53d1\u9001\u4e00\u6b21")
        self.send_btn.clicked.connect(self.send_once)
        self.snapshot_btn = QPushButton("\u8bfb\u53d6\u5f53\u524d\u5e27")
        self.snapshot_btn.clicked.connect(lambda: self._snapshot_once(True, 1.2))

        send_grid.addWidget(QLabel("\u901a\u9053"), 0, 0)
        send_grid.addWidget(self.channels_edit, 0, 1)
        send_grid.addWidget(QLabel("\u53c2\u6570\u503c"), 0, 2)
        send_grid.addWidget(self.values_edit, 0, 3)
        send_grid.addWidget(self.send_btn, 0, 4)
        send_grid.addWidget(self.snapshot_btn, 0, 5)
        v.addWidget(send_box)

        auto_box = QGroupBox("\u81ea\u52a8\u6574\u5b9a")
        auto_grid = QGridLayout(auto_box)
        self.init_edit = QLineEdit("1.0,0.001,1.0")
        self.step_edit = QLineEdit("0.2,0.0005,0.2")
        self.min_edit = QLineEdit("0.1,0.0000,0.1")
        self.max_edit = QLineEdit("5.0,1.0,5.0")
        self.rounds_edit = QLineEdit("30")
        self.decay_edit = QLineEdit("0.85")
        self.settle_edit = QLineEdit("0.4")
        self.measure_edit = QLineEdit("1.0")
        self.value_idx_edit = QLineEdit("0")
        self.target_idx_edit = QLineEdit("1")
        self.patience_edit = QLineEdit("6")
        self.min_improve_edit = QLineEdit("0.001")

        self.start_btn = QPushButton("\u5f00\u59cb\u81ea\u52a8\u6574\u5b9a")
        self.start_btn.clicked.connect(self.start_auto)
        self.stop_btn = QPushButton("\u505c\u6b62\u81ea\u52a8\u6574\u5b9a")
        self.stop_btn.clicked.connect(self.stop_auto)
        self.freeze_btn = QPushButton("\u51bb\u7ed3\u6700\u4f18\u5e76\u5bfc\u51fa")
        self.freeze_btn.clicked.connect(self.freeze_best)

        self.low_freq_check = QCheckBox("\u4f4e\u9891\u7ed8\u56fe")
        self.low_freq_check.setChecked(True)
        self.draw_every_edit = QLineEdit("3")

        auto_grid.addWidget(QLabel("\u521d\u59cb\u503c(init)"), 0, 0)
        auto_grid.addWidget(self.init_edit, 0, 1)
        auto_grid.addWidget(QLabel("\u6b65\u957f(step)"), 0, 2)
        auto_grid.addWidget(self.step_edit, 0, 3)

        auto_grid.addWidget(QLabel("\u4e0b\u9650(min)"), 1, 0)
        auto_grid.addWidget(self.min_edit, 1, 1)
        auto_grid.addWidget(QLabel("\u4e0a\u9650(max)"), 1, 2)
        auto_grid.addWidget(self.max_edit, 1, 3)

        auto_grid.addWidget(QLabel("\u8fed\u4ee3\u8f6e\u6570(rounds)"), 2, 0)
        auto_grid.addWidget(self.rounds_edit, 2, 1)
        auto_grid.addWidget(QLabel("\u6b65\u957f\u8870\u51cf(decay)"), 2, 2)
        auto_grid.addWidget(self.decay_edit, 2, 3)

        auto_grid.addWidget(QLabel("\u7a33\u5b9a\u65f6\u95f4(settle)"), 3, 0)
        auto_grid.addWidget(self.settle_edit, 3, 1)
        auto_grid.addWidget(QLabel("\u91c7\u6837\u65f6\u95f4(measure)"), 3, 2)
        auto_grid.addWidget(self.measure_edit, 3, 3)

        auto_grid.addWidget(QLabel("\u53cd\u9988\u5217\u7d22\u5f15(value-index)"), 4, 0)
        auto_grid.addWidget(self.value_idx_edit, 4, 1)
        auto_grid.addWidget(QLabel("\u76ee\u6807\u5217\u7d22\u5f15(target-index)"), 4, 2)
        auto_grid.addWidget(self.target_idx_edit, 4, 3)

        auto_grid.addWidget(QLabel("\u65e9\u505c\u8010\u5fc3\u8f6e\u6570(patience)"), 5, 0)
        auto_grid.addWidget(self.patience_edit, 5, 1)
        auto_grid.addWidget(QLabel("\u6700\u5c0f\u6539\u8fdb\u9608\u503c(min-improve)"), 5, 2)
        auto_grid.addWidget(self.min_improve_edit, 5, 3)

        auto_grid.addWidget(self.start_btn, 6, 0)
        auto_grid.addWidget(self.stop_btn, 6, 1)
        auto_grid.addWidget(self.freeze_btn, 6, 2)

        auto_grid.addWidget(self.low_freq_check, 7, 0)
        auto_grid.addWidget(QLabel("\u6bcf N \u6b21\u5237\u65b0\u56fe\u5f62"), 7, 1)
        auto_grid.addWidget(self.draw_every_edit, 7, 2)

        v.addWidget(auto_box)

        plot_box = QGroupBox("\u5b9e\u65f6\u66f2\u7ebf")
        plot_layout = QHBoxLayout(plot_box)

        pg.setConfigOptions(antialias=False)
        self.err_plot = pg.PlotWidget(title="\u8bef\u5dee MAE")
        self.err_curve = self.err_plot.plot(pen=pg.mkPen("#1f77b4", width=2))

        self.pid_plot = pg.PlotWidget(title="PID \u53c2\u6570")
        self.kp_curve = self.pid_plot.plot(pen=pg.mkPen("#d62728", width=2), name="Kp")
        self.ki_curve = self.pid_plot.plot(pen=pg.mkPen("#2ca02c", width=2), name="Ki")
        self.kd_curve = self.pid_plot.plot(pen=pg.mkPen("#9467bd", width=2), name="Kd")

        plot_layout.addWidget(self.err_plot)
        plot_layout.addWidget(self.pid_plot)
        v.addWidget(plot_box, stretch=2)

        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        v.addWidget(self.log_text, stretch=2)

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log_text.appendPlainText(f"[{ts}] {msg}")
        doc = self.log_text.document()
        if doc.blockCount() > self.max_log_lines:
            cur = self.log_text.textCursor()
            cur.movePosition(cur.Start)
            for _ in range(doc.blockCount() - self.max_log_lines):
                cur.select(cur.LineUnderCursor)
                cur.removeSelectedText()
                cur.deleteChar()

    def refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        current = self.port_combo.currentText()
        self.port_combo.clear()
        self.port_combo.addItems(ports)
        if current and current in ports:
            self.port_combo.setCurrentText(current)
        elif ports:
            self.port_combo.setCurrentText(ports[0])
        self._log(f"\u4e32\u53e3\u5217\u8868: {ports if ports else '\u65e0'}")

    def _set_connected_ui(self) -> None:
        if self.connected:
            self.state_label.setText("\u5df2\u8fde\u63a5")
            self.state_label.setStyleSheet("color:#2e8b57;")
            self.conn_btn.setText("\u65ad\u5f00")
        else:
            self.state_label.setText("\u672a\u8fde\u63a5")
            self.state_label.setStyleSheet("color:#b22222;")
            self.conn_btn.setText("\u8fde\u63a5")

    def _tool_cmd_prefix(self) -> list[str]:
        return [sys.executable, "-u", str(Path(__file__).with_name("pid_autotune_serial.py"))]

    def _base_args(self) -> list[str]:
        if not self.connected:
            raise ValueError("\u8bf7\u5148\u8fde\u63a5\u4e32\u53e3")
        port = self.port_combo.currentText().strip()
        baud = self.baud_edit.text().strip()
        if not port:
            raise ValueError("\u4e32\u53e3\u53f7\u4e0d\u80fd\u4e3a\u7a7a")
        if not baud.isdigit():
            raise ValueError("\u6ce2\u7279\u7387\u5fc5\u987b\u662f\u6574\u6570")
        return ["--port", port, "--baud", baud]

    def _close_monitor_serial(self) -> None:
        if self.serial_mon is not None:
            try:
                self.serial_mon.close()
            except Exception:
                pass
        self.serial_mon = None
        self.rx_buf = bytearray()

    def toggle_connection(self) -> None:
        if self.auto_proc is not None:
            QMessageBox.warning(self, "\u63d0\u793a", "\u81ea\u52a8\u6574\u5b9a\u8fd0\u884c\u4e2d\uff0c\u8bf7\u5148\u505c\u6b62\u518d\u8fde\u63a5\u6216\u65ad\u5f00\u3002")
            return

        if self.connected:
            self._close_monitor_serial()
            self.connected = False
            self._set_connected_ui()
            self._log("\u4e32\u53e3\u5df2\u65ad\u5f00")
            return

        port = self.port_combo.currentText().strip()
        baud_s = self.baud_edit.text().strip()
        if not port:
            QMessageBox.critical(self, "\u8fde\u63a5\u5931\u8d25", "\u4e32\u53e3\u53f7\u4e0d\u80fd\u4e3a\u7a7a")
            return
        if not baud_s.isdigit():
            QMessageBox.critical(self, "\u8fde\u63a5\u5931\u8d25", "\u6ce2\u7279\u7387\u5fc5\u987b\u662f\u6574\u6570")
            return

        try:
            self.serial_mon = serial.Serial(port=port, baudrate=int(baud_s), timeout=0.02)
            self.serial_mon.reset_input_buffer()
            self.rx_buf = bytearray()
            self.connected = True
            self._set_connected_ui()
            self._log("\u4e32\u53e3\u8fde\u63a5\u6210\u529f")
        except Exception as exc:
            self._close_monitor_serial()
            self.connected = False
            self._set_connected_ui()
            QMessageBox.critical(self, "\u8fde\u63a5\u5931\u8d25", str(exc))

    def _run_cmd(self, args: list[str]) -> subprocess.CompletedProcess[str]:
        self._log("\u6267\u884c\u547d\u4ee4: " + shlex.join(args))
        return subprocess.run(args, capture_output=True, text=True, check=False)

    def send_once(self) -> None:
        try:
            args = self._tool_cmd_prefix() + self._base_args() + [
                "send",
                "--channels",
                self.channels_edit.text().strip(),
                "--values",
                self.values_edit.text().strip(),
            ]
            cp = self._run_cmd(args)
            if cp.stdout.strip():
                for line in cp.stdout.strip().splitlines():
                    self._log(line)
            if cp.stderr.strip():
                for line in cp.stderr.strip().splitlines():
                    self._log("\u9519\u8bef: " + line)
            if cp.returncode != 0:
                QMessageBox.critical(self, "\u53d1\u9001\u5931\u8d25", f"\u9000\u51fa\u7801: {cp.returncode}")
        except Exception as exc:
            QMessageBox.critical(self, "\u53d1\u9001\u5931\u8d25", str(exc))

    def _snapshot_once(self, log_if_empty: bool, duration: float) -> None:
        try:
            _ = self._base_args()
            if self.serial_mon is None:
                raise RuntimeError("\u4e32\u53e3\u53e5\u67c4\u4e0d\u53ef\u7528\uff0c\u8bf7\u91cd\u65b0\u8fde\u63a5")

            end_t = time.time() + duration
            lines: list[str] = []
            while time.time() < end_t:
                d = self.serial_mon.read(self.serial_mon.in_waiting or 1)
                if not d:
                    continue
                self.rx_buf.extend(d)
                while b"\n" in self.rx_buf:
                    one, _, rest = self.rx_buf.partition(b"\n")
                    self.rx_buf = bytearray(rest)
                    line = one.decode("utf-8", errors="ignore").strip()
                    if line:
                        lines.append(line)

            if not lines:
                if log_if_empty:
                    self._log("\u5f53\u524d\u5e27\u8bfb\u53d6\u5931\u8d25: \u6682\u65e0\u4e32\u53e3\u6570\u636e")
                else:
                    now = time.time()
                    if now - self.last_no_data_log_ts > 3.0:
                        self.last_no_data_log_ts = now
                        self._log("\u8f6e\u8be2\u63d0\u793a: \u6682\u65e0\u4e32\u53e3\u6570\u636e")
                return

            last = lines[-1]
            if log_if_empty:
                self._log("\u539f\u59cb\u6700\u65b0\u5e27: " + last)

            parts = [p.strip() for p in last.split(",") if p.strip()]
            nums: list[float] = []
            for p in parts:
                try:
                    nums.append(float(p))
                except ValueError:
                    nums = []
                    break
            if len(nums) >= 9 and log_if_empty:
                f, l0, r0, _u1, _u2, _u3, kp_u, ki_u, kd_u = nums[:9]
                self._log(
                    f"\u89e3\u6790\u6570\u636e: F={f:.0f}, L0={l0:.0f}, R0={r0:.0f}, "
                    f"Kp={kp_u/1000.0:.3f}, Ki={ki_u/1000.0:.3f}, Kd={kd_u/1000.0:.3f}"
                )
        except Exception as exc:
            if log_if_empty:
                QMessageBox.critical(self, "\u8bfb\u53d6\u5931\u8d25", str(exc))

    def _parse_metric(self, line: str) -> dict[str, float] | None:
        if not line.startswith("METRIC,"):
            return None
        data: dict[str, float] = {}
        for token in line.split(",")[1:]:
            if "=" not in token:
                continue
            k, v = token.split("=", 1)
            try:
                data[k.strip()] = float(v.strip())
            except ValueError:
                return None
        req = ("mae", "p0", "p1", "p2", "score")
        if not all(k in data for k in req):
            return None
        if any(not math.isfinite(data[k]) for k in req):
            return None
        return data

    def _update_plots(self) -> None:
        self.err_curve.setData(list(self.mae_history))
        self.kp_curve.setData(list(self.kp_history))
        self.ki_curve.setData(list(self.ki_history))
        self.kd_curve.setData(list(self.kd_history))

    def _ingest_metric_line(self, line: str) -> None:
        m = self._parse_metric(line)
        if m is None:
            return
        self.mae_history.append(m["mae"])
        self.kp_history.append(m["p0"])
        self.ki_history.append(m["p1"])
        self.kd_history.append(m["p2"])

        score = m["score"]
        if self.best_score is None or score < self.best_score:
            self.best_score = score
            self.best_params = (m["p0"], m["p1"], m["p2"])

        self.metric_count += 1
        try:
            draw_every = max(1, int(self.draw_every_edit.text().strip()))
        except Exception:
            draw_every = 1
        if (not self.low_freq_check.isChecked()) or (self.metric_count % draw_every == 0):
            self._update_plots()

    def start_auto(self) -> None:
        if self.auto_proc is not None:
            QMessageBox.warning(self, "\u63d0\u793a", "\u81ea\u52a8\u6574\u5b9a\u5df2\u5728\u8fd0\u884c\u4e2d")
            return
        try:
            self.user_stop_requested = False
            self.mae_history.clear()
            self.kp_history.clear()
            self.ki_history.clear()
            self.kd_history.clear()
            self.best_score = None
            self.best_params = None
            self.metric_count = 0
            self._update_plots()

            self._close_monitor_serial()

            args = self._tool_cmd_prefix() + self._base_args() + [
                "auto",
                "--channels",
                self.channels_edit.text().strip(),
                "--init",
                self.init_edit.text().strip(),
                "--step",
                self.step_edit.text().strip(),
                "--min",
                self.min_edit.text().strip(),
                "--max",
                self.max_edit.text().strip(),
                "--rounds",
                self.rounds_edit.text().strip(),
                "--decay",
                self.decay_edit.text().strip(),
                "--settle",
                self.settle_edit.text().strip(),
                "--measure",
                self.measure_edit.text().strip(),
                "--value-index",
                self.value_idx_edit.text().strip(),
                "--target-index",
                self.target_idx_edit.text().strip(),
                "--patience",
                self.patience_edit.text().strip(),
                "--min-improve",
                self.min_improve_edit.text().strip(),
                "--print-metric",
            ]

            self._log("\u6267\u884c\u547d\u4ee4: " + shlex.join(args))
            self.auto_proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            def _reader(proc: subprocess.Popen[str], q: "queue.Queue[str]") -> None:
                assert proc.stdout is not None
                for line in proc.stdout:
                    q.put(line.rstrip("\r\n"))

            threading.Thread(target=_reader, args=(self.auto_proc, self.log_queue), daemon=True).start()
            self.conn_btn.setEnabled(False)
            self._log("\u81ea\u52a8\u6574\u5b9a\u5df2\u542f\u52a8")
        except Exception as exc:
            self.auto_proc = None
            self.conn_btn.setEnabled(True)
            QMessageBox.critical(self, "\u542f\u52a8\u5931\u8d25", str(exc))

    def stop_auto(self) -> None:
        if self.auto_proc is None:
            self._log("\u5f53\u524d\u6ca1\u6709\u8fd0\u884c\u4e2d\u7684\u81ea\u52a8\u6574\u5b9a\u8fdb\u7a0b")
            return
        self.user_stop_requested = True
        self.auto_proc.terminate()
        self._log("\u5df2\u53d1\u9001\u505c\u6b62\u4fe1\u53f7")

    def freeze_best(self) -> None:
        if self.best_params is None:
            QMessageBox.warning(self, "\u63d0\u793a", "\u6682\u65e0\u6700\u4f18\u53c2\u6570")
            return
        try:
            p0, p1, p2 = self.best_params
            score = self.best_score if self.best_score is not None else float("nan")
            channels = self.channels_edit.text().strip()
            values = f"{p0:.6f},{p1:.6f},{p2:.6f}"
            args = self._tool_cmd_prefix() + self._base_args() + [
                "send",
                "--channels",
                channels,
                "--values",
                values,
            ]
            cp = self._run_cmd(args)
            if cp.returncode != 0:
                raise RuntimeError(cp.stdout + "\n" + cp.stderr)

            out = Path(__file__).with_name("pid_best_params.txt")
            out.write_text(
                "\n".join(
                    [
                        "# PID best params",
                        f"channels={channels}",
                        f"values={values}",
                        f"score={score:.6f}",
                        f"time={time.strftime('%Y-%m-%d %H:%M:%S')}",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            self._log(f"\u5df2\u51bb\u7ed3\u6700\u4f18\u53c2\u6570\u5e76\u5bfc\u51fa: {out}")
        except Exception as exc:
            QMessageBox.critical(self, "\u5bfc\u51fa\u5931\u8d25", str(exc))

    def _on_timer(self) -> None:
        while not self.log_queue.empty():
            line = self.log_queue.get_nowait()
            self._log(line)
            self._ingest_metric_line(line)

        if self.auto_proc is not None:
            rc = self.auto_proc.poll()
            if rc is not None:
                if self.user_stop_requested:
                    self._log("\u81ea\u52a8\u6574\u5b9a\u5df2\u7531\u7528\u6237\u505c\u6b62")
                else:
                    self._log(f"\u81ea\u52a8\u6574\u5b9a\u7ed3\u675f\uff0c\u9000\u51fa\u7801={rc}")
                self.auto_proc = None
                self.user_stop_requested = False
                self.conn_btn.setEnabled(True)
                if self.connected:
                    try:
                        port = self.port_combo.currentText().strip()
                        baud_s = self.baud_edit.text().strip()
                        if port and baud_s.isdigit():
                            self.serial_mon = serial.Serial(port=port, baudrate=int(baud_s), timeout=0.02)
                            self.serial_mon.reset_input_buffer()
                            self.rx_buf = bytearray()
                            self._log("\u5df2\u6062\u590d\u4e32\u53e3\u76d1\u542c")
                    except Exception as exc:
                        self._log("\u6062\u590d\u4e32\u53e3\u76d1\u542c\u5931\u8d25: " + str(exc))

        if self.connected and self.poll_check.isChecked() and self.auto_proc is None:
            now = time.time()
            try:
                poll_interval = max(0.1, float(self.poll_interval_edit.text().strip()))
            except Exception:
                poll_interval = 0.25
            if now - self.last_poll_ts >= poll_interval:
                self.last_poll_ts = now
                self._snapshot_once(False, 0.15)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        try:
            if self.auto_proc is not None:
                self.auto_proc.terminate()
        except Exception:
            pass
        self._close_monitor_serial()
        super().closeEvent(event)


def main() -> int:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
