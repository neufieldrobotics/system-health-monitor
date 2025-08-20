#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import socket
import time
import subprocess
from typing import Optional, Dict

import psutil
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# ==============================
# Settings & thresholds
# ==============================
DEFAULT_PERIOD_SEC = 2.0

CPU_WARN_PCT = 90.0
MEM_WARN_PCT = 90.0
DISK_WARN_PCT = 95.0

GPU_WARN_UTIL = 90.0
GPU_WARN_TEMP = 85.0
CPU_WARN_TEMP = 85.0

# NTP check is optional and DISABLED by default (safe offline)
ENABLE_NTP_CHECK = False
NTP_SERVER = "pool.ntp.org"
NTP_TIMEOUT = 1.5  # seconds

# ==============================
# Status producer
# ==============================
class ComputerMonitor:
    """
    Produces a DiagnosticStatus with key system metrics.
    Designed to be called periodically by a ROS 2 node.
    """
    def __init__(self, node: Node):
        self.node = node
        self.start_time = time.time()

    # ---- Optional NTP (disabled unless ENABLE_NTP_CHECK=True) ----
    def _get_ntp_offset(self) -> Optional[float]:
        if not ENABLE_NTP_CHECK:
            return None
        try:
            import ntplib  # lazy import; only if enabled
            c = ntplib.NTPClient()
            r = c.request(NTP_SERVER, version=3, timeout=NTP_TIMEOUT)
            return float(r.offset)  # server_time - local_time (seconds)
        except Exception:
            return None

    # ---- GPU stats: nvidia-smi (dGPU) or tegrastats (Jetson) ----
    def _read_gpu_stats(self) -> Dict[str, Optional[float]]:
        stats = {"util": None, "temp": None, "mem_used_pct": None}

        # Try nvidia-smi first
        try:
            out = subprocess.check_output(
                [
                    "nvidia-smi",
                    "--query-gpu=utilization.gpu,temperature.gpu,memory.used,memory.total",
                    "--format=csv,noheader,nounits",
                ],
                stderr=subprocess.DEVNULL,
                timeout=1.0,
            ).decode().strip()
            util, temp, mem_used, mem_total = map(float, out.split(", "))
            stats["util"] = util
            stats["temp"] = temp
            stats["mem_used_pct"] = 100.0 * mem_used / max(mem_total, 1.0)
            return stats
        except Exception:
            pass

        # Fallback: tegrastats (Jetson)
        try:
            out = subprocess.check_output(
                ["tegrastats", "--interval", "1000", "--count", "1"],
                stderr=subprocess.DEVNULL,
                timeout=2.0,
            ).decode()

            util = None
            temp = None

            # Util: look for "GR3D_FREQ 23%@..."
            toks = out.replace(",", " ").split()
            if "GR3D_FREQ" in toks:
                i = toks.index("GR3D_FREQ")
                if i + 1 < len(toks) and "%" in toks[i + 1]:
                    util = float(toks[i + 1].split("%")[0])

            # Temp: token like "GPU@50.0C"
            for t in toks:
                if t.startswith("GPU@") and t.endswith("C"):
                    try:
                        temp = float(t.split("GPU@")[1].rstrip("C"))
                    except Exception:
                        pass

            stats["util"] = util
            stats["temp"] = temp
            # mem_used_pct often not available on Jetson; leave None
            return stats
        except Exception:
            return stats

    def _first_cpu_temp(self) -> Optional[float]:
        try:
            temps = psutil.sensors_temperatures()
            if not temps:
                return None
            # Try common keys; otherwise first sensor
            for key in ("coretemp", "cpu-thermal", "x86_pkg_temp", "k10temp", "soc_thermal"):
                if key in temps and temps[key]:
                    return float(temps[key][0].current)
            k = next(iter(temps))
            if temps[k]:
                return float(temps[k][0].current)
        except Exception:
            return None
        return None

    def get_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "computer_monitor"
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        # --- CPU ---
        cpu_percent = psutil.cpu_percent(interval=0.1)
        status.values.append(KeyValue(key="cpu_usage_pct", value=f"{cpu_percent:.1f}"))
        if cpu_percent > CPU_WARN_PCT:
            status.level = DiagnosticStatus.WARN
            status.message = "High CPU usage"

        # --- Memory ---
        mem = psutil.virtual_memory()
        status.values.append(KeyValue(key="mem_usage_pct", value=f"{mem.percent:.1f}"))
        if mem.percent > MEM_WARN_PCT:
            status.level = max(status.level, DiagnosticStatus.WARN)
            status.message = ("High memory usage" if status.message == "OK"
                              else status.message + " + High memory usage")

        # --- Disk (root) ---
        disk = psutil.disk_usage("/")
        status.values.append(KeyValue(key="disk_root_usage_pct", value=f"{disk.percent:.1f}"))
        if disk.percent > DISK_WARN_PCT:
            status.level = max(status.level, DiagnosticStatus.WARN)
            status.message = ("Low disk space" if status.message == "OK"
                              else status.message + " + Low disk space")

        # --- Network counters ---
        net = psutil.net_io_counters()
        status.values.append(KeyValue(key="net_tx_mb", value=f"{net.bytes_sent / 1e6:.1f}"))
        status.values.append(KeyValue(key="net_rx_mb", value=f"{net.bytes_recv / 1e6:.1f}"))

        # --- Process uptime ---
        status.values.append(KeyValue(key="process_uptime_s", value=f"{time.time() - self.start_time:.0f}"))

        # --- Optional NTP offset (safe when offline) ---
        ntp_off = self._get_ntp_offset()
        if ntp_off is None:
            status.values.append(KeyValue(key="ntp_offset_s", value="unavailable"))
        else:
            status.values.append(KeyValue(key="ntp_offset_s", value=f"{ntp_off:.3f}"))
            # No WARN by default; you can add a threshold here if desired

        # --- GPU ---
        gpu = self._read_gpu_stats()
        status.values.append(KeyValue(key="gpu_util_pct", value="unavailable" if gpu["util"] is None else f"{gpu['util']:.1f}"))
        status.values.append(KeyValue(key="gpu_temp_c", value="unavailable" if gpu["temp"] is None else f"{gpu['temp']:.1f}"))
        if gpu["mem_used_pct"] is not None:
            status.values.append(KeyValue(key="gpu_mem_used_pct", value=f"{gpu['mem_used_pct']:.1f}"))

        if (gpu["util"] is not None and gpu["util"] > GPU_WARN_UTIL) or \
           (gpu["temp"] is not None and gpu["temp"] > GPU_WARN_TEMP):
            status.level = max(status.level, DiagnosticStatus.WARN)
            status.message = ("High GPU load/temp" if status.message == "OK"
                              else status.message + " + High GPU load/temp")

        # --- CPU temperature ---
        cpu_temp = self._first_cpu_temp()
        status.values.append(KeyValue(key="cpu_temp_c", value="unavailable" if cpu_temp is None else f"{cpu_temp:.1f}"))
        if cpu_temp is not None and cpu_temp > CPU_WARN_TEMP:
            status.level = max(status.level, DiagnosticStatus.WARN)
            status.message = ("High CPU temp" if status.message == "OK"
                              else status.message + " + High CPU temp")

        return status

# ==============================
# ROS 2 Node wrapper & main
# ==============================
class ComputerMonitorNode(Node):
    def __init__(self):
        super().__init__("computer_monitor")
        self.declare_parameter("period_sec", DEFAULT_PERIOD_SEC)

        self.monitor = ComputerMonitor(self)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)

        self.hostname = socket.gethostname()
        period = float(self.get_parameter("period_sec").get_parameter_value().double_value)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(f"ComputerMonitorNode running on {self.hostname}, period={period:.2f}s, ntp_enabled={ENABLE_NTP_CHECK}")

    def _tick(self):
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        status = self.monitor.get_status()
        status.hardware_id = status.hardware_id or self.hostname

        diag.status.append(status)
        self.diag_pub.publish(diag)

def main():
    rclpy.init()
    node = ComputerMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
