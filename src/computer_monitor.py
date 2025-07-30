import psutil
import ntplib
import time
import subprocess
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


class ComputerMonitor:
    def __init__(self, node: Node):
        self.node = node
        self.ntp_server = 'pool.ntp.org'
        self.ntp_timeout = 2.0
        self.start_time = time.time()

    def get_status(self) -> DiagnosticStatus:
        status = DiagnosticStatus()
        status.name = "ComputerMonitor"
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        # --- CPU Usage ---
        cpu_percent = psutil.cpu_percent(interval=0.1)
        status.values.append(KeyValue("CPU Usage (%)", f"{cpu_percent:.1f}"))
        if cpu_percent > 90:
            status.level = DiagnosticStatus.WARN
            status.message = "High CPU usage"

        # --- Memory Usage ---
        mem = psutil.virtual_memory()
        status.values.append(KeyValue("Memory Usage (%)", f"{mem.percent:.1f}"))
        if mem.percent > 90:
            status.level = DiagnosticStatus.WARN
            status.message += " + High memory usage"

        # --- Disk Space ---
        disk = psutil.disk_usage('/')
        status.values.append(KeyValue("Disk Usage (%)", f"{disk.percent:.1f}"))
        if disk.percent > 95:
            status.level = DiagnosticStatus.WARN
            status.message += " + Low disk space"

        # --- Network Stats ---
        net = psutil.net_io_counters()
        status.values.append(KeyValue("Network Sent (MB)", f"{net.bytes_sent / 1e6:.1f}"))
        status.values.append(KeyValue("Network Received (MB)", f"{net.bytes_recv / 1e6:.1f}"))

        # --- Uptime ---
        uptime = time.time() - self.start_time
        status.values.append(KeyValue("Process Uptime (s)", f"{uptime:.0f}"))

        # --- NTP Drift ---


        # --- GPU (if available) ---
        try:
            output = subprocess.check_output(['nvidia-smi', '--query-gpu=utilization.gpu,temperature.gpu,memory.used,memory.total',
                                              '--format=csv,noheader,nounits'])
            util, temp, mem_used, mem_total = map(float, output.decode().strip().split(', '))
            gpu_mem_pct = 100 * mem_used / mem_total
            status.values.extend([
                KeyValue("GPU Utilization (%)", f"{util:.1f}"),
                KeyValue("GPU Temperature (C)", f"{temp:.1f}"),
                KeyValue("GPU Memory Used (%)", f"{gpu_mem_pct:.1f}")
            ])
            if util > 90 or temp > 85:
                status.level = DiagnosticStatus.WARN
                status.message += " + High GPU load or temp"
        except Exception:
            status.values.append(KeyValue("GPU Info", "nvidia-smi not available"))

        # --- Thermal Monitoring (CPU Temp) ---
        try:
            temps = psutil.sensors_temperatures()
            cpu_temp = temps.get('coretemp', [])[0].current if 'coretemp' in temps else None
            if cpu_temp:
                status.values.append(KeyValue("CPU Temperature (C)", f"{cpu_temp:.1f}"))
                if cpu_temp > 85:
                    status.level = DiagnosticStatus.WARN
                    status.message += " + High CPU temperature"
        except Exception as e:
            status.values.append(KeyValue("Thermal Info", f"Unavailable: {e}"))

        return status
