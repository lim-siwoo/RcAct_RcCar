# utils/system_monitor.py
import os
import psutil

class SystemMonitor:
    @staticmethod
    def get_cpu_temperature():
        temp = os.popen("cat /sys/class/thermal/thermal_zone0/temp").readline()
        return round(float(temp) / 1000.0, 2)

    @staticmethod
    def get_cpu_usage():
        return round(psutil.cpu_percent(interval=None, percpu=False), 2)