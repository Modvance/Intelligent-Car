#!/usr/bin/env python3
"""Low-overhead profiling helpers for real easy-mode runs."""
import json
import math
import os
import shutil
import subprocess
import time
from collections import defaultdict
from pathlib import Path


PROFILE_DIR_ENV = "OLD_CAR_PROFILE_DIR"


def configure_profile(enabled, base_dir=None):
    """Create one run directory and expose it to child processes by env var."""
    if not enabled:
        os.environ.pop(PROFILE_DIR_ENV, None)
        return None
    root = Path(base_dir) if base_dir else Path.cwd() / "logs" / "performance"
    run_dir = root / time.strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    os.environ[PROFILE_DIR_ENV] = str(run_dir)
    return run_dir


def profile_directory():
    value = os.environ.get(PROFILE_DIR_ENV, "")
    return Path(value) if value else None


def profile_enabled():
    return profile_directory() is not None


def summarize_samples(values):
    samples = sorted(float(value) for value in values)
    if not samples:
        return {"count": 0}

    def percentile(ratio):
        index = max(0, min(len(samples) - 1, math.ceil(len(samples) * ratio) - 1))
        return samples[index]

    return {
        "count": len(samples),
        "min": samples[0],
        "avg": sum(samples) / len(samples),
        "p50": percentile(0.50),
        "p95": percentile(0.95),
        "max": samples[-1],
    }


class PerformanceRecorder:
    """Aggregate per-frame samples and append one JSONL line per reporting window."""

    def __init__(self, component, directory=None, flush_interval_s=1.0):
        self.component = str(component)
        self.directory = Path(directory) if directory else profile_directory()
        self.enabled = self.directory is not None
        self.path = None
        self.flush_interval_s = float(flush_interval_s)
        self._samples = defaultdict(list)
        self._window_started = time.perf_counter()
        if self.enabled:
            self.directory.mkdir(parents=True, exist_ok=True)
            self.path = self.directory / f"{self.component}_{os.getpid()}.jsonl"

    def observe(self, **metrics):
        if not self.enabled:
            return None
        for name, value in metrics.items():
            if value is None:
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(number):
                self._samples[str(name)].append(number)
        if time.perf_counter() - self._window_started >= self.flush_interval_s:
            return self.flush()
        return None

    def event(self, event_name, **data):
        if not self.enabled:
            return None
        record = {
            "type": "event",
            "time": time.time(),
            "component": self.component,
            "event": str(event_name),
            "data": data,
        }
        self._append(record)
        return record

    def flush(self):
        if not self.enabled or not self._samples:
            self._window_started = time.perf_counter()
            return None
        now = time.perf_counter()
        record = {
            "type": "summary",
            "time": time.time(),
            "component": self.component,
            "window_s": now - self._window_started,
            "metrics": {name: summarize_samples(values) for name, values in self._samples.items()},
        }
        self._append(record)
        self._samples.clear()
        self._window_started = now
        return record

    def _append(self, record):
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")


class SystemSampler:
    def __init__(self):
        self._last_cpu = None
        self._last_process = {}
        self._clock_ticks = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

    @staticmethod
    def _read_cpu_totals():
        try:
            parts = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()[1:]
            values = [int(value) for value in parts]
        except (OSError, IndexError, ValueError):
            return None
        total = sum(values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return total, idle

    @staticmethod
    def _read_meminfo():
        values = {}
        try:
            for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
                name, value = line.split(":", 1)
                values[name] = int(value.strip().split()[0])
        except (OSError, ValueError):
            return {}
        total = values.get("MemTotal", 0) / 1024.0
        available = values.get("MemAvailable", 0) / 1024.0
        return {"memory_total_mb": total, "memory_used_mb": max(0.0, total - available)}

    @staticmethod
    def _read_process(pid):
        try:
            stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
            tail = stat[stat.rfind(")") + 2:].split()
            ticks = int(tail[11]) + int(tail[12])
            rss_kb = 0
            for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8").splitlines():
                if line.startswith("VmRSS:"):
                    rss_kb = int(line.split()[1])
                    break
            return ticks, rss_kb / 1024.0
        except (OSError, IndexError, ValueError):
            return None

    def sample(self, process_pids):
        metrics = self._read_meminfo()
        metrics["load1"] = os.getloadavg()[0]
        current_cpu = self._read_cpu_totals()
        if current_cpu is not None and self._last_cpu is not None:
            total_delta = current_cpu[0] - self._last_cpu[0]
            idle_delta = current_cpu[1] - self._last_cpu[1]
            if total_delta > 0:
                metrics["host_cpu_pct"] = 100.0 * (1.0 - idle_delta / total_delta)
        self._last_cpu = current_cpu

        now = time.perf_counter()
        for name, pid in process_pids.items():
            value = self._read_process(pid)
            if value is None:
                continue
            ticks, rss_mb = value
            previous = self._last_process.get(name)
            if previous is not None:
                previous_ticks, previous_time = previous
                elapsed = now - previous_time
                if elapsed > 0:
                    metrics[f"{name}_cpu_pct"] = 100.0 * (ticks - previous_ticks) / self._clock_ticks / elapsed
            metrics[f"{name}_rss_mb"] = rss_mb
            self._last_process[name] = (ticks, now)
        metrics.update(read_npu_metrics())
        return metrics


def parse_npu_smi(output):
    """Parse a standard npu-smi table without guessing column positions."""
    lines = output.splitlines()
    for index, line in enumerate(lines):
        if "AICore(%)" not in line or "|" not in line:
            continue
        headers = [cell.strip() for cell in line.strip().strip("|").split("|")]
        columns = {name: position for position, name in enumerate(headers)}
        for row in lines[index + 1:]:
            if "|" not in row:
                continue
            cells = [cell.strip() for cell in row.strip().strip("|").split("|")]
            if len(cells) != len(headers) or not cells or not cells[0].isdigit():
                continue
            metrics = {}
            for output_name, column_name in {
                "npu_power_w": "Power(W)",
                "npu_temperature_c": "Temp(C)",
                "npu_aicore_pct": "AICore(%)",
            }.items():
                position = columns.get(column_name)
                if position is None:
                    continue
                try:
                    metrics[output_name] = float(cells[position])
                except ValueError:
                    pass
            return metrics
    return {}


def read_npu_metrics():
    command = shutil.which("npu-smi")
    if not command:
        return {"npu_available": 0.0}
    try:
        result = subprocess.run(
            [command, "info"],
            check=False,
            capture_output=True,
            text=True,
            timeout=1.5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return {"npu_available": 0.0}
    if result.returncode != 0:
        return {"npu_available": 0.0}

    output = result.stdout + result.stderr
    metrics = {"npu_available": 1.0}
    metrics.update(parse_npu_smi(output))
    return metrics


def run_system_profiler(stop_sign, process_pids, interval_s=1.0):
    recorder = PerformanceRecorder("system")
    sampler = SystemSampler()
    while not stop_sign.value:
        recorder.observe(**sampler.sample(process_pids))
        time.sleep(float(interval_s))
    recorder.flush()
