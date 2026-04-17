import json
import time
from typing import List, Optional

import rclpy
from cuas_msgs.msg import DetectionReport

try:
    import serial
except Exception:  # pragma: no cover - optional at runtime
    serial = None


def report_to_json(report: DetectionReport) -> str:
    payload = {
        "node_id": report.node_id,
        "lat": report.lat,
        "lon": report.lon,
        "alt": report.alt,
        "azimuth_deg": report.azimuth_deg,
        "elevation_deg": report.elevation_deg,
        "confidence": report.confidence,
        "modality": report.modality,
        "stamp_sec": int(report.stamp.sec),
        "stamp_nanosec": int(report.stamp.nanosec),
    }
    return json.dumps(payload)


def json_to_report(line: str) -> DetectionReport:
    payload = json.loads(line)
    msg = DetectionReport()
    msg.node_id = str(payload["node_id"])
    msg.lat = float(payload["lat"])
    msg.lon = float(payload["lon"])
    msg.alt = float(payload["alt"])
    msg.azimuth_deg = float(payload["azimuth_deg"])
    msg.elevation_deg = float(payload["elevation_deg"])
    msg.confidence = float(payload["confidence"])
    msg.modality = str(payload["modality"])
    msg.stamp.sec = int(payload["stamp_sec"])
    msg.stamp.nanosec = int(payload["stamp_nanosec"])
    return msg


class LoRaTransport:
    """Transport abstraction for ROS-sim topic and serial LoRa JSON."""

    def __init__(self, serial_port: str, baud: int = 115200, use_sim: bool = True) -> None:
        self.serial_port = serial_port
        self.baud = baud
        self.use_sim = use_sim
        self.ser = None
        if not self.use_sim and serial is not None:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=0.01)

    def send_detection(self, report: DetectionReport) -> None:
        if self.use_sim:
            return
        if self.ser is None:
            return
        line = report_to_json(report) + "\n"
        self.ser.write(line.encode("utf-8"))

    def read_reports(self) -> List[DetectionReport]:
        if self.use_sim or self.ser is None:
            return []
        reports: List[DetectionReport] = []
        while self.ser.in_waiting > 0:
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue
            try:
                reports.append(json_to_report(raw))
            except Exception:
                continue
        return reports

    @staticmethod
    def is_fresh(report: DetectionReport, now_sec: float, max_skew_sec: float = 2.0) -> bool:
        report_sec = float(report.stamp.sec) + float(report.stamp.nanosec) * 1e-9
        return abs(now_sec - report_sec) <= max_skew_sec
