import math
from collections import deque
from typing import Optional

import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport
from rclpy.node import Node

try:
    import librosa
except Exception:  # pragma: no cover - optional dependency at runtime
    librosa = None


class AcousticNode(Node):
    """Acoustic node with placeholder energy detector and mel front-end."""

    def __init__(self) -> None:
        super().__init__("node_acoustic")
        self.declare_parameter("node_id", "acoustic_1")
        self.declare_parameter("gps_lat", 30.0)
        self.declare_parameter("gps_lon", 31.0)
        self.declare_parameter("gps_alt", 0.0)
        self.declare_parameter("wav_path", "")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("threshold", 0.25)

        self.node_id = str(self.get_parameter("node_id").value)
        self.gps_lat = float(self.get_parameter("gps_lat").value)
        self.gps_lon = float(self.get_parameter("gps_lon").value)
        self.gps_alt = float(self.get_parameter("gps_alt").value)
        self.wav_path = str(self.get_parameter("wav_path").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.threshold = float(self.get_parameter("threshold").value)

        self.pub = self.create_publisher(DetectionReport, "/detections", 20)
        self.timer = self.create_timer(0.5, self._tick)  # 2 Hz
        self.audio = self._load_audio()
        self.ptr = 0
        self.window = int(self.sample_rate * 2.0)

    def _load_audio(self) -> np.ndarray:
        if librosa is None or not self.wav_path:
            rng = np.random.default_rng(42)
            return rng.normal(0.0, 0.02, self.sample_rate * 60).astype(np.float32)
        y, _ = librosa.load(self.wav_path, sr=self.sample_rate, mono=True)
        return y.astype(np.float32)

    def _tick(self) -> None:
        if len(self.audio) == 0:
            return
        end = self.ptr + self.window
        if end >= len(self.audio):
            self.ptr = 0
            end = self.window
        chunk = self.audio[self.ptr:end]
        self.ptr = end
        prob = self._classify(chunk)
        if prob < self.threshold:
            return
        msg = DetectionReport()
        msg.node_id = self.node_id
        msg.lat = self.gps_lat
        msg.lon = self.gps_lon
        msg.alt = self.gps_alt
        msg.azimuth_deg = 0.0
        msg.elevation_deg = 0.0
        msg.confidence = float(prob)
        msg.modality = "acoustic"
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _classify(self, chunk: np.ndarray) -> float:
        if librosa is None:
            rms = float(np.sqrt(np.mean(np.square(chunk))))
            return float(np.clip(rms * 20.0, 0.0, 1.0))

        mel = librosa.feature.melspectrogram(
            y=chunk, sr=self.sample_rate, n_mels=64, hop_length=512, n_fft=1024
        )
        mel_db = librosa.power_to_db(mel, ref=np.max)
        # Placeholder classifier - band energy proxy for 80-400 Hz drone band.
        freqs = librosa.mel_frequencies(n_mels=64, fmin=0.0, fmax=self.sample_rate / 2.0)
        band_idx = np.where((freqs >= 80.0) & (freqs <= 400.0))[0]
        if len(band_idx) == 0:
            return 0.0
        band_energy = float(np.mean(mel_db[band_idx, :]))
        normalized = (band_energy + 80.0) / 80.0
        return float(np.clip(normalized, 0.0, 1.0))


def main() -> None:
    rclpy.init()
    node = AcousticNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
