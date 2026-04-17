from dataclasses import dataclass
from typing import Iterable, List


MODAL_WEIGHTS = {
    "rgb": 1.0,
    "thermal": 0.9,
    "rf": 0.4,
    "acoustic": 0.5,
}

BEARING_MODALITIES = {"rgb", "thermal"}


@dataclass
class VoteResult:
    combined_confidence: float
    track_state: str
    bearing_reports: List
    num_sensors: int


def compute_modal_vote(reports: Iterable) -> VoteResult:
    reports = list(reports)
    if not reports:
        return VoteResult(0.0, "lost", [], 0)

    weighted_sum = 0.0
    weight_total = 0.0
    bearing_reports = []
    unique_nodes = set()
    bearing_nodes = set()
    for report in reports:
        w = MODAL_WEIGHTS.get(report.modality, 0.2)
        weighted_sum += w * max(0.0, min(1.0, float(report.confidence)))
        weight_total += w
        unique_nodes.add(report.node_id)
        if report.modality in BEARING_MODALITIES:
            bearing_reports.append(report)
            bearing_nodes.add(report.node_id)
    combined = weighted_sum / weight_total if weight_total > 0 else 0.0

    if combined >= 0.6 and len(bearing_nodes) >= 2:
        state = "confirmed"
    elif combined >= 0.35:
        state = "tentative"
    else:
        state = "lost"
    return VoteResult(
        combined_confidence=combined,
        track_state=state,
        bearing_reports=bearing_reports,
        num_sensors=len(unique_nodes),
    )
