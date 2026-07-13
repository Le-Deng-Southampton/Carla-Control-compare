from dataclasses import dataclass
import hashlib
import json
import math
import os
from types import MappingProxyType

import numpy as np


FORMAT_VERSION = 1
_NON_DETERMINISTIC_METADATA_KEYS = {
    "execution",
    "output_dir",
    "output_path",
    "planning_duration_s",
    "timestamp",
}


def _normalize_angle(angle_rad):
    return (float(angle_rad) + np.pi) % (2.0 * np.pi) - np.pi


def _json_safe(value):
    if isinstance(value, MappingProxyType):
        value = dict(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _deterministic_metadata(metadata):
    return {
        str(key): _json_safe(value)
        for key, value in dict(metadata or {}).items()
        if str(key) not in _NON_DETERMINISTIC_METADATA_KEYS
    }


def _road_option_name(road_option):
    name = getattr(road_option, "name", None)
    if name:
        return str(name)
    text = str(road_option)
    return text.rsplit(".", 1)[-1]


@dataclass(frozen=True)
class TrajectorySample:
    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    curvature_1pm: float
    curvature_rate_1pm2: float
    left_clearance_m: float
    right_clearance_m: float
    speed_cap_mps: float
    source_index: int
    road_option: str


@dataclass(frozen=True)
class TrajectoryLocation:
    x: float
    y: float
    z: float = 0.0

    def distance(self, other):
        dx = self.x - float(other.x)
        dy = self.y - float(other.y)
        dz = self.z - float(getattr(other, "z", 0.0))
        return float(math.sqrt(dx * dx + dy * dy + dz * dz))


@dataclass(frozen=True)
class TrajectoryRotation:
    yaw: float


@dataclass(frozen=True)
class TrajectoryTransform:
    location: TrajectoryLocation
    rotation: TrajectoryRotation


@dataclass(frozen=True)
class TrajectoryWaypoint:
    transform: TrajectoryTransform
    id: int
    lane_width: float
    source_index: int
    centerline_offset_m: float
    left_clearance_m: float
    right_clearance_m: float
    curvature_1pm: float = 0.0
    curvature_rate_1pm2: float = 0.0
    speed_cap_mps: float = 0.0


@dataclass(frozen=True)
class ReferenceTrajectory:
    s_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    z_m: np.ndarray
    yaw_rad: np.ndarray
    curvature_1pm: np.ndarray
    curvature_rate_1pm2: np.ndarray
    left_clearance_m: np.ndarray
    right_clearance_m: np.ndarray
    speed_cap_mps: np.ndarray
    source_index: np.ndarray
    road_options: tuple
    metadata: dict

    def __post_init__(self):
        float_fields = (
            "s_m",
            "x_m",
            "y_m",
            "z_m",
            "yaw_rad",
            "curvature_1pm",
            "curvature_rate_1pm2",
            "left_clearance_m",
            "right_clearance_m",
            "speed_cap_mps",
        )
        arrays = {}
        for name in float_fields:
            array = np.array(getattr(self, name), dtype=float, copy=True).reshape(-1)
            arrays[name] = array
        arrays["source_index"] = np.array(self.source_index, dtype=np.int64, copy=True).reshape(-1)

        lengths = {len(array) for array in arrays.values()}
        lengths.add(len(self.road_options))
        if len(lengths) != 1:
            raise ValueError("ReferenceTrajectory arrays and road_options must have the same length.")
        count = next(iter(lengths))
        if count < 2:
            raise ValueError("ReferenceTrajectory requires at least two samples.")
        if any(not np.all(np.isfinite(array)) for array in arrays.values()):
            raise ValueError("ReferenceTrajectory arrays must contain only finite values.")
        if np.any(np.diff(arrays["s_m"]) <= 0.0):
            raise ValueError("ReferenceTrajectory arc length must be strictly increasing.")

        arrays["yaw_rad"] = np.unwrap(arrays["yaw_rad"])
        for name, array in arrays.items():
            array.setflags(write=False)
            object.__setattr__(self, name, array)
        object.__setattr__(self, "road_options", tuple(str(value) for value in self.road_options))
        object.__setattr__(self, "metadata", MappingProxyType(_deterministic_metadata(self.metadata)))

    @property
    def length_m(self):
        return float(self.s_m[-1])

    def sample_at(self, query_s_m):
        query = float(np.clip(float(query_s_m), self.s_m[0], self.s_m[-1]))
        index = min(int(np.searchsorted(self.s_m, query, side="right") - 1), len(self.s_m) - 2)
        index = max(index, 0)
        span = max(float(self.s_m[index + 1] - self.s_m[index]), 1e-12)
        ratio = (query - float(self.s_m[index])) / span

        def interpolate(values):
            return float(values[index] + ratio * (values[index + 1] - values[index]))

        yaw = interpolate(self.yaw_rad)
        nearest = index if ratio < 0.5 else index + 1
        return TrajectorySample(
            s_m=query,
            x_m=interpolate(self.x_m),
            y_m=interpolate(self.y_m),
            z_m=interpolate(self.z_m),
            yaw_rad=_normalize_angle(yaw),
            curvature_1pm=interpolate(self.curvature_1pm),
            curvature_rate_1pm2=interpolate(self.curvature_rate_1pm2),
            left_clearance_m=interpolate(self.left_clearance_m),
            right_clearance_m=interpolate(self.right_clearance_m),
            speed_cap_mps=interpolate(self.speed_cap_mps),
            source_index=int(self.source_index[nearest]),
            road_option=self.road_options[nearest],
        )

    def waypoint_at(self, s_m):
        sample = self.sample_at(s_m)
        lane_width = sample.left_clearance_m + sample.right_clearance_m
        centerline_offset = 0.5 * (sample.right_clearance_m - sample.left_clearance_m)
        return TrajectoryWaypoint(
            transform=TrajectoryTransform(
                location=TrajectoryLocation(sample.x_m, sample.y_m, sample.z_m),
                rotation=TrajectoryRotation(math.degrees(sample.yaw_rad)),
            ),
            id=int(round(sample.s_m * 1000.0)),
            lane_width=float(lane_width),
            source_index=sample.source_index,
            centerline_offset_m=float(centerline_offset),
            left_clearance_m=sample.left_clearance_m,
            right_clearance_m=sample.right_clearance_m,
            curvature_1pm=sample.curvature_1pm,
            curvature_rate_1pm2=sample.curvature_rate_1pm2,
            speed_cap_mps=sample.speed_cap_mps,
        )

    def to_route_trace(self):
        return [
            (self.waypoint_at(float(s_m)), self.road_options[index])
            for index, s_m in enumerate(self.s_m)
        ]

    def _canonical_bytes(self):
        chunks = [b"reference-trajectory-v1\0"]
        float_fields = (
            self.s_m,
            self.x_m,
            self.y_m,
            self.z_m,
            self.yaw_rad,
            self.curvature_1pm,
            self.curvature_rate_1pm2,
            self.left_clearance_m,
            self.right_clearance_m,
            self.speed_cap_mps,
        )
        for values in float_fields:
            canonical = np.asarray(values, dtype="<f8")
            chunks.append(np.asarray([canonical.size], dtype="<i8").tobytes())
            chunks.append(canonical.tobytes(order="C"))
        source = np.asarray(self.source_index, dtype="<i8")
        chunks.append(source.tobytes(order="C"))
        chunks.append(json.dumps(list(self.road_options), ensure_ascii=False, separators=(",", ":")).encode("utf-8"))
        chunks.append(
            json.dumps(
                _deterministic_metadata(self.metadata),
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
        )
        return b"".join(chunks)

    def content_hash(self):
        return hashlib.sha256(self._canonical_bytes()).hexdigest()

    def to_dict(self):
        return {
            "format_version": FORMAT_VERSION,
            "metadata": _json_safe(self.metadata),
            "content_hash": self.content_hash(),
            "trajectory": {
                "s_m": self.s_m.tolist(),
                "x_m": self.x_m.tolist(),
                "y_m": self.y_m.tolist(),
                "z_m": self.z_m.tolist(),
                "yaw_rad": self.yaw_rad.tolist(),
                "curvature_1pm": self.curvature_1pm.tolist(),
                "curvature_rate_1pm2": self.curvature_rate_1pm2.tolist(),
                "left_clearance_m": self.left_clearance_m.tolist(),
                "right_clearance_m": self.right_clearance_m.tolist(),
                "speed_cap_mps": self.speed_cap_mps.tolist(),
                "source_index": self.source_index.tolist(),
                "road_options": list(self.road_options),
            },
        }

    @classmethod
    def from_dict(cls, payload):
        if int(payload.get("format_version", 0)) != FORMAT_VERSION:
            raise ValueError("Unsupported reference trajectory format version.")
        values = payload["trajectory"]
        trajectory = cls(
            s_m=values["s_m"],
            x_m=values["x_m"],
            y_m=values["y_m"],
            z_m=values["z_m"],
            yaw_rad=values["yaw_rad"],
            curvature_1pm=values["curvature_1pm"],
            curvature_rate_1pm2=values["curvature_rate_1pm2"],
            left_clearance_m=values["left_clearance_m"],
            right_clearance_m=values["right_clearance_m"],
            speed_cap_mps=values["speed_cap_mps"],
            source_index=values["source_index"],
            road_options=tuple(values["road_options"]),
            metadata=payload.get("metadata", {}),
        )
        expected_hash = payload.get("content_hash")
        if expected_hash and trajectory.content_hash() != expected_hash:
            raise ValueError("Reference trajectory content hash mismatch.")
        return trajectory


def trajectory_from_route_trace(route_trace, metadata=None):
    if len(route_trace) < 2:
        raise ValueError("A route trace requires at least two waypoints.")
    x_values = []
    y_values = []
    z_values = []
    yaw_values = []
    left_clearance = []
    right_clearance = []
    source_indices = []
    road_options = []
    s_values = [0.0]

    for index, (waypoint, road_option) in enumerate(route_trace):
        transform = waypoint.transform
        location = transform.location
        x_values.append(float(location.x))
        y_values.append(float(location.y))
        z_values.append(float(getattr(location, "z", 0.0)))
        yaw_values.append(math.radians(float(transform.rotation.yaw)))
        lane_width = float(getattr(waypoint, "lane_width", 3.5) or 3.5)
        centerline_offset = float(getattr(waypoint, "centerline_offset_m", 0.0))
        left_clearance.append(float(getattr(waypoint, "left_clearance_m", lane_width * 0.5 - centerline_offset)))
        right_clearance.append(float(getattr(waypoint, "right_clearance_m", lane_width * 0.5 + centerline_offset)))
        source_indices.append(int(getattr(waypoint, "source_index", index)))
        road_options.append(_road_option_name(road_option))
        if index:
            dx = x_values[index] - x_values[index - 1]
            dy = y_values[index] - y_values[index - 1]
            dz = z_values[index] - z_values[index - 1]
            s_values.append(s_values[-1] + math.sqrt(dx * dx + dy * dy + dz * dz))

    s_array = np.asarray(s_values, dtype=float)
    if np.any(np.diff(s_array) <= 1e-9):
        raise ValueError("Route trace contains duplicate consecutive waypoint locations.")
    yaw_array = np.unwrap(np.asarray(yaw_values, dtype=float))
    curvature = np.gradient(yaw_array, s_array, edge_order=1)
    curvature_rate = np.gradient(curvature, s_array, edge_order=1)
    speed_cap = np.asarray(
        [float(getattr(waypoint, "speed_cap_mps", 1_000_000.0)) for waypoint, _ in route_trace],
        dtype=float,
    )
    return ReferenceTrajectory(
        s_m=s_array,
        x_m=x_values,
        y_m=y_values,
        z_m=z_values,
        yaw_rad=yaw_array,
        curvature_1pm=curvature,
        curvature_rate_1pm2=curvature_rate,
        left_clearance_m=left_clearance,
        right_clearance_m=right_clearance,
        speed_cap_mps=speed_cap,
        source_index=source_indices,
        road_options=tuple(road_options),
        metadata=metadata or {},
    )


def save_reference_trajectory(path, trajectory, execution_metadata=None):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    payload = trajectory.to_dict()
    payload["execution"] = _json_safe(execution_metadata or {})
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)


def load_reference_trajectory(path):
    with open(path, "r", encoding="utf-8") as handle:
        return ReferenceTrajectory.from_dict(json.load(handle))
