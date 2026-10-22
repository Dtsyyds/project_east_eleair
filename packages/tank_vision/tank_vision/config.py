"""YAML configuration loading, defaults, merging, and validation."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml


@dataclass(slots=True)
class CameraConfig:
    model: Optional[str] = None
    vendor: Optional[str] = None
    width: int = 4096
    height: int = 3072
    sensor_format: Optional[str] = None
    color: bool = True
    global_shutter: bool = True
    transport: Optional[str] = None
    exposure_us: Optional[float] = None
    exposure_mode: Optional[str] = None
    gain_db: Optional[float] = None
    calibration_file: Optional[str] = None
    calibration_version: Optional[str] = None
    source: str = "folder"
    frame_id: str = "camera"
    serial_number: Optional[str] = None
    sdk_root: Optional[str] = None
    resolution_index: Optional[int] = None
    pixel_format: str = "bgr8"
    white_balance: Optional[str] = None
    fps: float = 15.0
    trigger_mode: str = "continuous"
    trigger_source: Optional[str] = None
    timeout_ms: int = 1000
    warmup_frames: int = 0
    roi: Optional[tuple[int, int, int, int]] = None


@dataclass(slots=True)
class PipelineConfig:
    mode: str = "natural_seam"
    use_undistort: bool = False
    roi: Optional[tuple[int, int, int, int]] = None
    thumbnail_max_dimension: int = 1600
    save_overlay: bool = True
    save_json: bool = True
    save_masks: bool = True


@dataclass(slots=True)
class SeamConfig:
    detector: str = "classical"
    reference_x_ratio: float = 0.5
    reference_y_ratio: float = 0.7
    expected_direction: str = "vertical"
    min_confidence: float = 0.5
    lost_frame_threshold: int = 3
    color_channel: str = "gray"
    clahe_clip_limit: float = 2.0
    clahe_grid_size: int = 8
    filter_type: str = "gaussian"
    gaussian_kernel: int = 5
    bilateral_sigma: float = 35.0
    local_contrast_kernel: int = 41
    local_contrast_weight: float = 0.75
    gradient_weight: float = 0.25
    threshold_method: str = "otsu"
    adaptive_block_size: int = 31
    adaptive_constant: float = -2.0
    morphology_kernel: int = 5
    morphology_open_kernel: int = 3
    morphology_iterations: int = 2
    minimum_response: float = 4.0
    response_percentile: float = 99.5
    min_component_area_ratio: float = 0.001
    min_component_span_ratio: float = 0.25
    center_reasonable_margin_ratio: float = 0.45
    centerline_step_px: int = 4
    fit_degree: int = 2
    ransac_iterations: int = 120
    ransac_residual_px: float = 4.0
    max_fit_residual_px: float = 12.0
    continuity_max_gap_px: float = 2.5
    contrast_reference: float = 35.0
    contrast_ring_kernel: int = 11


@dataclass(slots=True)
class DefectConfig:
    backend: str = "classical"
    tile_size: int = 1024
    overlap: int = 128
    confidence_threshold: float = 0.5
    min_area_px: int = 80
    max_area_ratio: float = 0.2
    local_contrast_kernel: int = 31
    anomaly_z_threshold: float = 2.5
    texture_z_offset: float = 0.5
    color_difference_threshold: float = 28.0
    morphology_open_kernel: int = 3
    morphology_close_kernel: int = 7
    morphology_close_iterations: int = 2
    color_confidence_scale: float = 80.0
    exclusion_dilation_kernel: int = 13
    nms_iou_threshold: float = 0.35
    model_path: Optional[str] = None


@dataclass(slots=True)
class LaserConfig:
    enabled: bool = False
    wavelength_nm: int = 450
    calibrated: bool = False
    blue_difference_threshold: int = 35
    hsv_h_min: int = 90
    hsv_h_max: int = 140
    saturation_min: int = 70
    value_min: int = 60
    min_points: int = 20
    ransac_residual_px: float = 2.0
    ransac_iterations: int = 100
    morphology_open_kernel: int = 3
    morphology_close_width: int = 5
    morphology_close_height: int = 3
    quadratic_fit_min_points: int = 20


@dataclass(slots=True)
class QualityConfig:
    enable_blur_check: bool = True
    enable_exposure_check: bool = True
    underexposed_threshold: int = 12
    overexposed_threshold: int = 245
    max_underexposed_ratio: float = 0.65
    max_overexposed_ratio: float = 0.25
    min_laplacian_variance: float = 20.0
    min_contrast_std: float = 8.0


@dataclass(slots=True)
class PlatformConfig:
    name: str = "auto"
    architecture: Optional[str] = None
    ros_distro: Optional[str] = None


@dataclass(slots=True)
class RealtimeSeamConfig:
    enabled: bool = True
    queue_size: int = 1
    drop_policy: str = "drop_oldest"
    process_latest_only: bool = True


@dataclass(slots=True)
class InspectionConfig:
    enabled: bool = True
    queue_size: int = 2
    drop_policy: str = "drop_newest"
    tile_size: int = 1024
    overlap: int = 128
    trigger_mode: str = "frame_interval"
    trigger_every_n_frames: int = 15
    trigger_distance_mm: Optional[float] = None


@dataclass(slots=True)
class InferenceConfig:
    backend: str = "opencv"
    model_path: Optional[str] = None
    model_metadata_path: Optional[str] = None
    target_arch: Optional[str] = None
    input_format: str = "bgr"
    model_calibrated: bool = False


@dataclass(slots=True)
class StorageConfig:
    save_all_raw: bool = False
    save_abnormal_raw: bool = True
    normal_frame_interval: int = 15
    pre_event_frames: int = 15
    post_event_frames: int = 15
    compression: str = "jpg"
    output_queue_size: int = 32
    drop_policy: str = "drop_newest"
    disk_low_space_threshold_mb: int = 1024


@dataclass(slots=True)
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    pipeline: PipelineConfig = field(default_factory=PipelineConfig)
    seam: SeamConfig = field(default_factory=SeamConfig)
    defect: DefectConfig = field(default_factory=DefectConfig)
    laser: LaserConfig = field(default_factory=LaserConfig)
    quality: QualityConfig = field(default_factory=QualityConfig)
    platform: PlatformConfig = field(default_factory=PlatformConfig)
    realtime_seam: RealtimeSeamConfig = field(default_factory=RealtimeSeamConfig)
    inspection: InspectionConfig = field(default_factory=InspectionConfig)
    inference: InferenceConfig = field(default_factory=InferenceConfig)
    storage: StorageConfig = field(default_factory=StorageConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _section(section_type: type[Any], data: dict[str, Any], name: str) -> Any:
    allowed = set(section_type.__dataclass_fields__)
    unknown = set(data) - allowed
    if unknown:
        raise ValueError(f"Unknown keys in '{name}': {sorted(unknown)}")
    return section_type(**data)


def validate_config(config: AppConfig) -> None:
    if config.camera.width <= 0 or config.camera.height <= 0:
        raise ValueError("camera width and height must be positive")
    if config.camera.resolution_index is not None and config.camera.resolution_index < 0:
        raise ValueError("camera resolution_index must be non-negative")
    if config.pipeline.roi is not None:
        if len(config.pipeline.roi) != 4 or any(int(v) < 0 for v in config.pipeline.roi):
            raise ValueError("pipeline.roi must be [x, y, width, height] with non-negative values")
        if config.pipeline.roi[2] <= 0 or config.pipeline.roi[3] <= 0:
            raise ValueError("ROI width and height must be positive")
        config.pipeline.roi = tuple(int(v) for v in config.pipeline.roi)
    if config.camera.roi is not None:
        if len(config.camera.roi) != 4 or any(int(v) < 0 for v in config.camera.roi):
            raise ValueError("camera.roi must be [x, y, width, height] with non-negative values")
        if config.camera.roi[2] <= 0 or config.camera.roi[3] <= 0:
            raise ValueError("camera ROI width and height must be positive")
        config.camera.roi = tuple(int(v) for v in config.camera.roi)
    for name, value in (
        ("seam.reference_x_ratio", config.seam.reference_x_ratio),
        ("seam.reference_y_ratio", config.seam.reference_y_ratio),
        ("seam.min_confidence", config.seam.min_confidence),
        ("defect.confidence_threshold", config.defect.confidence_threshold),
    ):
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be in [0, 1]")
    if config.seam.expected_direction not in {"vertical", "horizontal"}:
        raise ValueError("seam.expected_direction must be 'vertical' or 'horizontal'")
    if config.seam.color_channel not in {"gray", "blue", "green", "red"}:
        raise ValueError("seam.color_channel must be gray, blue, green, or red")
    if config.seam.filter_type not in {"gaussian", "bilateral"}:
        raise ValueError("seam.filter_type must be gaussian or bilateral")
    if config.seam.threshold_method not in {"otsu", "adaptive"}:
        raise ValueError("seam.threshold_method must be otsu or adaptive")
    if config.seam.fit_degree not in {1, 2}:
        raise ValueError("seam.fit_degree must be 1 or 2")
    if not 0.0 < config.seam.response_percentile <= 100.0:
        raise ValueError("seam.response_percentile must be in (0, 100]")
    if config.seam.local_contrast_weight < 0 or config.seam.gradient_weight < 0:
        raise ValueError("seam response weights cannot be negative")
    if config.seam.local_contrast_weight + config.seam.gradient_weight <= 0:
        raise ValueError("at least one seam response weight must be positive")
    for name, value in (
        ("seam.gaussian_kernel", config.seam.gaussian_kernel),
        ("seam.local_contrast_kernel", config.seam.local_contrast_kernel),
        ("seam.adaptive_block_size", config.seam.adaptive_block_size),
        ("seam.morphology_kernel", config.seam.morphology_kernel),
        ("seam.morphology_open_kernel", config.seam.morphology_open_kernel),
        ("seam.contrast_ring_kernel", config.seam.contrast_ring_kernel),
        ("defect.local_contrast_kernel", config.defect.local_contrast_kernel),
        ("defect.morphology_open_kernel", config.defect.morphology_open_kernel),
        ("defect.morphology_close_kernel", config.defect.morphology_close_kernel),
        ("defect.exclusion_dilation_kernel", config.defect.exclusion_dilation_kernel),
        ("laser.morphology_open_kernel", config.laser.morphology_open_kernel),
        ("laser.morphology_close_width", config.laser.morphology_close_width),
        ("laser.morphology_close_height", config.laser.morphology_close_height),
    ):
        if value < 1 or value % 2 == 0:
            raise ValueError(f"{name} must be a positive odd integer")
    if config.defect.tile_size <= 0 or not 0 <= config.defect.overlap < config.defect.tile_size:
        raise ValueError("defect overlap must satisfy 0 <= overlap < tile_size")
    if config.seam.lost_frame_threshold < 1 or config.seam.ransac_iterations < 1:
        raise ValueError("seam lost-frame threshold and RANSAC iterations must be positive")
    if config.defect.min_area_px < 1 or config.defect.morphology_close_iterations < 1:
        raise ValueError("defect area and morphology iteration settings must be positive")
    if config.laser.min_points < 1 or config.laser.ransac_iterations < 1:
        raise ValueError("laser point and RANSAC iteration settings must be positive")
    if not 0 <= config.laser.hsv_h_min <= config.laser.hsv_h_max <= 179:
        raise ValueError("laser HSV hue bounds must satisfy 0 <= min <= max <= 179")
    if config.pipeline.thumbnail_max_dimension < 0:
        raise ValueError("pipeline.thumbnail_max_dimension cannot be negative")
    if config.camera.fps <= 0 or config.camera.timeout_ms < 1:
        raise ValueError("camera fps and timeout must be positive")
    exposure_mode = config.camera.exposure_mode
    if exposure_mode is None:
        # Preserve legacy profiles: null exposure means AE, numeric means manual.
        exposure_mode = "manual" if config.camera.exposure_us is not None else "auto"
    if exposure_mode not in {"auto", "manual"}:
        raise ValueError("camera exposure_mode must be auto or manual")
    if exposure_mode == "manual" and (config.camera.exposure_us is None or config.camera.exposure_us <= 0):
        raise ValueError("manual camera exposure_mode requires a positive exposure_us")
    if exposure_mode == "auto" and config.camera.exposure_us is not None:
        raise ValueError("auto camera exposure_mode requires exposure_us=null")
    if config.camera.warmup_frames < 0:
        raise ValueError("camera warmup_frames cannot be negative")
    if config.camera.trigger_mode not in {"continuous", "software", "hardware"}:
        raise ValueError("camera.trigger_mode must be continuous, software, or hardware")
    for name, queue_size in (
        ("realtime_seam.queue_size", config.realtime_seam.queue_size),
        ("inspection.queue_size", config.inspection.queue_size),
        ("storage.output_queue_size", config.storage.output_queue_size),
    ):
        if queue_size < 1:
            raise ValueError(f"{name} must be positive")
    for name, policy in (
        ("realtime_seam.drop_policy", config.realtime_seam.drop_policy),
        ("inspection.drop_policy", config.inspection.drop_policy),
        ("storage.drop_policy", config.storage.drop_policy),
    ):
        if policy not in {"drop_oldest", "drop_newest", "block"}:
            raise ValueError(f"{name} must be drop_oldest, drop_newest, or block")
    if config.realtime_seam.process_latest_only and config.realtime_seam.drop_policy != "drop_oldest":
        raise ValueError("process_latest_only requires realtime_seam.drop_policy=drop_oldest")
    if config.inspection.tile_size <= 0 or not 0 <= config.inspection.overlap < config.inspection.tile_size:
        raise ValueError("inspection overlap must satisfy 0 <= overlap < tile_size")
    if config.inspection.trigger_mode not in {"always", "frame_interval", "distance", "external"}:
        raise ValueError("inspection.trigger_mode must be always, frame_interval, distance, or external")
    if config.inspection.trigger_every_n_frames < 1:
        raise ValueError("inspection.trigger_every_n_frames must be positive")
    if config.inspection.trigger_distance_mm is not None and config.inspection.trigger_distance_mm <= 0:
        raise ValueError("inspection.trigger_distance_mm must be positive when configured")
    if config.storage.normal_frame_interval < 1:
        raise ValueError("storage.normal_frame_interval must be positive")
    if config.storage.pre_event_frames < 0 or config.storage.post_event_frames < 0:
        raise ValueError("storage event windows cannot be negative")
    if config.storage.compression not in {"jpg", "png", "none"}:
        raise ValueError("storage.compression must be jpg, png, or none")
    if config.storage.disk_low_space_threshold_mb < 0:
        raise ValueError("storage.disk_low_space_threshold_mb cannot be negative")
    if config.inference.backend not in {"opencv", "onnx", "d_robotics_bpu", "rknn"}:
        raise ValueError("inference.backend must be opencv, onnx, d_robotics_bpu, or rknn")
    if config.inference.input_format.lower() not in {"bgr", "rgb", "nv12", "gray"}:
        raise ValueError("inference.input_format must be BGR, RGB, NV12, or gray")
    platform_name = config.platform.name.lower()
    if platform_name not in {"auto", "x86", "s100p", "rk3588"}:
        raise ValueError("platform.name must be auto, x86, s100p, or rk3588")
    if config.inference.backend == "d_robotics_bpu":
        if config.inference.target_arch not in {None, "nash_m"}:
            raise ValueError("S100P BPU models must target nash_m; other RDK board models are incompatible")
        if platform_name == "rk3588":
            raise ValueError("d_robotics_bpu backend cannot be selected for RK3588")
    if config.inference.backend == "rknn" and platform_name == "s100p":
        raise ValueError("RKNN models cannot be selected for S100P")


def load_config(path: str | Path | None = None) -> AppConfig:
    raw: dict[str, Any] = {}
    if path is not None:
        config_path = Path(path)
        if not config_path.is_file():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        with config_path.open("r", encoding="utf-8") as handle:
            loaded = yaml.safe_load(handle) or {}
        if not isinstance(loaded, dict):
            raise ValueError("Configuration root must be a mapping")
        raw = loaded
    allowed_sections = {
        "camera",
        "pipeline",
        "seam",
        "defect",
        "laser",
        "quality",
        "platform",
        "realtime_seam",
        "inspection",
        "inference",
        "storage",
    }
    unknown_sections = set(raw) - allowed_sections
    if unknown_sections:
        raise ValueError(f"Unknown configuration sections: {sorted(unknown_sections)}")
    config = AppConfig(
        camera=_section(CameraConfig, raw.get("camera", {}), "camera"),
        pipeline=_section(PipelineConfig, raw.get("pipeline", {}), "pipeline"),
        seam=_section(SeamConfig, raw.get("seam", {}), "seam"),
        defect=_section(DefectConfig, raw.get("defect", {}), "defect"),
        laser=_section(LaserConfig, raw.get("laser", {}), "laser"),
        quality=_section(QualityConfig, raw.get("quality", {}), "quality"),
        platform=_section(PlatformConfig, raw.get("platform", {}), "platform"),
        realtime_seam=_section(RealtimeSeamConfig, raw.get("realtime_seam", {}), "realtime_seam"),
        inspection=_section(InspectionConfig, raw.get("inspection", {}), "inspection"),
        inference=_section(InferenceConfig, raw.get("inference", {}), "inference"),
        storage=_section(StorageConfig, raw.get("storage", {}), "storage"),
    )
    validate_config(config)
    return config
