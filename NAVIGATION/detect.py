"""YOLO detection helpers.

The module does not load a model or process an image during import. This keeps
it safe to reuse from the pipeline and from tests.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
from ultralytics import YOLO


@dataclass(frozen=True)
class Detection:
	"""One detected object in source-image pixel coordinates."""

	class_id: int
	class_name: str
	confidence: float
	x1: float
	y1: float
	x2: float
	y2: float


@dataclass
class DetectionResult:
	"""Detections and the annotated image produced for one source image."""

	image_path: str
	image_width: int
	image_height: int
	detections: list[Detection]
	annotated_image: object


def load_model(model_path: str = "yolo26n.pt") -> YOLO:
	"""Load a YOLO model. Ultralytics downloads missing model weights."""

	return YOLO(model_path)


def detect_image(
	image_path: str,
	model: YOLO,
	confidence: float = 0.25,
	classes: Optional[Sequence[int]] = None,
) -> DetectionResult:
	"""Detect objects in one image and return its annotated image."""

	source_path = Path(image_path)
	if not source_path.is_file():
		raise FileNotFoundError(f"Image not found: {source_path}")

	image = cv2.imread(str(source_path))
	if image is None:
		raise ValueError(f"Unable to read image: {source_path}")

	height, width = image.shape[:2]
	result = model.predict(
		source=str(source_path),
		conf=confidence,
		classes=classes,
		verbose=False,
	)[0]

	names = result.names
	detections: list[Detection] = []
	if result.boxes is not None:
		for box in result.boxes:
			class_id = int(box.cls[0].item())
			x1, y1, x2, y2 = box.xyxy[0].tolist()
			detections.append(
				Detection(
					class_id=class_id,
					class_name=str(names[class_id]),
					confidence=float(box.conf[0].item()),
					x1=x1,
					y1=y1,
					x2=x2,
					y2=y2,
				)
			)

	return DetectionResult(
		image_path=str(source_path),
		image_width=width,
		image_height=height,
		detections=detections,
		annotated_image=result.plot(),
	)


def save_annotated_image(result: DetectionResult, output_path: str) -> None:
	"""Save the marked image returned by YOLO."""

	output = Path(output_path)
	output.parent.mkdir(parents=True, exist_ok=True)
	if not cv2.imwrite(str(output), result.annotated_image):
		raise OSError(f"Unable to save annotated image: {output}")