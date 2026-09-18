"""Convert YOLO bounding boxes into a global obstacle matrix."""

from typing import Sequence

from detect import DetectionResult


def create_obstacle_matrix(
    detection_grid: Sequence[Sequence[DetectionResult]],
    grid_rows: int,
    grid_cols: int,
) -> list[list[int]]:
    """Return ``1 = obstacle`` and ``0 = free`` for the complete map.

    A cell is marked when any detection bounding box overlaps its area. Boxes
    are scaled from the original image size to the resized map-cell image.
    """

    if not detection_grid or not detection_grid[0]:
        raise ValueError("Detection grid cannot be empty.")
    if grid_rows <= 0 or grid_cols <= 0:
        raise ValueError("Grid dimensions must be positive.")
    source_cols = len(detection_grid[0])
    if any(len(row) != source_cols for row in detection_grid):
        raise ValueError("All detection-grid rows must have the same length.")

    total_rows = len(detection_grid) * grid_rows
    total_cols = source_cols * grid_cols
    matrix = [[0 for _ in range(total_cols)] for _ in range(total_rows)]

    for image_row, row in enumerate(detection_grid):
        for image_col, result in enumerate(row):
            cell_width = result.image_width / grid_cols
            cell_height = result.image_height / grid_rows

            for detection in result.detections:
                left = max(0, int(detection.x1 / cell_width))
                right = min(grid_cols - 1, int((detection.x2 - 1) / cell_width))
                top = max(0, int(detection.y1 / cell_height))
                bottom = min(grid_rows - 1, int((detection.y2 - 1) / cell_height))

                for local_row in range(top, bottom + 1):
                    for local_col in range(left, right + 1):
                        global_row = image_row * grid_rows + local_row
                        global_col = image_col * grid_cols + local_col
                        matrix[global_row][global_col] = 1

    return matrix