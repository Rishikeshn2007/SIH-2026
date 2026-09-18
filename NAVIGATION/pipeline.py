"""End-to-end image detection, mapping, obstacle extraction, and simulation."""

from pathlib import Path
from typing import Optional, Sequence

from detect import DetectionResult, detect_image, load_model, save_annotated_image
from map import create_global_image_grid
from obstacles import create_obstacle_matrix
from simulation import find_path, obstacle_to_free_grid, simulate_car


def run_pipeline(
    source: Sequence[Sequence[str]],
    model_path: str = "yolo26n.pt",
    output_dir: str = "output",
    image_width: int = 400,
    image_height: int = 400,
    grid_rows: int = 10,
    grid_cols: int = 10,
    start: tuple[int, int] = (0, 0),
    goal: Optional[tuple[int, int]] = None,
    confidence: float = 0.25,
    show_simulation: bool = True,
) -> dict:
    """Run the complete navigation flow and return all generated artifacts."""

    if not source or not source[0]:
        raise ValueError("Source cannot be empty.")
    if any(len(row) != len(source[0]) for row in source):
        raise ValueError("All source rows must have the same length.")

    output_path = Path(output_dir)
    annotated_dir = output_path / "detections"
    model = load_model(model_path)

    detection_grid: list[list[DetectionResult]] = []
    annotated_source: list[list[str]] = []
    for image_row, row in enumerate(source):
        detected_row = []
        annotated_row = []
        for image_col, image_path in enumerate(row):
            result = detect_image(image_path, model, confidence=confidence)
            annotated_path = annotated_dir / f"detected_{image_row:02d}_{image_col:02d}.jpg"
            save_annotated_image(result, str(annotated_path))
            detected_row.append(result)
            annotated_row.append(str(annotated_path))
        detection_grid.append(detected_row)
        annotated_source.append(annotated_row)

    combined_image, coordinate_matrix = create_global_image_grid(
        source=annotated_source,
        output_dir=output_dir,
        image_width=image_width,
        image_height=image_height,
        grid_rows=grid_rows,
        grid_cols=grid_cols,
    )
    obstacle_matrix = create_obstacle_matrix(detection_grid, grid_rows, grid_cols)
    free_grid = obstacle_to_free_grid(obstacle_matrix)

    if goal is None:
        goal = (len(free_grid) - 1, len(free_grid[0]) - 1)
    path = find_path(free_grid, start, goal)

    if show_simulation:
        if path is None:
            raise RuntimeError(f"No path found from {start} to {goal}.")
        simulate_car(free_grid, start, goal, path)

    return {
        "annotated_source": annotated_source,
        "combined_image": combined_image,
        "coordinate_matrix": coordinate_matrix,
        "obstacle_matrix": obstacle_matrix,
        "free_grid": free_grid,
        "path": path,
    }


if __name__ == "__main__":
    result = run_pipeline(
        source=[["img.jpeg","img.jpeg"],["img.jpeg","img.jpeg"]],
        start=(0, 0),
        goal=(10,10),
        show_simulation=True,
    )
    print("Obstacle matrix:")
    for row in result["obstacle_matrix"]:
        print(row)