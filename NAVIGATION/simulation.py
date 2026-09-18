"""Path finding and interactive simulation for an obstacle grid."""

from collections import deque
from typing import MutableSequence, Sequence

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def find_path(
    free: Sequence[Sequence[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
) -> list[tuple[int, int]] | None:
    """Find a shortest 4-direction path through cells where ``1`` is free."""

    if not free or not free[0]:
        raise ValueError("Grid cannot be empty.")
    if any(len(row) != len(free[0]) for row in free):
        raise ValueError("All grid rows must have the same length.")

    rows = len(free)
    cols = len(free[0])
    for point, name in ((start, "Start"), (goal, "Destination")):
        if not (0 <= point[0] < rows and 0 <= point[1] < cols):
            raise ValueError(f"{name} is outside the grid.")
    if free[start[0]][start[1]] == 0:
        raise ValueError("Starting point is blocked.")
    if free[goal[0]][goal[1]] == 0:
        raise ValueError("Destination is blocked.")

    queue = deque([start])
    parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    directions = ((-1, 0), (1, 0), (0, -1), (0, 1))

    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for row_step, col_step in directions:
            next_cell = (current[0] + row_step, current[1] + col_step)
            if not (0 <= next_cell[0] < rows and 0 <= next_cell[1] < cols):
                continue
            if free[next_cell[0]][next_cell[1]] == 0 or next_cell in parent:
                continue
            parent[next_cell] = current
            queue.append(next_cell)

    if goal not in parent:
        return None

    path: list[tuple[int, int]] = []
    current: tuple[int, int] | None = goal
    while current is not None:
        path.append(current)
        current = parent[current]
    path.reverse()
    return path


def obstacle_to_free_grid(
    obstacle_matrix: Sequence[Sequence[int]],
) -> list[list[int]]:
    """Convert ``1 = obstacle, 0 = free`` to the BFS convention."""

    if not obstacle_matrix or not obstacle_matrix[0]:
        raise ValueError("Obstacle matrix cannot be empty.")
    if any(len(row) != len(obstacle_matrix[0]) for row in obstacle_matrix):
        raise ValueError("All obstacle-matrix rows must have the same length.")
    return [[0 if cell else 1 for cell in row] for row in obstacle_matrix]


def replan_path(
    free: MutableSequence[MutableSequence[int]],
    current: tuple[int, int],
    goal: tuple[int, int],
    blocked_cell: tuple[int, int],
) -> list[tuple[int, int]] | None:
    """Block one cell in ``free`` and find a new path to ``goal``."""

    rows = len(free)
    cols = len(free[0]) if rows else 0
    row, col = blocked_cell
    if not (0 <= row < rows and 0 <= col < cols):
        raise ValueError("Blocked cell is outside the grid.")
    if blocked_cell == current:
        raise ValueError("The car's current cell cannot be blocked.")
    if blocked_cell == goal:
        raise ValueError("The destination cell cannot be blocked.")

    free[row][col] = 0
    return find_path(free, current, goal)


def simulate_car(
    free: Sequence[Sequence[int]],
    start: tuple[int, int],
    goal: tuple[int, int],
    path: Sequence[tuple[int, int]],
    interval: int = 500,
) -> FuncAnimation:
    """Animate the car and replan when the user clicks a new obstacle cell."""

    if not path:
        raise ValueError("Path cannot be empty.")

    mutable_grid = [list(row) for row in free]
    current_path = list(path)
    state = {"path_index": 0, "running": True}
    animation: FuncAnimation | None = None
    figure, axis = plt.subplots()
    grid_image = axis.imshow(mutable_grid, cmap="Greys", origin="upper", vmin=0, vmax=1)
    axis.scatter(start[1], start[0], color="green", s=120, label="Start")
    axis.scatter(goal[1], goal[0], color="red", s=120, label="Destination")
    path_line, = axis.plot([], [], color="blue", linewidth=2, linestyle="--")
    car, = axis.plot([], [], marker="o", markersize=12, color="orange", label="Car")
    axis.set_xlim(-0.5, len(mutable_grid[0]) - 0.5)
    axis.set_ylim(len(mutable_grid) - 0.5, -0.5)
    axis.set_aspect("equal")
    axis.legend()

    def stop_animation() -> None:
        if animation is not None and animation.event_source is not None:
            animation.event_source.stop()

    def handle_click(event) -> None:
        if event.inaxes is not axis or event.xdata is None or event.ydata is None:
            return
        blocked_cell = (int(event.ydata + 0.5), int(event.xdata + 0.5))
        current = current_path[state["path_index"]]
        if not (0 <= blocked_cell[0] < len(mutable_grid)):
            return
        if not (0 <= blocked_cell[1] < len(mutable_grid[0])):
            return
        if mutable_grid[blocked_cell[0]][blocked_cell[1]] == 0:
            return
        if blocked_cell in (current, goal):
            return

        new_path = replan_path(mutable_grid, current, goal, blocked_cell)
        if new_path is None:
            state["running"] = False
            axis.set_title(f"No path from {current} to {goal}")
            stop_animation()
            return

        current_path[:] = new_path
        state["path_index"] = 0
        grid_image.set_data(mutable_grid)
        figure.canvas.draw_idle()

    figure.canvas.mpl_connect("button_press_event", handle_click)

    def update(frame: int):
        if not state["running"]:
            return (car, path_line, grid_image)
        row, col = current_path[state["path_index"]]
        car.set_data([col], [row])
        path_rows = [point[0] for point in current_path]
        path_cols = [point[1] for point in current_path]
        path_line.set_data(path_cols, path_rows)
        axis.set_title(f"Car position: ({row}, {col}) | Click a cell to block and replan")
        if state["path_index"] < len(current_path) - 1:
            state["path_index"] += 1
        else:
            state["running"] = False
            stop_animation()
        return (car, path_line, grid_image)

    animation = FuncAnimation(
        figure,
        update,
        frames=None,
        interval=interval,
        repeat=True,
    )
    plt.show()
    return animation


if __name__ == "__main__":
    demo_grid = [[1, 1, 1], [0, 0, 1], [1, 1, 1]]
    demo_path = find_path(demo_grid, (0, 0), (2, 2))
    if demo_path is not None:
        simulate_car(demo_grid, (0, 0), (2, 2), demo_path)
