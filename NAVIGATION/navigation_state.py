"""Mission state and replanning logic for the Flask navigation server."""

from dataclasses import dataclass, field
from typing import Sequence

from simulation import find_path

Coordinate = tuple[int, int]


@dataclass
class MissionState:
    """Mutable state shared by the ESP and camera API endpoints."""

    free_grid: list[list[int]]
    start: Coordinate
    destination: Coordinate
    current: Coordinate
    path: list[Coordinate] = field(default_factory=list)
    blocked_cells: set[Coordinate] = field(default_factory=set)
    camera_blocked: bool = False
    reached: bool = False
    running: bool = False
    command_id: int = 0
    last_command: dict | None = None

    def __post_init__(self) -> None:
        self.path = find_path(self.free_grid, self.start, self.destination) or []
        if not self.path:
            raise ValueError("No path exists from start to destination.")

    @property
    def status(self) -> str:
        if self.reached:
            return "reached"
        if not self.running:
            return "stopped"
        if not self.path:
            return "no_path"
        return "running"

    def set_car_status(self, coordinates: Coordinate, reached: bool, running: bool) -> None:
        if coordinates != self.current:
            self.last_command = None
        self.current = coordinates
        self.reached = reached or coordinates == self.destination
        self.running = running and not self.reached
        if self.reached:
            self.last_command = {"command_id": self.command_id, "command": "STOP"}

    def mark_blocked_window(self, offsets: Sequence[int] = (-1, 0, 2)) -> list[Coordinate]:
        """Block cells around the next planned cell and replan."""

        if self.current not in self.path:
            raise ValueError("Current coordinate is not in the current path.")
        next_index = self.path.index(self.current) + 1
        blocked: list[Coordinate] = []
        for offset in offsets:
            path_index = next_index + offset
            if not (0 <= path_index < len(self.path)):
                continue
            cell = self.path[path_index]
            if cell in (self.current, self.destination):
                continue
            if cell not in self.blocked_cells:
                self.free_grid[cell[0]][cell[1]] = 0
                self.blocked_cells.add(cell)
                blocked.append(cell)

        self.camera_blocked = True
        self.path = find_path(self.free_grid, self.current, self.destination) or []
        self.last_command = None
        return blocked

    def next_command(self) -> dict:
        """Return one movement command for the next path cell."""

        if self.reached or not self.running or not self.path:
            return {"command_id": self.command_id, "command": "STOP"}
        if self.last_command is not None:
            return self.last_command
        try:
            current_index = self.path.index(self.current)
        except ValueError:
            return {"command_id": self.command_id, "command": "STOP"}
        if current_index >= len(self.path) - 1:
            return {"command_id": self.command_id, "command": "STOP"}

        target = self.path[current_index + 1]
        direction = {
            (-1, 0): "UP",
            (1, 0): "DOWN",
            (0, -1): "LEFT",
            (0, 1): "RIGHT",
        }.get((target[0] - self.current[0], target[1] - self.current[1]))
        if direction is None:
            return {"command_id": self.command_id, "command": "STOP"}

        self.command_id += 1
        self.last_command = {
            "command_id": self.command_id,
            "command": direction,
            "target": list(target),
        }
        return self.last_command

    def as_dict(self) -> dict:
        return {
            "current": list(self.current),
            "start": list(self.start),
            "destination": list(self.destination),
            "path": [list(cell) for cell in self.path],
            "blocked_cells": [list(cell) for cell in sorted(self.blocked_cells)],
            "camera_blocked": self.camera_blocked,
            "reached": self.reached,
            "running": self.running,
            "status": self.status,
            "last_command": self.last_command,
        }
