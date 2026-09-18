"""Flask API for ESP8266 car control and camera blockage events."""

from threading import Lock
from typing import Any

from flask import Flask, jsonify, request

from navigation_state import Coordinate, MissionState


def _coordinate(value: Any, field_name: str) -> Coordinate:
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field_name} must be [row, column].")
    if not all(isinstance(item, int) and not isinstance(item, bool) for item in value):
        raise ValueError(f"{field_name} must contain integer coordinates.")
    return value[0], value[1]


def _json_body() -> dict:
    body = request.get_json(silent=True)
    if not isinstance(body, dict):
        raise ValueError("Request body must be a JSON object.")
    return body


def create_app() -> Flask:
    app = Flask(__name__)
    state_holder: dict[str, MissionState | None] = {"mission": None}
    state_lock = Lock()

    def mission() -> MissionState:
        current = state_holder["mission"]
        if current is None:
            raise ValueError("No mission has been created.")
        return current

    @app.errorhandler(ValueError)
    def handle_value_error(error: ValueError):
        return jsonify({"error": str(error)}), 400

    @app.post("/api/mission")
    def create_mission():
        body = _json_body()
        grid = body.get("free_grid")
        if not isinstance(grid, list) or not grid or not all(isinstance(row, list) for row in grid):
            raise ValueError("free_grid must be a non-empty matrix.")
        free_grid = [[int(cell) for cell in row] for row in grid]
        start = _coordinate(body.get("start", [0, 0]), "start")
        destination = _coordinate(body.get("destination"), "destination")
        with state_lock:
            state_holder["mission"] = MissionState(
                free_grid=free_grid,
                start=start,
                destination=destination,
                current=start,
                running=True,
            )
            state = state_holder["mission"]
            assert state is not None
            return jsonify(state.as_dict()), 201

    @app.get("/api/state")
    def get_state():
        with state_lock:
            return jsonify(mission().as_dict())

    @app.post("/api/car/status")
    def car_status():
        body = _json_body()
        coordinates = _coordinate(body.get("coordinates"), "coordinates")
        reached = body.get("reached")
        running = body.get("running")
        if not isinstance(reached, bool) or not isinstance(running, bool):
            raise ValueError("reached and running must be boolean values.")
        with state_lock:
            state = mission()
            state.set_car_status(coordinates, reached, running)
            return jsonify({"ok": True, "state": state.as_dict()})

    @app.get("/api/car/command")
    def car_command():
        with state_lock:
            state = mission()
            return jsonify(state.next_command())

    @app.post("/api/camera/status")
    def camera_status():
        body = _json_body()
        blocked = body.get("blocked")
        if not isinstance(blocked, bool):
            raise ValueError("blocked must be a boolean value.")
        with state_lock:
            state = mission()
            if blocked and not state.camera_blocked:
                blocked_cells = state.mark_blocked_window()
            else:
                blocked_cells = []
                state.camera_blocked = blocked
            return jsonify({"blocked_cells": [list(cell) for cell in blocked_cells], "state": state.as_dict()})

    @app.post("/api/mission/stop")
    def stop_mission():
        with state_lock:
            state = mission()
            state.running = False
            return jsonify({"ok": True, "command": "STOP", "state": state.as_dict()})

    return app


app = create_app()


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
