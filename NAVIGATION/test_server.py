"""Small manual client for testing the Flask navigation server without hardware."""

import argparse
import json
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


# Tweak these values for different tests.
GRID_SIZE = 8
START = [0, 0]
DESTINATION = [GRID_SIZE - 1, GRID_SIZE - 1]
BLOCK_ON_STEP = 1  # Set to None to test without a camera blockage.


def request_json(base_url: str, method: str, path: str, payload: dict | None = None) -> dict:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = Request(
        f"{base_url}{path}",
        data=body,
        method=method,
        headers={"Content-Type": "application/json"} if body else {},
    )
    with urlopen(request, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def print_json(label: str, value: dict) -> None:
    print(f"\n{label}")
    print(json.dumps(value, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(description="Test the Flask car-navigation server.")
    parser.add_argument("--url", default="http://127.0.0.1:5000", help="Flask server URL")
    args = parser.parse_args()
    base_url = args.url.rstrip("/")
    grid = [[1 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

    try:
        mission = request_json(
            base_url,
            "POST",
            "/api/mission",
            {"free_grid": grid, "start": START, "destination": DESTINATION},
        )
        print_json("Mission created", mission)

        position = START[:]
        for step in range(GRID_SIZE * GRID_SIZE):
            status = request_json(
                base_url,
                "POST",
                "/api/car/status",
                {"coordinates": position, "reached": position == DESTINATION, "running": True},
            )
            print_json(f"ESP status at step {step}", status)

            if BLOCK_ON_STEP == step:
                camera = request_json(
                    base_url,
                    "POST",
                    "/api/camera/status",
                    {"blocked": True},
                )
                print_json("Camera reported blockage and server replanned", camera)

            command = request_json(base_url, "GET", "/api/car/command")
            print_json("Command", command)
            if command.get("command") == "STOP":
                break

            target = command.get("target")
            if not isinstance(target, list) or len(target) != 2:
                raise RuntimeError(f"Server returned an invalid target: {command}")
            position = target

        print_json("Final server state", request_json(base_url, "GET", "/api/state"))
        print("\nServer test completed successfully.")
    except (HTTPError, URLError, TimeoutError, RuntimeError) as error:
        print(f"\nServer test failed: {error}")
        print("Start the server first with: python server.py")
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
