# SIH-2026

SIH-2026 is an unmanned ground vehicle software project. It combines computer
vision, free-space detection, grid-based route planning, vehicle simulation,
and optional motor control through an ESP8266.

## System Overview

The repository contains two navigation implementations:

- **Vision navigation** uses a live camera, YOLO object detection, and MiDaS
	relative depth to decide whether the vehicle should move forward, turn, or
	stop. It can log commands for testing or send them to an ESP8266.
- **Map navigation** processes a grid of images with YOLO, builds an obstacle
	map, converts it into free and blocked cells, finds a route, and simulates or
	serves movement commands to a vehicle.

The two systems are separate programs. The vision system makes decisions from
live video, while the map system makes decisions from a prepared image grid or
free-space grid.

## Repository Layout

```text
NAVIGATION/
	detect.py          YOLO image detection
	map.py             Image-grid composition and coordinate mapping
	obstacles.py       Detection results to obstacle matrix
	simulation.py      Grid pathfinding and vehicle simulation
	navigation_state.py Mission state and movement commands
	pipeline.py        End-to-end image-grid navigation pipeline
	server.py          Flask API for missions and vehicle status
	test_server.py     API test client
	output/            Generated detections and maps

Unmanned_vehicle-v2/
	main.py            Real-time perception and control entry point
	config.py          Camera, model, geometry, and driving settings
	vision/            Camera, depth, YOLO, traversability, planner, and display code
	yolo11n-seg.pt     YOLO segmentation model
```

## Vision Navigation

The vision program in `Unmanned_vehicle-v2` processes each camera frame through
these stages:

1. Capture a phone IP-camera stream, webcam, video file, or RTSP stream.
2. Estimate relative inverse depth with MiDaS.
3. Detect semantic objects with YOLO11 segmentation.
4. Analyse the lower image region as drivable ground and identify obstacles,
	 walls, and blocked areas.
5. Plan a `STOP`, `FORWARD`, `LEFT`, `RIGHT`, or `REVERSE` vehicle command.
6. Display the camera, depth, traversability, and command information.
7. Optionally send motor commands to an ESP8266.

By default, the program is perception-only and does not move the vehicle.
Driving requires `--drive`, starts disarmed, and requires the `a` key to arm.
The space bar performs an emergency stop and `q` quits safely. The ESP8266 also
stops the motors if command packets stop arriving.

Example commands:

```powershell
cd Unmanned_vehicle-v2
python main.py
python main.py --source 0
python main.py --source clip.mp4 --headless --frames 60
python main.py --drive
```

Set the camera source, model options, camera height, camera tilt, processing
resolution, and speed limits in `Unmanned_vehicle-v2/config.py`. Camera pose is
important because ground distance and wall detection depend on it.

## Map Navigation

The `NAVIGATION` pipeline accepts a rectangular matrix of image paths. For each
image it detects objects, saves an annotated copy, combines the images into a
global map, creates an obstacle matrix, and converts that matrix into a free
grid. A pathfinder then searches from a start cell to a destination cell. The
result can be visualised with the built-in simulator.

The navigation server exposes the same grid-based state to an ESP8266 or test
client. Start it with:

```powershell
cd NAVIGATION
python server.py
```

The server listens on port `5000` and provides these endpoints:

| Method | Endpoint | Purpose |
| --- | --- | --- |
| `POST` | `/api/mission` | Create a mission from a free-cell grid, start, and destination |
| `GET` | `/api/state` | Return the current mission state |
| `GET` | `/api/car/command` | Return the next movement command |
| `POST` | `/api/car/status` | Report vehicle coordinates and running status |
| `POST` | `/api/camera/status` | Report a newly detected blockage and replan |
| `POST` | `/api/mission/stop` | Stop the active mission |

Example mission request:

```json
{
	"free_grid": [[1, 1, 1], [1, 0, 1], [1, 1, 1]],
	"start": [0, 0],
	"destination": [2, 2]
}
```

Coordinates use `[row, column]`. A value of `1` is free and `0` is blocked.
When the camera reports a blockage, the server marks nearby cells as blocked
and recalculates a route from the vehicle's latest position.

## Setup

Use Python 3 and create a separate virtual environment for each subsystem:

```powershell
cd Unmanned_vehicle-v2
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

For the grid navigation subsystem:

```powershell
cd NAVIGATION
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

The first vision run may download model dependencies or weights. A working
camera source is required for live perception; the fallback webcam or a video
file can be used for development.

## Safety

Test all motor commands with the vehicle wheels raised first. Keep the vehicle
disarmed while checking camera and planning behaviour. When driving, use a low
speed limit, keep a person beside the vehicle, and keep the emergency-stop key
available. Never operate the vehicle without confirming that the camera pose,
motor direction, wireless connection, and failsafe behaviour are correct.