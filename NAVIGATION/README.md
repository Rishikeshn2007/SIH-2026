# NAVIGATION

## Flask car controller

Install the server dependency inside the project virtual environment:

```powershell
.\venv\Scripts\python.exe -m pip install flask
```

Start the server:

```powershell
.\venv\Scripts\python.exe server.py
```

The server listens on `0.0.0.0:5000`.

### Test without ESP8266

Keep the Flask server running in one terminal. In a second terminal run:

```powershell
.\venv\Scripts\python.exe test_server.py
```

The test client simulates ESP coordinates, polls movement commands, reports a
camera blockage, and prints the replanned state. Edit `GRID_SIZE`, `START`,
`DESTINATION`, and `BLOCK_ON_STEP` at the top of `test_server.py` to change the
test. Set `BLOCK_ON_STEP = None` to test a clear route.

### Create a mission

Send the free-cell grid produced by the existing pipeline. Coordinates use
`[row, column]`, and `1` means free while `0` means blocked.

```http
POST /api/mission
Content-Type: application/json
```

```json
{
	"free_grid": [[1, 1, 1], [1, 1, 1], [1, 1, 1]],
	"start": [0, 0],
	"destination": [2, 2]
}
```

### ESP8266 status

```http
POST /api/car/status
```

```json
{
	"coordinates": [0, 1],
	"reached": false,
	"running": true
}
```

The ESP8266 polls its next command:

```http
GET /api/car/command
```

Example response:

```json
{
	"command_id": 1,
	"command": "RIGHT",
	"target": [0, 2]
}
```

### Camera status

```http
POST /api/camera/status
```

```json
{
	"blocked": true
}
```

When `blocked` changes to `true`, the server marks the path cells at offsets
`(-1, 0, +2)` around the next planned cell, ignoring the current and
destination cells, then recalculates BFS from the latest ESP coordinate.

Useful monitoring and safety endpoints:

```text
GET  /api/state
POST /api/mission/stop
```