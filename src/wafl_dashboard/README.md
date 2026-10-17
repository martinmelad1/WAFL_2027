# WAFL 2027 — Web Dashboard

Web dashboard (HTML/CSS/JS + FastAPI backend) for the warehouse forklift: live map, camera, robot status, Auto/Manual mode, manual drive and E-STOP.

> The root README (§5, D4) still says PySide6. The agreed decision is **HTML + FastAPI**.

## Status

| Part | Status |
|------|--------|
| `frontend/` layout, styling, responsive (desktop + phone) | Done |
| Tab switching, Auto/Manual toggle (UI only) | Done |
| `backend/main.py` (FastAPI + rclpy) | Not started |
| Live data, commands, map and camera rendering | Not started |
| Tab 2 (Logs), Tab 3 (WMS) | Placeholders |

## Structure

```
forklift_dashboard/
├── backend/main.py        ← FastAPI + ROS 2 bridge (todo)
└── frontend/
    ├── index.html         ← structure
    ├── style.css          ← design
    └── script.js          ← behaviour
```

## Run

Open `frontend/index.html` in a browser. For a phone on the same Wi-Fi:

```bash
cd frontend && python3 -m http.server 8000
```

then open `http://<laptop-ip>:8000` (`hostname -I` shows the IP).

## For the backend: how the page is wired

`script.js` finds elements by `id` (do not rename them):

- **Values:** `status-state`, `status-position`, `status-heading`, `status-speed`, `status-battery`, `status-fork`, `status-package`, `status-quantity`, `status-weight`, `status-eta`, `mission-id`, `mission-from`, `mission-to`
- **LEDs** (`system-led`, `mission-progress`, `mission-done`, `safety-normal`, `safety-obstacle`, `safety-estop`): add the class `green`, `amber` or `red`.
- **Battery bar:** `battery-bar` — set `width` in %.
- **Map / camera:** `map-canvas`, `camera-feed` (set `src`).
- **Buttons:** `estop`, `mode-auto`, `mode-manual`. Manual buttons use `data-cmd`: `forward`, `backward`, `left`, `right`, `lift-up`, `lift-down`, `lights-on`, `lights-off`.

## Data format (JSON) — proposal, needs team approval

Backend → dashboard:

```json
{
  "system_online": true, "mode": "auto", "state": "NAVIGATE",
  "battery": 85, "speed": 0.8, "fork_height_cm": 0, "heading_deg": 90,
  "position": { "x": 1.2, "y": 3.4 },
  "safety": "normal",
  "mission": { "id": "M-001", "from": "Zone A", "to": "Zone B", "status": "in_progress", "eta_s": 120 },
  "package": { "id": "P-042", "quantity": 4, "weight_kg": 120 }
}
```

`safety`: `normal` | `obstacle` | `estop` · `mission.status`: `in_progress` | `completed`

Dashboard → backend:

```json
{ "type": "estop",  "value": true }
{ "type": "mode",   "value": "manual" }
{ "type": "move",   "command": "forward" }
{ "type": "lift",   "command": "up" }
{ "type": "lights", "command": "on" }
```

## ROS 2 topics

| Subscribe | Used for |
|-----------|----------|
| `/map`, TF `map→base_footprint`, `/plan` | map, robot pose, path |
| `/odometry/filtered` | speed |
| `/robot/state`, `/fork_height`, `/battery/state` | status |
| `/wms/mission_status` | mission |

| Publish | When |
|---------|------|
| `/e_stop` (`Bool`), `/gui/mode` (`String`) | E-STOP, mode toggle — use `transient_local` QoS |
| `/manual_cmd_vel` (`Twist`) | drive arrows, **Manual only** |
| `/lift_cmd` (`Int32`), `/lights_cmd` (`Int32MultiArray`) | fork / lights, **Manual only** |

Drive must stop when the button is released or the connection drops.
