# WAFL 2025 — Launch Guide

> **Start here if you just cloned this repo.**  
> No ROS installation required — everything runs inside Docker.

---

## What you need

| Requirement | Notes |
|-------------|-------|
| Linux (Ubuntu 20.04 or 22.04) | Windows/macOS not supported for GUI |
| Docker Engine | Install guide below |
| A monitor / desktop | GUI windows won't work on a headless server |

---

## Step 1 — Install Docker (skip if already installed)

```bash
# Install Docker
sudo apt update
sudo apt install -y docker.io

# Allow your user to run Docker without sudo
sudo usermod -aG docker $USER

# Apply group change (or log out and back in)
newgrp docker

# Verify
docker --version
```

---

## Step 2 — Clone the repo

```bash
git clone <repo-url>
cd <repo-folder-name>
```

> WARNING: **All commands in this guide must be run from inside the repo root folder** (the folder you just `cd` into above — the one that contains `docker/` and `LAUNCH.md`). Do not run them from inside a subfolder.

---

## Step 3 — Allow GUI windows

Run this **once per login session** (every time you restart your PC):

```bash
xhost +local:docker
```

---

## Step 4 — Build the Docker image

Run this **once** (first time only), from the **repo root folder**:

```bash
# Make sure you are in the repo root (contains docker/ and LAUNCH.md)
bash docker/run.sh build
```

> [time] Takes ~10 minutes — downloads Ubuntu 20.04 + ROS Noetic + Gazebo (~2 GB total).  
> After the first build, subsequent builds are instant (cached).

You should see at the end:
```
OK:  Build complete! Image: wafl2025:latest
```

---

## Step 5 — Run it

> All commands below must be run from the **repo root folder** (same place you ran `build`).

### See the robot model (GUI with joint sliders)

```bash
bash docker/run.sh gui
```

Opens **RViz** showing the full 3D forklift model. Use the slider panel to move any joint interactively.

---

### See the robot in Gazebo simulation

```bash
bash docker/run.sh gazebo
```

Opens **Gazebo** with the robot spawned in an empty world. Also opens RViz.

---

### Drive the robot manually with the keyboard ← *Start here for testing*

```bash
bash docker/run.sh teleop
```

Opens Gazebo + RViz, then starts keyboard teleoperation in the same terminal.

**Click the terminal window, then use:**

| Key | Action |
|-----|--------|
| `W` | Move forward |
| `S` | Move backward |
| `A` | Turn left |
| `D` | Turn right |
| `Q` | Strafe left (crab mode) |
| `E` | Strafe right (crab mode) |
| `Space` | **Emergency STOP** |
| `Z` | Zero / reset steering |
| `+` | Speed up (+0.1 m/s) |
| `-` | Slow down (−0.1 m/s) |
| `Ctrl-C` | Exit |

> **Wait ~5–6 seconds** after the command for Gazebo to fully load before typing keys.

---

### Open a shell inside the container (for debugging)

```bash
bash docker/run.sh shell
```

Drops you into a bash shell with ROS fully sourced. You can run any `rostopic`, `rosnode`, `roslaunch` commands manually.

---

## All available commands

```
bash docker/run.sh build    # Build image (once)
bash docker/run.sh gui      # URDF viewer + joint sliders
bash docker/run.sh gazebo   # Gazebo simulation
bash docker/run.sh teleop   # Gazebo + keyboard driving  ← best for testing
bash docker/run.sh slam     # Hector SLAM (needs real LiDAR)
bash docker/run.sh nav      # Autonomous navigation (needs real LiDAR)
bash docker/run.sh shell    # Interactive shell
```

---

## Troubleshooting

### "Cannot connect to display" / windows don't open

```bash
# Run this first, then retry:
xhost +local:docker
```

### "docker: Got permission denied"

```bash
sudo usermod -aG docker $USER
newgrp docker   # or log out and back in
```

### Gazebo opens but robot doesn't appear

Wait 10–15 seconds. Gazebo can be slow to spawn models on the first run.  
If it still doesn't appear, check the terminal for error messages.

### "Image not found" / "No such image"

You haven't built yet. Run:
```bash
bash docker/run.sh build
```

### Teleop keys don't do anything

Make sure you clicked the **terminal window** (not the Gazebo or RViz window) before pressing keys. The keyboard listener reads from the terminal where you launched `teleop`.

### Build fails midway (network error)

Just run `build` again — Docker caches completed steps, so it will resume from where it failed.

---

## What's inside the Docker image

| Component | Version |
|-----------|---------|
| OS | Ubuntu 20.04 LTS |
| ROS | Noetic (desktop-full) |
| Gazebo | 11 |
| Python | 3.8 |
| WAFL2025 package | Built with catkin_make |

> **Note:** ROS 1 Noetic reached end-of-life in May 2025. This only means no new official updates — the software works perfectly and everything is already installed inside the image.

---

## For software testing — what to verify

| Test | How |
|------|-----|
| Robot model loads correctly | `bash docker/run.sh gui` — check RViz shows the full forklift |
| Gazebo simulation works | `bash docker/run.sh gazebo` — robot appears in world |
| Manual control works | `bash docker/run.sh teleop` — robot responds to WASD keys |
| ROS nodes are running | `bash docker/run.sh shell` then `rosnode list` |
| Topics are publishing | `bash docker/run.sh shell` then `rostopic list` |

---

## File structure (docker/)

```
docker/
├── Dockerfile       # Builds the ROS Noetic environment + WAFL2025 package
├── entrypoint.sh    # Auto-sources ROS + workspace on container start
└── run.sh           # User-friendly launcher script
```

---

*For full technical documentation, see [WAFL2025/README.md](WAFL2025-forklift2425-patch-manual-mapping/WAFL2025/README.md)*
