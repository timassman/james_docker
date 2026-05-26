# robojames-docker

Docker image for the James robot: Roomba 500 base, OAK-D Pro camera, OpenManipulator-X arm, Xbox controller.
Running ROS2 Humble on a Jetson Orin Nano (ARM64, CUDA).

---

## Hardware

| Component | USB connection | Speed | Device in container |
|---|---|---|---|
| OAK-D Pro | USB-C port → USB 3.0 hub | 5000M (USB 3.0) | `/dev/bus/usb` |
| OAK-D Lite | any USB-A port | 480M (USB 2.0 max by design) | `/dev/bus/usb` |
| OpenManipulator-X | any USB-A port | 480M | `/dev/openmanipulator` (udev symlink) |
| Roomba 500 | any USB-A port | 12M | `/dev/roomba` (udev symlink) |
| Xbox controller | USB hub (via USB-C) | 12M | `/dev/input/xbox-controller` (udev symlink) |
| Mouse + keyboard | USB hub (via USB-C) | 1.5M | — |

Jetson SSH: `ssh jetson-orin` (configured in `~/.ssh/config`)

### USB ports — important

The Jetson Orin Nano Developer Kit case labels all four USB-A ports as "USB 3.0", but
this is misleading. All four USB-A ports are physically wired to the USB 2.0 controller
and max out at 480 Mbps regardless of cable quality. This is a confirmed hardware routing
decision on the carrier board, not a driver or cable issue.

**Only the USB-C port provides real USB 3.0 (5 Gbps).**

- **OAK-D Pro** must be on the USB-C port (directly or via USB 3.0 hub). It needs USB 3.0
  for stereo depth + RGB at full resolution (~1-3 Gbps).
- **OAK-D Lite** is USB 2.0 only by design — any USB-A port is fine.

Reference: https://forums.developer.nvidia.com/t/orin-nano-8gb-no-usb-3-0/313345

### Verifying USB connections

Use `lsusb -t` on the Jetson to verify all devices are on the correct bus:

```
Bus 02 (USB 3.0) → hub → OAK-D Pro at 5000M   ← correct
Bus 01 (USB 2.0) → OAK-D Lite at 480M          ← correct (USB 2.0 by design)
                 → Roomba at 12M
                 → OpenManipulator at 480M
                 → hub → Xbox controller, mouse, keyboard
```

Note: the OAK-D Pro shows at **480M in bootloader mode** before the depthai driver starts.
It re-enumerates at **5000M after firmware upload** when `ros2 launch depthai_ros_driver camera.launch.py`
is running inside the container. Always verify with the driver active.

---

## Scripts overview

| Script | Where to run | Purpose |
|---|---|---|
| `setup_build_on_laptop.sh` | laptop | One-time: install QEMU + swap for ARM64 cross-compilation |
| `build_on_laptop.sh` | laptop | Build ARM64 Docker image via QEMU (slow, ~30 min) |
| `build_on_jetson.sh` | Jetson | Build Docker image natively on the Jetson (faster, no QEMU) |
| `deploy_to_jetson.sh` | laptop | Transfer built image from laptop to Jetson via SSH |
| `setup_jetson.sh` | Jetson | One-time: Docker, NVIDIA toolkit, udev rules, swap, hostname |
| `setup_monitor_laptop.sh` | laptop | One-time: ROS2 + Fast DDS config to see Jetson topics on laptop |
| `run.sh` | Jetson | Start the robot container |
| `test_container.sh` | Jetson | Smoke test: GPU, ROS2, camera, serial devices |
| `sync_james.sh` | laptop | Sync local changes to Jetson via rsync (no commit needed) |

---

## Scenario A — First-time setup

### 1. Set up the Jetson (run once on the Jetson)

```bash
bash setup_jetson.sh
```

Installs Docker, NVIDIA Container Toolkit, udev rules for all hardware,
8 GB swap, sets hostname to `jetson-orin`, tunes ROS2 DDS network buffers.

### 2. Set up the laptop for cross-compilation (run once on the laptop)

```bash
./setup_build_on_laptop.sh
```

Installs QEMU v7.0.0 (pinned — newer versions cause ARM64 segfaults on recent kernels),
creates a 16 GB swap file.

### 3. Set up the laptop for ROS2 monitoring (run once on the laptop)

```bash
./setup_monitor_laptop.sh
```

Configures ROS2 Humble and Fast DDS in `~/.bashrc`.
Fast DDS is configured to use only real LAN interfaces (excludes docker0)
so that ROS2 topics published by the Jetson container are discoverable on the laptop.

---

## Scenario B — Build and deploy the Docker image

### Option 1: build on the laptop, deploy to Jetson (recommended during development)

The laptop stays free after deployment. The Jetson is not tied up during the build.

```bash
# On the laptop:
./build_on_laptop.sh       # ~30 min, runs at low priority so laptop stays usable
./deploy_to_jetson.sh      # streams image via SSH (~10 min depending on network speed)
```

### Option 2: build directly on the Jetson (faster, no QEMU)

Requires the repo to be present on the Jetson (`git clone` or `rsync` from laptop).
The Jetson is tied up during the build and may run warm.

```bash
# On the Jetson:
git clone https://github.com/timassman/james_docker.git
cd james_docker
./build_on_jetson.sh       # ~20 min native ARM64, no QEMU crashes
```

---

## Scenario C — Start the robot

```bash
# On the Jetson:
./run.sh
```

Removes any existing container with the same name and starts a fresh one.
After the first run, Docker restarts the container automatically on reboot
(controlled by `RESTART_POLICY` in `run.sh`; default is `on-failure:3` during development).

---

## Scenario D — Verify the container

```bash
# On the Jetson, copy the test script into the running container:
docker cp test_container.sh robojames:/tmp/
docker exec -it robojames bash /tmp/test_container.sh
```

Checks: GPU (nvidia-smi), ROS2 environment, all expected ROS2 packages,
Python packages (depthai, open3d, ultralytics), OAK-D camera detection, serial devices.

---

## Scenario E — Iterative script development (no Dockerfile change)

When editing scripts (`run.sh`, `setup_jetson.sh`, etc.) but not the Dockerfile,
there is no need to rebuild the image. Use `sync_james.sh` to push changes directly
to the Jetson for testing, then commit once it works.

```bash
# On the laptop — after editing a script:
./sync_james.sh            # rsync changed files to Jetson (no commit needed)

# On the Jetson — test the change:
./run.sh                   # or whichever script was changed

# Back on the laptop — once it works:
git add .
git commit -m "..."
git push

# On the Jetson — sync back to the committed version:
git fetch && git reset --hard origin/main
```

`git fetch` downloads the latest commits from GitHub.
`git reset --hard origin/main` discards the rsync'd files and replaces them with
the committed version — keeping the Jetson in sync with git.

---

## Scenario F — After a Dockerfile change

```bash
# On the laptop:
./build_on_laptop.sh       # rebuilds only changed layers (Docker cache)
./deploy_to_jetson.sh      # redeploy

# On the Jetson:
./run.sh                   # start fresh container from new image
```

---

## Scenario G — Monitor from the laptop (RViz2)

### 1. Start the driver on the Jetson (inside the container)

```bash
ros2 launch depthai_ros_driver camera.launch.py pointcloud.enable:=true
```

### 2. Open a new terminal on the laptop

```bash
source ~/.bashrc           # applies LARGE_DATA mode and ROS_DOMAIN_ID
ros2 daemon stop           # restart daemon to pick up new DDS settings
ros2 topic list            # should show topics from the Jetson
rviz2
```

### 3. Configure RViz2

1. **Global Options → Fixed Frame**: change `map` to `oak`
2. Click **Add → By topic → /oak/points → PointCloud2**
3. In the display panel: set **Reliability Policy** to **Best Effort**

The point cloud updates live at ~15 Hz (XYZRGB, full 720p resolution).

Key topics published by the OAK-D Pro:
- `/oak/points` — XYZRGB point cloud (enable with `pointcloud.enable:=true`)
- `/oak/rgb/image_raw` — RGB camera
- `/oak/stereo/image_raw` — depth image (16-bit)
- `/oak/nn/spatial_detections` — neural network 3D detections
- `/oak/imu/data` — IMU

> **Note:** Fast DDS LARGE_DATA mode (set by `setup_monitor_laptop.sh` and `run.sh`)
> is required for large topics like point clouds. It uses TCP for data transport,
> avoiding UDP fragmentation loss.
>
> **WiFi limitation:** point clouds (28 MB/frame) and depth images (1.8 MB/frame) are
> too large for reliable WiFi delivery — UDP fragments are systematically dropped even
> with LARGE_DATA mode. A wired LAN connection between laptop and Jetson is required
> for monitoring. Connect both to the same switch via ethernet cable.

---

## Known limitations

- `open_manipulator_x_gui` is skipped: Qt5 fails to compile under QEMU ARM64 and is
  useless on a headless Jetson.
- depthai-ros v2.12.2 is used (not v3): the dustynv base image ships libdepthai 2.x;
  v3 requires libdepthai 3.x built from source.
- Open3D and rtabmap are installed from apt/pip (CPU only). See Dockerfile comments
  for overlay build instructions to enable CUDA acceleration.

---

## Future improvements

- **depthai-ros v3** (currently using v2.12.2):
  v3 requires libdepthai 3.x which is not in the dustynv base image.
  To upgrade: add a Dockerfile step to build libdepthai 3.x from source, then change
  the git clone tag to v3.x-humble. Adds ~1–2h to build time.

- **rtabmap with CUDA** (currently using apt, no CUDA):
  The apt version works but does not use the Jetson GPU for feature extraction.
  To enable: build rtabmap from source as an overlay image with `-DWITH_CUDA=ON`.
  See the commented overlay instructions in the Dockerfile RTAB-Map section.

- **Open3D with CUDA** (currently using CPU pip wheel):
  For GPU-accelerated point cloud processing: build Open3D from source with
  `-DBUILD_CUDA_MODULE=ON`. See the commented overlay instructions in the Dockerfile
  PCL section.

- **open_manipulator_x_gui** (currently skipped):
  Qt5 fails to compile under QEMU ARM64. Options:
  - Build directly on the Jetson (`build_on_jetson.sh`, no QEMU), or
  - Set up proper cross-compilation with a Qt5 sysroot.

- **Build directly on the Jetson** (`build_on_jetson.sh`):
  Eliminates all QEMU overhead and segfaults. Much faster and more stable,
  but the Jetson is tied up during the build. Ensure swap is configured first
  (`setup_jetson.sh`).
