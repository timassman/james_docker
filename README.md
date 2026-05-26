# robojames-docker

Docker image for the James robot: Roomba 500 base, OAK-D Pro camera, OpenManipulator-X arm, Xbox controller.
Running ROS2 Humble on a Jetson Orin Nano (ARM64, CUDA).

---

## Hardware

| Component | Device in container |
|---|---|
| Roomba 500 | `/dev/ttyUSB0` |
| OpenManipulator-X | `/dev/ttyUSB1` |
| OAK-D Pro camera | `/dev/bus/usb` (dynamic cgroup rule) |
| Xbox controller | `/dev/input/xbox-controller` (udev symlink) |

Jetson SSH: `ssh jetson-orin` (configured in `~/.ssh/config`)

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

## Scenario E — After a Dockerfile change

```bash
# On the laptop:
./build_on_laptop.sh       # rebuilds only changed layers (Docker cache)
./deploy_to_jetson.sh      # redeploy

# On the Jetson:
./run.sh                   # start fresh container from new image
```

---

## Scenario F — Monitor from the laptop (RViz2)

With the robot container running on the Jetson and `setup_monitor_laptop.sh` applied:

```bash
# Open a new terminal on the laptop (Fast DDS config is applied via ~/.bashrc):
ros2 topic list            # should show topics from the Jetson
rviz2                      # open RViz2 and add camera/point cloud displays
```

Key topics published by the OAK-D Pro:
- `/oak/rgb/image_raw` — RGB camera
- `/oak/stereo/image_raw` — depth image (16-bit)
- `/oak/nn/spatial_detections` — neural network 3D detections
- `/oak/imu/data` — IMU

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
