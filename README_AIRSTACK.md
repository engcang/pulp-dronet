# PULP-DroNet v3 with AirStack

The worker uses the official float ResBlock or Tiny v3 checkpoint from
[pulp-platform/pulp-dronet](https://github.com/pulp-platform/pulp-dronet).
Camera RGB is converted to grayscale; the network predicts steering and
collision probability. The AirStack adapter converts these outputs to flight
commands. It receives no route, goal or ground-truth obstacle information.

## Installation

Use Linux, Docker, NVIDIA Container Toolkit and a working AirStack Isaac/PX4
setup. Keep `AirStack/` and `pulp-dronet/` beside each other.

```bash
git clone --recursive --branch eungchang/adv-ws2 git@github.com:castacks/AirStack.git
git clone git@github.com:engcang/pulp-dronet.git
cd pulp-dronet
python3 airstack_integration/check_weights.py
docker build -t pulp-dronet-airstack:1.0 -f docker/Dockerfile.airstack .
```

Prepare the simulator/robot containers and bridge as described in
`AirStack/tools/ws2_bench/README.md`. The upstream checkpoints are already in
`tiny-pulp-dronet-v3/model/`: `pulp-dronet-v3-resblock-1.0.pth` and
`tiny-pulp-dronet-v3-dw-pw-0.125.pth`. No training dataset or NEMO installation
is needed. This worker uses CPU inference; Isaac still requires the GPU.

## Run

From `AirStack/`, use separate terminals for the viewer and flight:

```bash
source .venv-ws2/bin/activate
python3 tools/ws2_bench/dashboard.py --port 8892
python3 tools/ws2_bench/episode.py tools/ws2_bench/scenarios/corridor_pulp.yaml \
  --no-bag --camera follow --record-video
```

Open `http://127.0.0.1:8892`. The bench handles takeoff, worker startup,
execution, evaluation and landing. Recording requires `ffmpeg` or
`imageio-ffmpeg`. Omit `--no-bag` to also save a ROS bag.

For recorded common-map clean and sensor-attack tests:

```bash
python3 tools/ws2_bench/corridor_suite.py --targets pulp_dronet --tiers easy \
  --output robot/ros_ws/ws2_runtime/campaigns/pulp_recorded
```

The suite first checks clean traversal. Completed runs are reused on resume;
use a new output directory after changing settings. Noise and delay are
supported; Rui's FCRN patch is rejected for PULP. The viewer's Start tests form
runs Office campaigns; corridor suites are launched through the CLI.

## Configuration

- `AirStack/tools/ws2_bench/scenarios/corridor_pulp.yaml`: `pulp_variant`,
  `maximum_speed`, `yaw_gain_deg_s`, `input_fov_deg`, `command_smoothing`,
  `trajectory_horizon`, altitude and timeout. The corridor preset selects
  `tiny`, FOV 78°, yaw gain 90°/s and maximum speed 0.35 m/s.
- `AirStack/tools/ws2_bench/scenarios/corridor_suite.yaml`: shared geometry,
  lighting, noise/delay and observer camera. It overrides the target YAML's
  scene in suite runs.
- Standalone worker: run `python pulp_dronet_airstack.py --help` inside
  `pulp-dronet-airstack:1.0`. `--variant tiny --headless --execute` needs an
  already running bridge and airborne vehicle; use the bench for full flights.

No model retraining is performed. The command adapter integrates yaw using
simulation time and bounds tracking error; it is separate from the network.
Flight results and limits are recorded in AirStack
`tools/ws2_bench/CORRIDOR_VALIDATION.md`. Keep generated data, videos and caches
out of Git. Upstream attribution is in `airstack_integration/LICENSE.upstream`.
