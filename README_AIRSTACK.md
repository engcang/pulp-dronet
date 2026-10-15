# PULP-DroNet v3 with AirStack

This integration runs the official pretrained **float PULP-DroNet v3 ResBlock
1.0** or **Tiny v3 depthwise 0.125 (no bypass)** checkpoint. It does not retrain the model and does not require NEMO or
the embedded build tools. Isaac camera images become grayscale input; the
network predicts steering and collision probability. Its outputs control an
Iris through the existing AirStack controller and PX4.

**Status: experimental.** The first Office trials did not establish reliable
clean navigation. Bent-corridor qualification and matched attack testing are
documented in AirStack's `tools/ws2_bench/CORRIDOR_VALIDATION.md`. A successful
build or model inference is not evidence of successful flight.

## Installation

Requirements: Linux, Docker, NVIDIA Container Toolkit, and a working local
AirStack Isaac/PX4 setup. Follow the AirStack repository's installation guide
first. Keep these repositories beside each other:

```text
workspace/
  AirStack/       # eungchang/adv-ws2, including the PULP adapter
  pulp-dronet/    # this fork
```

From your chosen workspace directory:

```bash
git clone --recursive --branch eungchang/adv-ws2 git@github.com:castacks/AirStack.git
git clone git@github.com:engcang/pulp-dronet.git
cd pulp-dronet
python3 airstack_integration/check_weights.py
docker build -t pulp-dronet-airstack:1.0 -f docker/Dockerfile.airstack .
```

Skip a clone if that checkout already exists. The bench examples require the
accompanying PULP adapter and corridor changes in AirStack; until those changes
are published, use the existing WS2 development checkout. The standalone HTTP
worker also works with an already configured compatible vision bridge.

The small official checkpoint is
already tracked by upstream at
`tiny-pulp-dronet-v3/model/pulp-dronet-v3-resblock-1.0.pth`; the Tiny variant is
`tiny-pulp-dronet-v3/model/tiny-pulp-dronet-v3-dw-pw-0.125.pth`. No second download,
training dataset, CUDA runtime, or model API credential is required for this
worker. Its CPU inference leaves the GPU available for Isaac. The bench mounts
this checkout into the image, so source edits do not require an image rebuild.

Optional control-interface smoke test after building:

```bash
docker run --rm -v "$PWD:/workspace/planner:ro" \
  pulp-dronet-airstack:1.0 python -m unittest airstack_integration.test_policy
```

## Run through the common bench

From `AirStack/`, save this as a scenario YAML:

```yaml
name: pulp_rounded_u_clean
planner: pulp_dronet
height: 1.2
timeout: 500
maximum_speed: 0.35
input_fov_deg: 78
yaw_gain_deg_s: 90
pulp_variant: tiny # held fixed throughout a study
condition:
  name: clean
  layout: corridor
  layout_seed: 42
  seed: 42
  light: 1800
  corridor:
    shape: u_obstacles
    width: 4.0
    obstacles: 4
    style: hallway
    bend_radius: 4.0
    route_scale: 1.5
```

```bash
python3 tools/ws2_bench/episode.py scenario.yaml --resolve-only
python3 tools/ws2_bench/episode.py scenario.yaml --no-bag --camera follow
```

This profile completed two fresh clean U-corridor flights with four blocks,
plus one bare zigzag traversal (38.00m measured motion,210.00 simulation seconds).
Matched sensor checks are listed below. These small checks do not establish
general reliability in other layouts or Office. The
runner resets the simulator and worker, takes off, executes the model, scores
all-scene PhysX contacts and saves configuration, source hashes, trajectories,
metrics and diagnostic images. Omit `--no-bag` if a ROS bag is also needed.
Corridor missions require ordered traversal to the end; merely remaining in
the air is insufficient. Other supported shapes are `curve`, `zigzag`, and
`elbow`. Corridor geometry is **not** sent to this reactive model.

For repeated clean checks followed by isolated noise/delay comparisons:

```bash
python3 tools/ws2_bench/corridor_study.py scenario.yaml \
  --output robot/ros_ws/ws2_runtime/campaigns/pulp_corridor_study \
  --clean-repeats 2 --attack rgb_noise --attack delay
```

Use a new output directory when changing settings. A failed clean check stops
that study; it does not establish an attack effect. The study saves an English
`vulnerability_report.md` and JSON results. The viewer observes and controls
these CLI-launched corridor runs; its Office launch form is a separate workflow.

```bash
python3 tools/ws2_bench/dashboard.py
```

Open the displayed local viewer URL (normally `http://127.0.0.1:8892`) to see
simulator and inference images. The registered target is `pulp_dronet`.
Random, search, feedback and optional Claude use the same adapter. Supported
perturbations are RGB noise and sensor delay. Rui's FCRN patch is rejected for
DroNet. Always establish a clean reference before interpreting attack effects.

## Observed corridor results (2026-10-14)

The Tiny/FOV78/yaw90 profile above was held fixed throughout a six-flight
U-course study, with fresh simulator and worker starts. The four clean flights
(two qualification runs plus one control per perturbation) all completed.

| Condition | Observed result |
|---|---|
| Clean, four wall-attached blocks |4/4 completed the29.56m scoring route |
| RGB Gaussian noise, sigma32 on0–255 pixels |Collision at22.08% route progress; one matched comparison |
| Additional sensor delay,0.5 simulation seconds |Completed;3.45s longer than its clean control |
| Separate bare zigzag clean check |38.00m actual motion,210.00s,21/21 gates,no contact |

Noise and delay were tested separately. The noise result is a candidate
vulnerability observation, not a statistical robustness estimate; the passing
delay run does not prove immunity to latency. These results concern the larger
AirStack Iris, rendered corridor and documented control adapter, not a
reproduction of the paper's original nano-UAV performance. The earlier FOV60
zigzag failure and unsuccessful ResBlock/Office trials remain in the bench log.
The L-corridor ResBlock pass is not evidence that Tiny has passed that scene.

## Standalone worker

For an **already running** AirStack HTTP vision bridge, inspect the options:

```bash
docker run --rm --network host \
  -v "$PWD:/workspace/planner:ro" \
  pulp-dronet-airstack:1.0 python pulp_dronet_airstack.py --help
```

For the corridor-tested profile, replace `--help` with
`--server http://127.0.0.1:8765 --variant tiny --input-fov 78 --yaw-gain 90
--maximum-speed 0.35 --headless --execute` only when the simulator and flight
controller are ready. Omitting the variant selects the legacy ResBlock model;
it is not the Tiny profile tested above. This worker does not
start Isaac, arm the drone, record a bag, or perform landing by itself; the
episode runner is the recommended entry point.

## What changes from the original nano-UAV implementation?

- **Camera:** a calibrated square crop from the Isaac RGB camera is converted
  to grayscale, resized to 200×200 and divided by 255. `input_fov_deg` records
  its angular extent. The default 60° approximates the original Himax center
  crop; photometric equivalence is not claimed. No histogram equalization.
- **Navigation mapping:** `v = (1 - p_collision) * maximum_speed` and
  `yaw_rate = steering * yaw_gain_deg_s`, with positive yaw meaning left.
  The original `IOTJ_TEST_U` controller uses gain 120°/s and old-value filter
  weight 0.3. Our initial speed cap 0.35m/s is below its 0.5m/s.
- **Control interface:** body-forward velocity and yaw rate become map-frame
  trajectories. Camera pose is used only for this coordinate conversion.
  `yaw_reference_mode: integrated` integrates yaw rate with simulation
  timestamps and bounds reference error to 45°. This prevents repeated
  replanning from erasing the requested turn.
- **Tracking:** the PULP adapter sets `WS2_MIN_TRACKING_SPEED=0.01`; AirStack's
  original 0.2m/s stop threshold otherwise suppresses DroNet's slow commands.
  The bridge requests 10 frames per simulation second. Actual rates are logged.
- **Evaluation:** GT contacts and corridor gates are scoring information only.
  No hidden route follower, depth brake or external obstacle detector steers
  this model. The substantially larger Iris is not the original Crazyflie.

Episode controls: `maximum_speed`, `trajectory_horizon`, `yaw_gain_deg_s`
(30–180), `input_fov_deg` (40–85), `command_smoothing` (0–0.95), and
`yaw_reference_mode` (`integrated` or `trajectory`). These target parameters
remain fixed during a campaign; the attack agent does not tune the controller.
Select the architecture with YAML `pulp_variant: resblock|tiny`, or standalone
`--variant resblock|tiny`. Every inference record includes the selected variant.
Both adaptations passed strict loading and exact output comparison against
the official Python model on eight seeded tensors each. This is an integration
check, not a navigation performance comparison.

`WS2_INFERENCE_DIR` exports a JPEG and JSON with actual input, crop, network
outputs, commands and latency. `WS2_RUN_ID` associates them with an episode.

## Reproducibility and attribution

- [Official upstream](https://github.com/pulp-platform/pulp-dronet/tree/master/tiny-pulp-dronet-v3),
  revision `98f15cfe2e9c34c3a546358e22e45b3b09f11a79`.
- [Paper](https://arxiv.org/abs/2407.12675): *Distilling Tiny and Ultra-fast
  Deep Neural Networks for Autonomous Navigation on Nano-UAVs*.
- Checkpoint SHA256:
  `943ab4cbbc28cd711e874559dcb3ba0419a0ae1250dc8c94665f4a46d878ff6e`.
- `airstack_integration/model.py` preserves the official layer names and
  parameters. Unquantized NEMO residual addition becomes ordinary addition;
  strict state-dictionary loading is required. Upstream copyright and license
  remain in the source and `airstack_integration/LICENSE.upstream`.
- Runtime logs, bags, generated textures, previews and experiment results
  belong in AirStack's ignored runtime directory. Do not commit generated
  data or duplicate downloadable weights here.
