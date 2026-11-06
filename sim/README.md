# JAKA simulation and teleoperation

This repository contains the C++ controller demo and its bundled MuJoCo SDK,
robot assets, ONNX policies and sample motion files.

- [Demo setup and commands](demo/README.md)
- [Motion buffer timing](demo/docs/motion_buffer_timing.md)
- [Actual Pico publisher protocol](demo/docs/publisher_protocol.md)
- [Recovered baseline provenance](demo/docs/history_provenance.md)

## Run

```bash
cd demo
./scripts/run_demo.sh --realtime --duration 0 --mode jakaminimimic_zmq
```

This is the simulation receiver. The config retains the remote Pico endpoint.
For a local NPZ replay, with controller-first startup and realtime pacing:

```bash
KPI_NPZ_VIEWER=0 ./scripts/run_npz_demo.sh scripts/newsmallstep.npz --duration 18
```

## SuperMimic: controller and NPZ publisher in separate terminals

Terminal 1 starts the C++ controller in the actual mode
`jakaminimimic_superzmq`:

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
./scripts/run_demo.sh --viewer --realtime --duration 0 \
  --mode jakaminimimic_superzmq
```

Wait for both `[FSMJakaMiniSuperZmq] action_dim=27` and
`[StatePublisher] bound to`, then start terminal 2:

```bash
cd /home/cxr0726/workspace/JAKA_comparison/sim/demo
.venv-pub-motion/bin/python scripts/pub_motion_super_npz.py scripts/motion_pico.npz \
  --layout isaac --bind tcp://127.0.0.1:28701 --hz 50 \
  --height-offset 0 --startup-sleep 0.25
```

The checked-in SuperMimic receiver connects to `tcp://127.0.0.1:28701`.
This sample has 577 frames at 50 Hz (11.52 seconds from first to last frame)
in unnamed legacy Isaac order; the dedicated publisher converts it to JAKA
wire order with the base reference in slot 0. Named Khan training files can
use `--layout auto`. Publication is single-pass, then holds the final pose at
50 Hz until Ctrl+C. Stop both terminals separately; stopping the publisher
alone leaves the controller holding the last reference.

The current SuperMimic configuration selects the v6 80k ONNX and disables
target clipping. The default XML is `sdk/assets/khan_mini/Khan_mini_contact.xml`;
append `--xml /absolute/path/model.xml` to terminal 1 to use another model.
The one-command `run_super_npz_demo.sh` uses motion port 28721 by default;
these manual commands use the receiver's configured port 28701.
See [the demo README](demo/README.md#supermimic两个终端分别启动控制器和-npz-发布器)
for configuration and lifecycle details.

The actual sender is `demo/scripts/pico_pub_copy.py`. Use its existing sender
environment and explicitly select `--robot jaka`; the custom JAKA sim2real/GMR
configuration is an external dependency, not invented or vendored here.
Its original defaults and robot motion logic are retained.

## History

The initial commit imports a verified, recovered pre-source-timeline baseline.
It is not a pristine snapshot predating the whole investigation: earlier replay
fixes had already been made before complete source backups existed. The second
commit contains the source timeline, timing protocols, tests and current docs.
The provenance document identifies exact recovered files and this limitation.

Generated builds, local Python environments, logs and recorded episode sessions
are ignored. Required SDK libraries, policies, meshes and standalone sample
motions are intentionally versioned.
