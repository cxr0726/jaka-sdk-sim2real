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
motions are intentionally versioned. No remote is configured by this setup.
