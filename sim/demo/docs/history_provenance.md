# Recovered baseline provenance

Repository initialized on 2026-10-31. The baseline is the **verified recoverable
pre-source-timeline implementation**, not a claim that every file is the
pristine version from before the full conversation.

## Exact code recovery

Eight files are restored byte-for-byte from the pre-timeline snapshots, with
their saved SHA-256 fingerprints checked:

- `demo/CMakeLists.txt`
- `demo/include/RealtimeMotionBuffer.hpp`
- `demo/include/RLController.hpp`
- `demo/src/RealtimeMotionBuffer.cpp`
- `demo/src/FSMMimicJakaMiniZmq.cpp`
- `demo/src/FSMMimicJakaMiniZmqFixwrist.cpp`
- `demo/src/GlobalVars.cpp`
- `demo/config/KpiConfig.yaml`

The baseline's `demo/scripts/pico_retarget_pub.py` is the exact pre-protocol G1
reference sender. The baseline's actual `demo/scripts/pico_pub_copy.py` is the
exact pre-adaptation file supplied on 2026-10-31. The latter was independently
matched to the desktop application's pre-turn capture.

Two files were reconstructed by reversing only known edits and independently
verified against fingerprints in the earliest saved runtime manifest:

| File | Bytes | SHA-256 |
|---|---:|---|
| `demo/scripts/pub_motion_npz.py` | 6057 | `71362967b2fae91fe8c9b8e975c130e8164ca217c58efac8efd7287575d4bcfa` |
| `demo/scripts/run_npz_demo.sh` | 5726 | `dae60d03c09eeab26a92c589f0dd445f45bdf43837a93809d2e87e1c88d51d04` |

The first file precedes timestamp/seq/UUID additions. The launcher precedes the
headless/realtime option, but **already includes the earlier controller-first
startup-order fix**.

Exact baseline fingerprints and recovery descriptions are recorded in
[`baseline_sources.json`](baseline_sources.json). The Git baseline itself
contains the recovered source blobs, so those versions no longer depend on
external backup directories.

## What could not be recovered

Complete versions preceding earlier replay-loader/anchor, launcher-startup and
record-documentation fixes were not found. For `pub_motion_record.py`,
`run_record_demo.sh`, `test_record_anchor.py` and unchanged auxiliary files, the
earliest available versions are retained. The baseline README is an editorial
recovery that removes the later timing/headless documentation; it has no
independent original full-file fingerprint. Repository metadata and this
provenance document are new import bookkeeping.

The new SourceTimeline header, timing tests and current timing documentation
are introduced by the modified-code commit. SDK, model and policy files were
not changed by the timing work.

## Repository scope

`sim/.git` is independent of the parent workspace Git repository; the parent
branch/history is not changed. No remote is created and nothing is pushed.
Virtual environments, build directories, logs and personal recorded episodes
remain on disk but are ignored. SDK libraries, ONNX files, XML/meshes and the
standalone NPZ examples are retained for reproducibility.

After the two initial commits, compare code with:

```bash
git log --oneline
git diff HEAD~1 HEAD -- demo/src demo/include demo/scripts
git show HEAD~1:demo/src/RealtimeMotionBuffer.cpp
```
