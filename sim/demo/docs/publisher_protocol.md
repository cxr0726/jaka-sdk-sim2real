# Pico publisher timestamp protocol

The actual JAKA-capable publisher is
[`pico_pub_copy.py`](../scripts/pico_pub_copy.py). It maps `--robot jaka` to
GMR `jaka_mini`, retains canonical joint-name mapping and missing-joint defaults,
and uses the sender machine's customized `sim2real`/GMR robot configuration.
The file's inherited CLI defaults remain `robot=g1` and 30 Hz; select the
actual robot explicitly in the existing sender environment.

Timestamp placement:

1. Immediately after `get_raw_body_data()` returns, before coordinate conversion
   and GMR, record `pico_recv_time_ns` (wall) and `pico_recv_monotonic_ns`.
2. Live `motion_t_ns` is this pre-GMR monotonic sample time.
3. Once GMR/FK and payload construction finish, `_stamp_publication()` records
   `publish_t_ns` and `publish_monotonic_ns` immediately before JSON/send.
4. `sample_to_publish_ms` measures the post-read processing interval on one
   monotonic clock. It excludes JSON/network transport and earlier XR age.

Each stream declares `source_time_field=motion_t_ns`, a `source_clock_id` tied
to its per-process UUID, and `stream_id`. The C++ default `source` mode uses
this contract. Sender and receiver monotonic epochs still differ; the receiver
maps them rather than directly subtracting them.

Before the first X press, the actual sender emits no motion packets. After live
has started, X pause continues fresh static-command heartbeats in the same
clock/stream. In pause, `motion_source=paused_hold`, Pico read timestamps are
null, `smplx_t_ns=0`, and controller time is separate. This is not a new XR
capture. Root yaw-only pause handling, preserved joints/xyz, A-button collection
toggle and robot motion transformations are unchanged.

`pico_retarget_pub.py` remains an older G1 reference script with the same timing
contract. Do not confuse its robot configuration with the actual publisher.

On C++ startup, `source_field=motion_t_ns` confirms selection of the new sample
axis; `publish_t_ns` means an undeclared legacy stream or forced publish mode.
Source timestamp correctness does not ensure XR freshness: repeated SDK latest
poses still require independent source timestamps or update counters to diagnose.
