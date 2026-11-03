# Motion buffer timing

The active modes are `jakaminimimic_zmq` and `jakaminimimic_zmq_fixwrist`.
Their `clock` configuration is in [KpiConfig.yaml](../config/KpiConfig.yaml).

`warmup_s` starts at the first fully validated and accepted motion packet,
not at controller startup. With no motion input, `SourceTimeline::observe()`
has not set an origin and the buffer stays `Buffering`. The controller continues
its balance policy with the configured standing reference (or held reference
after a reset).

For example, if the controller starts at 0 seconds and the first motion packet
arrives at 10 seconds, a 0.5-second warmup cannot lock before 10.5 seconds.
Locking also requires `min_samples` valid packets and is triggered by packet
reception. Receiving one packet and then waiting is insufficient. At 50 Hz,
26 packets normally span 500 ms. Window coverage must also be complete.

The actual [Pico publisher](../scripts/pico_pub_copy.py) is silent before the
first X press. Controller-state messages on other ports do not start motion
warmup. The 500 ms warmup is not an additional steady playback delay: reference
history accumulates during warmup.

Let source and receiver epochs be `S0` and `R0`, and accepted packet times be
`Si` and `Ri`. The buffer stores immutable keys `si = Si - S0` and estimates
the phase distribution `ei = (Ri - R0) - si`. After warmup, a recent low quantile
provides the target phase `b_target`. At each query, phase changes are limited
to `max_slew * elapsed_receiver_time`.

The query source time is `tau = (steady_now - R0) - b`. For the default 20 ms
reference interval, five steps `[0,1,2,3,4]` and 40 ms tail tolerance:

```text
delay = 4 * 20 + 40 = 120 ms
targets = [tau-120, tau-100, tau-80, tau-60, tau-40] ms
```

Five reference samples span 80 ms. The 120 ms delay remains intentional.
At source time 500 ms, the five targets are 380/400/420/440/460 ms; warmup and
delay do not require a combined 620 ms wait.

`source` mode latches the sender's declared field and clock identity for a
generation. Undeclared legacy streams latch `publish_t_ns`. Explicit `publish`
and `arrival` modes force those axes. Missing selected fields never cause a
per-packet clock-domain fallback. New sessions, accepted-packet gaps and
detected discontinuities rebuffer with a new generation. Both active FSMs use
one atomic window per tick and rebuild alignment/history when a new generation
becomes ready.

Theoretical improvement over local receipt timestamps is reduced trajectory
time distortion from variable processing/network/parse delays. It does not
remove those delays or recover missing XR samples. If source and transport are
steady, the difference can be small. Source timestamp quality, sufficient tail
tolerance and policy/model consistency remain necessary.

At 30 Hz with 20 ms active processing after a low-latency pause, default 40 ms
tolerance may underrun. A deterministic buffer test found 36 underruns among
91 queries at 40 ms and none at 60 ms; the latter adds 20 ms of playback delay.
The default remains 40 ms, pending measurements of the actual sender.

## Validation

From the `sim` repository root:

```bash
source /opt/ros/humble/setup.bash
cmake -S demo -B demo/build -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
cmake --build demo/build --target KpiMujocoSimDemo SourceTimelineTest RealtimeMotionBufferTest -j4
ctest --test-dir demo/build --output-on-failure
```

The publisher timestamp tests execute actual Python methods with substituted
XR/GMR/clock dependencies; they do not instantiate real Pico or GMR hardware.
