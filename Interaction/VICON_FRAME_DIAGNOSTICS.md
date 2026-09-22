# Optional Vicon frame diagnostics

`--vicon-source-metadata` is off by default. It requires `--log`, Vicon
capture, and the patched `Hamedamz/libmotioncapture` build exposing
`MotionCapture.frameMetadata()`. Existing pose, velocity-filter and FC-send
timestamps remain based on Pi receipt; this flag only adds fields to the
`mocap_timing` log group. Do not enable it during flight until its logging cost
has been measured in a prop-off run.

Each diagnostic row has the existing Pi monotonic `wait_return_monotonic_s`
and local-loop interval, plus `source_metadata`, `source_frame_delta`, and
`source_metadata_read_duration_s`. Source metadata may contain:

- `frame_number` and `hardware_frame_number`, whose gaps can reveal frames
  skipped before the Pi consumer;
- `timecode`, only when Vicon supplies one; this is a **frame timecode**, not
  a packet-send timestamp or a Pi-clock timestamp;
- `latency_total_s` and `latency_samples_s`, Vicon's reported pipeline
  latencies, not an end-to-end network delay.

Unavailable source fields are omitted. If reading optional metadata fails,
`source_metadata_error` is logged and position forwarding continues. A Pi
inter-arrival gap without a source-frame gap still does not, by itself,
distinguish network buffering from Pi scheduling. Comparing absolute source
and Pi times requires synchronized clocks.
