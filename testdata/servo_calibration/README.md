# Recorded LED bloom regression

The three JPEGs are original, unmodified frames 10, 100, and 600 from lb7's
failed calibration recording on 2026-11-11. `bloom_reference.json` contains
the front-camera intrinsics and the SFL reference used for that run.

The LED cores clip to white. The remaining saturated color pixels form
separate halos on either side of each short colored half. Fitting a separate
PCA axis to each half wrongly classifies these genuine observations as
off-axis objects. The red/cyan rod was rejected in every replayed batch.

These fixtures verify that the detector uses the complete directed rod to
check color-component alignment, excludes tiny compression artifacts, and
recovers both angles. Synthetic tests separately retain coverage for colored
background distractors, disconnected LEDs, perspective, lens distortion,
missing rods, and angles crossing 360 degrees.
