# MotionSpoofer v1.6.0

A single standalone TrollFools dylib for app-scoped motion and location testing. It combines the CoreMotion sensor controls and CoreLocation adjustment controls in one injected library. It does not build a jailbreak `.deb` and contains no HealthKit, Steps, or Distance hooks.

## CoreMotion controls

The in-app panel has per-app scalar and per-axis controls for accelerometer, gyroscope, magnetometer/compass, user acceleration, gravity, rotation rate, calibrated magnetometer, and attitude, plus pressure and relative altitude. Settings persist in the injected app's standard defaults container by bundle ID.

Reducing an accelerometer multiplier changes motion readings; it does not smooth GPS coordinates. Use the separate GPS controls for location fixes.

## GPS controls

GPS adjustments are off by default. Turn on **Enable GPS adjustments** in the injected app's panel, then adjust:

- **Jitter smoothing**: a time-aware velocity low-pass filter. `0` passes normal movement through; higher values reduce fix-to-fix wobble. Large implausible jumps are dropped.
- **GPS speed scale**: scales each filtered location step, so the returned coordinates move more slowly—not just the `speed` field. Below `1×`, the adjusted path can trail the real fixes. Returned speed and course are derived from that adjusted path.

The filter is app-local and resets/rebases velocity on long gaps or when GPS adjustments are toggled; stale or out-of-order historical fixes pass through unchanged. GPS filtering and CoreMotion multipliers are independent controls; test the combination in an app/device you own or are authorized to test.

## Build and download

The GitHub Actions workflow builds one standalone `MotionSpoofer-v<version>.dylib` and uploads it directly plus inside `MotionSpoofer-v<version>.zip`. After pushing the source, download the artifact from **Build MotionSpoofer standalone dylib**. The dylib uses Logos' internal generator and does not include HealthKit, Steps, Distance, or a substrate-hook dependency.

Main sensor sliders cover `-100` to `100`, per-axis sliders `-5` to `5`, and typed sensor values `-1000` to `1000`. The GPS smoothing slider is `0` to `0.95`; GPS speed scale is `0.1×` to `1.0×`. Tap **Save & Apply** to persist the current app's settings.
