# SensorBender v1.8.0

A single standalone TrollFools dylib for app-scoped CoreMotion and CoreLocation testing. There is no jailbreak `.deb`, Settings bundle, HealthKit, Steps, or Distance code.

## Controls

The in-app panel has per-app scalar and per-axis controls for accelerometer, gyroscope, magnetometer/compass, user acceleration, gravity, rotation rate, calibrated magnetometer, and attitude, plus barometer and location controls. Settings persist in the injected app's defaults container.

GPS adjustments start off. Enable **GPS adjustments** for the app, then use **Jitter smoothing** and **GPS speed scale**. GPS values are not clamped to the slider endpoints: tap either displayed number to enter any finite number, including `-100`; normal values expand the slider range as needed. For values beyond UISlider's float range, the exact typed value remains displayed and saved while the slider thumb stays at an endpoint. The slider's original range is only a quick-adjustment range.

Smoothing `0` passes movement through. Positive values increase filter memory; negative values amplify incoming velocity changes and can make jitter much worse. Speed scale multiplies the adjusted path: `0` pauses it, negative values reverse it, and values above `1` speed it up. These GPS controls are independent from acceleration scaling. Extreme values can produce very large or invalid movement, so start with small changes.

The GPS section now displays live Core Location getter reads, adjusted-fix count, and raw-versus-output coordinates. In Google Maps, enable GPS adjustments, save, move around, then reopen the panel to check that the read/fix counters and output coordinates change. If the read counter remains at zero while the map's blue dot updates, the app is not reaching a hooked `CLLocation.coordinate` getter; the panel exposes that for diagnosis. The hook also scans Core Location subclasses for coordinate overrides.

Historical or out-of-order fixes pass through unchanged; timestamps, accuracy, and altitude are not rewritten.

## Interface

The panel uses native iOS system colors and typography, with a flat accent color and grouped controls. It follows light/dark mode and uses no gradients. The menu opens when the injected app launches; tap the floating **SB** button to reopen it. Use **Save changes** to persist settings.

## Build

GitHub Actions builds one `SensorBender-v<version>.dylib` and uploads it directly and inside `SensorBender-v<version>.zip`. Push the source, then download the artifact from **Build SensorBender standalone dylib**. The dylib uses Logos' internal generator and has no linked tweak-hook runtime dependency.

Main sensor sliders cover `-100` to `100`, per-axis sliders `-5` to `5`, and typed sensor values `-1000` to `1000`. The GPS sliders initially cover smoothing `0` to `0.95` and speed scale `0.1×` to `1.0×`; tapping the GPS numbers expands them beyond those ranges.
