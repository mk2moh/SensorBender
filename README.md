# SensorBender v1.7.0

A single standalone TrollFools dylib for app-scoped CoreMotion and CoreLocation testing. There is no jailbreak `.deb`, Settings bundle, HealthKit, Steps, or Distance code.

## Controls

The in-app panel has per-app scalar and per-axis controls for accelerometer, gyroscope, magnetometer/compass, user acceleration, gravity, rotation rate, calibrated magnetometer, and attitude, plus barometer and location controls. Settings persist in the injected app's defaults container.

GPS adjustments start off. Turn on **GPS adjustments** to use the time-aware jitter smoother and speed scale. Smoothing reduces fix-to-fix wobble; the speed scale acts on location steps, so it changes returned coordinates as well as derived speed/course. Below `1×`, the adjusted path can trail the source fixes. Historical or out-of-order fixes pass through unchanged; timestamps, accuracy, and altitude are not rewritten.

## Interface

The panel uses native iOS system colors and typography, with a flat accent color and grouped controls. It follows light/dark mode and uses no gradients. The menu opens when the injected app launches; tap the floating **SB** button to reopen it. Use **Save changes** to persist settings.

## Build

GitHub Actions builds one `SensorBender-v<version>.dylib` and uploads it directly and inside `SensorBender-v<version>.zip`. Push the source, then download the artifact from **Build SensorBender standalone dylib**. The dylib uses Logos' internal generator and has no linked tweak-hook runtime dependency.

Main sensor sliders cover `-100` to `100`, per-axis sliders `-5` to `5`, and typed sensor values `-1000` to `1000`. GPS smoothing is `0` to `0.95`; GPS speed scale is `0.1×` to `1.0×`.
