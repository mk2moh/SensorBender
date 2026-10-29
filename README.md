# SensorBender v1.10.0

A single standalone TrollFools dylib for app-scoped CoreMotion and CoreLocation testing. There is no jailbreak `.deb`, Settings bundle, HealthKit, Steps, or Distance code.

**v1.10.0** adds metre/time-based GPS stability controls, optional reported-location-field transforms, and best-effort floating-panel collision handling for apps that have more than one injected overlay. The v1.9.0 source features remain. This update has not been compiled or device-tested in this workspace.

## Per-app targeting

The injected process is detected automatically from its own bundle identifier (`NSBundle.mainBundle.bundleIdentifier`). The panel displays that identifier, and settings are stored under that app's existing per-app preferences. Inject SensorBender into each app you want to configure. The dylib does not enumerate every unrelated running app; that is not a normal public iOS capability for an injected app.

## GPS controls

GPS adjustments are off by default. Enable **GPS adjustments** for the target app and save before testing. The path controls are independent of accelerometer scaling.

- **Smoothing response:** `0` follows incoming movement without smoothing. Positive values add velocity-filter memory and lag; the filter's time constant is approximately `0.05 + 4 × value` seconds. Negative values amplify changes. This is not a GPS callback-rate control.
- **Moving distance scale:** multiplies each filtered path step. `1×` keeps the full step, `0.5×` halves all moving distance, `0` holds the output, values above `1` amplify movement, and negative values reverse the path.
- **Stationary radius (m):** the raw coordinate must leave this net-displacement radius and exceed the moving-speed threshold for the confirmation period before the output switches to moving. Default: `10 m`.
- **Stationary drift cap (m/min):** caps movement caused by residual stationary wander. `0` means no output drift while classified stationary.
- **Movement confirmation (s)** and **moving threshold (m/s):** require candidate movement to persist and exceed a speed threshold. The defaults are `3 s` and `0.5 m/s`.
- **Max output step (m):** caps an applied coordinate change. `0` disables the cap; default: `10 m`. Unapplied movement is carried forward.
- **Min time between changes (s):** holds intermediate coordinate results and applies pending movement on the next fix at or after the selected interval. `0` disables throttling. This does not reduce Core Location callback frequency.
- **Accuracy-aware filtering:** enabled by default; it holds fixes whose reported horizontal accuracy is worse than the cutoff and, when smoothing is non-negative, smooths accepted lower-confidence fixes more strongly. Default cutoff: `50 m`.

Advanced **Reported GPS values and offset** controls transform altitude, horizontal/vertical accuracy, reported speed, and course, or add a fixed east/north coordinate offset in metres. These affect fields returned to the injected app; they do not change the device's actual GPS hardware or every possible location API.

Tap a displayed GPS value to enter any finite number, including numbers outside the slider range. The numeric value is retained while the slider stays at an endpoint when necessary. Threshold-like values use their absolute magnitude; offsets can be negative; negative movement scale reverses direction; negative smoothing amplifies changes. Extreme values can cause nonsensical location results, so use small test values.

The live diagnostic shows hooked coordinate reads, adjusted fixes, raw/output coordinates, stationary/moving classification, raw distance from the stationary anchor, and rejected poor-accuracy fixes. If Google Maps moves while the hooked-read counter stays at zero, these adjustments are not reaching the getter path being monitored; that diagnostic does not establish why.

## Usage privacy controls

The **Usage privacy · best effort** section is separate from all sensor and GPS settings. Its master switch is off by default. Optional signals are:

- **UIApplication reports Active** — makes this process's `UIApplication.applicationState` getter report active.
- **Scenes report Foreground Active** — makes `UIScene.activationState` report foreground-active.
- **Protected data reports available** — makes the public protected-data availability query report `YES`; it does not unlock protected files, Keychain items, or the device.
- **Screen brightness reports 100%** — changes the brightness getter result only; it does not turn on the physical display or change the lock state.
- **Suppress background notifications** — suppresses selected public `NSNotificationCenter` background, scene-deactivation, protected-data, and brightness-change broadcasts.

These masks cannot stop iOS from suspending the process, suppress every lifecycle delegate callback, or prevent private APIs and other signals from revealing that the app is no longer visible. Notification suppression can affect app code that relies on those broadcasts; leave it off unless you specifically want to test it.

## Multiple injected overlays

The floating SensorBender button now searches for an available spot among visible small windows, avoids making the floating button key by default, raises its panel only while open, restores the previous key window when closed, and avoids auto-opening over another visible full-screen overlay. This is best-effort coordination: other injected dylibs may use different window/touch handling, so real-device testing is still needed.

## Sensors and interface

The panel retains per-app scalar and per-axis controls for accelerometer, gyroscope, magnetometer/compass, user acceleration, gravity, rotation rate, calibrated magnetometer, and attitude, plus barometer controls. The panel uses native iOS system colors and typography, a flat accent color, and grouped controls; it follows light/dark mode and uses no gradients.

## Build

GitHub Actions builds one `SensorBender-v<version>.dylib` and uploads it directly and inside `SensorBender-v<version>.zip`. Push the source, then download the artifact from **Build SensorBender standalone dylib**. This workspace cannot run the GitHub workflow or compile against an iOS SDK, so the v1.10.0 GPS state machine, overlay changes, and the carried-forward v1.8.1 ARC fix all still require CI/build and device verification. The dylib uses Logos' internal generator and has no linked tweak-hook runtime dependency.

Main sensor sliders cover `-100` to `100`, per-axis sliders `-5` to `5`, and typed sensor values `-1000` to `1000`. Slider ranges are quick-adjustment ranges; GPS controls also support finite typed values outside them.
