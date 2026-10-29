# SensorBender v1.9.0

A single standalone TrollFools dylib for app-scoped CoreMotion and CoreLocation testing. There is no jailbreak `.deb`, Settings bundle, HealthKit, Steps, or Distance code.

**v1.9.0** adds optional, best-effort app-usage state masking to the in-dylib panel. The v1.8.1 ARC-safe runtime-class enumeration fix remains in the source, but this update has not been compiled or tested on a device in this workspace.

## Per-app targeting

The injected process is detected automatically from its own bundle identifier (`NSBundle.mainBundle.bundleIdentifier`). The panel displays that identifier, and settings are stored under that app's existing per-app preferences. Inject SensorBender into each app you want to configure. The dylib does not enumerate every unrelated running app; that is not a normal public iOS capability for an injected app.

## Usage privacy controls

The **Usage privacy · best effort** section is separate from all sensor and GPS settings. Its master switch is off by default. Optional signals are:

- **UIApplication reports Active** — makes this process's `UIApplication.applicationState` getter report active.
- **Scenes report Foreground Active** — makes `UIScene.activationState` report foreground-active.
- **Protected data reports available** — makes the public protected-data availability query report `YES`; it does not unlock protected files, Keychain items, or the device.
- **Screen brightness reports 100%** — changes the brightness getter result only; it does not turn on the physical display or change the lock state.
- **Suppress background notifications** — suppresses selected public `NSNotificationCenter` background, scene-deactivation, protected-data, and brightness-change broadcasts.

The app/scene state hooks are independent of the existing CoreMotion, barometer, and GPS hooks. Turning usage masking on does not disable or alter accelerometer, gyroscope, magnetometer/compass, attitude, or location spoofing. The separate **Enable for this app** switch still gates the app's spoofing controls.

These are property/notification masks, not a way to keep an iOS process alive. iOS may suspend the app, lifecycle delegate callbacks can still reveal the transition, and private APIs or other signals may expose that it is no longer visible. Notification suppression can also affect app code that relies on those broadcasts; leave that option off unless you specifically want to test it. A successful build/runtime result is not claimed here.

## Sensor and location controls

The in-app panel has per-app scalar and per-axis controls for accelerometer, gyroscope, magnetometer/compass, user acceleration, gravity, rotation rate, calibrated magnetometer, and attitude, plus barometer and location controls. Usage-state options do not change these settings.

GPS adjustments start off. Enable **GPS adjustments** for the app, then use **Jitter smoothing** and **GPS speed scale**. GPS values are not clamped to the slider endpoints: tap either displayed number to enter any finite number, including `-100`; normal values expand the slider range as needed. For values beyond UISlider's float range, the exact typed value remains displayed and saved while the slider thumb stays at an endpoint. The slider's original range is only a quick-adjustment range.

Smoothing `0` passes movement through. Positive values increase filter memory; negative values amplify incoming velocity changes and can make jitter much worse. Speed scale multiplies the adjusted path: `0` pauses it, negative values reverse it, and values above `1` speed it up. These GPS controls are independent from acceleration scaling. Extreme values can produce very large or invalid movement, so start with small changes.

The GPS section displays live Core Location getter reads, adjusted-fix count, and raw-versus-output coordinates. In Google Maps, enable GPS adjustments, save, move around, then reopen the panel to check whether the read/fix counters and output coordinates change. If the read counter remains at zero while the map's blue dot updates, the app is not reaching a hooked `CLLocation.coordinate` getter; this diagnostic does not establish why. The hook also scans Core Location subclasses for coordinate overrides. No runtime cause has been confirmed without a device test.

Historical or out-of-order fixes pass through unchanged; timestamps, accuracy, and altitude are not rewritten.

## Interface

The panel uses native iOS system colors and typography, with a flat accent color and grouped controls. It follows light/dark mode and uses no gradients. The menu opens when the injected app launches; tap the floating **SB** button to reopen it. Use **Save changes** to persist settings.

## Build

GitHub Actions builds one `SensorBender-v<version>.dylib` and uploads it directly and inside `SensorBender-v<version>.zip`. Push the source, then download the artifact from **Build SensorBender standalone dylib**. This workspace cannot run the GitHub workflow or compile against an iOS SDK, so the v1.8.1 ARC fix and the new v1.9.0 hooks still require CI/build and device verification. The dylib uses Logos' internal generator and has no linked tweak-hook runtime dependency.

Main sensor sliders cover `-100` to `100`, per-axis sliders `-5` to `5`, and typed sensor values `-1000` to `1000`. The GPS sliders initially cover smoothing `0` to `0.95` and speed scale `0.1×` to `1.0×`; tapping the GPS numbers expands them beyond those ranges.
