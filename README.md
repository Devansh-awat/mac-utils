# mac-utils

Tools for the sensors Apple put inside Apple Silicon Macs and never told anyone about.

The IMU, the lid angle sensor, the ambient light sensor, the light sensor's
thermometer and the Force Touch trackpad's pressure sensors are all present and
readable — there is just no public API for most of them. This is a working set
of tools that read them anyway.

Everything here is read-only and makes no sound.

## what's in it

| file | what it is |
|---|---|
| `spu_tools.py` | the main app — one window, eight tabs, everything live |
| `angle_meter.py` | minimal: just the screen angle, two numbers and a diagram |
| `trackpad.py` | Force Touch pressure and haptics, runnable standalone as a calibration tool |
| `sound.py` | microphone analysis, runnable standalone as a terminal tuner |
| `macimu/` | the sensor-reading library (see [credits](#credits)) |

## requirements

- An Apple Silicon Mac (M-series). The sensor layout is M2/M3/M4/M5 era.
- macOS 15 or newer. Tested on macOS 27.2.
- Python 3.9+
- **Tkinter** — Homebrew's Python ships without it: `brew install python-tk@3.14`
- **Swift toolchain** for the sensor helper — comes with Xcode Command Line
  Tools (`xcode-select --install`). Without it the app falls back to a slower
  pure-Python path.
- **Microphone access**, only if you open the sound tab. macOS prompts the first
  time. Nothing is recorded.

## install and run

```bash
git clone https://github.com/Devansh-awat/mac-utils
cd mac-utils
python3 -m venv .venv && source .venv/bin/activate
pip install -e .

python3 spu_tools.py          # the full app
python3 angle_meter.py        # just the screen angle
python3 trackpad.py           # trackpad calibration / field inspector
```

**No `sudo` is needed on macOS 27.** Older releases gated the HID access behind
root; if you are on one, the app tells you rather than failing silently.

`spu_tools.py --rate 200` halves the CPU if you don't need the full 400 Hz.

## the tabs

| tab | what it shows |
|---|---|
| **angles** | the hinge angle (keyboard↔screen) and the screen-to-table angle, drawn as a side view |
| **level** | spirit level for the keyboard half, two bubble tubes plus a circular one |
| **quake** | seismograph — gravity removed, with STA/LTA event detection |
| **knock** | taps on the chassis, with interval timing and debouncing. `↑`/`↓` adjust the threshold 1 mg at a time |
| **heartbeat** | ballistocardiogram — rest your wrists on the machine and it finds your pulse |
| **3d** | a model of the machine that tilts, turns and opens with the real one |
| **ambient** | temperature and light inside the case |
| **trackpad** | Force Touch pressure in grams, every raw sensor field, a contact map, and haptic clicks |
| **sound** | microphone: level, 40-band spectrum, and the note you are playing |

Keys, where a tab has them: `↑`/`↓` (knock threshold), `[`/`]` (trackpad field),
`r` (reset peak), `x`/`y`/`s` (3d sensor axis mapping).

## how it works

Two parts, and it's worth knowing which is which.

**Reading the sensors.** macOS 27 exposes all the internal sensors through
`CoreHID` — Apple's public HID framework — as built-in devices with a dedicated
`.spu` transport:

```
accel  gyro  als  als-temp  las  cma  devmotion6  wakehint
```

`macimu/_spu_hid.swift` is a small Swift helper that subscribes to those and
streams samples to stdout; `macimu/_spu_hid.py` pumps them into shared memory.
The original pure-Python `ctypes`/IOKit path is kept as `macimu/_spu.py` and
still works — set `MACIMU_BACKEND=ctypes` to force it.

**The payloads are still undocumented.** CoreHID gives you the transport, not
the meaning. Every report layout here was reverse-engineered and is documented
in the source: the accelerometer is a 22-byte report with three little-endian
int32 at offset 6 in q16 fixed point; the ambient light sensor is 122 bytes;
the lid angle is 3 bytes; the temperature is a q16 value at offset 2.

**Two of the eight devices do not work.** `devmotion6` (Apple's fused motion)
and `wakehint` enumerate and open but never emit a single report. Tested
several ways — subscribing alongside active accel/gyro, seizing the device,
reading feature reports, writing the reporting properties. Property writes come
back `kIOReturnUnsupported`. They appear to be inert.

## the sound tab

Opens the microphone **only when you actually open that tab** — the rest of the
app is not listening. Capture and a 4096-point FFT run in Swift via vDSP
(`_sound.swift`); doing that in pure Python would not keep up. Results stream to
`sound.py`.

It is a tuner. Play a note, sing, or whistle and it names the pitch with the
deviation in cents. Detection needs the spectral peak to clear both a ratio test
against the rest of the spectrum *and* a floor that adapts to your room's own
noise — a fixed threshold either names a note for room hum or goes deaf to quiet
playing.

Audio is analysed in memory and discarded; nothing is recorded, saved or sent.
The build embeds an `NSMicrophoneUsageDescription` in the helper binary, without
which macOS kills the process on launch.

## haptics

The trackpad has no moving parts. The click you feel is a **linear resonant
actuator** — the Taptic Engine — shaking the glass. It is output, not input:
nothing is measured, hardware is driven.

```python
pad = Trackpad(); pad.start()
pad.click('weak')      # or 'strong', 'full'
```

The one place in this project that **writes** to hardware rather than reading it.

The catch, which cost real time: actuation silently returns `False` until you
call `MTActuatorRequestHostClickControl` first. That control must then be handed
back with `MTActuatorReclaimHostClickControl`, or the system's own click feedback
stays suppressed and the trackpad feels dead. `click()` takes control, actuates,
and hands it straight back — it never holds it.

Also: the actuator is reached through the *device* (`MTDeviceGetMTActuator`), not
by device id — `MTActuatorCreateFromDeviceID` returns null.

## the trackpad

There is no public API for the Force Touch pressure sensors. The only route is
the private `MultitouchSupport.framework`, and on macOS 27 its per-finger
struct does not match the layouts published by other projects. The offsets in
`trackpad.py` were solved here by calibration and are documented in the file:

```
offset 52  float   pressure in grams
offset 56  float   contact ellipse orientation (radians)
offset 60  float   ellipse major axis radius
offset 64  float   ellipse minor axis radius
offset 32  float   position x (normalized 0..1)
offset 36  float   position y
```

Offset 52 was identified as the only field reading exactly 0 with no finger,
tens of grams at a light rest, and ~980 g under a hard press — 1 kg being the
pad's rated maximum.

`run trackpad.py` to see every raw field live; if a future macOS shuffles the
struct, that is how you re-solve it.

**Weighing things is limited by physics.** The pad senses capacitance, so it
only reports while something conductive is in contact. An inert object sitting
alone on the glass produces no readings at all.

## credits and license

This project is **AGPL-3.0** (see `LICENSE`).

It bundles **`macimu`**, the sensor library by **olvvier**, from
[apple-silicon-accelerometer](https://github.com/olvvier/apple-silicon-accelerometer),
which is **MIT** licensed. MIT and AGPL are compatible, so the combination is
fine — but the MIT copyright notice has to be preserved, and it is: see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

The CoreHID backend (`macimu/_spu_hid.py`, `macimu/_spu_hid.swift`), the sensor
fusion, and all three apps are new work here.

Using this means complying with AGPL-3.0, including section 13: if you run a
modified version and let other people interact with it over a network, you have
to offer them the source.

## caveats

- Everything here relies on undocumented and private interfaces. A macOS update
  can break any of it.
- The 3D view's sensor axis mapping is a best guess — Apple does not document
  which way the accelerometer's axes point. If the model tilts the wrong way,
  the `x`/`y`/`s` keys fix it live.
- Yaw in the 3D view drifts slowly. There is no magnetometer, so nothing
  anchors rotation about the vertical axis. That's physics, not a bug.
- The heartbeat detector needs you to be still. It is a curiosity, not a
  medical device.
