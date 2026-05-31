# Pebble Fluid Sim

A real-time fluid simulation watchapp for Pebble smartwatches. Tilt to slosh,
tap to splash, and hold the buttons to fire colliding dye jets — all running on
a fixed-point [Stam-style](https://www.dgp.toronto.edu/people/stam/reality/Research/pdf/GDC03.pdf)
Navier–Stokes solver tuned to fit the watch's RAM and CPU.

## Features

- **Fixed-point fluid solver** — diffusion, semi-Lagrangian advection, and Hodge
  projection (incompressibility), all in integer math. Grid is sized per platform
  at 4px cells.
- **Colliding jets** — two angled streams that meet in the middle and fan out
  ("splash") against the back wall.
- **Touch injection** *(Pebble Time 2)* — tap or drag to inject a swirling blob of
  dye at your finger. Each splash gets rotational + drift motion so it keeps
  flowing instead of sitting still.
- **Tilt gravity** — a constant turbulent force from the accelerometer; tilt the
  watch and the whole tank flows that way.
- **Color dithering** — blue→cyan→white density gradient with ordered (Bayer)
  dithering on color watches, 4-level grayscale on B&W.

## Controls

| Input | Action |
| --- | --- |
| Hold **Up** | Fire the top jet |
| Hold **Down** | Fire the bottom jet |
| Hold **Select** | Stir (rotational impulse) |
| **Tap / drag** the screen *(Time 2)* | Inject swirling dye at the touch point |
| **Tilt** the watch | Gravity pulls the fluid; jitter adds turbulence |

## Supported platforms

`aplite`, `basalt`, `chalk`, `diorite`, `emery`. Touch is compiled in only where
the hardware has it (gated behind `PBL_TOUCH` — currently the Pebble Time 2 /
`emery`). The accelerometer gravity force works on every platform.

## Building

This app uses the **TouchService** API, which requires the current Core Devices
PebbleOS SDK (4.9+). The legacy `rebble/pebble-sdk` Docker image is stuck at SDK
4.3 and has no touch support, so this repo ships its own build image.

Build the SDK image once:

```sh
docker build -t pebble-sdk-touch .
```

Then build the app:

```sh
docker run --rm -i -w /app -v "$PWD:/app" pebble-sdk-touch pebble build
# or on Windows:
build.cmd
```

If you have the Pebble SDK installed natively instead, `pebble build` works
directly (Python 3.10–3.13 + `uv tool install pebble-tool`, then
`pebble sdk install latest`).

## Installing

Enable the developer connection in the Pebble phone app to get its IP, then:

```sh
docker run --rm -i -w /app -v "$PWD:/app" pebble-sdk-touch pebble install --phone <phone-ip>
# or on Windows (caches the IP in .pebble-ip):
install.cmd <phone-ip>
```

## Tuning

Most of the feel lives in a handful of constants in `src/c/main.c`:

- `INJECT_DENSITY` — jet/touch dye brightness.
- `GRAVITY_SCALE` — tilt strength (**smaller = stronger** pull).
- The `fast_rand(...)` calls in `apply_gravity` / `apply_noise` — turbulence amount.
- The `v / 32` velocity bleed in `fluid_step` — damping (bigger divisor = looser,
  longer-lived sloshing).
- Swirl/drift magnitudes in `add_fluid_at` — how much touched dye churns and travels.

## Layout

- `src/c/main.c` — the entire simulation, rendering, and input handling.
- `Dockerfile` — touch-capable build environment (`pebble-sdk-touch`).
- `build.cmd` / `install.cmd` / `screenshot.cmd` — Windows helper scripts.
