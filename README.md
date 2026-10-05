# rocketlab

![rocketlab banner — animated ASCII art over a starfield](assets/demo.gif)

A space mission simulator: launch rockets and probes, assemble your own, and
write flight software for them. Two clients watch the same simulation — a
terminal dashboard and a graphical map view.

## Building

Requires a C++20 compiler, CMake ≥ 3.24 and Ninja. Catch2 is fetched
automatically at configure time.

```sh
cmake --preset dev
cmake --build --preset dev
ctest --test-dir build/dev --output-on-failure
```

`build/dev/tests/rocketlab_tests` runs the suite directly, with Catch2's own
tag filtering (`[orbital]`, `[time]`).

## Layout

```
libs/core/      simulation core: no UI, no third-party dependencies
libs/scenario/  scenario JSON, the only place nlohmann_json is allowed
libs/proto/     snapshot and command types shared by the daemon and clients
libs/simhost/   the SimSource implementations: an in-process world, later a socket
libs/flight/    the flight computer: Lua behind a Lua-free interface
libs/render/    camera and scene building, renderer-agnostic
libs/hud/       readout, list model and the assembly editor, shared by every client
apps/simd/      headless daemon: owns the world, ticks the physics (not built yet)
apps/cli/       headless client: bodies, parts, scenario, run, vessel, map, fly
apps/tui/       FTXUI client: telemetry, entity list, map view, assembly editor
apps/gui/       graphical client: same camera, rasterised to a window, orbitable
scenarios/      vessel and mission definitions
scripts/        Lua flight programs
tests/          unit tests, numeric
```

## Architecture

Four decisions shape everything below. They were made deliberately and are
worth re-reading before changing anything structural.

### The daemon is headless, and the clients are separate processes

`simd` owns the world and never links a UI library. The TUI and the GUI are
independent clients of it. This is not just tidiness:

- FTXUI takes over stdout and the terminal. Any `std::cout` from any thread —
  including a logging library's — corrupts its rendering. Keeping it in its
  own process makes that class of bug impossible rather than merely unlikely.
- A crash in the graphical client, or in a user's flight software, does not
  take the simulation with it.
- The core can be regression-tested headless, and the TUI can be detached into
  a tmux pane while the map view keeps running.

State flows out over double-buffered shared memory: the daemon publishes a POD
snapshot at frame rate, clients read it without locking. Commands flow back
over a Unix domain socket, which is low-rate and therefore allowed to be
simple.

### Clients never do physics

The snapshot carries instantaneous state only. Anything predictive — the
osculating orbit, sphere-of-influence transitions, closest approach, a
manoeuvre plan — is answered by an explicit query to the daemon.

The tempting alternative is to have the renderer propagate the orbit itself
from the elements in the snapshot. That duplicates the patched-conic logic
into every client and guarantees that the two drift apart. One source of truth
means the TUI and the GUI cannot disagree about where the rocket will be.

### Simulation time is a first-class value, and the wall clock is quarantined

All simulation time is TDB seconds since J2000, carried in `Instant`. Nothing
in the core reads a real clock; `FrameTimer` is the single exception and exists
only so a client can convert a real frame duration into a simulation span via
`SimClock::frame_span`.

Two consequences worth preserving:

- **Propagation is analytic.** `propagate()` moves a state along its osculating
  conic exactly, for any step size, so a warp factor of 100000 costs the same as
  1. Anything under thrust, drag or third-body perturbation cannot use it and
  needs a numerical integrator instead.
- **Precision is spent where it is needed.** Positions are `double` metres in a
  single root frame; the render origin is shifted to the camera target before
  anything reaches a vertex buffer. Time is never round-tripped through a
  single Julian date, because a double JD resolves to only ~50 µs near the
  present day.

### The flat map is the 3D camera at zero pitch

An orbit reads as an ellipse, there is no depth-buffer precision problem across
nine orders of magnitude, and line widths and minimum marker sizes are trivial
in screen space — so the map is orthographic, and it is the *only* projection
rather than the first of two. `Camera3D` at `pitch = 0` reduces to `Camera2D`
exactly, and a test builds a frame through each and compares them primitive for
primitive. Tilting therefore moves the camera without moving the world, and the
flat and tilted views cannot disagree about where anything is, because there is
one piece of code that decides.

A renderer interface keeps the camera and trajectory code independent of the
rasteriser, so the same map is drawn into an FTXUI canvas in the terminal
client and into an ImDrawList in the windowed one — the same primitives, four
backends, and no client ever decides where anything goes.

The camera's centre tracks a *selectable* target and sits at a distance from
it. Crucially, that selection is client state, not world state: the daemon does
not know or care what anyone is looking at.

## Roadmap

- **M0 core** — time, two-body Kepler propagation, root frame, headless CLI
- **M1 TUI** — telemetry, entity list, target selection, canvas map view
- **M2** — scenarios, parts, staging, delta-v accounting
- **M3** — Lua flight computer: sandboxed, deterministic, with an instruction budget
- **M4 GUI** — window, same camera, smooth target transitions
- **M5** — patched conics, atmospheres, planets and moons
- **M6** — visual assembly editor, 3D view, docking

## Status

**M0 through M6 are done.** 131 test cases and 4140 assertions, green, with the
project's own warning set promoted to errors on the development preset.

- **M0** — the time base, two-body Kepler propagation for every conic family,
  the world and its root frame, the tick loop and the headless CLI.
- **M1** — the FTXUI client: telemetry, entity list, target selection, a 2D map
  with a tracking camera, cursor-anchored zoom and click-to-select.
- **M2** — scenarios, the part catalogue, stacks, staging and delta-v
  accounting, checked against an actual burn.
- **M3** — a Lua flight computer that is sandboxed (the dangerous libraries are
  not even in the binary), deterministic (no clock, no filesystem, no RNG) and
  bounded (an instruction budget and a memory cap). `scripts/hohmann.lua` flies
  the tug from a 400 km parking orbit to a circular 1200 km one.
- **M4** — the graphical client: a GLFW window drawing the same map from the
  same camera through Dear ImGui, with the camera gliding between targets
  instead of cutting.
- **M5** — the whole solar system and the air around part of it. Eighteen
  bodies: the Sun, the planets, and nine moons whose elements are published in
  their primary's equatorial plane and rotated into the ecliptic once, when the
  catalogue is built. Sphere-of-influence transitions are *solved for* rather
  than noticed — a step that straddles a boundary is cut at it and flown as two
  arcs — so `max_step` is an event granularity and not an accuracy setting.
  Earth, Venus, Mars and Titan have exponential atmospheres, interpolated
  log-linearly between tabulated rows, and drag acts on the velocity relative to
  the co-rotating air, which is enough to bring a 400 km orbit down and enough
  to make a retrograde one come down faster.

- **M6** — the vessels you fly are now yours to build. Both clients edit a
  stack with the same state machine in `libs/hud`: a part picker, the stack, its
  weights, and a stage table that shouts about the one mistake worth catching (an
  engine whose own stage carries no propellant). The editor edits the scenario
  *document* — a stack is what a vessel is built from, and the snapshot rightly
  carries only what it is doing — so building writes the document and restarts
  the mission, while saving writes the file and does neither. Two vessels can
  dock: the predicate is one set of numbers in `libs/core` that the host applies
  to two entities and the client applies to two published states, measured on the
  gap between the surfaces rather than the distance between the centres. And the
  map is now a 3D camera that happens to be looking straight down, so `v` tilts
  it, the right mouse button orbits, and the terminal's flat map is the same
  camera at zero pitch rather than a second renderer.

`apps/simd`, the daemon that would let the two clients be separate processes,
is not built yet — both currently drive an in-process `simhost::LocalSimSource`
behind the `proto::SimSource` interface, which is the seam the daemon will slot
into.
