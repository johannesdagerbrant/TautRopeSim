# TautRope — agent guide

This plugin is a reverse-engineering attempt at the taut rope system from
Uncharted 4 ([GDC talk](https://gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4)).
It has known defects, and this document describes how to work on them.

The whole point of the setup below is that **you can iterate on the simulation
without Unreal**. A human captures a recording of a bug happening in the editor;
you replay it headlessly, in milliseconds, against code you just changed.

---

## Layout

| Path | What it is |
|---|---|
| `Source/TautRopeCore/` | The simulation. **No UE headers, ever.** Compiled by both UE and CMake. |
| `Source/TautRope/` | The Unreal glue: actors, shape extraction, CVars, debug rendering, the recorder. |
| `Standalone/` | CMake build for core, the replay CLI and the unit tests. No engine required. |

The split is what makes the loop fast. If a UE header reaches `TautRopeCore`,
the standalone build breaks — that is the intended alarm, not an inconvenience
to work around.

---

## The loop

**1. Human describes a bug and captures a recording.**
In the editor, with the level running: `tautrope.record 1`, reproduce the bug,
`tautrope.record 0`. Files land in `Saved/TautRopeRecordings/` as
`<ActorName>_<timestamp>.tautrope`. Stopping PIE also flushes.

**2. Verify the replay before trusting anything.**

```
cd Plugins/TautRope/Standalone
./b.bat
./build/tautrope-replay <recording> --verify
```

Expect `verify: PASS`. **Do not skip this and do not proceed past a FAIL.**
A failure here means your replay is not reproducing the editor, so every number
you would go on to analyse is meaningless. Diagnose the divergence first.

**3. Find the numbers that match the human's words.**
`--info` summarises a recording; the per-frame data is plain text, so grep and
script over it directly. The format is documented below.

**4. Establish quantifiable benchmarks, then form hypotheses.**
Turn "the rope slows down" into a number you can watch move — rope length drift
per frame, point count at a vertex, penetration depth. Write the benchmark down
before changing code, otherwise you cannot tell improvement from noise.

**5. Closed loop. No human.** For each hypothesis:

```
./b.bat                      # ~0.2 s for one core file
./build/tautrope-tests       # ~60 ms, must stay green
./build/tautrope-replay <recording> --verify
```

Then compare your benchmark against the target. Unsatisfactory means the next
hypothesis. Note that once you change core, `--verify` **will** fail against a
recording captured with the old code — that is expected, and it is why step 4's
benchmarks matter more than bit-identity at this stage.

**6. Report and hand back.** Say what you tried, what happened, what worked and
why it solved the problem. The human verifies in the editor. If the bug still
reproduces, they return to step 1 with a new recording and new observations.

**7. Harden, then delete.** Add a unit test that pinpoints the exact defect,
and delete the recordings used for debugging. Recordings are large and slow to
accumulate; the tests are the permanent record.

---

## Commands

```
tautrope-replay <recording>              replay, report what it produced
tautrope-replay <recording> --verify     compare against the captured output
tautrope-replay <recording> -o <path>    write the replay result out
tautrope-replay --info <recording>       summarise without replaying
tautrope-replay <recording> --analyse <what>
tautrope-tests                           run all tests
tautrope-tests <substring>               run tests whose name matches
```

Exit codes: `0` ok, `1` error, `2` usage, `3` verify failed.

### Measuring a recording

`--analyse` takes `penetration`, `edges`, `vertex`, `conditioning` or `all`. It
reads the captured data and does not replay, so it answers step 3 in
milliseconds.

**Do not write scripts to do this.** The measurements below exist because they
were first written as throwaway Python, which took minutes per pass against a
loop that is about two seconds end to end. If you need a measurement that is not
here, add it to `TautRopeCore/Analysis.h` so the next run is fast and the unit
tests can assert on it.

**Never re-implement production logic in an analysis.** A measurement must drive
the real functions in `Collision.h` / `Rope.h`. If a measurement cannot be
expressed by calling production code, that is a signal the production code needs
to expose something -- change it, do not mirror it.

This is not a style preference. A hand-rolled copy of the sweep loop in
`Analysis.cpp` silently omitted two things the real `SweepSegmentTriangleAgainstShape`
does -- it skips the edge each rope point is already attached to, and the B
triangle is only swept when the A triangle misses. The copy reported 24 lost
in-face edges with a ratio gap wider than 0.001; production reports 15, all
within 0.001, which is the opposite conclusion. A mirrored implementation
diverges from the thing you are trying to measure, and it fails silently: the
numbers still look plausible.

| what | answers |
|---|---|
| `penetration` | does the rope pass through a shape, from which frame, how deep, and has it recovered by the end |
| `edges` | how many shape edges lie flat across a face, and how often rope points rest on one |
| `vertex` | for adjacent points on edges sharing a vertex, how fast they are closing on it |
| `conditioning` | how many sweep tests are degenerate, and how many of those produce a hit |

Two things worth knowing about the numbers:

`penetration` measures the rope **line**, not just its points. Both endpoints can
sit on the surface while the segment between them cuts through the solid, which
is the usual form the defect takes. It also shrinks each shape by a small surface
tolerance first: rope points rest on the surface by design, so without that the
answer for a segment lying flat against a face flips between zero and the full
width of the face on rounding alone. A first attempt at this measurement, written
in Python without the tolerance, reported 27x more affected frames than there
are.

`edges` distinguishes a real silhouette edge from a triangulation diagonal by how
many vertices lie on the plane through it -- a face contains its own corners, an
edge only its two endpoints. Testing whether the plane merely supports the hull
is not enough: the bevel plane through a silhouette edge supports it too.

`conditioning` reports the distribution of the conditioning number by decade,
how many sweeps had `Det` *bitwise* zero, and how many of the coplanar ones were
against an in-face edge. The distribution is what tells you whether an epsilon
change is worth trying: on the recordings so far, over 99.8% of near-coplanar
sweeps have `Det` exactly zero, so they are unsolvable rather than badly
conditioned, and no threshold reaches them.

---

## Reading a recording

Plain text, whitespace-delimited, one record per line. Doubles are written
`%.17g` and floats `%.9g`, so everything round-trips exactly.

```
tautrope-recording 2                  format version
engine 5.8.2-...                      engine that captured it
shapes 4                              then, per shape:
  vertices N / v x y z
  edges N / e a b                     indices into vertices
  vertedges N / ve count e0 e1 ...    edges adjacent to each vertex
  edgerotations N / q x y z w
  corners N / c 0|1
initial 2 / p ...                     rope state entering frame 0
nextpointid 3                         point id allocator entering frame 0
frames 2335                           then, per frame:
  frame
  start x y z                         simulation inputs
  end x y z
  maxlength f
  deltatime f                         metadata only, see below
  movement N / p id x y z shape edge vert
  collision N / p ...
  pruning N / p ...
```

Each frame carries the rope **after each of the three phases**, not just at end
of frame. A point that the collision phase inserts and the pruning phase removes
within one frame is invisible in an end-of-frame snapshot — and that is exactly
what the convergence defect does. Look at all three.

`id` is stable for a point's lifetime and never reused, so diffing id sets
between consecutive snapshots tells you what was inserted and removed. `shape`,
`edge` and `vert` are `-1` when the point is not attached to that feature.

---

## Things that will bite you

**Build with `Standalone/b.bat`, not bare `cmake --build`.** The compiler needs
the MSVC environment, and `vcvars64.bat` costs ~1.5 s of what is otherwise a
~0.2 s loop. `b.bat` caches that environment into `msvcenv.txt` on first use
(gitignored, machine-specific) and configures the build dir if it is missing.
Calling `cmake --build build` from a shell without the environment fails with
`cannot open include file: 'cstdint'` -- and if you then run the test binary
anyway, you are testing the *previous* build. Check the build succeeded.

**`DeltaTime` is not a simulation input.** `UpdateRope(Start, End, MaxLength)`
does not take it. State carries frame to frame through the rope points.
Determinism needs the shape set, the initial state, `nextpointid`, and the
per-frame endpoints — nothing else. `deltatime` is recorded as metadata so a
recording can be related back to what the human saw.

**`nextpointid` cannot be derived from the initial points.** Ids belonging to
points pruned before recording started have already advanced the counter. This
was a real bug; the verification in step 2 is what caught it.

**Format version 2 rejects version 1 recordings.** By design — v1 lacks
`nextpointid` and would replay with different ids. Recapture rather than patch.

**Floating point is pinned, and the guards are in core.** `TautRopeCore/Core.h`
refuses to compile under `/fp:fast`, `/fp:strict`, a raised `/arch:` baseline, or
fast-math, and requires C++20 to match what UBT gives the editor. Do not
"simplify" these away: bit-identity between the editor and this build is the
foundation of the whole loop. Optimisation level is deliberately *not* pinned —
a Debug build reproduces the editor's optimised build bit-identically, so
debugging with `/Od` is safe.

**Core must depend on UE's `Core` module even though it includes no UE headers.**
UBT only wires up a module's `operator new`/`delete` when `CORE_API` is defined,
which happens only for modules that depend on `Core`. Without it, core allocates
on the CRT heap while the glue frees on `FMemory`'s, and every `std::vector`
crossing the boundary is a cross-heap free. That crashed on the first recording
flush. Do not remove the dependency to make core look purer.

**Do not define helpers in anonymous namespaces in the glue module.** UBT
compiles glue files together in unity translation units, where identically named
helpers in anonymous namespaces collide. Shared conversions live in
`TautRopeConvert.h`.

**Only convex and box simple collision are sampled.** Sphere and capsule are
ignored. A static mesh with only sphere collision contributes no shapes, and the
sampler logs that rather than failing silently.

**If a recording has 0 shapes, the rope cannot collide.** The human needs to
press **Populate Static Shapes** on each `ATautRopeCollisionVolumeActor` and save
the level. `--info` will tell you when a recording never gained an intermediate
point, which means it exercises only the straight-line case and is worthless to
debug against.

---

## Tests

`Standalone/Tests/`. Two kinds:

- `TEST(Name)` — must pass. A red suite means a regression.
- `TEST_PENDING(Name)` — documents behaviour known to be wrong. Runs and
  reports, but does not fail the suite.

The two known defects are seeded as `TEST_PENDING`:

- `Defect_RopeNeverPenetratesShape` — the rope sometimes intersects a shape after
  sliding over a vertex.
- `Defect_RopeSettlesWhenInputsStopMoving` — the rope slows as two or more points
  converge on edges toward the same vertex.

**Both currently pass**, which means the synthetic sweep in `SimulationTests.cpp`
does not reproduce either bug. The invariants and the probes that measure them
exist; the motion that triggers them does not. Getting a real repro into a test
is the highest-value thing available — until then those two tests assert nothing.

When you fix a defect, promote its test from `TEST_PENDING` to `TEST` so a
regression fails the suite. The runner reminds you when a pending test starts
passing.

The shape fixture in `Support.h` is captured from the real level. Prefer
capturing over hand-writing: the edge rotations come from UE's
`FRotationMatrix::MakeFromXZ`, and an approximation would give you a fixture that
quietly differs from what the simulation sees.
