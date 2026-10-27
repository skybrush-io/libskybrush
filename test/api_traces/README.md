# Golden traces of the API-level behaviour

The traces in this directory record what libskybrush *computes*, as opposed to
the traces of the binary show file format (maintained with the specification of
the format), which record what can be *read* from a show file. They are the
reference for other implementations of the library, e.g. the Rust rewrite: an
implementation is expected to produce the same results from the same inputs,
within the tolerances described below.

Every trace records the inputs of each computation next to its results, so it
can be replayed without the C library.

## Files

| File | Generator | Inputs |
|---|---|---|
| `<fixture>.json` | `generate_api_trace` | The show file `test/fixtures/<fixture>.skyb` |
| `scenarios.json` | `generate_api_scenarios` | Defined in the generator and recorded in the trace |
| `tolerances.json` | | Tolerances of the comparison |

The per-fixture traces contain the following sections, where applicable:

- `trajectory`: duration, end position and bounding box of the trajectory
- `stats`: trajectory statistics with various settings of the calculator; only
  the requested components are recorded
- `proposed_takeoff_times`, `proposed_landing_times`
- `cuts`: the trajectory cut at various times, sampled around the cut
- `landings`: the end of the trajectory replaced with a landing; the trajectory
  and the statistics are recorded on failure as well, since they must be left
  intact then
- `rth_trajectories`: every distinct entry of the RTH plan converted to a
  trajectory, starting from the position of the drone at the time of the entry
- `light_player`: the timestamps where the light player reports a change
- `event_player`: the index of the next event after seeking to each sample time

`scenarios.json` contains the following sections:

- `builder`: sequences of trajectory builder operations with the result of each
  operation and the segments of the resulting trajectory
- `rth_conversions`: RTH plan entries converted to trajectories
- `time_axes`: forward and reverse mapping of time axes
- `screenplays`: screenplays with multiple scenes, played by the show controller
- `motion`: the public motion helpers
- `colors`: the public color helpers

Trajectories and light programs used as inputs in `scenarios.json` are recorded
as the hex-encoded bodies of their blocks, as defined by the specification of
the binary show file format.

## Conventions

- Samples are taken every `sample_interval_ms` milliseconds. Arrays of samples
  that do not start at zero record their first timestamp in `samples_from_ms`.
- Positions are in millimeters, velocities in millimeters per second, angles in
  degrees and times in seconds or milliseconds, as indicated by the suffix of
  the key (see below).
- Non-finite floating-point values (infinities and NaN) are recorded as `null`.
- Errors are recorded by name: `SUCCESS`, `ENOMEM`, `EINVAL`, `EOPEN`, `ECLOSE`,
  `EREAD`, `EWRITE`, `EREADWRITE`, `EPARSE`, `TIMEOUT`, `ELOCKED`, `FAILURE`,
  `EUNSUPPORTED`, `EUNIMPLEMENTED`, `EPERM`, `EFULL`, `EEMPTY`, `EAGAIN`,
  `ENOENT`, `ECORRUPTED` and `EOVERFLOW`. Other implementations are expected to
  map their own errors to these names.
- The `generator` key is informational and is not compared.

## Tolerances

The tolerance of a number is determined by the suffix of its key, or the key of
the closest enclosing array; see `tolerances.json`. A number matches if it
differs from the golden value by at most `abs + rel * |golden value|`.
Integers without a known suffix (colors, timestamps and durations in
milliseconds, counts, indices and flags) must match exactly.

Two suffixes are specific to the API traces:

- `_units`: positions of built trajectories in the units of the scale of the
  trajectory, which may differ by one unit as coordinates are quantized
- `_approx8`: 8-bit color components computed with `powf()` or `logf()`, which
  may differ by one between math libraries

## Regenerating the traces

The traces are checked by the `api_traces` and `api_scenarios` tests. After an
intentional change of the behaviour, regenerate them with

    cmake --build <build directory> --target update_api_traces update_api_scenarios

and review the differences before committing them.
