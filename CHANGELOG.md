# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Breaking changes

- Removed support for light program opcodes 0x0E to 0x15 (reset clock, set or fade
  to a color taken from input channels, jump, triggered jump, and setting pyro
  channels). These opcodes originate from the legacy Arduino-based light controller
  and were never used in Skybrush drone shows. The light player now treats them as
  unknown commands and stops the execution of the light program when it encounters
  one.

- The legacy yaw track that old show files may embed in trajectory blocks is now
  ignored; the yaw of the drone is encoded in yaw control blocks instead. The yaw
  component of positions and velocities evaluated from a trajectory is always zero,
  `sb_trajectory_t.use_yaw` is always false, and the trajectory builder does not
  write yaw information any more. `SB_TRAJECTORY_USE_YAW` is deprecated and ignored.

- `sb_show_metadata_update_from_binary_file()`,
  `sb_gcs_light_control_setup_update_from_binary_file()` and their in-memory
  variants now reset the object to its defaults when the show file contains no
  corresponding block, as a missing block is equivalent to an empty one. They
  still return `SB_ENOENT` in this case.

- Event list blocks whose events are not sorted by timestamp are now rejected with
  `SB_ECORRUPTED` instead of `SB_EINVAL`, and the event list is left intact.
  Events with unknown types are not rejected any more; they are kept as opaque
  events.

### Added

- Added `sb_buffer_update()` to replace the contents of an already initialized
  buffer with the contents of another one; the destination buffer becomes a
  view if the source buffer is a view and it becomes an owned buffer holding
  a copy of the contents if the source buffer is owned.

- Added `sb_color_palette_update()` to replace the contents of an already
  initialized color palette with the contents of another one; the destination
  palette is backed by a view if the source palette is backed by a view and it
  owns a copy of the colors if the source palette owns its colors.

- Added `sb_event_list_update_from_buffer()`, which was declared in the public
  header but never implemented.

### Changed

- `sb_buffer_init_from_bytes()` now accepts an empty array (the buffer takes
  ownership of it and can grow later) and rejects a null pointer with
  `SB_EINVAL`. It used to reject empty arrays and accept null pointers.

- `sb_color_palette_update_from_bytes()` now accepts an empty buffer; the
  palette becomes empty and takes ownership of the buffer. It used to reject
  empty buffers with `SB_EINVAL`.

### Fixed

- Fixed an out-of-bounds write in the light player when a light program nests
  loops more than four levels deep. Exceeding the maximum nesting depth now stops
  the execution of the light program.

- Fixed out-of-bounds reads when a show file loaded into memory is shorter than
  the length declared in one of its block headers; such files are now rejected
  with `SB_EREAD`, consistently with show files read from a file descriptor.

- Fixed looking up a block in a show file read from a file descriptor when the
  file ends in the middle of a preceding block; this now fails with `SB_EREAD`
  instead of reporting that the block does not exist, consistently with show
  files loaded into memory.

- Fixed empty light program blocks being rejected when the show file is read
  from a file descriptor.

- Fixed out-of-bounds reads on truncated trajectory, yaw control and RTH plan
  blocks. Such blocks are now rejected with `SB_ECORRUPTED` and the object being
  updated is left intact.

- Fixed a light program fade following a zero-duration fade starting from the
  color before the zero-duration fade.

- Fixed event lists being left half-loaded when an event list block could not be
  parsed.

- Fixed the declaration of `sb_yaw_player_dump_current_setpoint()` in the public
  header, which used to refer to a non-existent function.

- Fixed the trajectory statistics calculator reporting the start of the final
  descent as the landing time when the preferred descent was zero. The landing
  time is now the end of the trajectory in this case, consistently with
  `sb_trajectory_propose_landing_time_sec()`.

- Fixed the trajectory statistics calculator reporting zero velocity at the
  landing time when the landing time is the end of the trajectory and the drone
  is still moving there. The velocity is now the terminal velocity of the last
  segment, so `sb_trajectory_replace_end_to_land_at()` stitches the landing
  segment smoothly to the end of such trajectories.

- Fixed `sb_trajectory_replace_end_to_land_at()` and its variant with a terminal
  velocity cutting the trajectory and modifying the statistics before failing
  when the new landing position is not below the position at the landing time.
  Such calls now fail with `SB_EINVAL` without modifying anything.

- Fixed the light player returning the color of the previous segment of a light
  program when queried exactly at a timestamp where multiple commands that take
  no time (e.g., loop markers) were due.

- Fixed the light player hanging forever on light programs containing an infinite
  loop whose body takes no time. Such loops now stop the execution of the light
  program when the end of their body is reached.

- Fixed `sb_trajectory_builder_move_to_in_time()` generating a trajectory that
  shoots far beyond the target when the maximum acceleration is so large that
  the acceleration phase is shorter than one millisecond.

## [5.3.1] - 2026-09-30

### Fixed

- Added missing sub-headers in top-level header file.

## [5.3.0] - 2026-09-30

### Added

- Added two new subtypes for light events: `SB_EVENT_SUBTYPE_LIGHT_SET_PIXEL_X` and
  `SB_EVENT_SUBTYPE_LIGHT_SET_PIXEL_Y`. They carry the X and Y coordinates of the pixel
  represented by the drone in interactive GCS-controlled mode as a 12-bit unsigned
  integer.

- Added a new show metadata module (`skybrush/metadata.h`) that parses
  the new show metadata block of the Skybrush binary show file format. The
  body of the block is a tag-length-value stream that currently carries the
  unique ID of the show (a four-byte binary blob) and the index of the drone
  within the show (an unsigned 16-bit little-endian integer).

## [5.2.0] - 2026-09-23

### Added

- Added support for marking a scene as "dynamic", meaning that users are free to adjust
  its duration when the underlying trajectory changes. Used by the ArduPilot firmware
  to identify the main show scene and whether it is allowed to extend or shorten its
  duration when the underlying trajectory is altered.

- Added a new event type (light control) and a new subtype for light events (set
  source). These will be used to implement interactive control of drone lights from a
  ground station.

- Added a new, self-contained tag-length-value (TLV) parser module
  (`skybrush/formats/tlv.h`) that will be used to parse the bodies of certain
  block types in the Skybrush binary show file format, starting with the new
  GCS light control setup block.

- Added a new color palette module (`skybrush/palette.h`) that represents a
  color palette as a raw buffer of packed RGB colors, with functions to clear
  a palette, to fill it with data from a buffer (either borrowed or owned,
  similar to how `sb_trajectory_t` handles its internal buffer), to destroy a
  palette, and to get the i-th color of a palette. Will be used by the new
  GCS light control setup block.

- Added a new GCS light control setup module (`skybrush/gcs_light_control.h`)
  that parses the new GCS light control setup block of the Skybrush binary
  show file format.

## [5.1.0] - 2026-09-15

### Added

- Added `sb_screenplay_find_first_scene_with_tag()` to find the first scene in a
  screenplay that has a given tag.

- Added `sb_rth_plan_entry_clear()` to clear an RTH plan entry (i.e. make it
  represent a landing in place without any delays).

- Added `sb_trajectory_replace_end_to_land_at_with_velocity()` to replace the
  landing part of a trajectory so it ends at another point with a given
  terminal velocity (not necessarily zero).

### Fixed

- Fixed a bug in `sb_trajectory_replace_end_to_land_at()` that resulted in
  incorrect durations when the Z coordinate of the final point was not zero.

## [5.0.0] - 2026-06-20

### Breaking changes

- Migrated several data structures to reference counting to make it easier to
  share them between different parts of an application without worrying about
  ownership and lifetimes. Functions returning pointers to reference-counted data
  structures clearly document whether they return a new reference (where the ownership
  of the reference is passed to the caller and the caller is responsible to call
  `SB_DECREF()` later) or a borrowed reference (where the caller does not own the
  object and _must_ call `SB_INCREF()` on it if it wants to hold on to the object
  for a longer time). Functions that take pointers to reference-counted objects
  assume that they receive a _borrowed_ reference and increase the reference count
  if needed, unless documented otherwise. The only exceptions are `_init()` functions
  for a reference-counted type; these assume that they receive an uninitialized
  object and the caller will own a reference to the initialized object when the
  initializer function returns.

- `sb_trajectory_t` is now reference-counted. Use `sb_trajectory_new()` to
  allocate a new trajectory on the heap or `sb_trajectory_init()` to initialize
  a trajectory that is allocated on the stack. `sb_trajectory_destroy()`is removed;
  you should call `SB_DECREF()` on the trajectory object instead when you want to
  release it. Trajectories without active references are automatically freed.

- `sb_light_program_t` is now reference-counted. Use `sb_light_program_new()` to
  allocate a new light program on the heap or `sb_light_program_init()` to
  initialize a light program that is allocated on the stack.
  `sb_light_program_destroy()` is removed; you should call `SB_DECREF()` on the
  light program instead when you want to release it. Light programs without
  active references are automatically freed.

- `sb_yaw_control_t` is now reference-counted. Use `sb_yaw_control_new()` to allocate
  a new yaw control object on the heap or `sb_yaw_control_init()` to initialize
  a yaw control object that is allocated on the stack. `sb_yaw_control_destroy()`
  is removed; you should call `SB_DECREF()` on the yaw control object instead when
  you want to release it. Yaw control objects without active references are
  automatically freed.

- `sb_event_list_t` is now reference-counted. Use `sb_event_list_new()` to allocate
  a new event list on the heap or `sb_event_list_init()` to initialize an event
  list that is allocated on the stack. `sb_event_list_destroy()` is removed;
  you should call `SB_DECREF()` on the event list instead when you want to
  release it. Event lists without active references are automatically freed.

- `sb_rth_plan_t` is now reference-counted. Use `sb_rth_plan_new()` to allocate
  a new RTH plan on the heap or `sb_rth_plan_init()` to initialize an RTH plan
  that is allocated on the stack. `sb_rth_plan_destroy()` is removed;
  you should call `SB_DECREF()` on the RTH plan instead when you want to
  release it. RTH plans without active references are automatically freed.

- `sb_trajectory_replace_end_to_land_at()` now takes an `sb_vector3_t` to specify the
  desired landing coordinate instead of an `sb_vector3_with_yaw_t`.

- `sb_trajectory_update_from_rth_plan_entry()` now takes an `sb_vector3_t` to specify
  the starting position instead of an `sb_vector3_with_yaw_t`.

### Added

- Added `sb_screenplay_t` and `sb_screenplay_scene_t` to manage a complex
  sequence of combined trajectory, light program, yaw and event list objects,
  with a varying setup of time axis objects.

- Added `sb_vector3_t` type to cater for use-cases where the yaw component is not
  needed.

- Added `sb_light_program_set_constant_color()` to set a constant color in a light
  program object.

- Added `sb_yaw_control_set_constant_yaw()` to set a constant yaw value in a yaw
  control object.

- Added `sb_trajectory_player_clone()` to create a clone of a trajectory player that
  allows one to move forward on the time axis without affecting the state of the
  original player that the clone was created from.

- Added `sb_trajectory_builder_move_to_in_time()` to create a three-phase transition
  in a trajectory from one point to another such that the initial and terminal
  velocities are zeros and the entire transition takes a specified amount of time.

- RTH plans now have a default landing altitude, a default landing velocity and a
  default acceleration limit. These are applied to RTH plan entries when they are
  extracted from the RTH plan.

### Changed

- `sb_realloc()` now reallocates memory using `sb_malloc()` and `sb_free()` instead of
  using `realloc()` directly. This is because on some platforms (such as the ArduPilot
  codebase with gcc 13) the `realloc()` implementation seems to work incorrectly and
  does not copy the final few bytes of small memory blocks. This might be a bug in
  ArduPilot or ChibiOS, or we might be doing something with `realloc()` that is not
  supported.

### Fixed

- Fixed a bug in `sb_scale_update_vector3()` that made the function ignore the Z
  coordinate.

## [4.3.0] - 2025-10-15

### Added

- Added `sb_trajectory_replace_end_to_land_at()` to "bend" the landing segment
  of a trajectory towards a nearby point in a smooth manner in order to
  achieve more accurate landing back into docking stations.

## [4.2.1] - 2025-10-05

### Fixed

- Added missing `sb_light_program_init_from_bytes()` declaration in the
  appropriate header.

## [4.2.0] - 2025-06-02

### Added

- Added some helper functions to manipulate event blocks to allow the
  insertion of "pyro off" events as needed.

## [4.1.0] - 2025-05-30

### Added

- Added support for event blocks in `.skyb` files and an event player that
  iterates over events from an event block.

## [4.0.0] - 2025-03-29

### Changed

- `sb_trajectory_init_from_binary_file_in_memory()` now creates a view into the
  existing buffer instead of making a copy.

- `sb_light_program_init_from_binary_file_in_memory()` now creates a view into the
  existing buffer instead of making a copy.

- `sb_yaw_control_init_from_binary_file_in_memory()` now creates a view into the
  existing buffer instead of making a copy.

- `sb_trajectory_destroy()` does not clear the trajectory any more.

This is the release that serves as a basis for changelog entries above. Refer
to the commit logs for changes affecting this version and earlier versions.
