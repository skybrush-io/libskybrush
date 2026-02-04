# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [main]

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
