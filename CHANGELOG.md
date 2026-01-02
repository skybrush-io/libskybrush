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
  object and _must_ call `SB_INCREF()` on it if it wants to hold on to the object for
  a longer time). Functions that take pointers to reference-counted objects assume
  that they receive a _borrowed_ reference and increase the reference count if needed,
  unless documented otherwise. The only exceptions are `_init()` functions for a
  reference-counted type; these assume that they receive an uninitialized object and
  the caller will own a reference to the initialized object when the initializer
  function returns.

- `sb_trajectory_t` is now reference-counted. `sb_trajectory_destroy()` is removed;
  you should call `SB_DECREF()` on the trajectory object instead when you want to
  release it. Trajectories without active references are automatically freed.

- `sb_light_program_t` is now reference-counted. `sb_light_program_destroy()` is removed;
  you should call `SB_DECREF()` on the light program instead when you want to
  release it. Light programs without active references are automatically freed.

- `sb_yaw_control_t` is now reference-counted. `sb_yaw_control_destroy()` is removed;
  you should call `SB_DECREF()` on the yaw control object instead when you want to
  release it. Yaw control objects without active references are automatically freed.

- `sb_event_list_t` is now reference-counted. `sb_event_list_destroy()` is removed;
  you should call `SB_DECREF()` on the event list instead when you want to
  release it. Event lists without active references are automatically freed.

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
