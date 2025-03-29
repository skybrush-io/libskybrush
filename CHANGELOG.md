# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
