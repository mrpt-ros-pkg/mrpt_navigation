# AGENTS.md

Guidance for AI coding agents working in this repository.

## Repository overview

`mrpt_navigation` is a ROS 2 metapackage wrapping [MRPT](https://github.com/MRPT/mrpt/)
localization and navigation functionality. Active development happens on the
`ros2` branch (this branch); `ros1` is frozen, no further development. Package
list and responsibilities:

- **mrpt_map_server** — node that loads/publishes a static metric map (gridmap,
  MRPT map, or mp2p_icp map), not limited to occupancy grids like classic
  ROS 1 `map_server`.
- **mrpt_pf_localization** — particle-filter 2D self-localization node (like
  `amcl`, but supports multiple PF algorithms, multi-height gridmaps,
  range-only localization, etc).
- **mrpt_pointcloud_pipeline** — maintains a local obstacle map from recent
  sensor readings; supports point cloud filtering pipelines (volume/area
  filters, downsampling, 2D scan obstacle memory, etc).
- **mrpt_reactivenav2d** — pure reactive navigator for polygonal robots in 2D.
- **mrpt_tps_astar_planner** — SE(2)-lattice A* path planner based on PTG
  trajectories.
- **mrpt_trajectory_follower** — node that accurately follows a reference
  pose+speed path (pure pursuit) with minimal predictive safety (footprint
  sweeps stop/slow before obstacles); wraps `mpp::TrajectoryFollower`.
- **mrpt_msgs_bridge** — C++ conversions between `mrpt_msgs` ROS messages and
  native MRPT classes.
- **mrpt_nav_interfaces** — msg/srv/action definitions shared by the other
  packages.
- **mrpt_tutorials** — launch/config files and example datasets/maps tying
  the other packages together.

All packages follow REP-2003 for ROS 2 topic QoS.

## Code style rules

- Format with `clang-format-14`; run `bash scripts/formatter.sh` (or
  `--check`) from the repo root before committing. CI enforces this
  (`.github/workflows/check-clang-format.yml`).
- No one-line statement bodies: `if (foo) bar;` → always brace and put the
  body on its own line.
- One variable declaration per line (no `int a, b;`).
- No em/en dashes ("—") in code or comments; use plain alternatives.
- American English spelling.
- Use anonymous namespaces instead of `static` for internal linkage.
- Comments explaining a fix should state generic reasoning, not
  dataset/case-specific detail (e.g. not "fixes XXX failing on dataset YYY").
- Avoid unnecessary complexity; keep changes minimal and scoped.
- At present, this package uses MRPT 2.x API. It will soon be ported to MRPT 3.x (at that moment, update this line).
- Don't sign commits/PRs as an AI, and don't reference internal plan/design
  document section numbers in commits, PRs, or code comments.
- If a change affects something documented here, keep this file in sync;
  but keep additions terse.

## Build & test

Standard colcon workspace build from the workspace root (not this package
root):

```bash
colcon build --symlink-install --packages-up-to mrpt_navigation
colcon test --packages-select <pkg>
colcon test-result --verbose
```

CI (`.github/workflows/build-ros.yml`) builds and tests against Humble and
Jazzy (stable + testing repos) on every push.

## Release process (maintainer only)

Releases are cut by the maintainer (Jose Luis Blanco-Claraco) using the
standard two-step ROS release flow. **Agents should not perform releases
unless explicitly asked to.**

1. **`catkin_prepare_release`** run from the repo root on the `ros2` branch:
   - Bumps `<version>` in every package's `package.xml` to the new version
     (all packages in this repo are versioned in lockstep).
   - Regenerates each package's `CHANGELOG.rst` from git log history since
     the last release tag (via `catkin_generate_changelog`), grouped by
     version with a `Contributors:` line.
   - Creates a commit (message = new version number, e.g. `2.5.0`) and an
     annotated git tag matching the version.
2. **`bloom-release mrpt_navigation --ros-distro <distro>`** (run once per
   supported ROS distro, e.g. `rolling`, `kilted`, `jazzy`, `humble`):
   - Pushes the release to the corresponding `*-release` repo
     (`ros2-gbp/mrpt_navigation-release`).
   - Opens a PR against `ros/rosdistro` bumping the version in
     `<distro>/distribution.yaml`. Once merged, the ROS buildfarm picks it up
     and produces binary `.deb` packages (tracked by the badges in
     `README.md`).

Consequences for day-to-day work:
- Never hand-edit `<version>` in `package.xml` or `CHANGELOG.rst` — these are
  machine-generated at release time from git history.
- Git tags matching package versions (e.g. `2.5.0`) are release markers; don't
  create/move them manually.
- PR/commit titles matter: `catkin_generate_changelog` pulls merge commit
  titles and first-line commit messages verbatim into `CHANGELOG.rst`, so
  keep them descriptive and free of internal doc/section references.
