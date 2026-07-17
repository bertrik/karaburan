# Repository instructions

- Write all Markdown documentation in English.
- Keep existing Markdown documents in a single language; do not mix Dutch and English within a document.
- Use clear, project-specific examples and keep command-line instructions directly executable where practical.

## Local CI check

Before handing off changes that affect ROS code, package metadata, launch files,
Docker tests, or GitHub Actions, run the complete local CI check from the
repository root:

```bash
docker build --pull --no-cache -f docker/Dockerfile.ros-jazzy-test -t karaburan-ros-test .
docker run --rm karaburan-ros-test
```

Both commands must succeed. The image builds every package in `ros_ws/src`; the
container then compiles the Python sources, runs all `colcon` tests, validates
the storage and instrument launch files, and creates and inspects a sample MCAP
recording. If packages are added or dependency metadata changes, also verify
that `.github/workflows/action.yml` includes the package and that `rosdep` can
resolve every key.
The `--pull --no-cache` flags are intentional: GitHub Actions installs current
ROS lint packages, so a cached local apt layer can otherwise miss new lint
rules. Treat all lint warnings as failures. In particular, ROS
`ament_flake8` requires a blank line between a class declaration and its first
method (`CNL100`).
