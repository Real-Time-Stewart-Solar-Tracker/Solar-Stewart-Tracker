# Contributing

## Purpose

This repository contains the ENG5220 real-time embedded programming project codebase.
Contributions should preserve the taught architecture:

- blocking I/O wakes threads
- callbacks pass events between classes
- setters send control outputs
- no polling loops in the realtime path
- no sleep-based fake realtime control paths
- reusable code stays modular
- tests and documentation stay in sync with the code

## Workflow

1. Create an issue before starting substantial work.
2. Use a short feature or fix branch.
3. Keep each branch focused on one change.
4. Make small, descriptive commits.
5. Open a pull request with a clear summary of:
   - what changed
   - why it changed
   - how it was tested

## Branch naming

Examples:

- `feature/manual-input-queue`
- `feature/latency-csv-export`
- `fix/imu-whoami-config`
- `fix/cmake-optional-deps`

## Coding expectations

### Realtime/event-driven structure

Use the taught userspace pattern:

- one blocking loop per event source
- short callback after wake-up
- heavier work moved into the appropriate worker thread

Examples of acceptable event sources:

- GPIO edge events
- camera callbacks / blocking frame acquisition
- blocking queue waits
- timerfd or equivalent blocking timing only where a real event source does not exist

Avoid:

- polling loops
- `sleep`, `usleep`, or `sleep_for` in the control path
- GUI-driven control execution
- heavy work inside callbacks

### C++ style

- Use C++17.
- Prefer `std::thread`, STL containers, and RAII.
- Prefer references and value types over raw owning pointers.
- Avoid `malloc`, `free`, and unnecessary `new` / `delete`.
- Keep interfaces narrow and responsibilities clear.
- Preserve existing naming and directory structure unless there is a strong reason to change it.

### Comments and documentation

- Keep comments short and technical.
- Do not leave patch-note comments in code.
- Public headers should use Doxygen comments.
- Update Doxygen when public APIs change.

## Tests

Before opening a pull request, run the relevant checks.

### Core build and tests

```bash
./scripts/build_core.sh build
./scripts/test_core.sh build
Raspberry Pi hardware smoke tests
./scripts/build_pi_debug.sh build-pi
./scripts/test_pi_hw.sh build-pi
Doxygen
./scripts/build_docs.sh
Latency evidence

If a change affects the realtime path, update the evidence.

Example:

./scripts/run_latency.sh build artefacts/latency.csv solar_tracker

Store:

raw CSV
exact command used
platform details
short summary of average, max, and jitter
Pull request checklist
 code builds
 relevant tests pass
 no debug prints left in place
 no temporary comments remain
 public headers are documented
 docs stay consistent with the code
 realtime path still follows blocking-I/O event-driven design