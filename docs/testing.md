# Testing and Reliability Strategy

This document describes the unit testing approach used in the Solar Stewart Tracker project.

The goal is to provide quantitative evidence of correctness and robustness in accordance with A1-level assessment criteria.

---

# 1. Testing Philosophy

The project follows a structured reliability approach:

- Unit tests for algorithmic components
- Deterministic synthetic inputs
- Automated execution via CTest
- Clear PASS/FAIL reporting
- No hardware dependency for logic validation

Hardware-dependent components (e.g., libcamera, servo driver) are isolated behind interfaces and not tested at unit level.

---

# 2. Test Architecture

Tests are implemented as a separate executable:

solar_tracker_tests

The test framework is a lightweight custom runner located in:

tests/test_main.cpp

Test macros are shared via:

tests/test_common.hpp

This avoids external dependencies while keeping the structure professional and extensible.

CTest integration is provided via CMake:

    ctest -C Debug --output-on-failure

---

# 3. Unit Tests Implemented

## 3.1 SunTracker Tests

File:
tests/test_suntracker.cpp

Validated behaviour:

- No bright pixels → confidence = 0
- Bright circular region → centroid approximately correct
- Larger bright region → higher confidence

Synthetic grayscale frames are generated programmatically.
This ensures fully deterministic and repeatable testing.

These tests validate:

- Thresholding logic
- Pixel counting
- Centroid calculation
- Confidence scaling behaviour

---

## 3.2 Controller Tests

File:
tests/test_controller.cpp

Validated behaviour:

- Low confidence → no motion
- Deadband region → no motion
- Outside deadband → motion generated
- Output clamping respected

These tests verify:

- Safety gating logic
- Proportional control response
- Saturation limits
- Deterministic output behaviour

---

# 4. Reproducible Test Execution

From build directory:

    cmake --build . --config Debug --target solar_tracker_tests
    ctest -C Debug --output-on-failure

Expected result:

    100% tests passed

Alternatively, run directly:

    .\Debug\solar_tracker_tests.exe

Which reports individual test cases.

---

# 5. Coverage Scope

Covered:

- Vision centroid and confidence logic
- Control law behaviour
- Safety constraints
- Clamp and deadband logic

Not covered (by design):

- Hardware drivers
- libcamera integration
- Servo hardware timing

These components are validated via runtime logging and latency monitoring instead.

---

# 6. Reliability Contribution

The testing framework ensures:

- Algorithm correctness
- Deterministic behaviour
- Safety boundary enforcement
- Confidence gating robustness

Combined with runtime latency measurement and structured state control,
this provides strong evidence of production-level embedded design.

---

# 7. Future Improvements

Potential extensions:

- Fault injection tests
- Timeout behaviour testing
- State machine tests
- Continuous integration (CI) automation

---

# Summary

The project includes:

- Automated unit testing
- Deterministic synthetic inputs
- Clear pass/fail reporting
- Reproducible CTest integration
- Separation of logic and hardware

This demonstrates a professional testing strategy suitable for A1 assessment.