# Contributing Guidelines

Thank you for contributing to the Solar Stewart Tracker project.

This project is structured as a real-time embedded, event-driven C++ system.
Please follow the guidelines below to ensure code quality, reproducibility,
and architectural consistency.

---

## 1. Project Philosophy

This project follows:

- SOLID principles
- Event-driven multi-threaded architecture
- Hardware abstraction layers
- Reproducible CMake builds
- Clear separation between:
  - sensors
  - control
  - actuators
  - system orchestration
  - UI

Do not introduce tight coupling between modules.

---

## 2. Branching Model

- `main` → stable, tested code only
- Feature branches → `feature/<short-description>`
- Bug fixes → `fix/<short-description>`

Never push directly to `main`.

Example:

    git checkout -b feature/qt-overlay
    git commit -m "Add Qt overlay rendering"
    git push origin feature/qt-overlay

Open a Pull Request for review before merging.

---

## 3. Code Style

- C++17 standard
- RAII preferred over manual memory management
- No raw owning pointers
- Use `std::unique_ptr` or `std::shared_ptr`
- Avoid global variables
- Use `const` wherever possible
- Prefer clear names over short names

Formatting:

- 4 spaces indentation
- Braces on same line
- One class per header/source pair

---

## 4. Threading Rules

This system is event-driven.

- No busy waiting
- No `sleep()` loops for logic timing
- Use condition variables or callbacks
- Avoid blocking inside callbacks

UI must never block control or camera threads.

---

## 5. Hardware Safety

When modifying actuator code:

- Always ensure safe clamping of servo limits
- Never bypass safety checks
- Ensure neutral output on stop()

Real hardware must never move unexpectedly.

---

## 6. Testing

Before submitting a PR:

- Build from clean directory:

      rm -rf build
      cmake -S . -B build
      cmake --build build -j4

- Ensure:
  - No warnings
  - All tests pass
  - Application runs correctly

---

## 7. Commit Message Format

Use clear and descriptive commit messages.

Good examples:

    Add YUV420 stride-safe mmap handling
    Fix Qt overlay alignment bug
    Refactor ServoDriver for better safety checks

Avoid:

    fix
    update
    stuff

---

## 8. Documentation

If you modify architecture:

- Update `docs/system_architecture.md`
- Update relevant diagrams
- Update Doxygen comments

Public APIs must be documented.

---

## 9. Large Changes

For architectural or structural changes:

- Open an issue first
- Explain design reasoning
- Wait for review before implementation

---

## 10. License

By contributing, you agree that your contributions will be licensed
under the MIT License of this project.

---

Thank you for maintaining code quality and system integrity.